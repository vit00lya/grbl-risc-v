/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


// Some useful constants.
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment 
#define REQ_MM_INCREMENT_SCALAR 1.25                                   
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the 
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
#define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)


// Stores the planner block Bresenham algorithm execution data for the segments in the segment 
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use. 
typedef struct {  
  uint8_t direction_bits;
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper 
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by 
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint16_t n_step;          // Number of step events to be executed for this segment
  uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
  // Used by the bresenham line algorithm
  uint32_t counter_x,        // Counter variables for the bresenham line tracer
           counter_y, 
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif
  
  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  uint8_t step_outbits;         // The next stepping-bits to be output
  uint8_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       // Steps remaining in line segment motion  
  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;   // Pointer to the block data for the segment being executed
  segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;
static stepper_t st;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// Step and direction port invert masks. 
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;   

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped 

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t st_block_index;  // Index of stepper common data block being prepped
  uint8_t flag_partial_block;  // Flag indicating the last block completed. Time to load a new one.

  float steps_remaining;
  float step_per_mm;           // Current planner block step/millimeter conversion scalar
  float req_mm_increment;
  float dt_remainder;
  
  uint8_t ramp_type;      // Current segment ramp state
  float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  float current_speed;    // Current speed at the end of the segment buffer (mm/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  float exit_speed;       // Exit speed of executing block (mm/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (mm)
  float decelerate_after; // Deceleration ramp start measured from end of block (mm)
} st_prep_t;
static st_prep_t prep;


/*    BLOCK VELOCITY PROFILE DEFINITION 
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity
  
  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the 
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and 
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  + 
                    +--------+ <- maximum_speed (= nominal_speed)          /|\                                         
                   /          \                                           / | \                      
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |                       
                  +-------------+                     current_speed -> +----+--+                   
                   time -->  ^  ^                                           ^  ^                       
                             |  |                                           |  |                       
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)
                    
  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters 
  are shown and defined in the above illustration.
*/


// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up() 
{
  // Enable stepper drivers.
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
  else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }

  if (sys.state & (STATE_CYCLE | STATE_HOMING)){
    // Initialize stepper output bits
    st.dir_outbits = dir_port_invert_mask; 
    st.step_outbits = step_port_invert_mask;
    
    // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
    #ifdef STEP_PULSE_DELAY
      // Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope.
      st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
      // Set delay between direction pin write and step command.
      OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
    #else // Normal operation
      // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
      st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
    #endif

    // Enable Stepper Driver Interrupt
    TIMSK1 |= (1<<OCIE1A);
  }
}


// Stepper shutdown
void st_go_idle() 
{
  // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.
  TIMSK1 &= ~(1<<OCIE1A); // Disable Timer1 interrupt
  TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Reset clock to no prescaling.
  busy = false;
  
  // Set stepper driver idle state, disabled or enabled, depending on settings and circumstances.
  bool pin_state = false; // Keep enabled.
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm) && sys.state != STATE_HOMING) {
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // Override. Disable steppers.
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } // Apply pin invert.
  if (pin_state) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
  else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }
}


/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. Grbl employs
   the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
   Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
   round-off errors and only requires fast integer counters, meaning low computational overhead
   and maximizing the Arduino's capabilities. However, the downside of the Bresenham algorithm
   is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step 
   pulse trains, or aliasing, which can lead to strange audible noises or shaking. This is 
   particularly noticeable or may cause motion issues at low step frequencies (0-5kHz), but 
   is usually not a physical problem at higher frequencies, although audible.
     To improve Bresenham multi-axis performance, Grbl uses what we call an Adaptive Multi-Axis
   Step Smoothing (AMASS) algorithm, which does what the name implies. At lower step frequencies,
   AMASS artificially increases the Bresenham resolution without effecting the algorithm's 
   innate exactness. AMASS adapts its resolution levels automatically depending on the step
   frequency to be executed, meaning that for even lower step frequencies the step smoothing 
   level increases. Algorithmically, AMASS is acheived by a simple bit-shifting of the Bresenham
   step count for each AMASS level. For example, for a Level 1 step smoothing, we bit shift 
   the Bresenham step event count, effectively multiplying it by 2, while the axis step counts 
   remain the same, and then double the stepper ISR frequency. In effect, we are allowing the
   non-dominant Bresenham axes step in the intermediate ISR tick, while the dominant axis is 
   stepping every two ISR ticks, rather than every ISR tick in the traditional sense. At AMASS
   Level 2, we simply bit-shift again, so the non-dominant Bresenham axes can step within any 
   of the four ISR ticks, the dominant axis steps every four ISR ticks, and quadruple the 
   stepper ISR frequency. And so on. This, in effect, virtually eliminates multi-axis aliasing 
   issues with the Bresenham algorithm and does not significantly alter Grbl's performance, but 
   in fact, more efficiently utilizes unused CPU cycles overall throughout all configurations.
     AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
   Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four 
   intermediate steps must be completed such that baseline Bresenham (Level 0) count is always 
   retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed. 
   Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
   be multiplied by any integer value, multiplication by powers of two are simply used to ease 
   CPU overhead with bitshift integer operations. 
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then 
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This 
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.
   
   NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick, 
   which for Grbl must be less than 33.3usec (@30kHz ISR rate). Oscilloscope measured time in 
   ISR is 5usec typical and 25usec maximum, well below requirement.
   NOTE: This ISR expects at least one step to be executed per segment.
*/
// TODO: Replace direct updating of the int32 position counters in the ISR somehow. Perhaps use smaller
// int8 variables and update position counters only when a segment completes. This can get complicated 
// with probing and homing cycles that require true real-time positions.
ISR(TIMER1_COMPA_vect)
{        
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR // Debug: Используется для определения времени ISR
  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt // Флаг занятости используется для того, чтобы избежать повторного ввода этого прерывания
  
  // Set the direction pins a couple of nanoseconds before we step the steppers // Установите направляющие штифты за пару наносекунд до того, как мы включим степперы
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);

  // Then pulse the stepping pins // Затем подайте импульс на шаговые штифты
  #ifdef STEP_PULSE_DELAY
    st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting. // Сохраните out_bits, чтобы предотвратить перезапись.
  #else  // Normal operation // Нормальная работа
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
  #endif  

  // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds, independent of the main Timer1 prescaler.
  // Включите таймер сброса шагового импульса, чтобы прерывание сброса шагового порта могло сбрасывать сигнал после
  // точных настроек.pulse_microseconds в микросекундах, независимо от основного таймера1.
  TCNT0 = st.step_pulse_time; // Reload Timer0 counter // Счетчик времени перезагрузки 0
  TCCR0B = (1<<CS01); // Begin Timer0. Full speed, 1/8 prescaler // Время начала 0. Полная скорость, предустановка на 1/8

  busy = true;
  sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time. 
         // NOTE: The remaining code in this ISR will finish before returning to main program.
         // Повторно включите прерывания, чтобы обеспечить своевременное срабатывание прерывания сброса шагового порта. 
         // ПРИМЕЧАНИЕ: Оставшийся код в этом ISR будет завершен до возврата к основной программе.
    
  // If there is no step segment, attempt to pop one from the stepper buffer
  // Если сегмента step нет, попробуйте извлечь его из буфера stepper
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    // Что-нибудь есть в буфере? Если да, загрузите и инициализируйте сегмент следующего шага.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      // Инициализируем новый сегмент шага и загружаем количество шагов для выполнения
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS is disabled, set timer prescaler for segments with slow step frequencies (< 250Hz).
        // Если функция AMASS отключена, установите предварительный масштабатор таймера для сегментов с низкой частотой шага (< 250 Гц).
        TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
      #endif

      // Initialize step segment timing per step and load number of steps to execute.
      // Инициализируйте синхронизацию сегмента шага для каждого шага и загрузите количество шагов для выполнения.
      OCR1A = st.exec_segment->cycles_per_tick;
      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow. // ПРИМЕЧАНИЕ: Иногда может быть равно нулю при медленном движении.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      // Если новый сегмент запускает новый блок планировщика, инициализируйте промежуточные переменные и счетчики.
      // ПРИМЕЧАНИЕ: Когда индекс данных сегмента изменяется, это указывает на новый блок планировщика.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];
        
        // Initialize Bresenham line and distance counters
        // Инициализировать счетчики линий и расстояний Брезенхэма
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask; 

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        // При включенном режиме НАКОПЛЕНИЯ отрегулируйте счетчики приращения по оси Брезенхэма в соответствии с уровнем НАКОПЛЕНИЯ.
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
      #endif
      
    } else {
      // Segment buffer empty. Shutdown.
      // Буфер сегмента пуст. Выключение.
      st_go_idle();
      bit_true_atomic(sys_rt_exec_state,EXEC_CYCLE_STOP); // Flag main program for cycle end // Помечает основную программу для завершения цикла
      return; // Nothing to do but exit. // Ничего не остается, как выйти.
    }  
  }
  
  
  // Check probing state. // Проверьте состояние зондирования.
  probe_state_monitor();
   
  // Reset step out bits. // Сбросить биты вывода шага.
  st.step_outbits = 0; 

  // Execute step displacement profile by Bresenham line algorithm // Выполнить профиль ступенчатого смещения с помощью линейного алгоритма Брезенхема
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif  
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
    else { sys.position[X_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif    
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
    else { sys.position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif  
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
    else { sys.position[Z_AXIS]++; }
  }  

  // During a homing cycle, lock out and prevent desired axes from moving. // Во время цикла наведения на цель заблокируйте и предотвратите перемещение нужных осей.
  if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }   

  st.step_count--; // Decrement step events count // Уменьшить количество событий шага
  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing. // Сегмент завершен. Отменить текущий сегмент и продолжить индексацию сегмента.
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  st.step_outbits ^= step_port_invert_mask;  // Apply step port invert mask   // Применить пошаговую маску инвертирования порта  
  busy = false;
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR // Debug: Используется для определения времени ISR
}


/* The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
   pulse. This should always trigger before the next Timer1 COMPA interrupt and independently
   finish, if Timer1 is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is 
   added to Grbl.
*/
// This interrupt is enabled by ISR_TIMER1_COMPAREA when it sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds) 
// completing one step cycle.

/* Прерывание сброса шагового порта: прерывание Timer0 OVF обрабатывает спадающий край шага
   пульс. Это всегда должно срабатывать перед следующим совместным прерыванием Timer1 и независимо от него
   завершаться, если Timer1 отключен после завершения перемещения.
   ПРИМЕЧАНИЕ: Конфликты прерываний между последовательным и шаговым прерываниями могут привести к задержкам на
несколько микросекунд, если они выполняются непосредственно друг перед другом. Это не имеет большого значения, но может
вызвать проблемы при высокой частоте выполнения, если
к Grbl добавлено другое высокочастотное асинхронное прерывание.
*/
// Это прерывание включается с помощью ISR_TIMER1_COMPAREA, когда оно устанавливает биты порта двигателя для выполнения
// шага. Это ISR сбрасывает порт двигателя через короткий промежуток времени (settings.pulse_microseconds) 
// завершение одноэтапного цикла.
ISR(TIMER0_OVF_vect)
{
  // Сбросить шаговые штифты (оставить направляющие штифты)
  // Reset stepping pins (leave the direction pins)
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK); 
  TCCR0B = 0; // Disable Timer0 to prevent re-entering this interrupt when it's not needed. // Отключите таймер0, чтобы предотвратить повторный ввод этого прерывания, когда оно не требуется.
}
#ifdef STEP_PULSE_DELAY

  // This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
  // initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
  // will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
  // The new timing between direction, step pulse, and step complete events are setup in the
  // st_wake_up() routine.

  // Это прерывание используется только тогда, когда включена функция STEP_PULSE_DELAY. В данном случае импульс step
  // инициируется по истечении периода времени STEP_PULSE_DELAY. Прерывание ISR TIMER2_OVF
  // затем активируется после соответствующих настроек.pulse_microseconds, как при нормальной работе.
  // Новый временной интервал между событиями direction, step pulse и step complete устанавливается в процедуре
  // st_wake_up().
  ISR(TIMER0_COMPA_vect) 
  { 
    STEP_PORT = st.step_bits; // Begin step pulse. // Запуск пошагового импульса.
  }
#endif


// Generates the step and direction port invert masks used in the Stepper Interrupt Driver.
// Генерирует маски инвертирования шага и направления порта, используемые в драйвере шагового прерывания.
void st_generate_step_dir_invert_masks()
{  
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= get_step_pin_mask(idx); }
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= get_direction_pin_mask(idx); }
  }
}


// Reset and clear stepper subsystem variables
// Сброс и очистка переменных шаговой подсистемы
void st_reset()
{
  // Initialize stepper driver idle state.
  // Инициализируйте состояние бездействия шагового драйвера.
  st_go_idle();
  
  // Initialize stepper algorithm variables.
  // Инициализируем переменные шагового алгоритма.
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer // Указатель блока планировщика, используемый буфером сегмента
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;
  
  st_generate_step_dir_invert_masks();
      
  // Initialize step and direction port pins. // Инициализируйте выводы порта шага и направления.
  STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask;
  DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;
}


// Initialize and start the stepper motor subsystem // Инициализируйте и запустите подсистему шагового двигателя
void stepper_init()
{
  // Configure step and direction interface pins // Настройка выводов интерфейса шага и направления
  STEP_DDR |= STEP_MASK;
  STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
  DIRECTION_DDR |= DIRECTION_MASK;

  // Configure Timer 1: Stepper Driver Interrupt // Настройка таймера 1: Прерывание работы шагового драйвера
  TCCR1B &= ~(1<<WGM13); // waveform generation = 0100 = CTC
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10)); 
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0)); // Disconnect OC1 output // Отсоединить выход OC1
  // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Set in st_go_idle().
  // TIMSK1 &= ~(1<<OCIE1A);  // Set in st_go_idle().
  
  // Configure Timer 0: Stepper Port Reset Interrupt // Настройка таймера 0: Прерывание сброса шагового порта
  TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); // Disconnect OC0 outputs and OVF interrupt. // Отключите выходы OC0 и прерывание OVF.
  TCCR0A = 0; // Normal operation // Нормальная работа
  TCCR0B = 0; // Disable Timer0 until needed // Отключите таймер0 до тех пор, пока это не потребуется
  TIMSK0 |= (1<<TOIE0); // Enable Timer0 overflow interrupt // Включить прерывание переполнения таймера 0
  #ifdef STEP_PULSE_DELAY
    TIMSK0 |= (1<<OCIE0A); // Enable Timer0 Compare Match A interrupt // Включить Таймер0 Для сравнения с прерыванием
  #endif
}
  

// Called by planner_recalculate() when the executing block is updated by the new plan.
// Вызывается функцией planner_recalculate(), когда исполняемый блок обновляется в соответствии с новым планом.
void st_update_plan_block_parameters()
{ 
  if (pl_block != NULL) { // Ignore if at start of a new block. // Игнорировать, если в начале нового блока.
    prep.flag_partial_block = true;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed. // Скорость ввода обновлений.
    pl_block = NULL; // Flag st_prep_segment() to load new velocity profile. // Установите флажок st_prep_segment() для загрузки нового профиля скорости.
  }
}


/* Prepares step segment buffer. Continuously called from main program. 

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it. 
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, millimeters, and minutes.
*/
/* Подготавливает буфер сегментов шагов. Постоянно вызывается из основной программы. 

   Буфер сегментов является промежуточным буферным интерфейсом между выполнением шагов
   алгоритмом stepper и профилями скорости, генерируемыми планировщиком. Шаговый буфер
   алгоритм выполняет шаги только в буфере сегментов и заполняется основной программой
   когда шаги "извлекаются" из первого блока в буфере планировщика. Это позволяет сохранить процессы
   оптимизации выполнения шага и планирования атомарными и защищенными друг от друга.
   Количество шагов, "извлеченного" из буфера планировщик и количество сегментов в
   буферный сегмент размеров и компьютерной такие, что никакой операции в основной программе принимает
   дольше, чем время, которое требуется шаговый алгоритм, чтобы очистить его перед заправкой его. 
   В настоящее время сегментный буфер обычно содержит шаги продолжительностью примерно 40-50 мс.
   ПРИМЕЧАНИЕ: Единицы измерения указаны в шагах, миллиметрах и минутах.
*/
void st_prep_buffer()
{

  if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) { 
    // Check if we still need to generate more segments for a motion suspend. // Проверьте, нужно ли нам по-прежнему генерировать дополнительные сегменты для приостановки движения.
    if (prep.current_speed == 0.0) { return; } // Nothing to do. Bail. // Ничего не нужно делать. Поручительство.
  }
  
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer. // Проверьте, нужно ли нам заполнять буфер.

    // Determine if we need to load a new planner block or if the block has been replanned. // Определите, нужно ли нам загружать новый блок планировщика или этот блок был перепланирован.
    if (pl_block == NULL) {
      pl_block = plan_get_current_block(); // Query planner for a queued block // Планировщик запросов для блока, находящегося в очереди
      if (pl_block == NULL) { return; } // No planner blocks. Exit. // Нет блоков планировщика. Выход.
                      
      // Check if the segment buffer completed the last planner block. If so, load the Bresenham
      // data for the block. If not, we are still mid-block and the velocity profile was updated. 
      // Проверьте, завершил ли буфер сегмента последний блок планировщика. Если да, то загрузите "Брезенхэм"
      // данные для блока. Если нет, то мы все еще находимся в середине квартала, и профиль скорости был обновлен.
      if (prep.flag_partial_block) {
        prep.flag_partial_block = false; // Reset flag
      } else {
        // Increment stepper common data index to store new planner block data. 
        // Увеличьте общий индекс данных stepper для хранения новых данных блока планировщика.
        if ( ++prep.st_block_index == (SEGMENT_BUFFER_SIZE-1) ) { prep.st_block_index = 0; }
        
        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded when the 
        // segment buffer finishes the prepped block, but the stepper ISR is still executing it. 
        // Подготовьте и скопируйте сегментные данные алгоритма Брезенхэма из нового блока планировщика, чтобы
        // когда сегментный буфер завершит блок планировщика, они могли быть отброшены, когда
        // сегментный буфер завершит подготовленный блок, но шаговый ISR все еще выполняет его.
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS];
          st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS];
          st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS];
          st_prep_block->step_event_count = pl_block->step_event_count;
        #else
          // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS 
          // level, such that we never divide beyond the original data anywhere in the algorithm.
          // If the original data is divided, we can lose a step from integer roundoff.
          // При включенной функции AMASS просто умножьте все данные Брезенхэма со сдвигом в битах на максимальное значение AMASS 
          // таким образом, чтобы нигде в алгоритме мы не производили деление дальше исходных данных.
          // Если исходные данные будут разделены, мы можем потерять шаг при целочисленном округлении. 
          st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif
        
        // Initialize segment buffer data for generating the segments.
        // Инициализируйте данные буфера сегментов для генерации сегментов.
        prep.steps_remaining = pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        
        prep.dt_remainder = 0.0; // Reset for new planner block // Сброс для нового блока планировщика

        if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) {
          // Override planner block entry speed and enforce deceleration during feed hold.
          // Переопределите скорость ввода блока планировщика и принудительно замедлите его во время задержки подачи.
          prep.current_speed = prep.exit_speed; 
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed; 
        }
        else { prep.current_speed = sqrt(pl_block->entry_speed_sqr); }
      }
     
      /* --------------------------------------------------------------------------------- 
         Compute the velocity profile of a new planner block based on its entry and exit
         speeds, or recompute the profile of a partially-completed planner block if the 
         planner has updated it. For a commanded forced-deceleration, such as from a feed 
         hold, override the planner velocities and decelerate to the target exit speed.
      */
     /* ---------------------------------------------------------------------------------
      Вычислите профиль скорости нового блока планировщика на основе его входа и выхода
              скорости, или повторно вычислите профиль частично завершенного блока
      планирования, если планировщик обновил его. Для принудительного замедления по команде, например, при загрузке 
              удерживайте, измените скорости планирования и снизьте скорость до заданной скорости на выходе.
      */
      prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block. // Профиль скорости по умолчанию завершен на расстоянии 0,0 мм от конца блока.
      float inv_2_accel = 0.5/pl_block->acceleration;
      if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) { // [Forced Deceleration to Zero Velocity] // [Принудительное замедление до нулевой скорости]
        // Compute velocity profile parameters for a feed hold in-progress. This profile overrides // Вычислить параметры профиля скорости для текущей задержки подачи. Этот профиль переопределяет
        // the planner block profile, enforcing a deceleration to zero speed. // планировщик блокирует профиль, принудительно снижая скорость до нуля.
        prep.ramp_type = RAMP_DECEL;
        // Compute decelerate distance relative to end of block. // Вычислить расстояние замедления относительно конца блока.
        float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < 0.0) {
          // Deceleration through entire planner block. End of feed hold is not in this block. // Замедление по всему блоку планирования. В этом блоке отсутствует время окончания задержки подачи.
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
        } else {
          prep.mm_complete = decel_dist; // End of feed hold. // Окончание удержания подачи.
          prep.exit_speed = 0.0;
        }
      } else { // [Normal Operation]
        // Compute or recompute velocity profile parameters of the prepped planner block.
        // [Нормальная работа]
        // Вычислите или повторно рассчитайте параметры профиля скорости для подготовленного блока планирования.
        prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp. // Инициализировать как рампу ускорения.
        prep.accelerate_until = pl_block->millimeters; 
        prep.exit_speed = plan_get_exec_block_exit_speed();   
        float exit_speed_sqr = prep.exit_speed*prep.exit_speed;
        float intersect_distance =
                0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
        if (intersect_distance > 0.0) {
          if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types // Трапециевидного или треугольного типа
            // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0. // ПРИМЕЧАНИЕ: Для типов "ускорение-круиз" и "только круиз" следующий расчет будет равен 0.0.
            prep.decelerate_after = inv_2_accel*(pl_block->nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) { // Trapezoid type // Трапециевидный тип
              prep.maximum_speed = sqrt(pl_block->nominal_speed_sqr);
              if (pl_block->entry_speed_sqr == pl_block->nominal_speed_sqr) { 
                // Cruise-deceleration or cruise-only type. // Круиз-торможение или только круиз-контроль.
                prep.ramp_type = RAMP_CRUISE;
              } else {
                // Full-trapezoid or acceleration-cruise types // Полностью трапециевидный или ускоренно-круизный типы
                prep.accelerate_until -= inv_2_accel*(pl_block->nominal_speed_sqr-pl_block->entry_speed_sqr); 
              }
            } else { // Triangle type // Треугольный тип
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
            }          
          } else { // Deceleration-only type // Тип только для замедления
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            prep.maximum_speed = prep.current_speed;
          }
        } else { // Acceleration-only type // Тип только для ускорения
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }  
    }

    // Initialize new segment // Инициализировать новый сегмент
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block. // Установите новый сегмент таким образом, чтобы он указывал на текущий блок данных сегмента.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create 
      a full segment based on the current ramp conditions. If the segment time is incomplete 
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if 
      an incomplete segment terminates at the end of the velocity profile, the segment is 
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at 
      the end of planner block (typical) or mid-block at the end of a forced deceleration, 
      such as from a feed hold.
    */

/*------------------------------------------------------------------------------------
        Вычислите среднюю скорость этого нового сегмента, определив общее расстояние
        пройденный отрезок времени DT_SEGMENT. Следующий код сначала пытается создать
        полный сегмент на основе текущих условий перехода. Если время выполнения сегмента
        не завершено при изменении состояния ramp, код продолжит цикл по переходящим
        состояниям ramp, чтобы заполнить оставшееся время выполнения сегмента. Однако, если 
        незавершенный сегмент завершается в конце профиля скорости, сегмент
        считается завершенным, несмотря на то, что время выполнения сокращенного сегмента меньше, чем DT_SEGMENT.
        Предполагается, что профиль скорости всегда выполняется по нарастающей последовательности:
        скорость разгона, режим движения в крейсерском режиме и скорость торможения. Расстояние прохождения каждой трассы
        может варьироваться от нуля до длины квартала. Профили скорости могут заканчиваться либо в
        конце блока планирования (как правило), либо в середине блока в конце принудительного торможения,
        например, при задержке подачи.
    */

    float dt_max = DT_SEGMENT; // Maximum segment time // Максимальное время работы сегмента
    float dt = 0.0; // Initialize segment time // Инициализировать время сегмента
    float time_var = dt_max; // Time worker variable // Переменная рабочего времени
    float mm_var; // mm-Distance worker variable // мм-Переменная удаленности работника
    float speed_var; // Speed worker variable   // Переменная скорости рабочего
    float mm_remaining = pl_block->millimeters; // New segment distance from end of block. // Новое расстояние сегмента от конца блока.
    float minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step. // Гарантируйте, по крайней мере, один шаг.
    if (minimum_mm < 0.0) { minimum_mm = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_ACCEL: 
          // NOTE: Acceleration ramp only computes during first do-while loop. // ПРИМЕЧАНИЕ: Амплитуда ускорения вычисляется только во время первого цикла do-while.
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp. // Конец разгонной рампы.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block. // Ускорение-круиз, ускорение-торможение на перекрестке с рампой или в конце квартала.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB // ПРИМЕЧАНИЕ: 0,0 при EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only.  // Только ускорение.
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE: 
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To 
          //   prevent this, simply enforce a minimum speed threshold in the planner.
          // ПРИМЕЧАНИЕ: mm_var используется для сохранения последнего значения mm_remaining для неполных вычислений time_var сегмента.
          // ПРИМЕЧАНИЕ: Если значение maximum_speed*time_var слишком низкое, округление может привести к тому, что значение mm_var не изменится. Чтобы
          // предотвратить это, просто установите минимальный порог скорости в планировщике.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise. 
            // Cruise-deceleration junction or end of block.
            // Конец маршрута. 
            // Маршрут - переход для снижения скорости или конец квартала.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only. // Только для круиза.    
            mm_remaining = mm_var; 
          } 
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed. // ПРИМЕЧАНИЕ: mm_var используется как дополнительная рабочая переменная для предотвращения ошибок при скорости, близкой к нулю.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min) // Используется в качестве дельта-скорости (мм/мин)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed. // Проверьте, находится ли скорость на нулевом уровне или ниже него.
            // Compute distance from end of segment to end of block. // Вычислить расстояние от конца сегмента до конца блока.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Deceleration only. // Только замедление.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop. // Сегмент завершен. Выйдите из инструкции switch-case. Продолжите цикл do-while.
            }
          } // End of block or end of forced-deceleration. // Сегмент завершен. Выйдите из инструкции switch-case. Продолжите цикл do-while.
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete; 
      }
      dt += time_var; // Add computed ramp time to total segment time. // Добавьте вычисленное время нарастания к общему времени сегмента.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction. // **Неполный** На перекрестке с пандусом.
      else {
        if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_mm or mm_complete.
          // Проверьте, нет ли очень медленных сегментов с нулевым шагом.
          // Увеличьте время прохождения сегмента, чтобы обеспечить хотя бы один шаг в сегменте. Переопределите и выполните цикл
          // выполняйте вычисления расстояния до тех пор, пока не будет достигнуто значение minimum_mm или mm_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else { 
          break; // **Complete** Exit loop. Segment execution time maxed. // **Завершить цикл** Exit. Время выполнения сегмента увеличено до максимального значения.
        }
      }
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete. // **Завершить** Цикл выхода. Профиль завершен.

   
    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance 
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions. 
       However, since floats have only 7.2 significant digits, long moves with extremely 
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
   /* -----------------------------------------------------------------------------------
       Вычислите частоту шагов сегмента, шаги для выполнения и примените необходимые корректировки скорости.
       ПРИМЕЧАНИЕ: Шаги вычисляются путем прямого скалярного преобразования миллиметрового расстояния
       , оставшегося в блоке, а не путем постепенного подсчета шагов, выполняемых за каждый сегмент.
       сегмент. Это помогает устранить проблемы с округлением чисел с плавающей запятой при некоторых добавлениях. 
       Однако, поскольку число с плавающей запятой состоит всего из 7,2 значащих цифр, long перемещается с чрезвычайно 
       большое количество шагов может превысить точность поплавков, что может привести к потере шагов.
       К счастью, этот сценарий крайне маловероятен и нереалистичен в станках с ЧПУ
       поддерживается Grbl (например, перемещение оси превышает 10 метров при 200 шагах/мм).
    */
    float steps_remaining = prep.step_per_mm*mm_remaining; // Convert mm_remaining to steps // Преобразовать mm_remaining в шаги
    float n_steps_remaining = ceil(steps_remaining); // Round-up current steps remaining // Подведение итогов оставшихся текущих шагов
    float last_n_steps_remaining = ceil(prep.steps_remaining); // Round-up last steps remaining // Подведение итогов последних оставшихся шагов
    prep_segment->n_step = last_n_steps_remaining-n_steps_remaining; // Compute number of steps to execute. // Вычислите количество шагов, которые необходимо выполнить.
    
    // Bail if we are at the end of a feed hold and don't have a step to execute. // Внести залог, если у нас заканчивается задержка подачи и нам не нужно выполнять какой-либо шаг.
    if (prep_segment->n_step == 0) {
      if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) {
        // Less than one step to decelerate to zero speed, but already very close. AMASS 
        // requires full steps to execute. So, just bail.
        // Менее одного шага до снижения скорости до нуля, но уже очень близко. накапливать 
        // для выполнения требуется выполнить все шаги. Итак, просто выйдите из игры.
        prep.current_speed = 0.0; // NOTE: (=0.0) Used to indicate completed segment calcs for hold. // ПРИМЕЧАНИЕ: (=0,0) Используется для обозначения завершенных расчетов сегмента для удержания.
        prep.dt_remainder = 0.0;
        prep.steps_remaining = n_steps_remaining;
        pl_block->millimeters = prep.steps_remaining/prep.step_per_mm; // Update with full steps. // Обновите с полными шагами.
        plan_cycle_reinitialize();         
        return; // Segment not generated, but current step data still retained. // Сегмент не сгенерирован, но данные текущего шага все еще сохраняются.
      }
    }

    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not 
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are 
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    // Вычислить скорость шага сегмента. Поскольку шаги являются целыми числами, а пройденные расстояния в миллиметрах - нет,
    // в конце каждого сегмента может быть частичный шаг различной величины, который
    // не выполняется, поскольку для пошагового ISR требуются целые шаги из-за алгоритма AMASS. Чтобы
    // компенсировать это, мы отслеживаем время выполнения частичного шага предыдущего сегмента и просто
    // применяем его с учетом расстояния частичного шага к текущему сегменту, чтобы оно было минимальным
    // корректируем скорость всего сегмента для обеспечения точности вывода шага. Эти корректировки скорости являются 
    // как правило, очень малы и не влияют отрицательно на производительность, но гарантируют, что Grbl
    // выдает точные профили ускорения и скорости, рассчитанные планировщиком.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time // Применить время выполнения частичного шага предыдущего сегмента

    float inv_rate = dt/(last_n_steps_remaining - steps_remaining); // Compute adjusted step rate inverse // Вычислить скорректированную скорость шага, обратную
    prep.dt_remainder = (n_steps_remaining - steps_remaining)*inv_rate; // Update segment partial step time // Время частичного шага обновления сегмента

    // Compute CPU cycles per step for the prepped segment. // Вычислять циклы процессора за каждый шаг для подготовленного сегмента.
    uint32_t cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); // (cycles/step)    

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING        
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      // Вычислите время выполнения шага и уровень многоосевого сглаживания.
      // ПРИМЕЧАНИЕ: AMASS увеличивает время работы таймера с каждым уровнем, поэтому требуется только один предварительный масштаб.
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }    
        cycles >>= prep_segment->amass_level; 
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // Just set the slowest speed possible. // Просто установите самую низкую скорость из возможных.
    #else 
      // Compute step timing and timer prescalar for normal step generation. // Вычислите время шага и предкаляр таймера для нормальной генерации шага.
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // prescaler: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // prescaler: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else { 
        prep_segment->prescaler = 3; // prescaler: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // Just set the slowest speed possible. (Around 4 step/sec.) // Вычислите время шага и предкаляр таймера для нормальной генерации шага.
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // Segment complete! Increment segment buffer indices. // Сегмент завершен! Увеличьте индексы буфера сегмента.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // Setup initial conditions for next segment. // Установите начальные условия для следующего сегмента.
    if (mm_remaining > prep.mm_complete) { 
      // Normal operation. Block incomplete. Distance remaining in block to be executed. // Нормальная работа. Блок не завершен. Расстояние, оставшееся в блоке для выполнения.
      pl_block->millimeters = mm_remaining;      
      prep.steps_remaining = steps_remaining;  
    } else { 
      // End of planner block or forced-termination. No more distance to be executed. // Завершение блока планирования или принудительное завершение. Больше не нужно выполнять дистанцию.
      if (mm_remaining > 0.0) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
        // the segment queue, where realtime protocol will set new state upon receiving the 
        // cycle stop flag from the ISR. Prep_segment is blocked until then.
        // В конце принудительного завершения.
        // Сбросьте подготовительные параметры для возобновления, а затем сбросьте bail. Разрешите завершить пошаговый ISR
        // очередь сегментов, в которой протокол реального времени установит новое состояние после получения
        // флага остановки цикла от ISR. До тех пор Prep_segment заблокирован.
        prep.current_speed = 0.0; // NOTE: (=0.0) Used to indicate completed segment calcs for hold. // ПРИМЕЧАНИЕ: (=0,0) Используется для обозначения завершенных расчетов сегмента для удержания.
        prep.dt_remainder = 0.0;
        prep.steps_remaining = ceil(steps_remaining);
        pl_block->millimeters = prep.steps_remaining/prep.step_per_mm; // Update with full steps. // Обновите с полными шагами.
        plan_cycle_reinitialize(); 
        return; // Bail! // Залог!
      } else { // End of planner block // Конец блока планирования
        // The planner block is complete. All steps are set to be executed in the segment buffer. // Блок планирования завершен. Все шаги настроены на выполнение в буфере сегмента.
        pl_block = NULL; // Set pointer to indicate check and load next planner block. // Установите указатель так, чтобы он указывал на проверку и загрузку следующего блока планировщика.
        plan_discard_current_block();
      }
    }

  } 
}      


// Called by realtime status reporting to fetch the current speed being executed. This value
// however is not exactly the current speed, but the speed computed in the last step segment
// in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
// divided by the ACCELERATION TICKS PER SECOND in seconds. 
// Вызывается с помощью realtime status reporting для получения текущей скорости выполнения. Однако это значение
// является не совсем текущей скоростью, а скоростью, вычисленной в сегменте последнего шага
// в буфере сегмента. Оно всегда будет отставать на количество блоков сегмента (-1).
// делится на ТАКТЫ УСКОРЕНИЯ В СЕКУНДУ в секундах.
#ifdef REPORT_REALTIME_RATE
  float st_get_realtime_rate()
  {
     if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_MOTION_CANCEL | STATE_SAFETY_DOOR)){
       return prep.current_speed;
     }
    return 0.0f;
  }
#endif
