#include "grbl.h"
#include "serial.h"
#include "report.h"
#include "machine.h"
#include "stepper.h"

void* machine_glb;
Timer16_HandleTypeDef timer_step;

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper 
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by 
// the planner, where the remaining planner block steps still can.
// Кольцевой буфер основного шагового сегмента. Содержит небольшие, короткие линейные сегменты для выполнения шагового алгоритма 
//, которые "извлекаются" постепенно, начиная с первого блока в буфере
// планировщика. После "извлечения" шаги в буфере сегментов не могут быть изменены с помощью 
// планировщика, в то время как остальные шаги блока планировщика все еще могут быть изменены.
typedef struct {
  uint16_t n_step;          // Number of step events to be executed for this segment // Количество пошаговых событий, которые должны быть выполнены для этого сегмента
  uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment. // Индекс данных шагового блока. Эта информация используется для выполнения данного сегмента.
  uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate. // Расстояние, пройденное за такт ISR, или скорость шага.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment // Указывает уровень накопления для ISR для выполнения этого сегмента
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing. // Без НАКОПЛЕНИЯ требуется предварительный масштабатор для настройки на медленное время.
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
// Структура данных / Stepper ISR. Содержит текущие данные для основного stepper ISR.
typedef struct {
  // Used by the bresenham line algorithm
  // Используется линейным алгоритмом Брезенхэма
  uint32_t counter_x,        // Counter variables for the bresenham line tracer // Переменные счетчика для трассировщика линии Брезенхэма
           counter_y, 
           counter_z;
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay // Сохраняет выходные данные out_bits для завершения задержки пошагового импульса
  #endif
  
  uint8_t execute_step;     // Flags step execution for each interrupt. // Помечает выполнение шага для каждого прерывания.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise // Время сброса ступенчатого импульса после повышения ступени
  uint8_t step_outbits;         // The next stepping-bits to be output // Следующие шаговые биты, которые будут выведены
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

// Индексы кольцевого буфера ступенчатого сегмента
// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_next_head;
static uint8_t segment_buffer_head;

// Stores the planner block Bresenham algorithm execution data for the segments in the segment 
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use. 
// Хранит данные о выполнении алгоритма Брезенхэма блока планировщика для сегментов в сегменте 
// buffer. Обычно этот буфер используется частично, но в худшем случае он будет
// никогда не превышайте количество доступных сегментов шагового буфера (SEGMENT_BUFFER_SIZE-1).
// ПРИМЕЧАНИЕ: Эти данные копируются из подготовленных блоков планировщика, чтобы блоки планировщика могли быть
// отброшены, когда они будут полностью использованы и заполнены буфером сегмента. Кроме того, AMASS изменяет эти
// данные для собственного использования.
struct st_block_t{  
  uint8_t direction_bits;
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
};
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

 void CheckLimits(){
  if (HAL_GPIO_LineInterruptState(X_LIMIT_LINE_IRQ)
      || HAL_GPIO_LineInterruptState(Y_LIMIT_LINE_IRQ)
      || HAL_GPIO_LineInterruptState(Z_LIMIT_LINE_IRQ)){

      uint8_t mask;
      mask |= 1 << (X_LIMIT_LINE_IRQ >> GPIO_IRQ_LINE_S);
      mask |= 1 << (Y_LIMIT_LINE_IRQ >> GPIO_IRQ_LINE_S);
      mask |= 1 << (Z_LIMIT_LINE_IRQ >> GPIO_IRQ_LINE_S);
      ClearGPIOInterruptLines(mask);
        
        if (static_cast<Machine*>(machine_glb)->GetMachineState() != STATE_ALARM) {  // Ignore if already in alarm state. // Игнорировать, если он уже находится в состоянии тревоги.
           static_cast<Machine*>(machine_glb)->Reset();
           static_cast<Machine*>(machine_glb)->SysRtExecAlarmSet((EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate hard limit critical event
        }
    }
 }

 void StepTimer(){
     // Обработчик прерываний шагового двигателя
                // if (busy) { return; } // Флаг занятости используется для избежания повторного входа в прерывание

                // Установите направляющие штифты за пару наносекунд до того, как мы включим степперы
                // DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);
                HAL_GPIO_WritePin(X_DIRECTION_PORT, X_DIRECTION_BIT, (st.dir_outbits & (1<<X_AXIS)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
                HAL_GPIO_WritePin(Y_DIRECTION_PORT, Y_DIRECTION_BIT, (st.dir_outbits & (1<<Y_AXIS)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW);
                HAL_GPIO_WritePin(Z_DIRECTION_PORT, Z_DIRECTION_BIT, (st.dir_outbits & (1<<Z_AXIS)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW);

                // Затем подайте импульс на шаговые штифты
                #ifdef STEP_PULSE_DELAY
                  st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Сохраните out_bits, чтобы предотвратить перезапись.
                #else  // Нормальная работа
                  // STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
                  if (st.step_outbits & (1<<X_AXIS)) HAL_GPIO_WritePin(STEP_PORT, X_STEP_BIT, GPIO_PIN_HIGH);
                  if (st.step_outbits & (1<<Y_AXIS)) HAL_GPIO_WritePin(STEP_PORT, Y_STEP_BIT, GPIO_PIN_HIGH);
                  if (st.step_outbits & (1<<Z_AXIS)) HAL_GPIO_WritePin(STEP_PORT, Z_STEP_BIT, GPIO_PIN_HIGH);
                #endif

                // Включите таймер сброса шагового импульса, чтобы прерывание сброса шагового порта могло сбрасывать сигнал после
                // точных настроек.pulse_microseconds в микросекундах, независимо от основного таймера1.
                // TCNT0 = st.step_pulse_time; // Счетчик времени перезагрузки 0
                // TCCR0B = (1<<CS01); // Время начала 0. Полная скорость, предустановка на 1/8

                #ifndef STEP_PULSE_DELAY
                  // Немедленный сброс импульсов
                  HAL_GPIO_WritePin(STEP_PORT, X_STEP_BIT, GPIO_PIN_LOW);
                  HAL_GPIO_WritePin(STEP_PORT, Y_STEP_BIT, GPIO_PIN_LOW);
                  HAL_GPIO_WritePin(STEP_PORT, Z_STEP_BIT, GPIO_PIN_LOW);
                #endif

                busy = true;
                // sei(); // Повторно включите прерывания, чтобы обеспечить своевременное срабатывание прерывания сброса шагового порта.
                       // ПРИМЕЧАНИЕ: Оставшийся код в этом ISR будет завершен до возврата к основной программе.
                
                // Если сегмента step нет, попробуйте извлечь его из буфера stepper
                if (st.exec_segment == NULL) {
                  // Что-нибудь есть в буфере? Если да, загрузите и инициализируйте сегмент следующего шага.
                  if (segment_buffer_head != segment_buffer_tail) {
                    // Инициализируем новый сегмент шага и загружаем количество шагов для выполнения
                    st.exec_segment = &segment_buffer[segment_buffer_tail];

                    #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                      // С AMASS отключена, установите предварительный масштабатор таймера для сегментов с низкой частотой шага (< 250 Гц).
                      // TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
                    #endif

                    // Инициализируйте синхронизацию сегмента шага для каждого шага и загрузите количество шагов для выполнения.
                    // OCR1A = st.exec_segment->cycles_per_tick;
                    // Установим новое значение периода и сравнения для таймера
                    HAL_Timer16_StartSetOnes_IT(timer_step, st.exec_segment->cycles_per_tick, st.exec_segment->cycles_per_tick/2);
                    st.step_count = st.exec_segment->n_step; // ПРИМЕЧАНИЕ: Может быть равно нулю при медленном движении.
                    
                    // Если новый сегмент запускает новый блок планировщика, инициализируйте промежуточные переменные и счетчики.
                    // ПРИМЕЧАНИЕ: Когда индекс данных сегмента изменяется, это указывает на новый блок планировщика.
                    if ( st.exec_block_index != st.exec_segment->st_block_index ) {
                      st.exec_block_index = st.exec_segment->st_block_index;
                      st.exec_block = &st_block_buffer[st.exec_block_index];
                      
                      // Инициализировать счетчики линий и расстояний Брезенхэма
                      st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
                    }
                    st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;

                    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                      // С AMASS включена, отрегулируйте счетчики приращения по оси Брезенхэма в соответствии с уровнем НАКОПЛЕНИЯ.
                      st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
                      st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
                      st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
                    #endif
                    
                  } else {
                    // Буфер сегмента пуст. Выключение.
                    st_go_idle();
                    static_cast<Machine*>(machine_glb)->SetMachineState(EXEC_CYCLE_STOP); // Помечает основную программу для завершения цикла
                    return; // Ничего не остается, как выйти.
                  }
                }
                
 }

extern "C"
{
    // Обработчик прерываний
    void trap_handler()
    {
        if (EPIC_CHECK_TIMER16_1())
        {
             if (__HAL_TIMER16_GET_FLAG_IT(&timer_step, TIMER16_FLAG_CMPM))
             {
               StepTimer();
                __HAL_TIMER16_CLEAR_FLAG(&timer_step, TIMER16_FLAG_CMPM);
             }
        }

        if (EPIC_CHECK_GPIO_IRQ())
        {
            CheckLimits();
        }

        // if (EPIC_CHECK_TIMER16_1())
        // {
            
        //      if (__HAL_TIMER16_GET_FLAG_IT(&timer_step, TIMER16_FLAG_CMPM))
        //      {
        //         // HAL_GPIO_TogglePin(GPIO_2, GPIO_PIN_7); /* Смена сигнала PORT1_3 на противоположный */
        //         __HAL_TIMER16_CLEAR_FLAG(&timer_step, TIMER16_FLAG_CMPM);
        //         // HAL_GPIO_WritePin(STEP_PORT, X_STEP_BIT, GPIO_PIN_HIGH);
        //         // HAL_DelayMs(10);
        //         // HAL_GPIO_WritePin(STEP_PORT, X_STEP_BIT, GPIO_PIN_LOW);
        //         // HAL_DelayMs(1000);                                                                                                    
        //         // HAL_Timer16_StartSetOnes_IT(timer_step, 0xFFFF, 0xFFFF / 2);
        //      }

        // }

        //   /* Сброс прерываний */
        //   // Денис рекомендовал следующую последовательность, сбросить флаг прерывания, затем его обрабатывать.
        //   // Чтобы было меньше багов
         HAL_EPIC_Clear(0xFFFFFFFF);
        // }
    }
}

int main()
{

    Serial serial;
    Printer printer(serial);
    Report report(printer);
    Machine machine(report);
    SystemClockConfig(timer_step);
    machine.Init();

    machine_glb = &machine;
    report.SetMachine(&machine);
    stepper_init();
    io_out(USER_LED);

    while (1)
    {

        io_set(USER_LED);
        HAL_DelayMs(2000);
        io_clr(USER_LED);
        HAL_DelayMs(2000);
        machine.PrintSettings();
        printer.String(machine.LimitsGetState());
        printer.String("\n");
    }
    return 0;
}

//   // Инициализация системы при включении питания
//   // Initialize system upon power-up.
//   stepper_init();  // Configure stepper pins and interrupt timers // Настройка выходов шаговых двигателей и таймеров прерываний
//   system_init();   // Configure pinout pins and pin-change interrupt // Настройка выводов распиновки и прерывания смены выводов

//   memset(&sys, 0, sizeof(system_t));  // Clear all system variables // Очистка всех системных переменных
//   sys.abort = true;   // Set abort to complete initialization // Установка значения переменной abort при окончании инициализации
//   sei(); // Enable interrupts //Включение перерываний

//   // Check for power-up and set system alarm if homing is enabled to force homing cycle
//   // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
//   // startup scripts, but allows access to settings and internal commands. Only a homing
//   // cycle '$H' or kill alarm locks '$X' will disable the alarm.
//   // NOTE: The startup script will run after successful completion of the homing cycle, but
//   // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
//   // things uncontrollably. Very bad.
//   // Проверяем включение питания и установите системный сигнал тревоги, если включен режим самонаведения, для принудительного цикла самонаведения
//   // установив состояние тревоги Grbl. Сигнал тревоги блокирует все команды g-кода, включая сценарии запуска
//   //, но позволяет получить доступ к настройкам и внутренним командам. Только режим самонаведения
//   // цикл "$H" или отключение блокировки сигнализации "$X" приведет к отключению сигнализации.
//   // ПРИМЕЧАНИЕ: Сценарий запуска будет запущен после успешного завершения цикла самонаведения, но
//   // не после отключения блокировок сигнализации. Предотвращает попадание блоков запуска motion в
//   // все происходит бесконтрольно. Очень плохо.
//   #ifdef HOMING_INIT_LOCK
//     if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
//   #endif

//   // Переводит Grbl в аварийное состояние после отключения питания или жесткого сброса.
//   // Force Grbl into an ALARM state upon a power-cycle or hard reset.
//   #ifdef FORCE_INITIALIZATION_ALARM
//     sys.state = STATE_ALARM;
//   #endif

//   // Цикл инициализации Grbl при включении питания или прерывании работы системы. В последнем случае все процессы
//   // возвращаются к этому циклу для повторной инициализации.
//   // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
//   // will return to this loop to be cleanly re-initialized.
//   for(;;) {

//     // TODO: Отдельная задача настройки, требующая отключения прерываний, особенно при
//     // прерывание работы системы и обеспечение того, чтобы все активные прерывания были полностью сброшены.
//     // TODO: Separate configure task that require interrupts to be disabled, especially upon
//     // a system abort and ensuring any active interrupts are cleanly reset.

//     // Reset Grbl primary systems. // Перезагрузите основные системы Grbl.
//     serial_reset_read_buffer(); // Clear serial read buffer //Очищает Com порт
//     gc_init(); // Set g-code parser to default state // Устанавливает g-code парсер в состояние по умолчанию
//     spindle_init();
//     coolant_init();
//     limits_init();
//     probe_init();
//     plan_reset(); // Clear block buffer and planner variables //Очистить буфер блоков и переменную планировщика
//     st_reset(); // Clear stepper subsystem variables. //Очистить переменные шаговой подсистемы

//     // Синхронизируйте очищенные позиции gcode и планировать с текущей позицией в системе.
//     // Sync cleared gcode and planner positions to current system position.
//     plan_sync_position();
//     gc_sync_position();

//     // Сброс системных переменных.
//     // Reset system variables.
//     sys.abort = false;
//     sys_rt_exec_state = 0;
//     sys_rt_exec_alarm = 0;
//     sys.suspend = false;
//     sys.soft_limit = false;

//     // Запустите основной цикл Grbl. Обрабатывает вводимые программой данные и выполняет их.
//     // Start Grbl main loop. Processes program inputs and executes them.
//     protocol_main_loop();
