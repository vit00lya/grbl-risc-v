/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud 
  Copyright (c) 2011 Jens Geisler 
  
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

#define SOME_LARGE_VALUE 1.0E+38 // Used by rapids and acceleration maximization calculations. Just needs
                                 // to be larger than any feasible (mm/min)^2 or mm/sec^2 value.
                                 // Используется при расчетах порогов и максимизации ускорения. Просто необходимо, чтобы
                                 // было больше любого допустимого значения (мм/мин)^2 или мм/сек^2.

/*
  Глобальные переменные планировщика.
*/
static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions // Кольцевой буфер для получения инструкций по перемещению
static uint8_t block_buffer_tail;     // Index of the block to process now // Индекс блока, который нужно обработать сейчас
static uint8_t block_buffer_head;     // Index of the next block to be pushed // Индекс следующего передаваемого блока
static uint8_t next_buffer_head;      // Index of the next buffer head  // Индекс следующей головки буфера
static uint8_t block_buffer_planned;  // Index of the optimally planned block // Индекс оптимально спланированного блока

// Define planner variables // Определение переменных планировщика
/*
  Структура планировщика, содержащая информацию о текущем состоянии планировщика.
  
  Поля:
    position[N_AXIS] - Позиция инструмента в абсолютных шагах. Хранится отдельно от позиции по g-коду
                       для перемещений, требующих многократных линейных перемещений (дуги, фиксированные
                       циклы, компенсация люфта).
    previous_unit_vec[N_AXIS] - Единичный вектор предыдущего отрезка траектории.
    previous_nominal_speed_sqr - Номинальная скорость предыдущего отрезка траектории (в квадрате).
*/
typedef struct {
  int32_t position[N_AXIS];          // The planner position of the tool in absolute steps. Kept separate
                                     // from g-code position for movements requiring multiple line motions,
                                     // i.e. arcs, canned cycles, and backlash compensation.
                                     // Положение инструмента в режиме планирования с абсолютными шагами. Сохранено отдельно
                                     // от положения по g-коду для перемещений, требующих многократных линейных перемещений,
                                     // например, дуг, фиксированных циклов и компенсации люфта.
  float previous_unit_vec[N_AXIS];   // Unit vector of previous path line segment // Единичный вектор предыдущего отрезка прямой траектории
  float previous_nominal_speed_sqr;  // Nominal speed of previous path line segment // Номинальная скорость предыдущего отрезка траектории
} planner_t;
static planner_t pl;


/*
  Возвращает индекс следующего блока в кольцевом буфере.
  Также вызывается буфером шагового сегмента.
  
  Параметры:
    block_index - Текущий индекс блока.
  
  Возвращает:
    Индекс следующего блока в кольцевом буфере.
*/
// Returns the index of the next block in the ring buffer. Also called by stepper segment buffer. // Возвращает индекс следующего блока в кольцевом буфере. Также вызывается буфером шагового сегмента.
uint8_t plan_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer // Возвращает индекс предыдущего блока в кольцевом буфере
static uint8_t plan_prev_block_index(uint8_t block_index) 
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


/*                            PLANNER SPEED DEFINITION                                              
                                     +--------+   <- current->nominal_speed
                                    /          \                                
         current->entry_speed ->   +            \                               
                                   |             + <- next->entry_speed (aka exit speed)
                                   +-------------+                              
                                       time -->                      
                                                  
  Recalculates the motion plan according to the following basic guidelines:
  
    1. Go over every feasible block sequentially in reverse order and calculate the junction speeds
        (i.e. current->entry_speed) such that:
      a. No junction speed exceeds the pre-computed maximum junction speed limit or nominal speeds of 
         neighboring blocks.
      b. A block entry speed cannot exceed one reverse-computed from its exit speed (next->entry_speed)
         with a maximum allowable deceleration over the block travel distance.
      c. The last (or newest appended) block is planned from a complete stop (an exit speed of zero).
    2. Go over every block in chronological (forward) order and dial down junction speed values if 
      a. The exit speed exceeds the one forward-computed from its entry speed with the maximum allowable
         acceleration over the block travel distance.
  
  When these stages are complete, the planner will have maximized the velocity profiles throughout the all
  of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In 
  other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements
  are possible. If a new block is added to the buffer, the plan is recomputed according to the said 
  guidelines for a new optimal plan.
  
  To increase computational efficiency of these guidelines, a set of planner block pointers have been
  created to indicate stop-compute points for when the planner guidelines cannot logically make any further
  changes or improvements to the plan when in normal operation and new blocks are streamed and added to the
  planner buffer. For example, if a subset of sequential blocks in the planner have been planned and are 
  bracketed by junction velocities at their maximums (or by the first planner block as well), no new block
  added to the planner buffer will alter the velocity profiles within them. So we no longer have to compute
  them. Or, if a set of sequential blocks from the first block in the planner (or a optimal stop-compute
  point) are all accelerating, they are all optimal and can not be altered by a new block added to the
  planner buffer, as this will only further increase the plan speed to chronological blocks until a maximum
  junction velocity is reached. However, if the operational conditions of the plan changes from infrequently
  used feed holds or feedrate overrides, the stop-compute pointers will be reset and the entire plan is  
  recomputed as stated in the general guidelines.
  
  Planner buffer index mapping:
  - block_buffer_tail: Points to the beginning of the planner buffer. First to be executed or being executed. 
  - block_buffer_head: Points to the buffer block after the last block in the buffer. Used to indicate whether
      the buffer is full or empty. As described for standard ring buffers, this block is always empty.
  - next_buffer_head: Points to next planner buffer block after the buffer head block. When equal to the 
      buffer tail, this indicates the buffer is full.
  - block_buffer_planned: Points to the first buffer block after the last optimally planned block for normal
      streaming operating conditions. Use for planning optimizations by avoiding recomputing parts of the 
      planner buffer that don't change with the addition of a new block, as describe above. In addition, 
      this block can never be less than block_buffer_tail and will always be pushed forward and maintain 
      this requirement when encountered by the plan_discard_current_block() routine during a cycle.
  
  NOTE: Since the planner only computes on what's in the planner buffer, some motions with lots of short 
  line segments, like G2/3 arcs or complex curves, may seem to move slow. This is because there simply isn't
  enough combined distance traveled in the entire buffer to accelerate up to the nominal speed and then 
  decelerate to a complete stop at the end of the buffer, as stated by the guidelines. If this happens and
  becomes an annoyance, there are a few simple solutions: (1) Maximize the machine acceleration. The planner
  will be able to compute higher velocity profiles within the same combined distance. (2) Maximize line 
  motion(s) distance per block to a desired tolerance. The more combined distance the planner has to use,
  the faster it can go. (3) Maximize the planner buffer size. This also will increase the combined distance
  for the planner to compute over. It also increases the number of computations the planner has to perform
  to compute an optimal plan, so select carefully. The Arduino 328p memory is already maxed out, but future
  ARM versions should have enough memory and speed for look-ahead blocks numbering up to a hundred or more.

*/

/* ОПРЕДЕЛЕНИЕ СКОРОСТИ ПЛАНИРОВЩИКА
                                         
                                     +--------+   <- текущая->номинальная скорость
                                    /          \                                
     текущая-> скорость входа ->   +            \                               
                                   |             + <-  следующая-> скорость входа (она же скорость выхода)
                                   +-------------+                              
                                       время -->     
                                                  
  Пересчитывает план движения в соответствии со следующими основными рекомендациями:
  
    1. Последовательно пройдитесь по каждому возможному блоку в обратном порядке и рассчитайте скорости перехода
        (т.е. текущую->начальную скорость) таким образом, чтобы:
      a. Ни одна скорость соединения не превышает предварительно рассчитанного максимального значения скорости соединения или номинальных скоростей
соседних блоков.
      b. Скорость входа в блок не может превышать скорость, вычисленную в обратном порядке по скорости выхода (далее->скорость входа).
         с максимально допустимым замедлением на протяжении всего пути движения блока.
      c. Последний (или самый новый добавленный) блок планируется с полной остановки (скорость выхода равна нулю).
    2. Просмотрите каждый блок в хронологическом (прямом) порядке и уменьшите значения скорости на перекрестке, если
a. Скорость на выходе превышает скорость, рассчитанную исходя из скорости на входе, с максимально допустимой
         ускорение на расстояние перемещения блока.
  
  Когда эти этапы будут завершены, планировщик максимизирует профили скоростей во всех
блоках планировщика, где каждый блок работает на максимально допустимых пределах ускорения. В 
  другими словами, для всех блоков в планировщике план является оптимальным, и дальнейшее повышение скорости
невозможно. Если в буфер добавляется новый блок, план пересчитывается в соответствии с указанными
рекомендациями для нового оптимального плана.
  
  Чтобы повысить вычислительную эффективность этих рекомендаций, был создан набор указателей блоков планировщика
, указывающих точки остановки вычислений, когда рекомендации планировщика не могут логически внести какие-либо дальнейшие
изменения или улучшения в план при нормальной работе, а новые блоки передаются в потоковом режиме и добавляются в систему.
  буфер планировщика. Например, если подмножество последовательных блоков в планировщике было запланировано и
заключено в квадратные скобки по максимальным скоростям пересечения (или по первому блоку планировщика), то ни один новый блок
, добавленный в буфер планировщика, не изменит профили скоростей внутри них. Таким образом, нам больше не нужно
их вычислять. Или, если набор последовательных блоков из первого блока в планировщике (или оптимальная
точка остановки вычислений) все ускоряются, все они оптимальны и не могут быть изменены новым блоком, добавленным в планировщик.
  буфер планирования, так как это только еще больше увеличит скорость планирования до хронологических блоков, пока не будет достигнута максимальная скорость соединения
  . Однако, если условия работы плана изменятся из-за нечасто
используемых задержек подачи или переопределения скорости подачи, указатели остановки вычислений будут сброшены, и весь план будет
рассчитан заново, как указано в общих рекомендациях.
  
  Отображение буферного индекса планировщика:
  - block_buffer_tail: указывает на начало буфера планировщика. Который должен быть выполнен первым или выполняется в настоящее время. 
  - block_buffer_head: указывает на блок буфера после последнего блока в буфере. Используется для указания на то, является ли
      буфер заполнен или пуст. Как описано для стандартных кольцевых буферов, этот блок всегда пуст.
  - next_buffer_head: указывает на следующий блок буфера планировщика после начального блока буфера. Если значение равно
конечному значению буфера, это означает, что буфер заполнен.
  - block_buffer_planned: указывает на первый блок буфера после последнего оптимально спланированного блока для нормального
      условия работы потоковой передачи. Используйте для планирования оптимизаций, избегая повторного вычисления частей 
      буфер планировщика, который не изменяется при добавлении нового блока, как описано выше. Кроме того,
этот блок никогда не может быть меньше, чем block_buffer_tail, и всегда будет выдвигаться вперед и поддерживать
это требование при выполнении процедуры plan_discard_current_block() в течение цикла.
  
  ПРИМЕЧАНИЕ: Поскольку планировщик вычисляет только то, что находится в буфере планировщика, некоторые движения с большим количеством коротких
отрезков, такие как дуги G2/3 или сложные кривые, могут показаться медленными. Это происходит потому, что в буфере просто нет
  суммарного расстояния, пройденного во всем буфере, достаточно, чтобы разогнаться до номинальной скорости, а затем
замедлиться до полной остановки в конце буфера, как указано в руководстве. Если это происходит и
становится помехой, есть несколько простых решений: (1) Максимально увеличить ускорение машины. Планировщик
сможет рассчитать более высокие профили скоростей при том же суммарном расстоянии. (2) Максимально увеличить расстояние между линейными
перемещениями на блок до желаемого допуска. Чем больше комбинированного расстояния должен использовать планировщик,
  тем быстрее это может произойти. (3) Увеличьте размер буфера планировщика. Это также увеличит общее расстояние
, которое планировщик должен преодолеть для выполнения вычислений. Это также увеличивает количество вычислений, которые планировщик должен выполнить
для расчета оптимального плана, поэтому выбирайте тщательно. Объем памяти Arduino 328p уже исчерпан, но в будущем
  В версиях ARM должно хватить памяти и быстродействия для прогнозных блоков численностью до ста и более.

*/
static void planner_recalculate() 
{   
  // Initialize block index to the last block in the planner buffer. // Инициализируйте индекс блока до последнего блока в буфере планировщика.
  uint8_t block_index = plan_prev_block_index(block_buffer_head);
         
  // Bail. Can't do anything with one only one plan-able block. // Залог. Ничего не могу сделать с одним-единственным планируемым блоком.
  if (block_index == block_buffer_planned) { return; }
      
  // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
  // block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
  // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
  // Обратный проход: грубо увеличьте все возможные кривые замедления, планируя движение назад, начиная с последнего
  // блокируйте в буфере. Прекратите планирование, когда будет достигнут последний оптимальный запланированный или конечный указатель.
  // ПРИМЕЧАНИЕ: Прямой проход позже уточнит и скорректирует обратный проход для создания оптимального плана.
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  // Calculate maximum entry speed for last block in buffer, where the exit speed is always zero.
  // Вычислить максимальную скорость входа для последнего блока в буфере, скорость выхода из которого всегда равна нулю.
  current->entry_speed_sqr = min( current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);
  
  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned) { // Only two plannable blocks in buffer. Reverse pass complete.
    // Check if the first block is the tail. If so, notify stepper to update its current parameters.
    // В буфере только два планируемых блока. Обратный проход завершен.
    // Проверьте, является ли первый блок конечным. Если это так, сообщите stepper, чтобы он обновил свои текущие параметры.
    if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  } else { // Three or more plan-able blocks // Три или более планируемых блока
    while (block_index != block_buffer_planned) { 
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      // Check if next block is the tail block(=planned block). If so, update current stepper parameters.
      // Проверьте, является ли следующий блок конечным (=запланированный блок). Если это так, обновите текущие параметры степпера.
      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); } 

      // Compute maximum entry speed decelerating over the current block from its exit speed.
      // Вычислите максимальную скорость входа, которая замедляется в текущем блоке по сравнению со скоростью выхода из него.
      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        } else {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }    

  // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
  // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
  // Переход вперед: планируйте кривую ускорения вперед от запланированного указателя и далее.
  // Также выполняется поиск оптимальных точек останова в плане и соответствующее обновление запланированного указателя.
  next = &block_buffer[block_buffer_planned]; // Begin at buffer planned pointer // Начать с запланированного указателя буфера
  block_index = plan_next_block_index(block_buffer_planned); 
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    
    // Any acceleration detected in the forward pass automatically moves the optimal planned
    // pointer forward, since everything before this is all optimal. In other words, nothing
    // can improve the plan from the buffer tail to the planned pointer by logic.
    // Любое ускорение, обнаруженное при прямом проходе, автоматически перемещает оптимальный запланированный
    // указатель вперед, поскольку все, что было до этого, является оптимальным. Другими словами, ничто
    // не может логически улучшить план от конца буфера до запланированного указателя.
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
      // If true, current block is full-acceleration and we can move the planned pointer forward.
      // Если true, то текущий блок находится в режиме полного ускорения, и мы можем переместить запланированный указатель вперед.
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // Always <= max_entry_speed_sqr. Backward pass sets this. // Всегда <= max_entry_speed_sqr. Это значение устанавливается при обратном переходе.
        block_buffer_planned = block_index; // Set optimal plan pointer. // Установите указатель оптимального плана.
      }
    }
    
    // Any block set at its maximum entry speed also creates an optimal plan up to this
    // point in the buffer. When the plan is bracketed by either the beginning of the
    // buffer and a maximum entry speed or two maximum entry speeds, every block in between
    // cannot logically be further improved. Hence, we don't have to recompute them anymore.
    // Любой блок, настроенный на максимальную скорость ввода, также создает оптимальный план для этого
    // точка в буфере. Когда план заключен в квадратные скобки либо началом буфера
    // и максимальной скоростью ввода, либо двумя максимальными скоростями ввода, каждый блок между ними
    / логически они не могут быть улучшены. Следовательно, нам больше не нужно их пересчитывать.
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index( block_index );
  } 
}


void plan_reset() 
{
  memset(&pl, 0, sizeof(planner_t)); // Clear planner struct // Четкая структура планировщика
  block_buffer_tail = 0;
  block_buffer_head = 0; // Empty = tail
  next_buffer_head = 1; // plan_next_block_index(block_buffer_head)
  block_buffer_planned = 0; // = block_buffer_tail;
}


void plan_discard_current_block() 
{
  if (block_buffer_head != block_buffer_tail) { // Discard non-empty buffer. // Удалить непустой буфер.
    uint8_t block_index = plan_next_block_index( block_buffer_tail );
    // Push block_buffer_planned pointer, if encountered. // Нажимаем указатель block_buffer_planned, если он встречается.
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index;
  }
}


plan_block_t *plan_get_current_block() 
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); } // Buffer empty // Буфер пуст 
  return(&block_buffer[block_buffer_tail]);
}


float plan_get_exec_block_exit_speed()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return( 0.0 ); }
  return( sqrt( block_buffer[block_index].entry_speed_sqr ) ); 
}


// Returns the availability status of the block ring buffer. True, if full.
// Возвращает статус доступности кольцевого буфера блоков. Значение True, если он заполнен.
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { return(true); }
  return(false);
}


/* Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position
   in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
   rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
   All position data passed to the planner must be in terms of machine position to keep the planner 
   independent of any coordinate system changes and offsets, which are handled by the g-code parser.
   NOTE: Assumes buffer is available. Buffer checks are handled at a higher level by motion_control.
   In other words, the buffer head is never equal to the buffer tail.  Also the feed rate input value
   is used in three ways: as a normal feed rate if invert_feed_rate is false, as inverse time if
   invert_feed_rate is true, or as seek/rapids rate if the feed_rate value is negative (and
   invert_feed_rate always false). */
/* Добавьте новое линейное перемещение в буфер. цель [N_AXIS] - это абсолютное целевое положение со знаком
   в миллиметрах. Скорость подачи определяет скорость перемещения. Если скорость подачи инвертирована, подача
   скорость принимается за "частоту" и завершает операцию за 1 минуту/feed_rate.
   Все данные о местоположении, передаваемые планировщику, должны быть указаны в терминах положения машины, чтобы планировщик
   не зависел от любых изменений системы координат и смещений, которые обрабатываются анализатором g-кода.
   ПРИМЕЧАНИЕ: Предполагается, что буфер доступен. Проверки буфера обрабатываются на более высоком уровне с помощью motion_control.
   Другими словами, начальный объем буфера никогда не равен конечному объему буфера.  Также входное значение скорости подачи
   используется тремя способами: как нормальная скорость подачи, если значение invert_feed_rate равно false, как обратное время, если
   значение invert_feed_rate равно true, или как скорость поиска/сброса, если значение feed_rate отрицательное (а
   значение invert_feed_rate всегда равно false). */
#ifdef USE_LINE_NUMBERS   
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number) 
#else
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate) 
#endif
{
  // Prepare and initialize new block // Подготовить и инициализировать новый блок
  plan_block_t *block = &block_buffer[block_buffer_head];
  block->step_event_count = 0;
  block->millimeters = 0;
  block->direction_bits = 0;
  block->acceleration = SOME_LARGE_VALUE; // Scaled down to maximum acceleration later // Позже уменьшено до максимального ускорения
  #ifdef USE_LINE_NUMBERS
    block->line_number = line_number;
  #endif

  // Compute and store initial move distance data.
  // TODO: After this for-loop, we don't touch the stepper algorithm data. Might be a good idea
  // to try to keep these types of things completely separate from the planner for portability.
    // Вычисляем и сохраняем начальные данные о расстоянии перемещения.
  // ЗАДАЧА: После этого цикла мы не будем касаться данных шагового алгоритма. Возможно, было бы хорошей идеей
  // попытаться полностью отделить эти типы данных от планировщика для удобства переноса.
  int32_t target_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;
  #ifdef COREXY
    target_steps[A_MOTOR] = lround(target[A_MOTOR]*settings.steps_per_mm[A_MOTOR]);
    target_steps[B_MOTOR] = lround(target[B_MOTOR]*settings.steps_per_mm[B_MOTOR]);
    block->steps[A_MOTOR] = labs((target_steps[X_AXIS]-pl.position[X_AXIS]) + (target_steps[Y_AXIS]-pl.position[Y_AXIS]));
    block->steps[B_MOTOR] = labs((target_steps[X_AXIS]-pl.position[X_AXIS]) - (target_steps[Y_AXIS]-pl.position[Y_AXIS]));
  #endif

  for (idx=0; idx<N_AXIS; idx++) {
    // Calculate target position in absolute steps, number of steps for each axis, and determine max step events.
    // Also, compute individual axes distance for move and prep unit vector calculations.
    // NOTE: Computes true distance from converted step values.
    // Вычислите положение цели в абсолютных шагах, количество шагов для каждой оси и определите максимальное количество шагов.
    // Кроме того, вычислите расстояние между отдельными осями для расчета единичных векторов перемещения и подготовки.
    // ПРИМЕЧАНИЕ: Истинное расстояние вычисляется на основе преобразованных значений шага.
    #ifdef COREXY
      if ( !(idx == A_MOTOR) && !(idx == B_MOTOR) ) {
        target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
        block->steps[idx] = labs(target_steps[idx]-pl.position[idx]);
      }
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      if (idx == A_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-pl.position[X_AXIS] + target_steps[Y_AXIS]-pl.position[Y_AXIS])/settings.steps_per_mm[idx];
      } else if (idx == B_MOTOR) {
        delta_mm = (target_steps[X_AXIS]-pl.position[X_AXIS] - target_steps[Y_AXIS]+pl.position[Y_AXIS])/settings.steps_per_mm[idx];
      } else {
        delta_mm = (target_steps[idx] - pl.position[idx])/settings.steps_per_mm[idx];
      }
    #else
      target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
      block->steps[idx] = labs(target_steps[idx]-pl.position[idx]);
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      delta_mm = (target_steps[idx] - pl.position[idx])/settings.steps_per_mm[idx];
    #endif
    unit_vec[idx] = delta_mm; // Store unit vector numerator. Denominator computed later.
    // Сохранить числитель единичного вектора. Знаменатель будет вычислен позже.
        
    // Set direction bits. Bit enabled always means direction is negative.
    // Установите биты направления. Включенный бит всегда означает, что направление отрицательное.
    if (delta_mm < 0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
    
    // Incrementally compute total move distance by Euclidean norm. First add square of each term.
    // Постепенно вычисляйте общее расстояние перемещения по евклидовой норме. Сначала добавьте квадрат каждого члена.
    block->millimeters += delta_mm*delta_mm;
  }
  block->millimeters = sqrt(block->millimeters); // Complete millimeters calculation with sqrt() // Полный расчет в миллиметрах с помощью sqrt()
  
  // Bail if this is a zero-length block. Highly unlikely to occur.
  // Внесите залог, если это блок нулевой длины. Маловероятно, что это произойдет.
  if (block->step_event_count == 0) { return; } 
  
  // Adjust feed_rate value to mm/min depending on type of rate input (normal, inverse time, or rapids)
  // TODO: Need to distinguish a rapids vs feed move for overrides. Some flag of some sort.
  // Измените значение скорости подачи на мм/мин в зависимости от типа ввода скорости (обычная, с обратным временем или с порогами)
  // ЗАДАЧА: Необходимо различать скорость и скорость перемещения подачи для переопределения. Какой-нибудь флаг.
  if (feed_rate < 0) { feed_rate = SOME_LARGE_VALUE; } // Scaled down to absolute max/rapids rate later // Уменьшено до абсолютного максимума/скорость прохождения порогов позже
  else if (invert_feed_rate) { feed_rate *= block->millimeters; }
  if (feed_rate < MINIMUM_FEED_RATE) { feed_rate = MINIMUM_FEED_RATE; } // Prevents step generation round-off condition. // Предотвращает возникновение условия округления шага.

  // Calculate the unit vector of the line move and the block maximum feed rate and acceleration scaled 
  // down such that no individual axes maximum values are exceeded with respect to the line direction. 
  // NOTE: This calculation assumes all axes are orthogonal (Cartesian) and works with ABC-axes,
  // if they are also orthogonal/independent. Operates on the absolute value of the unit vector.
  // Вычислите единичный вектор перемещения линии и максимальную скорость подачи блока и ускорение, уменьшенные в масштабе 
  // таким образом, чтобы не были превышены максимальные значения по отдельным осям относительно направления линии. 
  // ПРИМЕЧАНИЕ: В этом расчете предполагается, что все оси ортогональны (декартовы), и он работает с осями ABC,
// если они также ортогональны / независимы. Используется абсолютное значение единичного вектора.
  float inverse_unit_vec_value;
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple float divides	 // Обратные миллиметры для удаления множественных плавающих делений
  float junction_cos_theta = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (unit_vec[idx] != 0) {  // Avoid divide by zero. // Избегайте деления на ноль.
      unit_vec[idx] *= inverse_millimeters;  // Complete unit vector calculation // Полное вычисление единичного вектора
      inverse_unit_vec_value = fabs(1.0/unit_vec[idx]); // Inverse to remove multiple float divides. // Инверсия для удаления нескольких разделений с плавающей точкой.

      // Check and limit feed rate against max individual axis velocities and accelerations // Проверьте и ограничьте скорость подачи в соответствии с максимальными скоростями и ускорениями отдельных осей
      feed_rate = min(feed_rate,settings.max_rate[idx]*inverse_unit_vec_value);
      block->acceleration = min(block->acceleration,settings.acceleration[idx]*inverse_unit_vec_value);

      // Incrementally compute cosine of angle between previous and current path. Cos(theta) of the junction
      // between the current move and the previous move is simply the dot product of the two unit vectors, 
      // where prev_unit_vec is negative. Used later to compute maximum junction speed.
      // Постепенно вычисляем косинус угла между предыдущим и текущим траекториями. Cos(тета) пересечения
      // между текущим и предыдущим перемещениями - это просто скалярное произведение двух единичных векторов,
      // где значение prev_unit_vec отрицательно. Используется позже для вычисления максимальной скорости соединения.
      junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];
    }
  }
  
  // TODO: Need to check this method handling zero junction speeds when starting from rest.
  // ЗАДАЧА: Необходимо проверить, работает ли этот метод с нулевыми скоростями на перекрестке при запуске из состояния покоя.
  if (block_buffer_head == block_buffer_tail) {
  
    // Initialize block entry speed as zero. Assume it will be starting from rest. Planner will correct this later.
    // Инициализируйте скорость ввода блока равной нулю. Предположим, что она будет начинаться с rest. Планировщик исправит это позже.
    block->entry_speed_sqr = 0.0;
    block->max_junction_speed_sqr = 0.0; // Starting from rest. Enforce start from zero velocity.
    // Начинаем с нуля. Принудительно начните с нулевой скорости.
  
  } else {
    /* 
       Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
       Let a circle be tangent to both previous and current path line segments, where the junction 
       deviation is defined as the distance from the junction to the closest edge of the circle, 
       colinear with the circle center. The circular segment joining the two paths represents the 
       path of centripetal acceleration. Solve for max velocity based on max acceleration about the
       radius of the circle, defined indirectly by junction deviation. This may be also viewed as 
       path width or max_jerk in the previous Grbl version. This approach does not actually deviate 
       from path, but used as a robust way to compute cornering speeds, as it takes into account the
       nonlinearities of both the junction angle and junction velocity.

       NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path 
       mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
       stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
       is exactly the same. Instead of motioning all the way to junction point, the machine will
       just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
       a continuous mode path, but ARM-based microcontrollers most certainly do. 
       
       NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
       changed dynamically during operation nor can the line move geometry. This must be kept in
       memory in the event of a feedrate override changing the nominal speeds of blocks, which can 
       change the overall maximum entry speed conditions of all blocks.
    */
    // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
    /* 
       Вычислите максимально допустимую скорость въезда на перекресток, используя приближение центростремительного ускорения.
       Пусть окружность является касательной как к предыдущему, так и к текущему сегментам траектории, где
отклонение от пересечения определяется как расстояние от перекрестка до ближайшего края окружности,
расположенного на одной прямой с центром окружности. Круговой сегмент, соединяющий две траектории, представляет
собой траекторию центростремительного ускорения. Вычислите максимальную скорость, основываясь на максимальном ускорении относительно
радиуса окружности, которое косвенно определяется отклонением перекрестка. Это также можно рассматривать как 
       ширина траектории или max_jerk в предыдущей версии Grbl. На самом деле этот подход не отличается
от траектории, но используется как надежный способ расчета скорости на поворотах, поскольку учитывает
нелинейности как угла поворота, так и скорости на перекрестке.

       ПРИМЕЧАНИЕ: Если величина отклонения соединения конечна, Grbl выполняет перемещения в
режиме точной траектории (G61). Если значение отклонения соединения равно нулю, Grbl выполнит движение в точном направлении.
       способ остановки режима (G61.1). В будущем, если будет желателен непрерывный режим (G64), приведенная здесь математика
       это в точности то же самое. Вместо того, чтобы двигаться до самой точки соединения, машина будет
       просто следовать по описанной здесь дуге окружности. У Arduino нет необходимых циклов процессора для выполнения
       траектории в непрерывном режиме, но микроконтроллеры на базе ARM, безусловно, справляются. 
       
       ПРИМЕЧАНИЕ: Максимальная скорость соединения является фиксированным значением, поскольку пределы ускорения машины не могут быть
изменены динамически во время работы, равно как и геометрия линии не может изменяться. Это необходимо сохранить в
памяти на случай изменения номинальной скорости подачи блоков, что может привести к 
       измените общие условия максимальной скорости входа для всех блоков.
    */
    // ПРИМЕЧАНИЕ: Вычисляется без каких-либо дорогостоящих тригонометрических функций, sin() или acos(), путем определения половины тригонометрического угла cos(theta).
    if (junction_cos_theta > 0.999999) {
      //  For a 0 degree acute junction, just set minimum junction speed.
      // Для соединения под углом 0 градусов просто установите минимальную скорость соединения.
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      junction_cos_theta = max(junction_cos_theta,-0.999999); // Check for numerical round-off to avoid divide by zero. // Проверьте округление чисел, чтобы избежать деления на ноль.
      float sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta)); // Trig half angle identity. Always positive. // Идентичность половины угла тригонометрии. Всегда положительный результат.

      // TODO: Технически, ускорение, используемое в расчетах, должно быть ограничено минимумом из
      // двух переходов. Однако это не должно быть серьезной проблемой, за исключением экстремальных обстоятельств.
      // TODO: Technically, the acceleration used in calculation needs to be limited by the minimum of the
      // two junctions. However, this shouldn't be a significant problem except in extreme circumstances.
      block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                                   (block->acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2) );

    }
  }

  // Store block nominal speed // Номинальная скорость блока запоминания
  block->nominal_speed_sqr = feed_rate*feed_rate; // (mm/min). Always > 0

  // Вычислите максимальное значение скорости на перекрестке, основываясь на минимальной скорости на перекрестке и соседних номинальных скоростях.
  // Compute the junction maximum entry based on the minimum of the junction speed and neighboring nominal speeds.
  block->max_entry_speed_sqr = min(block->max_junction_speed_sqr, 
                                   min(block->nominal_speed_sqr,pl.previous_nominal_speed_sqr));

  // Обновить предыдущий вектор единицы измерения пути и номинальную скорость (в квадрате)
  // Update previous path unit_vector and nominal speed (squared)
  memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
  pl.previous_nominal_speed_sqr = block->nominal_speed_sqr;

  // Обновить позицию планировщика
  // Update planner position
  memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

  // Для нового блока все готово. Обновите индексы заголовка буфера и следующего заголовка буфера.
  // New block is all set. Update buffer head and next buffer head indices.
  block_buffer_head = next_buffer_head;  
  next_buffer_head = plan_next_block_index(block_buffer_head);

  // Завершите работу, пересчитав план с учетом нового блока.
  // Finish up by recalculating the plan with the new block.
  planner_recalculate();
}

// Сбросить векторы положения планировщика. Вызывается системной процедурой прерывания/инициализации.
// Reset the planner position vectors. Called by the system abort/initialization routine.
void plan_sync_position()
{

  // ЗАДАЧА: Для конфигураций двигателей, расположенных не в той же системе координат, что и положение станка,
  // эту функцию необходимо обновить, чтобы учесть разницу.
  // TODO: For motor configurations not in the same coordinate frame as the machine position,
  // this function needs to be updated to accomodate the difference. 
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
     if (idx==X_AXIS) { 
        pl.position[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys.position);
      } else if (idx==Y_AXIS) { 
        pl.position[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys.position);
      } else {
        pl.position[idx] = sys.position[idx];
      }
    #else
      pl.position[idx] = sys.position[idx];
    #endif
  }
}


// Returns the number of active blocks are in the planner buffer.
// Возвращает количество активных блоков, находящихся в буфере планировщика.
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head));
}


// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.
// Повторно инициализируйте план буферизации с помощью частично завершенного блока, который, как предполагается, находится в конце буфера.
// Вызывается после полной остановки степпера для удержания подачи и завершения цикла.
void plan_cycle_reinitialize()
{
  // Re-plan from a complete stop. Reset planner entry speeds and buffer planned pointer.
  // Перепланируйте планирование с полной остановки. Сбросьте скорость ввода в планировщик и буферизуйте указатель запланированного.
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();  
}
