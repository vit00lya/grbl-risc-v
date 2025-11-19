/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Simen Svale Skogsrud
  
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


// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
// NOTE: This is the primary gateway to the grbl planner. All line motions, including arc line 
// segments, must pass through this routine before being passed to the planner. The seperation of
// mc_line and plan_buffer_line is done primarily to place non-planner-type functions from being
// in the planner and to let backlash compensation or canned cycle integration simple and direct.
// Выполнить линейное перемещение в абсолютных миллиметровых координатах. Скорость подачи указана в миллиметрах в секунду
// если значение invert_feed_rate не равно true. Тогда скорость подачи означает, что перемещение должно быть завершено за
// (1 минуту)/время подачи.
// ПРИМЕЧАНИЕ: Это основной шлюз для планировщика grbl. Все линейные перемещения, включая дуговую линию 
// сегменты, должны проходить через эту процедуру перед передачей в планировщик. Разделение
// mc_line и plan_buffer_line сделано в первую очередь для того, чтобы исключить использование функций, не относящихся к типу planner.
// в планировщике, а также для обеспечения простой и понятной компенсации люфта или включения в рабочий цикл.
#ifdef USE_LINE_NUMBERS
  void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number)
#else
  void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate)
#endif
{
  // If enabled, check for soft limit violations. Placed here all line motions are picked up
  // from everywhere in Grbl.
  // Если этот параметр включен, проверьте, не нарушены ли плавные ограничения. Здесь отображаются все перемещения по линии
  // отовсюду в Grbl.
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) { limits_soft_check(target); }    
      
  // If in check gcode mode, prevent motion by blocking planner. Soft limits still work.
  // Если вы находитесь в режиме проверки gcode, предотвратите перемещение, заблокировав планировщик. Мягкие ограничения по-прежнему действуют.
  if (sys.state == STATE_CHECK_MODE) { return; }
    
  // NOTE: Backlash compensation may be installed here. It will need direction info to track when
  // to insert a backlash line motion(s) before the intended line motion and will require its own
  // plan_check_full_buffer() and check for system abort loop. Also for position reporting 
  // backlash steps will need to be also tracked, which will need to be kept at a system level.
  // There are likely some other things that will need to be tracked as well. However, we feel
  // that backlash compensation should NOT be handled by Grbl itself, because there are a myriad
  // of ways to implement it and can be effective or ineffective for different CNC machines. This
  // would be better handled by the interface as a post-processor task, where the original g-code
  // is translated and inserts backlash motions that best suits the machine. 
  // NOTE: Perhaps as a middle-ground, all that needs to be sent is a flag or special command that
  // indicates to Grbl what is a backlash compensation motion, so that Grbl executes the move but
  // doesn't update the machine position values. Since the position values used by the g-code
  // parser and planner are separate from the system machine positions, this is doable.

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Remain in this loop until there is room in the buffer.

  // ПРИМЕЧАНИЕ: Здесь может быть установлена компенсация люфта. Для отслеживания времени потребуется информация о направлении движения
  // чтобы вставить линейные движения с люфтом перед предполагаемым линейным движением, потребуется собственный
  // plan_check_full_buffer() и проверьте, нет ли системного цикла прерывания. Также для получения отчетов о местоположении 
  // необходимо также отслеживать шаги обратной связи, которые необходимо поддерживать на системном уровне.
  // Вероятно, есть и другие вещи, которые также необходимо будет отслеживать. Однако мы чувствуем
  // что компенсация люфта не должна выполняться самим Grbl, поскольку существует множество
  // множество способов его реализации и может быть эффективным или неэффективным для различных станков с ЧПУ. Этот
  // было бы лучше, если бы интерфейс выполнял задачу постпроцессора, где исходный g-код
  // переводится и вставляются движения обратной связи, которые наилучшим образом подходят для машины. 
  // ПРИМЕЧАНИЕ: Возможно, в качестве промежуточного решения все, что нужно отправить, - это флажок или специальную команду, которая
  // указывает Grbl, что такое движение для компенсации люфта, чтобы Grbl выполнил это движение, но
  // не обновлял значения положения станка. Поскольку значения положения, используемые g-кодом
  // анализатор и планировщик находятся отдельно от системных машин, это выполнимо.

  // Если буфер заполнен: хорошо! Это означает, что мы значительно опережаем робота. 
  // Оставайтесь в этом цикле, пока в буфере не останется места.
  do {
    protocol_execute_realtime(); // Check for any run-time commands // Проверьте наличие каких-либо команд во время выполнения
    if (sys.abort) { return; } // Bail, if system abort. // Залог, если система отключится.
    if ( plan_check_full_buffer() ) { protocol_auto_cycle_start(); } // Auto-cycle start when buffer is full. // Автоматический запуск цикла при заполнении буфера.
    else { break; }
  } while (1);

  // Plan and queue motion into planner buffer // Планирование и перемещение очереди в буфер планировщика
  #ifdef USE_LINE_NUMBERS
    plan_buffer_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    plan_buffer_line(target, feed_rate, invert_feed_rate);
  #endif
}


// Execute an arc in offset mode format. position == current xyz, target == target xyz, 
// offset == offset from current xyz, axis_X defines circle plane in tool space, axis_linear is
// the direction of helical travel, radius == circle radius, isclockwise boolean. Used
// for vector transformation direction.
// The arc is approximated by generating a huge number of tiny, linear segments. The chordal tolerance
// of each segment is configured in settings.arc_tolerance, which is defined to be the maximum normal
// distance from segment to the circle when the end points both lie on the circle.

// Выполнить дугу в формате режима смещения. позиция == текущее значение xyz, цель == целевое значение xyz,
// смещение == смещение от текущего значения xyz, axis_X определяет плоскость окружности в пространстве инструмента, axis_linear - это
// направление движения по спирали, радиус == радиус окружности, имеет логическое значение по часовой стрелке. Используемый
// для векторного преобразования направления.
// Дуга аппроксимируется путем создания огромного количества крошечных линейных сегментов. Допуск по хордам
// каждого сегмента настраивается в settings.arc_tolerance, который определяется как максимальное нормальное значение
// расстояние от сегмента до окружности, когда обе конечные точки лежат на окружности.
#ifdef USE_LINE_NUMBERS
  void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate, 
    uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc, int32_t line_number)
#else
  void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate,
    uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc)
#endif
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location // Радиус-вектор от центра к текущему местоположению
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required. // Угол CCW между позицией и целью от центра окружности. Требуется только одно тригонометрическое вычисление atan2().
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (is_clockwise_arc) { // Correct atan2 output per direction // Корректный выходной сигнал atan2 в каждом направлении
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel += 2*M_PI; }
  }

  // NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
  // (2x) settings.arc_tolerance. For 99% of users, this is just fine. If a different arc segment fit
  // is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
  // For the intended uses of Grbl, this value shouldn't exceed 2000 for the strictest of cases.
  // ПРИМЕЧАНИЕ: Конечные точки сегментов расположены на дуге, что может привести к уменьшению диаметра дуги на величину до
  // (2 раза) настройки.arc_tolerance. Для 99% пользователей это нормально. Если подходит другой сегмент дуги
  // желателен, т.е. метод наименьших квадратов, средняя точка на дуге, просто измените вычисление mm_per_arc_segment.
  // Для предполагаемого использования Grbl это значение не должно превышать 2000 в самых строгих случаях.
  uint16_t segments = floor(fabs(0.5*angular_travel*radius)/
                          sqrt(settings.arc_tolerance*(2*radius - settings.arc_tolerance)) );
  
  if (segments) { 
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    // Умножьте обратную скорость подачи, чтобы компенсировать тот факт, что это движение аппроксимируется
    // на количество отдельных сегментов. Обратная скорость подачи должна быть правильной для суммы
    // всех сегментов.
    if (invert_feed_rate) { feed_rate *= segments; }
   
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear])/segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;
       
       For arc generation, the center of the circle is the axis of rotation and the radius vector is 
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. Single precision values can accumulate error greater than tool precision in rare
       cases. So, exact arc path correction is implemented. This approach avoids the problem of too many very
       expensive trig operations [sin(),cos(),tan()] which can take 100-200 usec each to compute.
  
       Small angle approximation may be used to reduce computation overhead further. A third-order approximation
       (second order sin() has too much error) holds for most, if not, all CNC applications. Note that this 
       approximation will begin to accumulate a numerical drift error when theta_per_segment is greater than 
       ~0.25 rad(14 deg) AND the approximation is successively used without correction several dozen times. This
       scenario is extremely unlikely, since segment lengths and theta_per_segment are automatically generated
       and scaled by the arc tolerance setting. Only a very large arc tolerance setting, unrealistic for CNC 
       applications, would cause this numerical drift error. However, it is best to set N_ARC_CORRECTION from a
       low of ~4 to a high of ~20 or so to avoid trig operations while keeping arc generation accurate.
       
       This approximation also allows mc_arc to immediately insert a line segment into the planner 
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
       This is important when there are successive arc motions. 
    */
    // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
    /* Поворот вектора с помощью матрицы преобразования: r - исходный вектор, r_T - повернутый вектор,
     а phi - угол поворота. Подход к решению, предложенный Йенсом Гейслером.
           r_T = [cos(phi) -sin(phi)];
                  sin(phi) cos(phi] * r ;
       
       Для построения дуги центр окружности является осью вращения, а радиус-вектор
       определяется от центра окружности до начального положения. Каждый отрезок линии формируется путем последовательных поворотов вектора
       . Значения единичной точности могут накапливать погрешность, превышающую точность инструмента, в редких случаях
       случаи. Таким образом, реализована точная коррекция траектории дуги. Такой подход позволяет избежать проблемы слишком большого количества очень
       дорогостоящие тригонометрические операции [sin(),cos(),tan()], каждая из которых может занимать 100-200 секунд.
  
       Для дальнейшего сокращения вычислительных затрат можно использовать приближение под малым углом. Приближение третьего порядка
       (sin() второго порядка имеет слишком большую погрешность) применимо для большинства, если не для всех приложений с ЧПУ. Обратите внимание, что это
       приближение начнет накапливать числовую ошибку дрейфа, когда значение ta_per_segment будет больше, чем 
       ~0,25 рад (14°), И аппроксимация последовательно используется без коррекции несколько десятков раз. Этот
       сценарий крайне маловероятен, поскольку длина сегмента и значение ta_per_segment генерируются автоматически
       и масштабируются в соответствии с настройкой допуска дуги. Только очень большое значение допуска по дуге, нереалистичное для приложений с ЧПУ 
       , может привести к этой числовой ошибке смещения. Однако, лучше всего установить значение N_ARC_CORRECTION от
       низкого значения ~4 до максимального значения ~20 или около того, чтобы избежать триггерных операций и сохранить точность формирования дуги.
       
       Это приближение также позволяет mc_arc немедленно вставлять сегмент линии в планировщик
       без первоначальных затрат на вычисление cos() или sin(). К тому времени, когда потребуется применить arc
       с поправкой, планировщик должен был учесть задержку, вызванную первоначальными накладными расходами на mc_arc. 
       Это важно при последовательных перемещениях по дуге. 
    */
    // Вычисляет: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) за ~52 секунды
    float cos_T = 2.0 - theta_per_segment*theta_per_segment;
    float sin_T = theta_per_segment*0.16666667*(cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;
  
    for (i = 1; i<segments; i++) { // Increment (segments-1). // Приращение (сегменты-1).
      
      if (count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix. ~40 usec // Применить матрицу поворота вектора. ~40 секунд
        r_axisi = r_axis0*sin_T + r_axis1*cos_T;
        r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {      
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
	// Коррекция дуги в радиус-вектор. Вычисляется только с каждым шагом N_ARC_CORRECTION. ~375 секунд.
        // Точное местоположение вычисляется путем применения матрицы преобразования к исходному радиус-вектору (=-смещение).
        cos_Ti = cos(i*theta_per_segment);
        sin_Ti = sin(i*theta_per_segment);
        r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
        r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
        count = 0;
      }
  
      // Update arc_target location // Обновить местоположение arc_target
      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;
      
      #ifdef USE_LINE_NUMBERS
        mc_line(position, feed_rate, invert_feed_rate, line_number);
      #else
        mc_line(position, feed_rate, invert_feed_rate);
      #endif
      
      // Bail mid-circle on system abort. Runtime command check already performed by mc_line. // Заблокируйте середину круга при прерывании работы системы. Проверка команд во время выполнения уже выполнена mc_line.
      if (sys.abort) { return; }
    }
  }
  // Ensure last segment arrives at target location. // Убедитесь, что последний сегмент прибыл в целевое местоположение.
  #ifdef USE_LINE_NUMBERS
    mc_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    mc_line(target, feed_rate, invert_feed_rate);
  #endif
}


// Execute dwell in seconds. // Выполните задержку в считанные секунды.
void mc_dwell(float seconds) 
{
   if (sys.state == STATE_CHECK_MODE) { return; }
   
   uint16_t i = floor(1000/DWELL_TIME_STEP*seconds);
   protocol_buffer_synchronize();
   delay_ms(floor(1000*seconds-i*DWELL_TIME_STEP)); // Delay millisecond remainder. // Задержка на оставшуюся миллисекунду.
   while (i-- > 0) {
     // NOTE: Check and execute realtime commands during dwell every <= DWELL_TIME_STEP milliseconds. // ПРИМЕЧАНИЕ: Проверяйте и выполняйте команды реального времени во время задержки каждые миллисекунды <= DWELL_TIME_STEP.
     protocol_execute_realtime();
     if (sys.abort) { return; }
     _delay_ms(DWELL_TIME_STEP); // Delay DWELL_TIME_STEP increment // Задержка приращения ДЛИТЕЛЬНОСТИ_ВРЕМЕНИ_ШАГА
   }
}


// Perform homing cycle to locate and set machine zero. Only '$H' executes this command.
// NOTE: There should be no motions in the buffer and Grbl must be in an idle state before
// executing the homing cycle. This prevents incorrect buffered plans after homing.
// Выполните цикл самонаведения, чтобы найти и обнулить машину. Эту команду выполняет только '$H'.
// ПРИМЕЧАНИЕ: Перед выполнением цикла самонаведения в буфере не должно быть никаких перемещений, и Grbl должен находиться в состоянии ожидания.
// Это предотвращает неправильную буферизацию планов после восстановления.
void mc_homing_cycle()
{
  // Check and abort homing cycle, if hard limits are already enabled. Helps prevent problems
  // with machines with limits wired on both ends of travel to one limit pin.
  // TODO: Move the pin-specific LIMIT_PIN call to limits.c as a function.
  // Проверьте и прервите цикл самонаведения, если жесткие ограничения уже включены. Помогает предотвратить проблемы
  // для машин с ограничениями, подключенными на обоих концах пути к одному ограничивающему контакту.
  // ЗАДАЧА: Переместить вызов LIMIT_PIN, относящийся к конкретному контакту, в limits.c в качестве функции.
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES  
    if (limits_get_state()) { 
      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown. // Выполните сброс системы и убедитесь, что шпиндель и охлаждающая жидкость отключены.
      bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT));
      return;
    }
  #endif
   
  limits_disable(); // Disable hard limits pin change register for cycle duration // Отключить жесткие ограничения в регистре смены pin-кода на продолжительность цикла
    
  // -------------------------------------------------------------------------------------
  // Perform homing routine. NOTE: Special motion case. Only system reset works.
  
  // Search to engage all axes limit switches at faster homing seek rate.
  // -------------------------------------------------------------------------------------
  // Выполните процедуру самонаведения. ПРИМЕЧАНИЕ: Особый случай при движении. Работает только сброс системы.
  
  // Выполните поиск для включения концевых выключателей всех осей с более высокой скоростью поиска самонаведения.
  limits_go_home(HOMING_CYCLE_0);  // Homing cycle 0
  #ifdef HOMING_CYCLE_1
    limits_go_home(HOMING_CYCLE_1);  // Homing cycle 1
  #endif
  #ifdef HOMING_CYCLE_2
    limits_go_home(HOMING_CYCLE_2);  // Homing cycle 2
  #endif
    
  protocol_execute_realtime(); // Check for reset and set system abort. // Проверьте, выполняется ли сброс, и установите режим прерывания работы системы.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm. // Не завершено. Состояние тревоги устанавливается с помощью mc_alarm.

  // Homing cycle complete! Setup system for normal operation.
  // -------------------------------------------------------------------------------------

  // Gcode parser position was circumvented by the limits_go_home() routine, so sync position now.
  // Цикл самонаведения завершен! Система настроена на нормальную работу.
  // -------------------------------------------------------------------------------------

  // Позиция анализатора Gcode была изменена с помощью процедуры limits_go_home(), поэтому синхронизируйте позицию сейчас.
  gc_sync_position();

  // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
  // Если функция жестких ограничений включена, повторно включите регистр смены pin-кода с жесткими ограничениями после цикла самонаведения.
  limits_init();
}


// Perform tool length probe cycle. Requires probe switch.
// NOTE: Upon probe failure, the program will be stopped and placed into ALARM state.
// Выполните цикл измерения длины инструмента. Требуется переключение датчика.
// ПРИМЕЧАНИЕ: При неисправности датчика программа будет остановлена и переведена в аварийное состояние.
#ifdef USE_LINE_NUMBERS
  void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away, 
    uint8_t is_no_error, int32_t line_number)
#else
  void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away,
    uint8_t is_no_error)
#endif
{ 
  // TODO: Need to update this cycle so it obeys a non-auto cycle start. // TODO: Необходимо обновить этот цикл, чтобы он запускался не автоматически.
  if (sys.state == STATE_CHECK_MODE) { return; }

  // Finish all queued commands and empty planner buffer before starting probe cycle. // Завершите выполнение всех команд, находящихся в очереди, и очистите буфер планировщика перед началом цикла проверки.
  protocol_buffer_synchronize();

  // Initialize probing control variables // Инициализировать управляющие переменные зондирования
  sys.probe_succeeded = false; // Re-initialize probe history before beginning cycle. // Повторно инициализируйте историю зондирования перед началом цикла.
  probe_configure_invert_mask(is_probe_away);
  
  // After syncing, check if probe is already triggered. If so, halt and issue alarm.
  // NOTE: This probe initialization error applies to all probing cycles.
  // После синхронизации проверьте, запущен ли уже датчик. Если это так, остановите и выдайте сигнал тревоги.
  // ПРИМЕЧАНИЕ: Эта ошибка инициализации датчика относится ко всем циклам тестирования.
  if ( probe_get_state() ) { // Check probe pin state. // Проверьте состояние штифта датчика.
    bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_PROBE_FAIL);
    protocol_execute_realtime();
  }
  if (sys.abort) { return; } // Return if system reset has been issued. // Вернуть, если был произведен сброс системы.

  // Setup and queue probing motion. Auto cycle-start should not start the cycle. // Настройка и постановка в очередь пробного движения. Автоматический запуск цикла не должен приводить к запуску цикла.
  #ifdef USE_LINE_NUMBERS
    mc_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    mc_line(target, feed_rate, invert_feed_rate);
  #endif
  
  // Activate the probing state monitor in the stepper module. // Активируйте монитор состояния зондирования в шаговом модуле.
  sys_probe_state = PROBE_ACTIVE;

  // Perform probing cycle. Wait here until probe is triggered or motion completes. // Выполните цикл зондирования. Подождите здесь, пока не сработает датчик или не завершится перемещение.
  bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START);
  do {
    protocol_execute_realtime(); 
    if (sys.abort) { return; } // Check for system abort // Проверьте, не прервалась ли система
  } while (sys.state != STATE_IDLE);
  
  // Probing cycle complete!
  
  // Set state variables and error out, if the probe failed and cycle with error is enabled.
  // Цикл проверки завершен!
  
  // Установите переменные состояния и выведите ошибку, если проверка завершилась неудачно и включен цикл с ошибкой.
  if (sys_probe_state == PROBE_ACTIVE) {
    if (is_no_error) { memcpy(sys.probe_position, sys.position, sizeof(float)*N_AXIS); }
    else { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_PROBE_FAIL); }
  } else { 
    sys.probe_succeeded = true; // Indicate to system the probing cycle completed successfully. // Указывает системе на успешное завершение цикла зондирования.
  }
  sys_probe_state = PROBE_OFF; // Ensure probe state monitor is disabled. // Убедитесь, что монитор состояния датчика отключен.
  protocol_execute_realtime();   // Check and execute run-time commands // Проверка и выполнение команд во время выполнения
  if (sys.abort) { return; } // Check for system abort // Проверьте, не прервалась ли система

  // Reset the stepper and planner buffers to remove the remainder of the probe motion. // Сбросьте буферы шагового управления и планировщика, чтобы удалить оставшуюся часть движения зонда.
  st_reset(); // Reest step segment buffer. // Восстановите буфер сегмента шага.
  plan_reset(); // Reset planner buffer. Zero planner positions. Ensure probing motion is cleared. // Сбросить буфер планировщика. Обнулить позиции планировщика. Убедитесь, что движение зондирования отменено.
  plan_sync_position(); // Sync planner position to current machine position. // Синхронизируйте положение планировщика с текущим положением машины.

  // TODO: Update the g-code parser code to not require this target calculation but uses a gc_sync_position() call.
  // NOTE: The target[] variable updated here will be sent back and synced with the g-code parser.
  // ЗАДАЧА: Обновите код анализатора g-кода, чтобы он не требовал этого целевого вычисления, а использовал вызов gc_sync_position().
  // ПРИМЕЧАНИЕ: Измененная здесь переменная target[] будет отправлена обратно и синхронизирована с анализатором g-кода.
  system_convert_array_steps_to_mpos(target, sys.position);

  #ifdef MESSAGE_PROBE_COORDINATES
    // All done! Output the probe position as message. // Все готово! Выведите положение датчика в виде сообщения.
    report_probe_parameters();
  #endif
}


// Method to ready the system to reset by setting the realtime reset command and killing any
// active processes in the system. This also checks if a system reset is issued while Grbl
// is in a motion state. If so, kills the steppers and sets the system alarm to flag position
// lost, since there was an abrupt uncontrolled deceleration. Called at an interrupt level by
// realtime abort command and hard limits. So, keep to a minimum.
// Способ подготовить систему к сбросу, установив команду сброса в реальном времени и завершив все
// активные процессы в системе. Это также проверяет, выполняется ли сброс системы, пока Grbl
// находится в состоянии движения. Если это так, отключите переключатели и установите системный сигнал тревоги в положение флажка
// потерян, поскольку произошло резкое неконтролируемое замедление. Вызывается на уровне прерывания с помощью
// команды прерывания в реальном времени и жестких ограничений. Таким образом, сведите к минимуму.
void mc_reset()
{
  // Only this function can set the system reset. Helps prevent multiple kill calls.
  // Только с помощью этой функции можно выполнить сброс системы. Помогает предотвратить многократное отключение системы.
  if (bit_isfalse(sys_rt_exec_state, EXEC_RESET)) {
    bit_true_atomic(sys_rt_exec_state, EXEC_RESET);

    // Kill spindle and coolant.
    // Заглушите шпиндель и охлаждающую жидкость.
    spindle_stop();
    coolant_stop();

    // Kill steppers only if in any motion state, i.e. cycle, actively holding, or homing.
    // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
    // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
    // violated, by which, all bets are off.
    // Отключайте степперы, только если они находятся в любом состоянии движения, т.е. в цикле, активном удержании или в режиме самонаведения.
    // ПРИМЕЧАНИЕ: Если степперы остаются включенными с помощью параметра step idle delay, это также сохраняет
    // степперы активируются путем полного отказа от вызова go_idle, если только не нарушено состояние движения
    //, в результате чего все ставки отменяются.
    if ((sys.state & (STATE_CYCLE | STATE_HOMING)) || (sys.suspend == SUSPEND_ENABLE_HOLD)) {
      if (sys.state == STATE_HOMING) { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_HOMING_FAIL); }
      else { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_ABORT_CYCLE); }
      st_go_idle(); // Force kill steppers. Position has likely been lost. // Принудительное уничтожение шагающих. Скорее всего, позиция потеряна.
    }
  }
}
