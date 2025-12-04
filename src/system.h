#pragma once

#include "grbl.h"

// Define system executor bit map. Used internally by realtime protocol as realtime command flags, 
// which notifies the main program to execute the specified realtime command asynchronously.
// NOTE: The system executor uses an unsigned 8-bit volatile variable (8 flag limit.) The default
// flags are always false, so the realtime protocol only needs to check for a non-zero value to 
// know when there is a realtime command to execute.
// Определяет битовую карту системного исполнителя. Используется внутри протокола realtime в качестве флагов команд реального времени,
// которые уведомляют основную программу о необходимости асинхронного выполнения указанной команды реального времени.
// ПРИМЕЧАНИЕ: Системный исполнитель использует 8-разрядную переменную volatile без знака (ограничение в 8 флагов). По умолчанию
// флаги всегда имеют значение false, поэтому протоколу реального времени требуется только проверить ненулевое значение, чтобы 
// знать, когда требуется выполнить команду реального времени.

#define EXEC_STATUS_REPORT  bit(0) // bitmask 00000001
#define EXEC_CYCLE_START    bit(1) // bitmask 00000010
#define EXEC_CYCLE_STOP     bit(2) // bitmask 00000100
#define EXEC_FEED_HOLD      bit(3) // bitmask 00001000
#define EXEC_RESET          bit(4) // bitmask 00010000
#define EXEC_SAFETY_DOOR    bit(5) // bitmask 00100000
#define EXEC_MOTION_CANCEL  bit(6) // bitmask 01000000

// Alarm executor bit map.
// NOTE: EXEC_CRITICAL_EVENT is an optional flag that must be set with an alarm flag. When enabled,
// this halts Grbl into an infinite loop until the user aknowledges the problem and issues a soft-
// reset command. For example, a hard limit event needs this type of halt and aknowledgement.
// Битовая карта исполнителя Alarm.
// ПРИМЕЧАНИЕ: EXEC_CRITICAL_EVENT - это необязательный флаг, который должен быть установлен вместе с флагом alarm. Если он включен,
// это останавливает Grbl в бесконечном цикле, пока пользователь не распознает проблему и не выдаст мягкое предупреждение.-
// команда / reset. Например, для события с жестким ограничением требуется такой тип остановки и подтверждения.
#define EXEC_CRITICAL_EVENT     bit(0) // bitmask 00000001 (SPECIAL FLAG. See NOTE:)
#define EXEC_ALARM_HARD_LIMIT   bit(1) // bitmask 00000010
#define EXEC_ALARM_SOFT_LIMIT   bit(2) // bitmask 00000100
#define EXEC_ALARM_ABORT_CYCLE  bit(3) // bitmask 00001000
#define EXEC_ALARM_PROBE_FAIL   bit(4) // bitmask 00010000
#define EXEC_ALARM_HOMING_FAIL  bit(5) // bitmask 00100000

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
// Определяет битовую карту состояния системы. Переменная state в первую очередь отслеживает отдельные функции
// Grbl, чтобы управлять каждой из них без дублирования. Она также используется в качестве флага обмена сообщениями для
// критических событий.
#define STATE_IDLE          0      // Must be zero. No flags. // Должно быть равно нулю. Без флажков.
#define STATE_ALARM         bit(0) // In alarm state. Locks out all g-code processes. Allows settings access. // В аварийном состоянии. Блокирует все процессы с использованием g-кода. Разрешает доступ к настройкам.
#define STATE_CHECK_MODE    bit(1) // G-code check mode. Locks out planner and motion only. // Режим проверки G-кода. Блокирует только планировщик и движение.
#define STATE_HOMING        bit(2) // Performing homing cycle // Выполнение цикла самонаведения
#define STATE_CYCLE         bit(3) // Cycle is running or motions are being executed. // Цикл запущен или выполняются движения.
#define STATE_HOLD          bit(4) // Active feed hold // Активное удержание подачи
#define STATE_SAFETY_DOOR   bit(5) // Safety door is ajar. Feed holds and de-energizes system. // Защитная дверца приоткрыта. Система подачи задерживается и обесточивается.
#define STATE_MOTION_CANCEL bit(6) // Motion cancel by feed hold and return to idle.  // Отмените движение, удерживая подачу, и вернитесь в режим ожидания.

// Define system suspend states. // Определите состояния приостановки работы системы.
#define SUSPEND_DISABLE       0      // Must be zero. // Должно быть равно нулю.
#define SUSPEND_ENABLE_HOLD   bit(0) // Enabled. Indicates the cycle is active and currently undergoing a hold. // Включено. Указывает, что цикл активен и в настоящее время находится в режиме ожидания.
#define SUSPEND_ENABLE_READY  bit(1) // Ready to resume with a cycle start command. // Готов к возобновлению с помощью команды запуска цикла.
#define SUSPEND_ENERGIZE      bit(2) // Re-energizes output before resume. // Повторно активирует выход перед возобновлением работы.
#define SUSPEND_MOTION_CANCEL bit(3) // Cancels resume motion. Used by probing routine. // Отменяет возобновление движения. Используется в процедуре зондирования.

struct system_t{
  u8 abort;                 // System abort flag. Forces exit back to main loop for reset. // Флаг системного прерывания. Принудительный выход обратно в основной цикл для сброса.
  u8 state;                 // Tracks the current state of Grbl. // Отслеживает текущее состояние Grbl.
  u8 suspend;               // System suspend bitflag variable that manages holds, cancels, and safety door. // Системная переменная suspend bitflag, управляющая задержками, отменами и защитной дверью.
  u8 soft_limit;            // Tracks soft limit errors for the state machine. (boolean) // Отслеживает ошибки мягкого ограничения для конечного автомата. (логическое значение)
  
  i32 position[N_AXIS];      // Real-time machine (aka home) position vector in steps.  // Пошаговый вектор положения машины в реальном времени (он же home).
                                 // NOTE: This may need to be a volatile variable, if problems arise.    // ПРИМЕЧАНИЕ: Возможно, это должна быть переменная volatile, если возникнут проблемы.                          

  i32 probe_position[N_AXIS]; // Last probe position in machine coordinates and steps. // Последнее положение датчика в машинных координатах и шагах.
  u8 probe_succeeded;        // Tracks if last probing cycle was successful. // Отслеживает, был ли успешным последний цикл тестирования.
  u8 homing_axis_lock;       // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR. // Блокирует оси при включении ограничителей. Используется в качестве маски перемещения оси в шаговом режиме ISR.
};

extern system_t sys;

// uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR. // Значение состояния зондирования.  Используется для согласования цикла зондирования с шаговым управлением ISR.
// uint8_t sys_rt_exec_state;  // Global realtime executor bitflag variable for state management. See EXEC bitmasks. // Глобальная переменная bitflag executor в реальном времени для управления состоянием. Смотрите раздел Битовые маски EXEC.
// uint8_t sys_rt_exec_alarm;  // Global realtime executor bitflag variable for setting various alarms. // Глобальная переменная bitflag исполнителя в реальном времени для установки различных сигналов тревоги.

void SystemClockConfig();

// Initialize the serial protocol
// Инициализируйте последовательный протокол
void system_init();

// Returns if safety door is open or closed, based on pin state.
// Возвращается, если защитная дверца открыта или закрыта, в зависимости от состояния вывода.
uint8_t system_check_safety_door_ajar();

// Executes an internal system command, defined as a string starting with a '$'
// Выполняет внутреннюю системную команду, определенную как строка, начинающаяся с '$'
uint8_t system_execute_line(char *line);

// Execute the startup script lines stored in EEPROM upon initialization
// Выполнить строки сценария запуска, сохраненные в EEPROM при инициализации
void system_execute_startup(char *line);

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// Returns machine position of axis 'idx'. Must be sent a 'step' array.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx);

// Updates a machine 'position' array based on the 'step' array sent.
// Обновляет массив "положение" машины на основе отправленного массива "шаг".
void system_convert_array_steps_to_mpos(float *position, int32_t *steps);

// CoreXY calculation only. Returns x or y-axis "steps" based on CoreXY motor steps.
// Только расчет по методу CoreXY. Возвращает "шаги" по оси x или y на основе шагов двигателя CoreXY.
#ifdef COREXY
  int32_t system_convert_corexy_to_x_axis_steps(int32_t *steps);
  int32_t system_convert_corexy_to_y_axis_steps(int32_t *steps);
#endif

void SystemClockConfig();