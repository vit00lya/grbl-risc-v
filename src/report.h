#pragma once

#ifndef report_h
#define report_h

#include "grbl.h"
#include "settings.h"
#include "gcode.h"
#include "print.h"
#include "limits.h"
#include "system.h"
#include "serial.h"
#include "cstring"
#include "planner.h"

// Define Grbl status codes. // Определите коды состояния Grbl.
#define STATUS_OK 0
#define STATUS_EXPECTED_COMMAND_LETTER 1
#define STATUS_BAD_NUMBER_FORMAT 2
#define STATUS_INVALID_STATEMENT 3
#define STATUS_NEGATIVE_VALUE 4
#define STATUS_SETTING_DISABLED 5
#define STATUS_SETTING_STEP_PULSE_MIN 6
#define STATUS_SETTING_READ_FAIL 7
#define STATUS_IDLE_ERROR 8
#define STATUS_ALARM_LOCK 9
#define STATUS_SOFT_LIMIT_ERROR 10
#define STATUS_OVERFLOW 11
#define STATUS_MAX_STEP_RATE_EXCEEDED 12

#define STATUS_GCODE_UNSUPPORTED_COMMAND 20
#define STATUS_GCODE_MODAL_GROUP_VIOLATION 21
#define STATUS_GCODE_UNDEFINED_FEED_RATE 22
#define STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER 23
#define STATUS_GCODE_AXIS_COMMAND_CONFLICT 24
#define STATUS_GCODE_WORD_REPEATED 25
#define STATUS_GCODE_NO_AXIS_WORDS 26
#define STATUS_GCODE_INVALID_LINE_NUMBER 27
#define STATUS_GCODE_VALUE_WORD_MISSING 28
#define STATUS_GCODE_UNSUPPORTED_COORD_SYS 29
#define STATUS_GCODE_G53_INVALID_MOTION_MODE 30
#define STATUS_GCODE_AXIS_WORDS_EXIST 31
#define STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE 32
#define STATUS_GCODE_INVALID_TARGET 33
#define STATUS_GCODE_ARC_RADIUS_ERROR 34
#define STATUS_GCODE_NO_OFFSETS_IN_PLANE 35
#define STATUS_GCODE_UNUSED_WORDS 36
#define STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR 37

// Define Grbl alarm codes. // Определите коды аварийных сигналов Grbl.
#define ALARM_HARD_LIMIT_ERROR 1 
#define ALARM_SOFT_LIMIT_ERROR 2
#define ALARM_ABORT_CYCLE 3
#define ALARM_PROBE_FAIL 4
#define ALARM_HOMING_FAIL 5

// Define Grbl feedback message codes. // Определите коды сообщений обратной связи Grbl.
#define MESSAGE_CRITICAL_EVENT 1
#define MESSAGE_ALARM_LOCK 2
#define MESSAGE_ALARM_UNLOCK 3
#define MESSAGE_ENABLED 4
#define MESSAGE_DISABLED 5
#define MESSAGE_SAFETY_DOOR_AJAR 6
#define MESSAGE_PROGRAM_END 7
#define MESSAGE_RESTORE_DEFAULTS 8

class Report {
private:
    Printer &printer_;
public:
    Report(Printer &printer) : printer_(printer){} 
    // Prints system status messages. // Выводит сообщения о состоянии системы.
    void StatusMessage(uint8_t status_code);
    // Prints system alarm messages. // Выводит системные тревожные сообщения.
    void AlarmMessage(int8_t alarm_code);
    // Prints miscellaneous feedback messages. // Печатает различные сообщения обратной связи.
    void FeedbackMessage(uint8_t message_code);
    // Prints welcome message // Выводит приветственное сообщение
    void InitMessage();
    // Prints Grbl help and current global settings // Выводит справку Grbl и текущие глобальные настройки
    void GrblHelp();
    // Prints Grbl global settings // Выводит глобальные настройки Grbl
    void GrblSettings(const settings_t& settings);
    // Prints an echo of the pre-parsed line received right before execution. // Выводит эхо предварительно обработанной строки, полученной непосредственно перед выполнением.
    void EchoLineReceived(char *line);
    // Prints realtime status report // Выводит отчет о состоянии в режиме реального времени
    void RealtimeStatus(settings_t& settings);
    // Prints recorded probe position // Печатает записанное положение датчика
    void ProbeParameters();
    // Prints Grbl NGC parameters (coordinate offsets, probe) // Выводит параметры Grbl NGC (смещения координат, зонд)
    void NgcParameters();
    // Prints current g-code parser mode state // Выводит текущее состояние режима анализатора g-кода
    void GcodeModes();
    // Prints startup line // Выводит строку запуска
    void StartupLine(uint8_t n, char *line);
    // Prints build info and user info // Выводит информацию о сборке и пользователе
    void BuildInfo(char *line);
};

#endif
