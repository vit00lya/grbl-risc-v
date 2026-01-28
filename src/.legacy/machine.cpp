#include "machine.h"

void Machine::Init(){
    
    // if (!ReadGlobalSettings())
    // {
    //     report_.StatusMessage(STATUS_SETTING_READ_FAIL);
    //     SettingsRestore(SETTINGS_RESTORE_ALL); // Force restore all EEPROM data. // Принудительно восстановите все данные EEPROM.
    //     report_.GrblSettings(settings_);
    // }

    limits_[X_AXIS].LimitsInit(X_LIMIT_BIT_PIN, X_LIMIT_BIT_PORT, X_LIMIT_BIT_LINE_IRQ);
    limits_[Y_AXIS].LimitsInit(Y_LIMIT_BIT_PIN, Y_LIMIT_BIT_PORT, Y_LIMIT_BIT_LINE_IRQ);
    limits_[Z_AXIS].LimitsInit(Z_LIMIT_BIT_PIN, Z_LIMIT_BIT_PORT, Z_LIMIT_BIT_LINE_IRQ);

    init_ = true;
    
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
void Machine::Reset(){
      // Only this function can set the system reset. Helps prevent multiple kill calls.
  // Только с помощью этой функции можно выполнить сброс системы. Помогает предотвратить многократное отключение системы.
//   if (bit_isfalse(sys_rt_exec_state, EXEC_RESET)) {
//     bit_true_atomic(sys_rt_exec_state, EXEC_RESET);

    // Kill spindle and coolant.
    // Заглушите шпиндель и охлаждающую жидкость.
    // spindle_stop();
    // coolant_stop();

    // Kill steppers only if in any motion state, i.e. cycle, actively holding, or homing.
    // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
    // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
    // violated, by which, all bets are off.
    // Отключайте степперы, только если они находятся в любом состоянии движения, т.е. в цикле, активном удержании или в режиме самонаведения.
    // ПРИМЕЧАНИЕ: Если степперы остаются включенными с помощью параметра step idle delay, это также сохраняет
    // степперы активируются путем полного отказа от вызова go_idle, если только не нарушено состояние движения
    //, в результате чего все ставки отменяются.
//     if ((sys.state & (STATE_CYCLE | STATE_HOMING)) || (sys.suspend == SUSPEND_ENABLE_HOLD)) {
//       if (sys.state == STATE_HOMING) { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_HOMING_FAIL); }
//       else { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_ABORT_CYCLE); }
//       st_go_idle(); // Force kill steppers. Position has likely been lost. // Принудительное уничтожение шагающих. Скорее всего, позиция потеряна.
//     }
//   }
}

void Machine::SetMachineState(uint8_t value){
    machine_state_ = value;
}

uint8_t Machine::GetMachineState(){
    return machine_state_;
}

bool Machine::GetInit(){
    return init_;
}

void Machine::PrintSettings(){
    report_.GrblSettings(settings_);  
}
 std::bitset<8> Machine::SysRtExecAlarmGet(){
    return sys_rt_exec_alarm_;
}
void Machine::SysRtExecAlarmSet(uint8_t sys_rt_exec_alarm){
    sys_rt_exec_alarm_ = sys_rt_exec_alarm;
}

char* Machine::LimitsGetState(){
    static char result[N_AXIS + 1];  // +1 для нулевого символа
    for(size_t i = 0 ; i < N_AXIS; ++i){
        result[i] = limits_[i].LimitGetState() ? '1' : '0';
    };
    result[N_AXIS] = '\0';  // Добавляем нулевой символ в конце
    return result;
}

void Machine::SettingsRestore(uint8_t restore_flag)
{
    if (restore_flag & SETTINGS_RESTORE_DEFAULTS)
    {
        settings_.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS;
        settings_.stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME;
        settings_.step_invert_mask = DEFAULT_STEPPING_INVERT_MASK;
        settings_.dir_invert_mask = DEFAULT_DIRECTION_INVERT_MASK;
        settings_.status_report_mask = DEFAULT_STATUS_REPORT_MASK;
        settings_.junction_deviation = DEFAULT_JUNCTION_DEVIATION;
        settings_.arc_tolerance = DEFAULT_ARC_TOLERANCE;
        settings_.homing_dir_mask = DEFAULT_HOMING_DIR_MASK;
        settings_.homing_feed_rate = DEFAULT_HOMING_FEED_RATE;
        settings_.homing_seek_rate = DEFAULT_HOMING_SEEK_RATE;
        settings_.homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY;
        settings_.homing_pulloff = DEFAULT_HOMING_PULLOFF;

        settings_.flags = 0;
        if (DEFAULT_INVERT_ST_ENABLE)
        {
            settings_.flags |= BITFLAG_INVERT_ST_ENABLE;
        }
        if (DEFAULT_INVERT_LIMIT_PINS)
        {
            settings_.flags |= BITFLAG_INVERT_LIMIT_PINS;
        }
        if (DEFAULT_SOFT_LIMIT_ENABLE)
        {
            settings_.flags |= BITFLAG_SOFT_LIMIT_ENABLE;
        }
        if (DEFAULT_HARD_LIMIT_ENABLE)
        {
            settings_.flags |= BITFLAG_HARD_LIMIT_ENABLE;
        }
        if (DEFAULT_HOMING_ENABLE)
        {
            settings_.flags |= BITFLAG_HOMING_ENABLE;
        }

        // settings_.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
        // settings_.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
        // settings_.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
        // settings_.max_rate[X_AXIS] = DEFAULT_X_MAX_RATE;
        // settings_.max_rate[Y_AXIS] = DEFAULT_Y_MAX_RATE;
        // settings_.max_rate[Z_AXIS] = DEFAULT_Z_MAX_RATE;
        // settings_.acceleration[X_AXIS] = DEFAULT_X_ACCELERATION;
        // settings_.acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION;
        // settings_.acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION;
        // settings_.max_travel[X_AXIS] = (-DEFAULT_X_MAX_TRAVEL);
        // settings_.max_travel[Y_AXIS] = (-DEFAULT_Y_MAX_TRAVEL);
        // settings_.max_travel[Z_AXIS] = (-DEFAULT_Z_MAX_TRAVEL);

        // write_global_settings();
    }
}

bool Machine::ReadGlobalSettings()
{
    //   // Check version-byte of eeprom // Проверить версию-байт eeprom
    //   uint8_t version = eeprom_get_char(0);
    //   if (version == SETTINGS_VERSION) {
    //     // Read settings-record and check checksum // Настройки чтения - запись и проверка контрольной суммы
    //     if (!(memcpy_from_eeprom_with_checksum((char*)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t)))) {
    //       return(false);
    //     }
    //   } else {
    //     return(false);
    //   }
    //   return(true);
    return false;
}

    float Machine::GetPosition(uint8_t axis){
        float pos;
        #ifdef COREXY
            if (axis==X_AXIS) { 
            pos = (float)system_convert_corexy_to_x_axis_steps(steps) / settings.steps_per_mm[A_MOTOR];
            } else if (axis==Y_AXIS) {
            pos = (float)system_convert_corexy_to_y_axis_steps(steps) / settings.steps_per_mm[B_MOTOR];
            } else {
            pos = steps[axis]/settings.steps_per_mm[axis];
            }
        #else
            pos = position_[axis]/settings_.steps_per_mm[axis];
        #endif
        return(pos);
    }
    void Machine::SetPosition(uint8_t axis, int32_t value){
         position_[axis] = value;
    }

    void Machine::SetProbeSucceeded(uint8_t value){
        probe_succeeded_ = value;
    }

    uint8_t Machine::GetProbeSucceeded(){
        return probe_succeeded_;
    }
