#include "machine.h"

void Machine::Init(){
    
     if (!ReadGlobalSettings())
    {
        report_.StatusMessage(STATUS_SETTING_READ_FAIL);
        SettingsRestore(SETTINGS_RESTORE_ALL); // Force restore all EEPROM data. // Принудительно восстановите все данные EEPROM.
        report_.GrblSettings(settings_);
    }
}

void Machine::PrintSettings(){
    report_.GrblSettings(settings_);  
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

        settings_.steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM;
        settings_.steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM;
        settings_.steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM;
        settings_.max_rate[X_AXIS] = DEFAULT_X_MAX_RATE;
        settings_.max_rate[Y_AXIS] = DEFAULT_Y_MAX_RATE;
        settings_.max_rate[Z_AXIS] = DEFAULT_Z_MAX_RATE;
        settings_.acceleration[X_AXIS] = DEFAULT_X_ACCELERATION;
        settings_.acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION;
        settings_.acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION;
        settings_.max_travel[X_AXIS] = (-DEFAULT_X_MAX_TRAVEL);
        settings_.max_travel[Y_AXIS] = (-DEFAULT_Y_MAX_TRAVEL);
        settings_.max_travel[Z_AXIS] = (-DEFAULT_Z_MAX_TRAVEL);

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
