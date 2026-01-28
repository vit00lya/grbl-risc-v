#pragma once
#include <bitset>
#include "grbl.h"
#include "settings.h"
#include "report.h"
#include "settings.h"
#include "limits.h"


class Machine
{
private:
    std::array<Limits, N_AXIS>limits_;
    Report &report_;
    bool init_ = false;
    settings_t settings_;
    std::bitset<8> sys_rt_exec_alarm_;
    std::array<int32_t, N_AXIS> position_;
    uint8_t probe_succeeded_;
    uint8_t machine_state_;

public:
    Machine(Report& report):report_(report){};
    void Init();
    void SettingsRestore(uint8_t restore_flag);
    bool ReadGlobalSettings();
    void PrintSettings();
    std::bitset<8> SysRtExecAlarmGet();
    void SysRtExecAlarmSet(uint8_t);
    char* LimitsGetState();
    bool GetInit();
    float GetPosition(uint8_t axis);
    void SetPosition(uint8_t axis, int32_t value);
    uint8_t GetProbeSucceeded();
    void SetProbeSucceeded(uint8_t);
    uint8_t GetMachineState();
    void SetMachineState(uint8_t);
    void Reset();

};

