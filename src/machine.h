
#include "grbl.h"
#include "settings.h"
#include "report.h"
#include "settings.h"
#include "limits.h"

class Machine
{
private:
    std::array<Limits, N_AXIS>limits;
    Report &report_;
    bool init_ = false;
    settings_t settings_;
    std::bitset<8> sys_rt_exec_alarm_;


public:
    Machine(Report& report):report_(report){};
    void Init();
    void SettingsRestore(uint8_t restore_flag);
    bool ReadGlobalSettings();
    void PrintSettings();
    std::bitset<8> SysRtExecAlarmGet();
    void SysRtExecAlarmSet(uint8_t);
    std::bitset<8> LimitsGetState();

};

