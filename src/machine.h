
#include "grbl.h"
#include "settings.h"
#include "report.h"
#include "settings.h"

class Machine
{
private:
    Report &report_;
    bool init_ = false;
    settings_t settings_;

public:
    Machine(Report& report):report_(report){};
    void Init();
    void SettingsRestore(uint8_t restore_flag);
    bool ReadGlobalSettings();
    void PrintSettings();

};

