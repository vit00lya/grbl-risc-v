
#include "system.h"
 
// void SystemClockConfig()
// {
//     PCC_InitTypeDef PCC_OscInit = {0};

//     PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
//     PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
//     PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
//     PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
//     PCC_OscInit.AHBDivider = 0;
//     PCC_OscInit.APBMDivider = 0;
//     PCC_OscInit.APBPDivider = 0;
//     PCC_OscInit.HSI32MCalibrationValue = 128;
//     PCC_OscInit.LSI32KCalibrationValue = 8;
//     PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
//     PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
//     HAL_PCC_Config(&PCC_OscInit);

//     __HAL_PCC_GPIO_0_CLK_ENABLE(); // Включение тактирования
//     __HAL_PCC_GPIO_1_CLK_ENABLE();
//     __HAL_PCC_GPIO_2_CLK_ENABLE();

//     //Включение прерываний
//     __HAL_PCC_GPIO_IRQ_CLK_ENABLE();


// }
