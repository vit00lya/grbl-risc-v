/*
  nuts_bolts.c - Shared functions
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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


#define MAX_INT_DIGITS 8 // Maximum number of digits in int32 (and float) // Максимальное количество цифр в int32 (с плавающей точкой)
 
// void SysObj::Init(void* machine){
//   SystemClockConfig();
//   machine_ = machine;
// }
// void* SysObj::GetMachine() {
//   return machine_;
// }


static void Timer16_StepInit(Timer16_HandleTypeDef& timer_step)
{
    // timer_step.Instance = TIMER_STEP;

    // /* Настройка тактирования */
    // timer_step.Clock.Source = TIMER16_SOURCE_INTERNAL_SYSTEM;
    // timer_step.CountMode = TIMER16_COUNTMODE_INTERNAL; /* При тактировании от Input1 не имеет значения */
    // timer_step.Clock.Prescaler = TIMER16_PRESCALER_1;
    // timer_step.ActiveEdge = TIMER16_ACTIVEEDGE_RISING; /* Выбирается при тактировании от Input1 */

    // /* Настройка режима обновления регистра ARR и CMP */
    // timer_step.Preload = TIMER16_PRELOAD_AFTERWRITE;

    // /* Настройки фильтра */
    // timer_step.Filter.ExternalClock = TIMER16_FILTER_NONE;
    // timer_step.Filter.Trigger = TIMER16_FILTER_NONE;

    // /* Настройка режима энкодера */
    // timer_step.EncoderMode = TIMER16_ENCODER_DISABLE;

    // /* Выходной сигнал */
    // timer_step.Waveform.Enable = TIMER16_WAVEFORM_GENERATION_ENABLE;
    // timer_step.Waveform.Polarity = TIMER16_WAVEFORM_POLARITY_NONINVERTED;

    // HAL_Timer16_Init(&timer_step);
}

void SystemClockConfig(Timer16_HandleTypeDef& timer_step)
{

    HAL_Init();

    Timer16_StepInit(timer_step);
    PCC_InitTypeDef PCC_OscInit = {0};

    PCC_OscInit.OscillatorEnable = PCC_OSCILLATORTYPE_ALL;
    PCC_OscInit.FreqMon.OscillatorSystem = PCC_OSCILLATORTYPE_OSC32M;
    PCC_OscInit.FreqMon.ForceOscSys = PCC_FORCE_OSC_SYS_UNFIXED;
    PCC_OscInit.FreqMon.Force32KClk = PCC_FREQ_MONITOR_SOURCE_OSC32K;
    PCC_OscInit.AHBDivider = 0;
    PCC_OscInit.APBMDivider = 0;
    PCC_OscInit.APBPDivider = 0;
    PCC_OscInit.HSI32MCalibrationValue = 128;
    PCC_OscInit.LSI32KCalibrationValue = 8;
    PCC_OscInit.RTCClockSelection = PCC_RTC_CLOCK_SOURCE_AUTO;
    PCC_OscInit.RTCClockCPUSelection = PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;
    HAL_PCC_Config(&PCC_OscInit);

    __HAL_PCC_GPIO_0_CLK_ENABLE(); // Включение тактирования
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    __HAL_PCC_GPIO_2_CLK_ENABLE();

    //Включение прерываний
    __HAL_PCC_GPIO_IRQ_CLK_ENABLE();

    /* Разрешить прерывания по уровню для линии EPIC GPIO_IRQ */
    HAL_EPIC_MaskLevelSet(HAL_EPIC_GPIO_IRQ_MASK);
    /* Разрешить глобальные прерывания */
    HAL_IRQ_EnableInterrupts();

}


/**
 * @brief Считать текущее состояние выводов порта GPIO_x.
 * @param GPIO_x порт GPIO_x, где x может быть (0, 1, 2).
 * @param pin маска выводов порта GPIO_x, с которых считывание значение.
 * @return @ref GPIO_PIN_HIGH если с одного или больше выводов, указанных в pin, считалась 1. Иначе @ref GPIO_PIN_LOW.
 */
GPIO_PinState ReadPin(GPIO_TypeDef *GPIO_x, HAL_PinsTypeDef pin)
{
    GPIO_PinState bitStatus;

    if ((GPIO_x->STATE >> pin  &  1) != 0)
    {
        bitStatus = GPIO_PIN_HIGH;
    }
    else
    {
        bitStatus = GPIO_PIN_LOW;
    }
    return bitStatus;
}


bool PinHightLevel(const HAL_PinsTypeDef pin, GPIO_TypeDef* port){
  
    GPIO_PinState state = HAL_GPIO_ReadPin(port,pin);
    if(state == GPIO_PIN_LOW){
      return false;
    };
    return true;
    
}

// Delays variable defined milliseconds. Compiler compatibility fix for _delay_ms(),
// which only accepts constants in future compiler releases.
// Задерживает переменную, определяемую в миллисекундах. Исправлена ошибка совместимости с компилятором для функции _delay_ms(),
// которая принимает константы только в будущих версиях компилятора.

void delay_ms(uint16_t ms) 
{
   HAL_DelayMs(ms);
}


// Delays variable defined microseconds. Compiler compatibility fix for _delay_us(),
// which only accepts constants in future compiler releases. Written to perform more 
// efficiently with larger delays, as the counter adds parasitic time in each iteration.

// Задержка определяется переменной в микросекундах. Исправлена ошибка совместимости с компилятором для _delay_us(),
// которая принимает константы только в будущих версиях компилятора. Написано для выполнения большего 
// эффективно с большими задержками, поскольку счетчик добавляет паразитное время на каждой итерации.
void delay_us(uint32_t us) 
{
  while (us) {
    if (us < 10) { 
      HAL_DelayUs(1);
      us--;
    } else if (us < 100) {
      HAL_DelayUs(10);
      us -= 10;
    } else if (us < 1000) {
      HAL_DelayUs(100);
      us -= 100;
    } else {
      HAL_DelayMs(1);
      us -= 1000;
    }
  }
}


// Simple hypotenuse computation function.
// Простая функция вычисления гипотенузы.
float hypot_f(float x, float y) { return(sqrt(x*x + y*y)); }

// Функция для сброса произвольных линий прерываний
// lines - битовая маска, где каждый бит соответствует линии прерывания
// Например, для сброса линий 0, 3 и 7 нужно передать
// (1 << (X_LIMIT_LINE_IRQ >> GPIO_IRQ_LINE_S)) |
// (1 << (Y_LIMIT_LINE_IRQ >> GPIO_IRQ_LINE_S)) |
// (1 << (Z_LIMIT_LINE_IRQ >> GPIO_IRQ_LINE_S))
void ClearGPIOInterrupts(uint8_t line_mask) {
    GPIO_IRQ->CLEAR = line_mask;
}

// Функция для сброса конкретных линий прерываний, заданных через HAL_GPIO_Line
void ClearGPIOInterruptLines(uint8_t mask) {
    GPIO_IRQ->CLEAR = mask;
}

