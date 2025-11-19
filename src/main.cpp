extern "C" {
    #include <mik32_memory_map.h>
    #include <pad_config.h>
    #include <gpio.h>
    #include <power_manager.h>
    #include <wakeup.h>
}

void InitClock()
{
	PM->CLK_APB_P_SET |= PM_CLOCK_APB_P_UART_0_M | PM_CLOCK_APB_P_GPIO_0_M | PM_CLOCK_APB_P_GPIO_1_M | PM_CLOCK_APB_P_GPIO_2_M; // включение тактирования GPIO
	PM->CLK_APB_M_SET |= PM_CLOCK_APB_M_PAD_CONFIG_M | PM_CLOCK_APB_M_WU_M | PM_CLOCK_APB_M_PM_M;								// включение тактирования блока для смены режима выводов
}

class Gpio{                           // Появились классы (в C их нет)
    public :

    GPIO_TypeDef* port;           // Здесь хранится номер порта, например GPIO_0
    uint32_t pin;                 // Здесь хранится номер вывода у порта, например 9

    Gpio(GPIO_TypeDef* _port, uint32_t _pin) {  // Функция-конструктор
        port = _port;
        pin = _pin;
    }

    void toggle() {                // Метод класса для переключения уазанного GPIO
        port->OUTPUT ^= 1 << pin;
    }
};

int main() {
    InitClock(); // Включение тактирования GPIO
    
    auto LED = Gpio(GPIO_0, 9);     // Создание объекта LED

	PAD_CONFIG->PORT_0_CFG &= ~(0b11 << (2 * LED.pin));	// Установка вывода 9 порта 0 в режим GPIO

	LED.port->DIRECTION_OUT = 1 << LED.pin;	// Установка направления вывода 9 порта 0 на выход

    while(true) {
        LED.toggle();           // Вызов метода "переключить" для объекта LED

        for (volatile int i = 0; i < 500000; i++) ;  // 1 мс = 2000 проходов этого пустого цикла
    }
}