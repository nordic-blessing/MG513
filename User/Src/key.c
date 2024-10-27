#include "key.h"

//按键    ok
GPIO_PinState ReadKeyState() {
    GPIO_PinState mValue = GPIO_PIN_SET;
    if (HAL_GPIO_ReadPin(Key_OK_GPIO_Port, Key_OK_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(50);
        while (HAL_GPIO_ReadPin(Key_OK_GPIO_Port, Key_OK_Pin) == GPIO_PIN_RESET);
        mValue = GPIO_PIN_RESET;
    }
    return mValue;
}

//按键    on/off
uint16_t ReadKey2State() {
    static uint16_t mValue = 0;
    if (HAL_GPIO_ReadPin(Key_ON_GPIO_Port, Key_ON_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(50);
        while (HAL_GPIO_ReadPin(Key_ON_GPIO_Port, Key_ON_Pin) == GPIO_PIN_RESET);
        mValue = !mValue;
    }
    return mValue;
}
