#pragma once

#include <cstdint>
#include <string>
#include <mutex>

class GPIOManager {
public:
    GPIOManager(const int16_t xshutGPIOPin);

    /**
     * Power on the sensor by setting its XSHUT pin to high via host's GPIO.
     */
    void powerOn();

    /**
     * Power off the sensor by setting its XSHUT pin to low via host's GPIO.
     */
    void powerOff();

protected:
    int16_t xshutGPIOPin;

    // GPIO
    std::string gpioFilename;
    std::mutex fileAccessMutex;
    bool gpioInitialized = false;

    void initGPIO();
};