#include "GPIOManager.h"
#include <unistd.h>
#include <fstream>
#include <mutex>

GPIOManager::GPIOManager(const int16_t xshutGPIOPin) : xshutGPIOPin(xshutGPIOPin) {
}

void GPIOManager::initGPIO() {
    using namespace std::chrono_literals;
    if (this->gpioInitialized) {
        return;
    }

    // Set XSHUT pin mode (if pin set)
    if (this->xshutGPIOPin >= 0) {
        std::string gpioDirectionFilename = std::string("/sys/class/gpio/gpio") +
                                            std::to_string(this->xshutGPIOPin) +
                                            std::string("/direction");
        this->gpioFilename = std::string("/sys/class/gpio/gpio") +
                             std::to_string(this->xshutGPIOPin) + std::string("/value");

        std::lock_guard<std::mutex> guard(this->fileAccessMutex);

        std::ofstream file;
        for (int i = 0; i < 3; ++i) {
            file.open("/sys/class/gpio/export", std::ofstream::out);
            if (!file.is_open() || !file.good()) {
                file.close();
                if (i == 2) {
                    throw(std::runtime_error("Failed opening file: /sys/class/gpio/export"));
                }
                usleep(500000);
            } else {
                break;
            }
        }
        file << this->xshutGPIOPin;
        file.close();

        for (int i = 0; i < 3; ++i) {
            file.open(gpioDirectionFilename.c_str(), std::ofstream::out);
            if (!file.is_open() || !file.good()) {
                file.close();
                if (i == 2) {
                    throw(std::runtime_error(std::string("Failed opening file: ") +
                                             gpioDirectionFilename));
                }
                usleep(500000);
            } else {
                break;
            }
        }
        file << "out";
        file.close();
    }

    this->gpioInitialized = true;
}

void GPIOManager::powerOn() {
    this->initGPIO();

    if (this->xshutGPIOPin >= 0) {
        std::lock_guard<std::mutex> guard(this->fileAccessMutex);
        std::ofstream file;

        for (int i = 0; i < 5; ++i) {
            file.open(this->gpioFilename.c_str(), std::ofstream::out);
            if (!file.is_open() || !file.good()) {
                file.close();
                if (i == 4) {
                    throw(std::runtime_error(std::string("Failed opening file: ") +
                                             this->gpioFilename));
                }
                usleep(500000);
            }
            break;
        }

        file << "1";
        file.close();

        // t_boot is 1.2ms max, wait 2ms just to be sure
        usleep(2000);
    }
}

void GPIOManager::powerOff() {
    this->initGPIO();

    if (this->xshutGPIOPin >= 0) {
        std::lock_guard<std::mutex> guard(this->fileAccessMutex);
        std::ofstream file;

        for (int i = 0; i < 5; ++i) {
            file.open(this->gpioFilename.c_str(), std::ofstream::out);
            if (!file.is_open() || !file.good()) {
                file.close();
                if (i == 4) {
                    throw(std::runtime_error(std::string("Failed opening file: ") +
                                             this->gpioFilename));
                }
                usleep(500000);
            }
            break;
        }
        file << "0";
        file.close();
    }
}