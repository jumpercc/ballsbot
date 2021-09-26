#pragma once

#include <stdint.h>

class TCA9548 {
public:
    TCA9548(const uint8_t bus_number = 0, const uint8_t device_address = 0x77u)
        : bus_number_(bus_number), device_address_(device_address) {
    }
    TCA9548(const TCA9548&) = delete;
    TCA9548(const TCA9548&&) = delete;
    TCA9548& operator=(const TCA9548&) = delete;
    TCA9548& operator=(const TCA9548&&) = delete;
    ~TCA9548() {
        Finish();
    }

    void Start();
    void Finish();
    void SetState(const uint8_t new_state);
    uint8_t GetState();

private:
    uint8_t bus_number_;
    uint8_t device_address_;
    int file_descriptor_ = -1;
    uint8_t state_ = 0;
};
