#include <math.h>
#include <cstdint>

#ifndef _AMS_AS5048B_H_
#define _AMS_AS5048B_H_

// Default addresses for AS5048B
#define AS5048_ADDRESS 0x40         // 0b10000 + ( A1 & A2 to GND)
#define AS5048B_ANGLMSB_REG 0xFE    // bits 0..7
#define AS5048B_RESOLUTION 16384.0  // 14 bits

// unit consts - just to make the units more readable
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4

class AMS_AS5048B {
public:
    AMS_AS5048B(uint8_t i2c_bus_number = 0, uint8_t i2c_address = AS5048_ADDRESS)
        : i2c_file_descriptor_(-1),
          clock_wise_(true),
          i2c_address_(i2c_address),
          i2c_bus_number_(i2c_bus_number),
          last_angle_raw_(0.) {
    }

    void OpenSensor();
    void CloseSensor();
    void SetClockWise(bool cw = true);
    double GetAngle(int unit = U_RAW, bool new_val = true);

private:
    int i2c_file_descriptor_;
    bool clock_wise_;
    uint8_t i2c_address_;
    uint8_t i2c_bus_number_;
    double last_angle_raw_;

    // methods
    uint8_t ReadReg8(uint8_t address);
    // 16 bit value got from 2x8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
    uint16_t ReadReg16(uint8_t address);
    // RAW, TRN, DEG, RAD
    double ConvertAngle(int unit, double angle);
};

#endif
