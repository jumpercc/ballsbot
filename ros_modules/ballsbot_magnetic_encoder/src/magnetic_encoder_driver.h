#pragma once
#include <math.h>
#include <cstdint>

struct EncoderConstants {
    uint8_t i2c_address;
    uint8_t register_address;
    double resolution;
    uint16_t inverse_bits;
};

EncoderConstants GetAS5048BConstants();
EncoderConstants GetAS5600Constants();

// unit consts - just to make the units more readable
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4

class EncoderBase {
public:
    EncoderBase(const EncoderConstants constants, uint8_t i2c_bus_number)
        : i2c_file_descriptor_(-1),
          clock_wise_(true),
          constants_(constants),
          i2c_bus_number_(i2c_bus_number),
          last_angle_raw_(0.) {
    }

    void OpenSensor();
    void CloseSensor();
    void SetClockWise(bool cw = true);
    double GetAngle(int unit = U_RAW, bool new_val = true);

protected:
    int i2c_file_descriptor_;
private:
    bool clock_wise_;
    EncoderConstants constants_;
    uint8_t i2c_bus_number_;
    double last_angle_raw_;

    // methods
    virtual uint16_t ReadAngle(uint8_t address) = 0;
    // RAW, TRN, DEG, RAD
    double ConvertAngle(int unit, double angle);
};

class AMS_AS5048B : public EncoderBase {
public:
    AMS_AS5048B(uint8_t i2c_bus_number = 0) : EncoderBase(GetAS5048BConstants(), i2c_bus_number) {
    }

private:
    uint16_t ReadAngle(uint8_t address) override;
};

class AMS_AS5600 : public EncoderBase {
public:
    AMS_AS5600(uint8_t i2c_bus_number = 0) : EncoderBase(GetAS5600Constants(), i2c_bus_number) {
    }

private:
    uint16_t ReadAngle(uint8_t address) override;
};