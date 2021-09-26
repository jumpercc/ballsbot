#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
extern "C" {
#include <i2c/smbus.h>
}
#include <stdexcept>
#include "magnetic_encoder_driver.h"

EncoderConstants GetAS5048BConstants() {
    EncoderConstants result = {0x40, 0xFE, 16384.0, 0b11111111111111};
    return result;
}

EncoderConstants GetAS5600Constants() {
    EncoderConstants result = {0x36, 0x0E, 4096.0, 0b111111111111};
    return result;
}

void EncoderBase::OpenSensor() {
    clock_wise_ = false;
    last_angle_raw_ = 0.0;

    char file_name_buffer[32];
    sprintf(file_name_buffer, "/dev/i2c-%d", i2c_bus_number_);
    i2c_file_descriptor_ = open(file_name_buffer, O_RDWR);
    if (i2c_file_descriptor_ < 0) {
        throw std::runtime_error(std::string("Could not open the file /dev/i2c-xxx: ") +
                                 std::strerror(errno));
    }
    if (ioctl(i2c_file_descriptor_, I2C_SLAVE, constants_.i2c_address) < 0) {
        throw std::runtime_error(std::string("Could not open the device on the bus: ") +
                                 std::strerror(errno));
    }
}

void EncoderBase::CloseSensor() {
    if (i2c_file_descriptor_ > 0) {
        int close_return_value = close(i2c_file_descriptor_);
        i2c_file_descriptor_ = -1;
        if (close_return_value != 0) {
            throw std::runtime_error(std::string("close failed: ") + std::strerror(errno));
        }
    }
}

void EncoderBase::SetClockWise(bool cw) {
    clock_wise_ = cw;
    last_angle_raw_ = 0.0;
}

double EncoderBase::GetAngle(int unit, bool new_val) {
    double angle_raw;

    if (new_val) {
        if (clock_wise_) {
            angle_raw = static_cast<double>(constants_.inverse_bits -
                                            ReadAngle(constants_.register_address));
        } else {
            angle_raw = static_cast<double>(ReadAngle(constants_.register_address));
        }
        last_angle_raw_ = angle_raw;
    } else {
        angle_raw = last_angle_raw_;
    }

    return ConvertAngle(unit, angle_raw);
}

double EncoderBase::ConvertAngle(int unit, double angle) {
    // convert raw sensor reading into angle unit

    double result;

    switch (unit) {
        case U_RAW:
            // Sensor raw measurement
            result = angle;
            break;
        case U_TRN:
            // full turn ratio
            result = (angle / constants_.resolution);
            break;
        case U_DEG:
            // degree
            result = (angle / constants_.resolution) * 360.0;
            break;
        case U_RAD:
            // Radian
            result = (angle / constants_.resolution) * 2 * M_PI;
            break;
    }
    return result;
}

uint16_t AMS_AS5048B::ReadAngle(uint8_t address) {
    // 16 bit value got from 2 8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
    uint16_t result = i2c_smbus_read_word_data(i2c_file_descriptor_, address);
    result = (result & 0xFF) << 6 | (result & 0x3F00) >> 8;
    return result;
}

uint16_t AMS_AS5600::ReadAngle(uint8_t address) {
    // 16 bit value got from 2 8bits registers (7..0 MSB + 3..0 LSB) => 12 bits value
    uint16_t result = i2c_smbus_read_word_data(i2c_file_descriptor_, address);
    result = (result & 0xFF000) >> 8 | (result & 0x0F) << 8;
    return result;
}