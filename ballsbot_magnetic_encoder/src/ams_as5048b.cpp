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
#include "ams_as5048b.h"

void AMS_AS5048B::OpenSensor() {
    clock_wise_ = false;
    last_angle_raw_ = 0.0;

    char file_name_buffer[32];
    sprintf(file_name_buffer, "/dev/i2c-%d", i2c_bus_number_);
    i2c_file_descriptor_ = open(file_name_buffer, O_RDWR);
    if (i2c_file_descriptor_ < 0) {
        throw std::runtime_error(std::string("Could not open the file /dev/i2c-xxx: ") +
                                 std::strerror(errno));
    }
    if (ioctl(i2c_file_descriptor_, I2C_SLAVE, i2c_address_) < 0) {
        throw std::runtime_error(std::string("Could not open the device on the bus: ") +
                                 std::strerror(errno));
    }
}

void AMS_AS5048B::CloseSensor() {
    if (i2c_file_descriptor_ > 0) {
        int close_return_value = close(i2c_file_descriptor_);
        i2c_file_descriptor_ = -1;
        if (close_return_value != 0) {
            throw std::runtime_error(std::string("close failed: ") + std::strerror(errno));
        }
    }
}

void AMS_AS5048B::SetClockWise(bool cw) {
    clock_wise_ = cw;
    last_angle_raw_ = 0.0;
}

double AMS_AS5048B::GetAngle(int unit, bool new_val) {
    double angle_raw;

    if (new_val) {
        if (clock_wise_) {
            angle_raw = (double)(0b11111111111111 - AMS_AS5048B::ReadReg16(AS5048B_ANGLMSB_REG));
        } else {
            angle_raw = (double)AMS_AS5048B::ReadReg16(AS5048B_ANGLMSB_REG);
        }
        last_angle_raw_ = angle_raw;
    } else {
        angle_raw = last_angle_raw_;
    }

    return AMS_AS5048B::ConvertAngle(unit, angle_raw);
}

uint8_t AMS_AS5048B::ReadReg8(uint8_t address) {
    return i2c_smbus_read_byte_data(i2c_file_descriptor_, address);
}

uint16_t AMS_AS5048B::ReadReg16(uint8_t address) {
    // 16 bit value got from 2 8bits registers (7..0 MSB + 5..0 LSB) => 14 bits value
    uint16_t result = i2c_smbus_read_word_data(i2c_file_descriptor_, address);
    result = (result & 0xFF) << 6 | (result & 0x3F00) >> 8;
    return result;
}

double AMS_AS5048B::ConvertAngle(int unit, double angle) {
    // convert raw sensor reading into angle unit

    double result;

    switch (unit) {
        case U_RAW:
            // Sensor raw measurement
            result = angle;
            break;
        case U_TRN:
            // full turn ratio
            result = (angle / AS5048B_RESOLUTION);
            break;
        case U_DEG:
            // degree
            result = (angle / AS5048B_RESOLUTION) * 360.0;
            break;
        case U_RAD:
            // Radian
            result = (angle / AS5048B_RESOLUTION) * 2 * M_PI;
            break;
    }
    return result;
}