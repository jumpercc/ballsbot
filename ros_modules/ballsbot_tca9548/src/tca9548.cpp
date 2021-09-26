#include "tca9548.h"
#include <stdexcept>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
extern "C" {
#include <i2c/smbus.h>
}

void TCA9548::Start() {
    char file_name_buffer[32];
    sprintf(file_name_buffer, "/dev/i2c-%d", bus_number_);
    file_descriptor_ = open(file_name_buffer, O_RDWR);
    if (file_descriptor_ < 0) {
        throw std::runtime_error("Could not open /dev/i2c-*");
    }
    if (ioctl(file_descriptor_, I2C_SLAVE, device_address_) < 0) {
        throw std::runtime_error("Could not open the device on the bus");
    }
}

void TCA9548::Finish() {
    if (file_descriptor_ > 0) {
        close(file_descriptor_);
        // WARNING - This is not quite right, need to check for error first
        file_descriptor_ = -1 ;
    }
}

void TCA9548::SetState(const uint8_t new_state) {
    if (file_descriptor_ < 0) {
        throw std::runtime_error("You should call Start first");
    }
    int result = i2c_smbus_write_byte_data(file_descriptor_, 0, new_state);
    if (result < 0) {
        throw std::runtime_error("SetState failed");
    }
    state_ = new_state;
}

uint8_t TCA9548::GetState() {
    if (file_descriptor_ < 0) {
        throw std::runtime_error("You should call Start first");
    }
    // no real reading, just last set state
    return state_;
}