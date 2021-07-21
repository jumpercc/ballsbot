#include <VL53L0X.h>
#include "ros/ros.h"
#include <string>
#include "GPIOManager.h"
#include <cstdint>
#include <stdexcept>
#include <fstream>

const int16_t FRONT_GPIO_PIN = 216;
#define ADDRESS_DEFAULT 0x29
#define FRONT_ADDRESS 0x27
#define REAR_ADDRESS 0x28
const std::string ADDRESSES_CHANGED("/tmp/laser_sensors_addresses_changed");

void ChangeSensorsAddresses(unsigned char bus) {
    std::ifstream already_done(ADDRESSES_CHANGED.c_str());
    if (already_done.good()) {
        return;
    }

    ROS_INFO("changing addresses for sensors");

    ROS_INFO("setting off front sensor");
    GPIOManager manager(FRONT_GPIO_PIN);
    manager.powerOff();

    ROS_INFO("changing for rear");
    VL53L0X* rear_sensor = new VL53L0X(bus, ADDRESS_DEFAULT);
    if (!rear_sensor->openVL53L0X()) {
        ROS_INFO("Unable to open VL53L0X");
        throw std::runtime_error("Unable to open VL53L0X, rear");
    }
    rear_sensor->init();
    rear_sensor->setAddress(REAR_ADDRESS);
    rear_sensor->closeVL53L0X();
    delete rear_sensor;

    ROS_INFO("setting on front sensor");
    manager.powerOn();

    ROS_INFO("changing for front");
    VL53L0X* front_sensor = new VL53L0X(bus, ADDRESS_DEFAULT);
    if (!front_sensor->openVL53L0X()) {
        ROS_INFO("Unable to open VL53L0X");
        throw std::runtime_error("Unable to open VL53L0X, front");
    }
    front_sensor->init();
    front_sensor->setAddress(FRONT_ADDRESS);
    front_sensor->closeVL53L0X();
    delete front_sensor;

    ROS_INFO("done");

    std::ofstream mark_as_done(ADDRESSES_CHANGED.c_str(), std::ofstream::out);
    if (!mark_as_done.is_open() || !mark_as_done.good()) {
        mark_as_done.close();
        throw(std::runtime_error(std::string("Failed opening file: ") + ADDRESSES_CHANGED));
    }
    mark_as_done.close();
}

int main(int argc, char** argv) {
    unsigned char bus = 0;

    if (argc >= 2) {
        bus = atoi(argv[1]);
    }

    ChangeSensorsAddresses(bus);
}