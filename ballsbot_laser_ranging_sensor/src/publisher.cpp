#include <VL53L0X.h>
#include "ros/ros.h"
#include "ballsbot_laser_ranging_sensor/LaserDistance.h"
#include <string>
#include "GPIOManager.h"
#include <cstdint>
#include <stdexcept>
#include <fstream>

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

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
    uint8_t address = FRONT_ADDRESS;
    std::string direction("unknown");

    if (argc >= 2) {
        bus = atoi(argv[1]);
    }
    if (argc >= 3) {
        direction = argv[2];
        if (direction == "front") {
            address = FRONT_ADDRESS;
        } else {
            address = REAR_ADDRESS;
        }
    }

    ChangeSensorsAddresses(bus);

    ros::init(argc, argv, "ballsbot_laser_ranging_sensor");
    ros::NodeHandle n;
    ros::Publisher chatter_pub =
        n.advertise<ballsbot_laser_ranging_sensor::LaserDistance>("laser_distance", 2);
    ros::Rate loop_rate(10);

    VL53L0X* sensor = new VL53L0X(bus, address);
    if (!sensor->openVL53L0X()) {
        ROS_INFO("Unable to open VL53L0X");
        return -1;
    }
    sensor->init();
    sensor->setTimeout(500);
#if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
#endif

    while (ros::ok()) {
        ballsbot_laser_ranging_sensor::LaserDistance msg;

        uint16_t distance = sensor->readRangeSingleMillimeters();
        if (sensor->timeoutOccurred()) {
            ROS_INFO("Sensor timeout!");
            continue;
        } else if (distance > 2500) {
            distance = 0xFFFF;
        }

        msg.distance_in_mm = distance;
        msg.bus_number = bus;
        msg.direction = direction;
        ROS_INFO("%i bus %i, %s: %i mm", argc, msg.bus_number, msg.direction.c_str(),
                 msg.distance_in_mm);
        ROS_INFO("-");

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    sensor->closeVL53L0X();
    delete sensor;

    return 0;
}
