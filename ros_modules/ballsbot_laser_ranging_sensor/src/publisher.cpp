#include <VL53L0X.h>
#include "ros/ros.h"
#include "ballsbot_laser_ranging_sensor/LaserDistance.h"
#include <string>
#include <cstdint>
#include <stdexcept>
#include <fstream>
#include <memory>

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

#define FRONT_ADDRESS 0x27
#define REAR_ADDRESS 0x28

std::unique_ptr<VL53L0X> get_sensor(unsigned char bus, uint8_t address) {
    auto sensor = std::make_unique<VL53L0X>(bus, address);
    if (!sensor->openVL53L0X()) {
        ROS_INFO("Unable to open VL53L0X");
        exit(-1);
    }
    sensor->init();
    sensor->setTimeout(500);
#if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor->setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor->setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor->setMeasurementTimingBudget(200000);
#endif
    return sensor;
}

int main(int argc, char** argv) {
    unsigned char bus = 0;
    uint8_t address = FRONT_ADDRESS;
    std::string direction("unknown");
    std::string topic_name("laser_distance_");

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
    topic_name += direction;

    ros::init(argc, argv, "ballsbot_laser_ranging_sensor");

    auto sensor = get_sensor(bus, address);

    ros::NodeHandle n;
    ros::Publisher chatter_pub =
        n.advertise<ballsbot_laser_ranging_sensor::LaserDistance>(topic_name, 1);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ballsbot_laser_ranging_sensor::LaserDistance msg;

        uint16_t distance = sensor->readRangeSingleMillimeters();
        if (sensor->timeoutOccurred()) {
            ROS_INFO("Sensor timeout!");
            sensor->closeVL53L0X();
            sensor = get_sensor(bus, address);
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

    return 0;
}
