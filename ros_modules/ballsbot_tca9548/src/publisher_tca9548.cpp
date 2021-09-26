#include "tca9548.h"
#include "VL53L0X.h"
#include "magnetic_encoder_driver.h"
#include <stdint.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include "ros/ros.h"
#include "ballsbot_tca9548/LaserDistance.h"
#include "ballsbot_tca9548/EncoderAngle.h"
#include <memory>

enum kDeviceType {
    kLaserRangingSensor = 0,
    kMagneticEncoderAS5600 = 1,
    kMagneticEncoderAS5048B = 2,
};

struct I2CDevice {
    I2CDevice(kDeviceType device_type, std::string device_name)
        : device_type_(device_type), device_name_(device_name) {
    }

    kDeviceType device_type_;
    std::string device_name_;
    std::shared_ptr<ros::Publisher> publisher_ = nullptr;
};

struct ConfigItem {
    uint8_t state_bits;
    std::vector<I2CDevice> devices;
};

std::unique_ptr<VL53L0X> GetLaserSensor(uint8_t bus_number) {
    auto sensor = std::make_unique<VL53L0X>(bus_number, 0x29);
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

std::unique_ptr<EncoderBase> GetEncoder(uint8_t bus_number, kDeviceType device_type) {
    std::unique_ptr<EncoderBase> result;
    if (device_type == kMagneticEncoderAS5600) {
        result = std::make_unique<AMS_AS5600>(bus_number);
    } else {
        result = std::make_unique<AMS_AS5048B>(bus_number);
    }
    result->OpenSensor();
    result->SetClockWise(false);
    return result;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "USAGE: publisher_tca9548 <bus_number> <device_address>\n";
        exit(1);
    }
    uint8_t bus_number = atoi(argv[1]);
    uint8_t address = atoi(argv[2]);

    std::vector<ConfigItem> config = {
        {0b10000000u, {{kLaserRangingSensor, "front"}}},
        {0b01000000u, {{kLaserRangingSensor, "rear"}}},
        {0b00100000u,
         {
             {kLaserRangingSensor, "manipulator"},
             {kMagneticEncoderAS5600, "m0"},
         }},
    };

    ros::init(argc, argv, "ballsbot_tca9548");

    TCA9548 controller(bus_number, address);
    controller.Start();

    ballsbot_tca9548::LaserDistance ld_msg;
    ld_msg.bus_number = bus_number;
    uint16_t distance;

    ballsbot_tca9548::EncoderAngle enc_msg;
    double angle;

    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        for (auto config_item : config) {
            controller.SetState(config_item.state_bits);
            for (auto device : config_item.devices) {
                if (device.publisher_ == nullptr) {
                    if (device.device_type_ == kLaserRangingSensor) {
                        device.publisher_ = std::make_shared<ros::Publisher>(
                            n.advertise<ballsbot_tca9548::LaserDistance>(device.device_name_, 1));
                    } else if (device.device_type_ == kMagneticEncoderAS5600 ||
                               device.device_type_ == kMagneticEncoderAS5048B) {
                        device.publisher_ = std::make_shared<ros::Publisher>(
                            n.advertise<ballsbot_tca9548::EncoderAngle>(device.device_name_, 1));
                    }
                }

                if (device.device_type_ == kLaserRangingSensor) {
                    auto laser_sensor = GetLaserSensor(bus_number);
                    distance = laser_sensor->readRangeSingleMillimeters();
                    if (laser_sensor->timeoutOccurred()) {
                        ROS_INFO("Sensor timeout!");
                        laser_sensor = nullptr;
                        continue;
                    } else if (distance > 2500) {
                        distance = 0xFFFF;
                    }
                    laser_sensor->closeVL53L0X();

                    ld_msg.distance_in_mm = distance;
                    ld_msg.direction = device.device_name_;
                    ROS_INFO("%s: %i mm", ld_msg.direction.c_str(), ld_msg.distance_in_mm);
                    ROS_INFO("-");
                    device.publisher_->publish(ld_msg);
                } else if (device.device_type_ == kMagneticEncoderAS5600 ||
                           device.device_type_ == kMagneticEncoderAS5048B) {
                    auto encoder = GetEncoder(bus_number, device.device_type_);
                    angle = encoder->GetAngle(U_RAD, true);
                    encoder->CloseSensor();

                    enc_msg.angle = angle;
                    enc_msg.encoder_name = device.device_name_;
                    ROS_INFO("%s: %0.4f rad", enc_msg.encoder_name.c_str(), enc_msg.angle);
                    ROS_INFO("-");
                    device.publisher_->publish(enc_msg);
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    controller.SetState(0);
    controller.Finish();
    return 0;
}
