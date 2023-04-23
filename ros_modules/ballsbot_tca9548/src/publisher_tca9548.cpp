#include <stdint.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <future>
#include <thread>
#include <chrono>
#include <unordered_map>
#include "ros/ros.h"
#include "ballsbot_tca9548/LaserDistance.h"
#include "ballsbot_tca9548/EncoderAngle.h"
#include "tca9548.h"
#include "VL53L0X.h"
#include "magnetic_encoder_driver.h"
using namespace std::chrono_literals;

const uint16_t kNotAvailable = 0xFFFFu;

enum kDeviceType {
    kLaserRangingSensor = 0,
    kMagneticEncoderAS5600 = 1,
    kMagneticEncoderAS5048B = 2,
};

struct I2CDevice {
    kDeviceType device_type_;
    std::string device_name_;
};

struct ConfigItem {
    uint8_t state_bits;
    std::vector<I2CDevice> devices;
};

std::shared_ptr<VL53L0X> GetLaserSensor(uint8_t bus_number) {
    auto sensor = std::make_shared<VL53L0X>(bus_number, 0x29);
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

std::shared_ptr<EncoderBase> GetEncoder(uint8_t bus_number, kDeviceType device_type) {
    std::shared_ptr<EncoderBase> result;
    if (device_type == kMagneticEncoderAS5600) {
        result = std::make_shared<AMS_AS5600>(bus_number);
    } else {
        result = std::make_shared<AMS_AS5048B>(bus_number);
    }
    result->OpenSensor();
    result->SetClockWise(false);  // TODO from config
    return result;
}

size_t CountDevices(const std::vector<ConfigItem>& config, kDeviceType device_type) {
    size_t result = 0;
    for (auto lane : config) {
        for (auto device : lane.devices) {
            if (device.device_type_ == device_type) {
                result += 1;
            }
        }
    }
    return result;
}

std::unordered_map<uint8_t, std::shared_ptr<VL53L0X>> laser_sensors_cache;

uint16_t GetDistanceFromLaserSensor(uint8_t bus_number, uint8_t sensor_key) {
    if (laser_sensors_cache.find(sensor_key) == laser_sensors_cache.end()) {
        laser_sensors_cache[sensor_key] = GetLaserSensor(bus_number);
    }
    auto laser_sensor = laser_sensors_cache[sensor_key];
    uint16_t distance = laser_sensor->readRangeSingleMillimeters();
    if (laser_sensor->timeoutOccurred()) {
        ROS_INFO("Sensor timeout!");
        laser_sensor->closeVL53L0X();
        laser_sensors_cache.erase(laser_sensors_cache.find(sensor_key));
        return kNotAvailable;
    } else if (distance > 2500) {
        distance = 2500;
    }
    return distance;
}

size_t timeouts_count = 0;

uint16_t GetDistanceFromLaserSensorWithTimeout(uint8_t bus_number, uint8_t sensor_key) {
    std::packaged_task<uint16_t(uint8_t, uint8_t)> task(GetDistanceFromLaserSensor);
    auto future = task.get_future();
    std::thread thr(std::move(task), bus_number, sensor_key);
    if (future.wait_for(2s) != std::future_status::timeout) {
        thr.join();
        return future.get();
    } else {
        thr.detach();
        ROS_INFO("Sensor timeout, thread still running!");
        if (++timeouts_count > 15) {
            throw std::runtime_error("timeouts limit exceeded");
        }
        auto laser_sensor = laser_sensors_cache[sensor_key];
        if (laser_sensor) {
            laser_sensor->closeVL53L0X();
        }
        laser_sensors_cache.erase(laser_sensors_cache.find(sensor_key));
        return kNotAvailable;
    }
}

std::unordered_map<uint8_t, std::shared_ptr<EncoderBase>> encoders_cache;

double GetAngleFromMagneticEncoder(uint8_t bus_number, kDeviceType device_type,
                                   uint8_t sensor_key) {
    if (encoders_cache.find(sensor_key) == encoders_cache.end()) {
        encoders_cache[sensor_key] = GetEncoder(bus_number, device_type);
    }
    auto encoder = encoders_cache[sensor_key];
    double angle = encoder->GetAngle(U_RAD, true);
    return angle;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "USAGE: publisher_tca9548 <bus_number> <device_address>\n";
        exit(1);
    }
    uint8_t bus_number = atoi(argv[1]);
    uint8_t address = atoi(argv[2]);

    std::vector<ConfigItem> config = {  // TODO config to external text file
        {0b00000001u, {{kLaserRangingSensor, "rear-right"}}},
        {0b00000010u, {{kLaserRangingSensor, "rear-left"}}},
        {0b00000100u, {{kLaserRangingSensor, "front-left"}}},
        {0b00001000u, {{kLaserRangingSensor, "front-right"}}},
        {0b00010000u, {{kMagneticEncoderAS5600, "m-2"}}},
        {0b00100000u,
         {
             {kLaserRangingSensor, "manipulator"},
             {kMagneticEncoderAS5600, "m-claw"},
         }},
        {0b01000000u,
         {
             {kLaserRangingSensor, "front-center"},
             {kMagneticEncoderAS5600, "m-0"},
         }},
        {0b10000000u,
         {
             {kLaserRangingSensor, "rear-center"},
             {kMagneticEncoderAS5600, "m-1"},
         }},
    };

    ros::init(argc, argv, "ballsbot_tca9548");
    ros::NodeHandle n;

    TCA9548 controller(bus_number, address);
    controller.Start();

    ballsbot_tca9548::LaserDistance ld_msg;
    uint16_t distance;
    auto laser_distance_publisher = n.advertise<ballsbot_tca9548::LaserDistance>(
        "laser_distance", CountDevices(config, kLaserRangingSensor) || 1);

    ballsbot_tca9548::EncoderAngle enc_msg;
    double angle;
    auto magnetic_encoder_publisher = n.advertise<ballsbot_tca9548::EncoderAngle>(
        "magnetic_encoder", CountDevices(config, kMagneticEncoderAS5600) || 1);

    ros::Rate loop_rate(8);
    while (ros::ok()) {
        for (auto config_item : config) {
            controller.SetState(config_item.state_bits);
            for (auto device : config_item.devices) {
                if (device.device_type_ == kLaserRangingSensor) {
                    distance =
                        GetDistanceFromLaserSensorWithTimeout(bus_number, config_item.state_bits);
                    if (distance == kNotAvailable) {
                        continue;
                    }
                    ld_msg.distance_in_mm = distance;
                    ld_msg.sensor_name = device.device_name_;
                    ld_msg.header.stamp = ros::Time::now();
                    // ROS_INFO("%s: %i mm", ld_msg.sensor_name.c_str(), ld_msg.distance_in_mm);
                    laser_distance_publisher.publish(ld_msg);
                } else if (device.device_type_ == kMagneticEncoderAS5600 ||
                           device.device_type_ == kMagneticEncoderAS5048B) {
                    angle = GetAngleFromMagneticEncoder(bus_number, device.device_type_,
                                                        config_item.state_bits);
                    enc_msg.angle = angle;
                    enc_msg.sensor_name = device.device_name_;
                    enc_msg.header.stamp = ros::Time::now();
                    // ROS_INFO("%s: %0.4f rad", enc_msg.encoder_name.c_str(), enc_msg.angle);
                    magnetic_encoder_publisher.publish(enc_msg);
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
