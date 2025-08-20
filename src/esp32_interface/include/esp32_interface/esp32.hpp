#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <iostream>
#include "esp32_msgs/msg/imu_data.hpp"
#include <vector>
#include <unordered_map>
class ESP32Interface : public rclcpp::Node
{
public:
    struct packet_t {
        uint8_t addr;
        uint8_t size;
    };


    ESP32Interface(int argc, char **argv, rclcpp::NodeOptions &options);
    ~ESP32Interface();
    void readerThread(boost::asio::serial_port &serial);
    void read_register(uint8_t addr);
    void write_register(uint8_t addr, int16_t value);
    void read_batch(uint8_t addr);
    void readDataPublisher();
    void dataQuery();
    bool check_size(uint8_t addr, size_t actual_size);
    void handle_packet(const std::vector<uint8_t> &packet);
private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
    std::mutex data_mutex_;
    std::mutex serial_mutex;
    std::condition_variable queue_cv;
    std::queue<packet_t> packet_queue;

    rclcpp::TimerBase::SharedPtr read_publish_timer_;
    rclcpp::TimerBase::SharedPtr data_query_timer_;

    rclcpp::Publisher<esp32_msgs::msg::ImuData>::SharedPtr imu_data_publisher_;
    // rclcpp::Service<esp32_msgs::srv::Calibrate>::SharedPtr caliberation_service_;

    esp32_msgs::msg::ImuData imu_data_;
    std::thread reader_thread_;

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;
    int8_t imu_full_scale = 0;
    std::unordered_map<uint8_t, size_t> expected_sizes_;

};