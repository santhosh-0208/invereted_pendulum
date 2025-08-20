#include "esp32.hpp"

ESP32Interface::ESP32Interface(int argc, char **argv, rclcpp::NodeOptions &options)
    : Node("esp_32_interface", options), serial_(io_, "/dev/ttyUSB0")
{
    try
    {
        serial_.open("/dev/ttyUSB0");
        serial_.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));
    }
    catch (std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    reader_thread_ = std::thread(&ESP32Interface::readerThread, this, std::ref(serial_));
    data_query_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ESP32Interface::dataQuery, this));
    read_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&ESP32Interface::readDataPublisher, this));
    // caliberation_service_ = this->create_service<std>(
    //     "calibrate", std::bind(&ESP32Interface::calibrate, this, std::placeholders::_1, std::placeholders::_2));

    imu_data_publisher_ = this->create_publisher<esp32_msgs::msg::ImuData>("/imu_data", 10);
    imu_data_.ax = 0;
    imu_data_.ay = 0;
    imu_data_.az = 0;

    expected_sizes_ = {
        {0x01, 6},
        {0x02, 6},
        {0x03, 6},
        {0x10, 10}, // Example for batch read
    };
    // read imu full scale
    // read_register(0x1C);
}
ESP32Interface::~ESP32Interface()
{
    if (reader_thread_.joinable())
    {
        reader_thread_.join();
    }
    serial_.close();
}
// void ESP32Interface::calibrate(const std::shared_ptr<esp32_msgs::srv::Calibrate::Request> request,
//                                std::shared_ptr<esp32_msgs::srv::Calibrate::Response> response)
// {
//     // Implement calibration logic here
//     int samples = 50;

//     RCLCPP_INFO(this->get_logger(), "Calibration requested");
//     response->success = true; // Set to true if calibration is successful
// }
void ESP32Interface::readerThread(boost::asio::serial_port &serial)
{
    const uint8_t START_BYTE = 0x77;
    const uint8_t END_BYTE = 0xAA;

    std::vector<uint8_t> packet;
    bool reading_packet = false;

    uint8_t buf[256];
    while (true)
    {
        boost::system::error_code ec;
        size_t n = serial.read_some(boost::asio::buffer(buf), ec);
        if (ec)
        {
            std::cerr << "Serial error: " << ec.message() << "\n";
            break;
        }

        for (size_t i = 0; i < n; i++)
        {
            uint8_t b = buf[i];

            if (!reading_packet)
            {
                if (b == START_BYTE)
                {
                    packet.clear();
                    packet.push_back(b);
                    reading_packet = true;
                }
            }
            else
            {
                packet.push_back(b);

                if (b == END_BYTE)
                {
                    // complete packet received
                    handle_packet(packet);
                    reading_packet = false;
                }
            }
        }
    }
}

bool ESP32Interface::check_size(uint8_t addr, size_t actual_size)
{
    auto it = expected_sizes_.find(addr);
    if (it != expected_sizes_.end())
    {
        if (it->second != actual_size)
        {
            std::cerr << "Warning: packet size mismatch for addr "
                      << std::hex << (int)addr
                      << " expected " << it->second
                      << " got " << actual_size << "\n";
            return false;
        }
    }
    else
    {
        std::cerr << "Unknown address: " << std::hex << (int)addr << "\n";
        return false;
    }
    return true;
}
void ESP32Interface::handle_packet(const std::vector<uint8_t> &packet)
{
    uint8_t addr = packet[1];
    size_t actual_size = packet.size();

    uint8_t checksum = 0;
    if (!check_size(addr, actual_size))
    {
        return; // size mismatch, skip processing
    }
    for (size_t i = 1; i < actual_size - 2; i++)
    {
        checksum ^= packet[i];
    }
    if (checksum != packet[actual_size - 2])
    {
        std::cerr << "Checksum failed for address: " << std::hex << (int)addr << "\n";
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    switch (addr)
    {
    case 0x01: // Accelerometer X
    {
        ax = (packet[2] | (packet[3] << 8));
        break;
    }
    case 0x02: // Accelerometer Y
    {
        ay = (packet[2] | (packet[3] << 8));
        break;
    }
    case 0x03: // Accelerometer Z
    {
        az = (packet[2] | (packet[3] << 8));
        break;
    }
    case 0x10: // Batch read
    {
        ax = (packet[2] | (packet[3] << 8));
        ay = (packet[4] | (packet[5] << 8));
        az = (packet[6] | (packet[7] << 8));
        break;
    }
    case 0x1C: // Config
    {
        uint8_t accel_config = packet[2];
        imu_full_scale = (accel_config & 0x03);
        break;
    }
    default:
    {
        std::cerr << "Unhandled address: " << std::hex << (int)addr << "\n";
        break;
    }
    }
}

void ESP32Interface::read_register(uint8_t addr)

{
    const uint8_t START_BYTE = 0x77;
    const uint8_t END_BYTE = 0xAA;
    const size_t PACKET_SIZE = 6; // start + addr + low + high + checksum + end

    // Send request: 'R' + address
    uint8_t request[2] = {'R', addr};
    std::lock_guard<std::mutex> lock(serial_mutex);
    boost::asio::write(serial_, boost::asio::buffer(request, 2));
    // append address to pending read requests
}

void ESP32Interface::write_register(uint8_t addr, int16_t value)
{
    const size_t PACKET_SIZE = 4; // start + addr + low + high + checksum + end

    // Prepare packet
    uint8_t packet[PACKET_SIZE];
    packet[0] = 'W';
    packet[1] = addr;
    packet[2] = value & 0xFF;        // low byte
    packet[3] = (value >> 8) & 0xFF; // high byte

    // Send packet
    std::lock_guard<std::mutex> lock(serial_mutex);
    boost::asio::write(serial_, boost::asio::buffer(packet, PACKET_SIZE));
}
void ESP32Interface::read_batch(uint8_t addr)
{
    uint8_t request[2] = {'Q', addr};
    std::lock_guard<std::mutex> lock(serial_mutex);
    boost::asio::write(serial_, boost::asio::buffer(&request, 2));
}

void ESP32Interface::dataQuery()
{
    // read_register(0x01, 20, ax);
    // read_register(0x02, 20, ay);
    // read_register(0x03, 20, az);
    read_batch(0x10);
    // std::cout << "sending data query\n";
}
void ESP32Interface::readDataPublisher()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_data_.ax = ax;
    imu_data_.ay = ay;
    imu_data_.az = az;
    // std::cout << "AX="<<ax<<" AY="<<ay<<" AZ="<<az<<"\n";
    imu_data_publisher_->publish(imu_data_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<ESP32Interface>(argc, argv, options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    std::cout << "ESP32Interface node has been shut down." << std::endl;
    return 0;
}
