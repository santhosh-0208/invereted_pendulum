#include <iostream>
#include <boost/asio.hpp>
#include <chrono>
#include <thread>

#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>

struct Packet {
    uint8_t addr;
    int16_t value;
};

std::queue<Packet> packet_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;

enum esp_read_response_ {
    ESP_READ_SUCCESS = 0,
    ESP_READ_TIMEOUT = -1,
    ESP_READ_ERROR = -2
};

void readerThread(boost::asio::serial_port &serial)
{
    const uint8_t START_BYTE = 0x77;
    const uint8_t END_BYTE   = 0xAA;
    const size_t PACKET_SIZE = 6;

    uint8_t buf[1024];
    size_t buf_len = 0;

    while (true) {
        boost::system::error_code ec;
        size_t n = serial.read_some(boost::asio::buffer(buf + buf_len, sizeof(buf) - buf_len), ec);
        if (ec) {
            std::cerr << "Serial error: " << ec.message() << "\n";
            break;
        }
        buf_len += n;

        // try to parse packets out of buffer
        size_t i = 0;
        while (buf_len - i >= PACKET_SIZE) {
            if (buf[i] == START_BYTE && buf[i+5] == END_BYTE) {
                uint8_t addr     = buf[i+1];
                uint8_t val_low  = buf[i+2];
                uint8_t val_high = buf[i+3];
                uint8_t checksum = buf[i+4];
                if (checksum == (addr ^ val_low ^ val_high)) {
                    int16_t value = (int16_t)((val_high << 8) | val_low);
                    {
                        std::lock_guard<std::mutex> lock(queue_mutex);
                        packet_queue.push({addr, value});
                    }
                    queue_cv.notify_one();
                }
                i += PACKET_SIZE; // consume packet
            } else {
                i++; // resync
            }
        }
        // move leftover bytes to front
        if (i > 0) {
            memmove(buf, buf + i, buf_len - i);
            buf_len -= i;
        }
    }
}

esp_read_response_ read_register(boost::asio::serial_port &serial, uint8_t addr, int timeout_ms, int16_t &value)
{
    const uint8_t START_BYTE = 0x77;
    const uint8_t END_BYTE   = 0xAA;
    const size_t PACKET_SIZE = 6; // start + addr + low + high + checksum + end

    // Send request: 'R' + address
    uint8_t request[2] = {'R', addr};
    boost::asio::write(serial, boost::asio::buffer(request, 2));

    uint8_t resp[PACKET_SIZE];
    auto start_time = std::chrono::steady_clock::now();

    // === 1. Wait for START_BYTE ===
    // std::cout << "Waiting for start byte...\n";
    while (true) {
        uint8_t byte;
        boost::system::error_code ec;
        size_t n = serial.read_some(boost::asio::buffer(&byte, 1), ec);

        if (n > 0 && byte == START_BYTE) {
            resp[0] = byte;
            break;
        }

        // Timeout check
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count() > timeout_ms)
        {
            std::cerr << "Timeout waiting for start byte\n";
            return ESP_READ_TIMEOUT;
        }

        // Avoid CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // === 2. Read remaining bytes ===
    // std::cout << "Waiting for rest of packet...\n";
    size_t total_read = 1; // already got start byte
    while (total_read < PACKET_SIZE) {
        boost::system::error_code ec;
        size_t n = serial.read_some(boost::asio::buffer(resp + total_read, PACKET_SIZE - total_read), ec);

        if (ec) {
            std::cerr << "Error reading from serial port: " << ec.message() << "\n";
            return ESP_READ_ERROR;
        }

        if (n > 0) {
            total_read += n;
        }

        // Timeout check
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count() > timeout_ms)
        {
            std::cerr << "Timeout waiting for rest of packet\n";
            return ESP_READ_TIMEOUT;
        }

        // Avoid CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // === 3. Parse ===
    uint8_t recv_addr = resp[1];
    uint8_t val_low   = resp[2];
    uint8_t val_high  = resp[3];
    uint8_t checksum  = resp[4];
    uint8_t terminator= resp[5];

    // std::cout << "Received packet: "
    //           << std::hex << +recv_addr << " " << +val_low << " "
    //           << +val_high << " " << +checksum << " " << +terminator
    //           << std::dec << "\n";

    // === 4. Validate ===
    if (recv_addr != addr) {
        // std::cerr << "Address mismatch! Expected: " << +addr << " Got: " << +recv_addr << "\n";
        return ESP_READ_ERROR;
    }

    if (terminator != END_BYTE) {
        // std::cerr << "Invalid packet terminator: " << std::hex << +terminator << std::dec << "\n";
        return ESP_READ_ERROR;
    }

    if (checksum != (recv_addr ^ val_low ^ val_high)) {
        // std::cerr << "Checksum mismatch!\n";
        return ESP_READ_ERROR;
    }

    // === 5. Combine signed value ===
    value = (int16_t)((val_high << 8) | val_low);
    return ESP_READ_SUCCESS;
}


int main()
{
    try {
        boost::asio::io_service io;
        boost::asio::serial_port serial(io, "/dev/ttyUSB0");

        // Configure serial port
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::parity(
            boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));

        int16_t ax = 0; int16_t ay = 0; int16_t az = 0;
        esp_read_response_ response;
        std::thread reader(readerThread, std::ref(serial));

        while (true) {
            // send requests (non-blocking)
        uint8_t req1[2] = {'R', 0x01};
        uint8_t req2[2] = {'R', 0x02};
        uint8_t req3[2] = {'R', 0x03};
        boost::asio::write(serial, boost::asio::buffer(req1,2));
        boost::asio::write(serial, boost::asio::buffer(req2,2));
        boost::asio::write(serial, boost::asio::buffer(req3,2));

        // process all available packets
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cv.wait(lock, [] { return !packet_queue.empty(); });
            while (!packet_queue.empty()) {
                auto p = packet_queue.front();
                packet_queue.pop();
                switch (p.addr) {
                    case 0x01: ax = p.value; break;
                    case 0x02: ay = p.value; break;
                    case 0x03: az = p.value; break;
                }
            }
        }

        std::cout << "AX="<<ax<<" AY="<<ay<<" AZ="<<az<<"\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    } catch (std::exception &e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
