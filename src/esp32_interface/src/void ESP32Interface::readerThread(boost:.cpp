void ESP32Interface::readerThread(boost::asio::serial_port &serial)
{
    const uint8_t START_BYTE = 0x77;
    const uint8_t END_BYTE = 0xAA;
    const size_t PACKET_SIZE = 11;

    uint8_t buf[1024];
    size_t buf_len = 0;

    bool reading_packet = false;
    while (true)
    {
        boost::system::error_code ec;
        size_t len = serial.read_some(boost::asio::buffer(buf + buf_len, sizeof(buf) - buf_len), ec);
        // std::cout << "Read " << n << " bytes from serial port.\n";
        if (ec)
        {
            std::cerr << "Serial error: " << ec.message() << "\n";
            break;
        }
        buf_len += n;

        // try to parse packets out of buffer
        size_t i = 0;
        while(i <=  buf_len){
            if(buf[i] == START_BYTE){
                int8_t packet[];
                reading_packet = true;
                packet.push_back(buf[i]);
                i++;
                continue;
            }
            if(reading_packet){
                if(buf[i] == END_BYTE){
                    reading_packet = false;
                    packet.push_back(buf[i]);
                    handle_packet(packet);

                }else{
                    packet.push_back([buf[i]]);
                }
            }
        }
        // clear buffer
        buf_len = 0;
        
    }
}
void ESP32Interface::handle_packet(uint8_t &packet[]){
    // check second byte with address dictionary, confirm size and update the corresponding variables
}