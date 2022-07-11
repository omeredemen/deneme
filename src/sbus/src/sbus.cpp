#include "sbus.h"

#include "rclcpp/rclcpp.hpp" 
#include "sensor_msgs/msg/joy.hpp"  

namespace SBUS{
    bool SBusSerialPort::connectSerialPort() {
        // Open serial port
        // O_RDWR - Read and write
        // O_NOCTTY - Ignore special chars like CTRL-C
        serial_port_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  
        if (serial_port_fd_ == -1) {
            close(serial_port_fd_);
            printf("Serial Port Couldn't Open");
            return false;
        }
        
        if (!configureSerialPortForSBus()) {
            close(serial_port_fd_);
            printf("Serial Port Couldn't Configure");
            return false;
        } 
        return true;
    }
    
    void SBusSerialPort::disconnectSerialPort() {
        close(serial_port_fd_);
    }


    bool SBusSerialPort::configureSerialPortForSBus() const{
        // clear config
        fcntl(serial_port_fd_, F_SETFL, 0);
        // read non blocking
        fcntl(serial_port_fd_, F_SETFL, FNDELAY);

        struct termios2 uart_config;
        /* Fill the struct for the new configuration */
        ioctl(serial_port_fd_, TCGETS2, &uart_config);

        // Output flags - Turn off output processing
        // no CR to NL translation, no NL to CR-NL translation,
        // no NL to CR translation, no column 0 CR suppression,
        // no Ctrl-D suppression, no fill characters, no case mapping,
        // no local output processing
        //
        uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

        // Input flags - Turn off input processing
        // convert break to null byte, no CR to NL translation,
        // no NL to CR translation, don't mark parity errors or breaks
        // no input parity check, don't strip high bit off,
        // no XON/XOFF software flow control
        //
        uart_config.c_iflag &=
            ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

        //
        // No line processing:
        // echo off
        // echo newline off
        // canonical mode off,
        // extended input processing off
        // signal chars off
        //
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

        // Turn off character processing
        // Turn off odd parity
        uart_config.c_cflag &= ~(CSIZE | PARODD | CBAUD);

        // Enable parity generation on output and parity checking for input.
        uart_config.c_cflag |= PARENB;
        // Set two stop bits, rather than one.
        uart_config.c_cflag |= CSTOPB;
        // No output processing, force 8 bit input
        uart_config.c_cflag |= CS8;
        // Enable a non standard baud rate
        uart_config.c_cflag |= BOTHER;

        // Set custom baud rate of 100'000 bits/s necessary for sbus
        const speed_t spd = 100000;
        uart_config.c_ispeed = spd;
        uart_config.c_ospeed = spd;

        if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0) {
            printf("could not set configuration of serial port");
            return false;
        }

        return true;
    }


    void SBusSerialPort::transmitSerialSBusMessage() const {
        uint8_t buffer[kSbusFrameLength_];

        // SBUS header
        buffer[0] = kSbusHeaderByte_;

        // 16 channels of 11 bit data
        buffer[1] = (uint8_t)((channels[0] & 0x07FF));
        buffer[2] = (uint8_t)((channels[0] & 0x07FF) >> 8 |
                        (channels[1] & 0x07FF) << 3);
        buffer[3] = (uint8_t)((channels[1] & 0x07FF) >> 5 |
                        (channels[2] & 0x07FF) << 6);
        buffer[4] = (uint8_t)((channels[2] & 0x07FF) >> 2);
        buffer[5] = (uint8_t)((channels[2] & 0x07FF) >> 10 |
                        (channels[3] & 0x07FF) << 1);
        buffer[6] = (uint8_t)((channels[3] & 0x07FF) >> 7 |
                        (channels[4] & 0x07FF) << 4);
        buffer[7] = (uint8_t)((channels[4] & 0x07FF) >> 4 |
                        (channels[5] & 0x07FF) << 7);
        buffer[8] = (uint8_t)((channels[5] & 0x07FF) >> 1);
        buffer[9] = (uint8_t)((channels[5] & 0x07FF) >> 9 |
                        (channels[6] & 0x07FF) << 2);
        buffer[10] = (uint8_t)((channels[6] & 0x07FF) >> 6 |
                         (channels[7] & 0x07FF) << 5);
        buffer[11] = (uint8_t)((channels[7] & 0x07FF) >> 3);
        buffer[12] = (uint8_t)((channels[8] & 0x07FF));
        buffer[13] = (uint8_t)((channels[8] & 0x07FF) >> 8 |
                         (channels[9] & 0x07FF) << 3);
        buffer[14] = (uint8_t)((channels[9] & 0x07FF) >> 5 |
                         (channels[10] & 0x07FF) << 6);
        buffer[15] = (uint8_t)((channels[10] & 0x07FF) >> 2);
        buffer[16] = (uint8_t)((channels[10] & 0x07FF) >> 10 |
                         (channels[11] & 0x07FF) << 1);
        buffer[17] = (uint8_t)((channels[11] & 0x07FF) >> 7 |
                         (channels[12] & 0x07FF) << 4);
        buffer[18] = (uint8_t)((channels[12] & 0x07FF) >> 4 |
                         (channels[13] & 0x07FF) << 7);
        buffer[19] = (uint8_t)((channels[13] & 0x07FF) >> 1);
        buffer[20] = (uint8_t)((channels[13] & 0x07FF) >> 9 |
                         (channels[14] & 0x07FF) << 2);
        buffer[21] = (uint8_t)((channels[14] & 0x07FF) >> 6 |
                         (channels[15] & 0x07FF) << 5);
        buffer[22] = (uint8_t)((channels[15] & 0x07FF) >> 3);

        // SBUS flags
        // (bit0 = least significant bit)
        // bit0 = ch17 = digital channel (0x01)
        // bit1 = ch18 = digital channel (0x02)
        // bit2 = Frame lost, equivalent red LED on receiver (0x04)
        // bit3 = Failsafe activated (0x08)
        // bit4 = n/a
        // bit5 = n/a
        // bit6 = n/a
        // bit7 = n/a
        buffer[23] = 0x08;
        /*if (digital_channel_1) {
            buffer[23] |= 0x01;
        }
        if (digital_channel_2) {
            buffer[23] |= 0x02;
        }
        if (frame_lost) {
            buffer[23] |= 0x04;
        }
        if (failsafe) {
            buffer[23] |= 0x08;
        }*/

        // SBUS footer
        buffer[24] = kSbusFooterByte_;

        const int written = write(serial_port_fd_, buffer, kSbusFrameLength_);
        // tcflush(serial_port_fd_, TCOFLUSH); // There were rumors that this might
        // not work on Odroids...
        if (written != kSbusFrameLength_) {
            printf(" Wrote %d bytes but should have written %d \n", written, kSbusFrameLength_);
        }
        else{
            printf("Wrote %d bytes \n", written);
        }
    }
}

SBUS::SBusSerialPort sbus;

class sbus_node: public rclcpp::Node 
{
public:
    sbus_node() : Node("sbus_node") 
    {
        pub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
        std::bind(&sbus_node::callBack,this,std::placeholders::_1));
    }
     
private:
    void callBack(const sensor_msgs::msg::Joy::SharedPtr joy_msg){
        for(int i = 0; i<SIZE; i++){
            if(i<8){    
                new_x[i] = joy_msg->axes[i]; //axes
            }
            else{
                new_x[i] = joy_msg->buttons[i-8]; //buttons
            }
        }
        
        //Linear Interpolation  y = y0 + ((y1-y0)/(x1-x0)) * (x - x0);
        for(int j = 0; j<SIZE; j++){
            if(j<8){
                new_x[j] = min_sbus + ((max_sbus-min_sbus)/(max_axes-min_axes)) * (x[j]-min_axes);
            }
            else{
                new_x[j] = min_sbus +((max_sbus-min_sbus)/(max_button-min_button)) * (x[j]-min_button);
            }
        }

        for(int i = 0; i< SIZE; i++){
            if(new_x[i] == x[i]){
                equal = true;
            }
            else{
                break;
            }
        }
        
        for(int i = 0; i<SIZE; i++){
            if(equal){
                sbus.channels[i] = new_x[i];
            }
            else{
                sbus.channels[i] = x[i];
            }
        }
        
        sbus.transmitSerialSBusMessage();
        
        for(int i =0; i<16; i++){
            RCLCPP_INFO(this->get_logger()," sbus_.channels[%d] = %d", i, sbus.channels[i]);
            x[i] = new_x[i];
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr pub_;

    float x[16];
    float new_x[16]; 
    bool equal;
    

    //interpolasyon
    bool min_button = 0;
    bool max_button = 1;        

    int min_axes = -1;
    int max_axes =  1;

    int min_sbus = 1000;
    int max_sbus = 2000;

};
     
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sbus_node>());
    rclcpp::shutdown();
    return 0;
}