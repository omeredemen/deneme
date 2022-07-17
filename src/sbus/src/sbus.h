#ifndef SBUS_H_
#define SBUS_H_

#include <atomic>
#include <thread>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <deque>
#include <unistd.h> 
#include <chrono>

#define SIZE 16
namespace SBUS{

class SBusSerialPort {
public:
   bool connectSerialPort();
   int* transmitSerialSBusMessage(int channels[16]) const;
   int channels[16];

private:
   static constexpr int kSbusFrameLength_ = 25;
   static constexpr uint8_t kSbusHeaderByte_ = 0x0F;
   static constexpr uint8_t kSbusFooterByte_ = 0x00;
   static constexpr int kPollTimeoutMilliSeconds_ = 500;

   // Digital channels (ch17 and ch18)
   bool digital_channel_1;
   bool digital_channel_2;

   // Flags
   bool frame_lost;
   bool failsafe;
    
   int serial_port_fd_;

   void disconnectSerialPort();
   bool configureSerialPortForSBus() const;
   
};
}

#endif