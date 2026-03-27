#ifndef SAFEUART_H
#define SAFEUART_H

#include <Arduino.h>

class SafeUART 
{
    private:
    // Private variables
    Stream& uart;
    const uint8_t ackByte = 0x06;   // Acknowledge byte ASCII ACK
    const uint8_t nakByte = 0x15;   // Not-Acknowledge byte ASCII NAK
    const size_t maxBufferLen = 128;    // Max. data buffer length
    unsigned long ack_timeout = 100;    // Time to wait for an acknowledge from the other party

    // Private methods
    uint8_t calcCRC(uint8_t* data, size_t dataLength);

    public:
    // Constructor
    SafeUART(Stream& s);
    // Public methods
    int16_t sendData(uint8_t* sendBuffer, size_t sendBufferLen);
    int16_t receiveData(uint8_t* receiveBuffer, size_t receiveBufferLen);
    int16_t sendSafeData(uint8_t* sendBuffer, size_t sendBufferLen);
};

#endif