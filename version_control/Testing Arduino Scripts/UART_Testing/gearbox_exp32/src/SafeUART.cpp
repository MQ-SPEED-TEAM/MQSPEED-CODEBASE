#include "SafeUART.h"

// Constructor
SafeUART::SafeUART(Stream& s) : uart(s) {}


/* Calculate CRC8-ATM with polynomial 0x7 */
uint8_t SafeUART::calcCRC(uint8_t* data, size_t dataLength) 
{
    uint8_t crc_table[256] = {0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
        0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
        0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
        0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
        0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
        0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
        0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
        0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
        0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
        0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
        0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
        0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
        0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
        0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
        0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
        0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
        0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
        0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
        0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
        0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
        0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
        0xfa, 0xfd, 0xf4, 0xf3};
    uint8_t crc = 0x00;
    for(uint16_t i = 0; i < dataLength; i++)
    {
        crc = (crc_table[crc ^ data[i]]);
    }
    return crc;
}

/** sendData
 * @brief
 * 
 * @param sendBuffer: Buffer with the data that needs to be sent.
 * @param sendBufferLen: Size of buffer in bytes.
 * @return number of bytes sent
 */ 
int16_t SafeUART::sendData(uint8_t * sendBuffer, size_t sendBufferLen)
{
    uint8_t outputData[2];            // Buffer used to send CRC + terminator
    if(sendBufferLen > maxBufferLen)
    {
        return -1;
    }

    // Calculate CRC of the sendBuffer data
    outputData[0] = calcCRC(sendBuffer, sendBufferLen);
    outputData[1] = '\n';
    size_t bytesSent = uart.write(sendBuffer, sendBufferLen);
    bytesSent += uart.write(outputData, 2); // Also send CRC + terminator

    return bytesSent;
}

/** receiveData
 * @brief Receives bytes over the given serial interface and performs a CRC8-ATM check.
 * Data is read until the termination character '\n' is received.
 * Data has the form: [x data bytes][1 byte CRC][\n]
 * CRC of received data is checked. If the received CRC matches with the calculated one
 * an acknowledge response ([ACK][CRC][\n]) is sent back. If the CRC match fails an 
 * not-acknowledge response ([NAK][CRC][\n]) is sent.
 * @param receiveBuffer Buffer that will be filled with received bytes. Needs to be 
 *                       sufficiently big.
 * @param receiveBufferLen Size of buffer in bytes
 * @return  0: no data received
 *          >0: number of received data and acknowledge successfully sent
 *          -1: receiveBufferLen bigger than maxBufferLen (128)
 *              received data too big (receivedBytes bigger than receiveBufferLen)
 *              CRC check failed (data corrupted)

**/
int16_t SafeUART::receiveData(uint8_t* receiveBuffer, size_t receiveBufferLen)
{
    uint8_t outputData[8];
    if(receiveBufferLen > maxBufferLen)
    {
        return -1;
    }

    int availableBytes = uart.available();
    if(availableBytes > receiveBufferLen)
    {
        // Receive buffer smaller than received bytes, send NAK
        outputData[0] = nakByte;
        outputData[1] = calcCRC(outputData, 1);
        outputData[2] = '\n';
        Serial.write(outputData, 3);
        return -1;
    }
    else if(availableBytes > 0 && availableBytes <= receiveBufferLen)
    {
        // Read bytes into buffer until '\n' is read, returns the number of bytes read
        size_t bytesRead = uart.readBytesUntil('\n', receiveBuffer, receiveBufferLen);
        // Calculate CRC of received bytes without the last byte which is the received CRC itself
        uint8_t inputCRC = calcCRC(receiveBuffer, bytesRead - 1);

        if(inputCRC != receiveBuffer[bytesRead - 1])
        {
            // CRC check failed, send NAK
            outputData[0] = nakByte;
            outputData[1] = calcCRC(outputData, 1);
            outputData[2] = '\n';
            uart.write(outputData, 3);
            return -1;
        }
        else
        {
            // CRC check successful, send ACK
            outputData[0] = ackByte;
            outputData[1] = calcCRC(outputData, 1);
            outputData[2] = '\n';
            uart.write(outputData, 3);
            return bytesRead-1; // Return number of received bytes (without CRC byte)
        }
    }
    return 0;
}