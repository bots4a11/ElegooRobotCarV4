#ifndef _I2CDEV_H_
#define _I2CDEV_H_
#define I2CDEV_IMPLEMENTATION       I2CDEV_ARDUINO_WIRE
#define I2CDEV_IMPLEMENTATION_WARNINGS
#define I2CDEV_ARDUINO_WIRE         1
#define I2CDEV_BUILTIN_NBWIRE       2
#define I2CDEV_BUILTIN_FASTWIRE     3
#define I2CDEV_I2CMASTER_LIBRARY    4
#ifdef ARDUINO
    #if ARDUINO < 100
        #include "WProgram.h"
    #else
        #include "Arduino.h"
    #endif
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        #include <Wire.h>
    #endif
    #if I2CDEV_IMPLEMENTATION == I2CDEV_I2CMASTER_LIBRARY
        #include <I2C.h>
    #endif
#endif
#define I2CDEV_DEFAULT_READ_TIMEOUT     1000
class I2Cdev {
    public:
        I2Cdev();
        static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout=I2Cdev::readTimeout);
        static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
        static bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
        static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
        static bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
        static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
        static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
        static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
        static bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);
        static uint16_t readTimeout;
};
#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    #define TW_START                0x08
    #define TW_REP_START            0x10
    #define TW_MT_SLA_ACK           0x18
    #define TW_MT_SLA_NACK          0x20
    #define TW_MT_DATA_ACK          0x28
    #define TW_MT_DATA_NACK         0x30
    #define TW_MT_ARB_LOST          0x38
    #define TW_MR_ARB_LOST          0x38
    #define TW_MR_SLA_ACK           0x40
    #define TW_MR_SLA_NACK          0x48
    #define TW_MR_DATA_ACK          0x50
    #define TW_MR_DATA_NACK         0x58
    #define TW_OK                   0
    #define TW_ERROR                1
    class Fastwire {
        private:
            static boolean waitInt();
        public:
            static void setup(int khz, boolean pullup);
            static byte beginTransmission(byte device);
            static byte write(byte value);
            static byte writeBuf(byte device, byte address, byte *data, byte num);
            static byte readBuf(byte device, byte address, byte *data, byte num);
            static void reset();
            static byte stop();
    };
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_NBWIRE
    #define NBWIRE_BUFFER_LENGTH 32
    class TwoWire {
        private:
            static uint8_t rxBuffer[];
            static uint8_t rxBufferIndex;
            static uint8_t rxBufferLength;
            static uint8_t txAddress;
            static uint8_t txBuffer[];
            static uint8_t txBufferIndex;
            static uint8_t txBufferLength;
            static void (*user_onRequest)(void);
            static void (*user_onReceive)(int);
            static void onRequestService(void);
            static void onReceiveService(uint8_t*, int);
        public:
            TwoWire();
            void begin();
            void begin(uint8_t);
            void begin(int);
            void beginTransmission(uint8_t);
            uint8_t endTransmission(uint16_t timeout=0);
            void nbendTransmission(void (*function)(int)) ;
            uint8_t requestFrom(uint8_t, int, uint16_t timeout=0);
            void nbrequestFrom(uint8_t, int, void (*function)(int));
            void send(uint8_t);
            void send(uint8_t*, uint8_t);
            void send(char*);
            uint8_t available(void);
            uint8_t receive(void);
            void onReceive(void (*)(int));
            void onRequest(void (*)(void));
    };
    #define TWI_READY   0
    #define TWI_MRX     1
    #define TWI_MTX     2
    #define TWI_SRX     3
    #define TWI_STX     4
    #define TW_WRITE    0
    #define TW_READ     1
    #define TW_MT_SLA_NACK      0x20
    #define TW_MT_DATA_NACK     0x30
    #define CPU_FREQ            16000000L
    #define TWI_FREQ            100000L
    #define TWI_BUFFER_LENGTH   32
    #define TW_STATUS_MASK              (_BV(TWS7)|_BV(TWS6)|_BV(TWS5)|_BV(TWS4)|_BV(TWS3))
    #define TW_STATUS                   (TWSR & TW_STATUS_MASK)
    #define TW_START                    0x08
    #define TW_REP_START                0x10
    #define TW_MT_SLA_ACK               0x18
    #define TW_MT_SLA_NACK              0x20
    #define TW_MT_DATA_ACK              0x28
    #define TW_MT_DATA_NACK             0x30
    #define TW_MT_ARB_LOST              0x38
    #define TW_MR_ARB_LOST              0x38
    #define TW_MR_SLA_ACK               0x40
    #define TW_MR_SLA_NACK              0x48
    #define TW_MR_DATA_ACK              0x50
    #define TW_MR_DATA_NACK             0x58
    #define TW_ST_SLA_ACK               0xA8
    #define TW_ST_ARB_LOST_SLA_ACK      0xB0
    #define TW_ST_DATA_ACK              0xB8
    #define TW_ST_DATA_NACK             0xC0
    #define TW_ST_LAST_DATA             0xC8
    #define TW_SR_SLA_ACK               0x60
    #define TW_SR_ARB_LOST_SLA_ACK      0x68
    #define TW_SR_GCALL_ACK             0x70
    #define TW_SR_ARB_LOST_GCALL_ACK    0x78
    #define TW_SR_DATA_ACK              0x80
    #define TW_SR_DATA_NACK             0x88
    #define TW_SR_GCALL_DATA_ACK        0x90
    #define TW_SR_GCALL_DATA_NACK       0x98
    #define TW_SR_STOP                  0xA0
    #define TW_NO_INFO                  0xF8
    #define TW_BUS_ERROR                0x00
    #ifndef sbi
        #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
    #endif
    #ifndef cbi
        #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
    #endif
    extern TwoWire Wire;
#endif
#endif
