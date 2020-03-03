//author: Leonardo La Rocca
#ifndef MP_SAM_M8Q_H_INCLUDED
#define MP_SAM_M8Q_H_INCLUDED

#define DEFAULT_I2C_ADDRESS = 0x42;

#define DATA_STREAM_REGISTER = 0xFF;

#define AVAILABLE_BYTES_MSB 0xFD
#define AVAILABLE_BYTES_LSB 0xFE

enum class Status : int8_t {
  NoError = 0,
  ArgumentError = -3,
  ErrorReceiving = -4,
  OperationTimeOut = -5
};

enum class TimeRef : uint8_t {
  UTC = 0,
  GPS = 1,
  GLONASS = 2, //(not supported in protocol versions less than 18)
  BeiDou = 3,  //(not supported in protocol versions less than 18)
  Galileo = 4  //(not supported in protocol versions less than 18)
};

class SAM_M8Q {

  public :
    UbxMessage ubxmsg;
    PVTData pvtData;

  public :
    SAM_M8Q(uint8_t i2cAddress = DEFAULT_I2C_ADDRESS);

    int16_t getAvailableBytes();
    int8_t writeUbxMessage(UbxMessage &msg);
    int8_t readUbxMessage(UbxMessage &msg);

    int8_t waitForUbxMessage(UbxMessage &msg, uint timeoutMillis = 1000, uint intervalMillis = 50);
    bool waitForAcknowledge(uint8_t msgClass, uint8_t msgId);

    int8_t setCommunicationToUbxOnly();
    int8_t setMessageSendRate(uint8_t msgClass, uint8_t msgId, uint8_t sendRate = 0x01);
    int8_t setMeasurementFrequency(uint16_t measurementPeriodMs = 1000, uint16_t navigation_rate = 1, uint16_t timeref = 0);

    int8_t updatePVT(bool polling = true, uint16_t timeOutMillis = 1000);

    private :
    uint32_t extractU4FromUbxMessage(UbxMessage &msg, uint startIndex);
    uint16_t extractU2FromUbxMessage(UbxMessage &msg, uint startIndex);

};

#endif // MP_SAM_M8Q_H_INCLUDED
