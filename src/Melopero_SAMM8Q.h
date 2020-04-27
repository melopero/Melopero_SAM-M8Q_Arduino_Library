//author: Leonardo La Rocca
#ifndef Melopero_SAMM8Q_H_INCLUDED
#define Melopero_SAMM8Q_H_INCLUDED

#include "MP_UBX.h"

#define DEFAULT_I2C_ADDRESS 0x42

#define DATA_STREAM_REGISTER 0xFF

#define AVAILABLE_BYTES_MSB 0xFD
#define AVAILABLE_BYTES_LSB 0xFE

enum class Status : int8_t {
  NoError = 0,
  ArgumentError = -2,
  ErrorSending = -3,
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

class Melopero_SAMM8Q {

  public :
    UbxMessage ubxmsg;
    PVTData pvtData;
    uint8_t  i2cAddress;

  public :
    Melopero_SAMM8Q(uint8_t i2cAddress = DEFAULT_I2C_ADDRESS);

    uint16_t getAvailableBytes();
    Status writeUbxMessage(UbxMessage &msg);
    Status readUbxMessage(UbxMessage &msg);

    Status waitForUbxMessage(UbxMessage &msg, uint32_t timeoutMillis = 1000, uint32_t intervalMillis = 50);
    bool waitForAcknowledge(uint8_t msgClass, uint8_t msgId);

    Status setCommunicationToUbxOnly();
    Status setMessageSendRate(uint8_t msgClass, uint8_t msgId, uint8_t sendRate = 0x01);
    Status setMeasurementFrequency(uint16_t measurementPeriodMillis = 1000, uint8_t navigationRate = 1, TimeRef timeref = TimeRef::UTC);

    Status updatePVT(bool polling = true, uint16_t timeOutMillis = 1000);

    String getStatusDescription(Status status);

    private :
      uint32_t extractU4FromUbxMessage(UbxMessage &msg, uint16_t startIndex);
      uint16_t extractU2FromUbxMessage(UbxMessage &msg, uint16_t startIndex);

};

#endif // Melopero_SAMM8Q_H_INCLUDED
