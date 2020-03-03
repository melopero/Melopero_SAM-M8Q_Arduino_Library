//Author: Leonardo La Rocca
#include "SAM_M8Q.h"
#include "MP_UBX.h"

SAM_M8Q::SAM_M8Q(uint8_t i2cAddress){
  this->i2cAddress = i2cAddress;
  Wire.begin(); //Prepare I2C communication
}

/* returns the number of bytes available to read*/
uint16_t SAM_M8Q::getAvailableBytes(){

  Wire.beginTransmission(this->i2cAddress);
  Wire.write(AVAILABLE_BYTES_MSB);
  Wire.endTransmission();

  Wire.requestFrom(this->i2cAddress, 2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t) msb << 8 | lsb);
}

/* Reads a UBX message and populates the given UbxMessage*/
int8_t SAM_M8Q::readUbxMessage(UbxMessage &msg){
  uint16_t bytes = this->getAvailableBytes();

  if (bytes > MAX_MESSAGE_LENGTH || bytes <= 0)
    return Status::ErrorReceiving;

  Wire.requestFrom(this->i2cAddress, bytes);
  uint8_t syncChA = Wire.read(); // sync char a
  uint8_t syncChB = Wire.read(); // sync char b

  if (!(syncChA == SYNC_CHAR_A && syncChB == SYNC_CHAR_B))
    return Status::ErrorReceiving;

  msg.msgClass = Wire.read();
  msg.msgId = Wire.read();
  msg.length = bytes - 8;
  for (int i = 0; i < msg.length; i++)
    msg.payload[i] = Wire.read();

  msg.checksumA = Wire.read();
  msg.checkSumB = Wire.read();

  return Status::NoError;
}

int8_t SAM_M8Q::writeUbxMessage(UbxMessage &msg){
  computeChecksum(msg);
  Wire.beginTransmission(this->i2cAddress);

  Wire.write(SYNC_CHAR_1);
  Wire.write(SYNC_CHAR_2);
  Wire.write(msg.msgClass);
  Wire.write(msg.msgId);
  Wire.write((uint8_t) (msg.length >> 8)); // length msb
  Wire.write((uint8_t) (msg.length %(1 << 8))); // length lsb
  for (int i = 0; i < msg.length; i++)
    Wire.write(msg.payload[i]);

  Wire.write(msg.checksumA);
  Wire.write(msg.checksumB);

  Wire.endTransmission();

  return Status::NoError;
}

/* waits for a given message type (class and id).
  timeoutSeconds: the maximum amount of time to wait for the message to arrive in milliseconds.
  intervalSeconds: the interval in milliseconds between two readings.
*/
int8_t SAM_M8Q::waitForUbxMessage(UbxMessage &msg, uint timeoutMillis, uint intervalMillis){
  int startTime = millis();
  uint8_t desiredClass = msg.msgClass;
  uint8_t desiredId = msg.msgId;

  while (millis() - startTime < timeoutMillis){
      Status status = this->readUbxMessage(msg);
      if (status != Status::ErrorReceiving) {
        if (msg.msgClass == desiredClass && msg.msgId == desiredId)
          return Status::NoError;
      }

      delay(intervalMillis);
  }
  return Status::OperationTimeOut;
}

/* An acknowledge message (or a Not Acknowledge message) is sent everytime
a configuration message is sent.*/
bool SAM_M8Q::waitForAcknowledge(uint8_t msgClass, uint8_t msgId){
  this->ubxmsg.msg
  Status status = this->waitForUbxMessage(this->ubxmsg, 1000, 50);
  //see if a message is received
  if (status == Status::OperationTimeOut)
    return false;

  //see if the received message is an acknowledge message
  if (this->ubxmsg.msgClass == ACK_CLASS && this->ubxmsg.msgId == ACK_ACK)
    if (this->ubxmsg.length >= 2)
      if (this->ubxmsg.payload[0] == msgClass && this->ubxmsg.payload[1] == msgId)
        return true;

  return false;
}

/*Sets the communication protocol to UBX (only) both for input and output*/
int8_t SAM_M8Q::setCommunicationToUbxOnly(){
  this->ubxmsg = { CFG_CLASS, CFG_PRT, 20, { 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 }, 0, 0 };
  return this->writeUbxMessage(this->ubxmsg);
}

/* Send rate is relative to the event a message is registered on.
For example, if the rate of a navigation message is set to 2,
the message is sent every second navigation solution */
int8_t SAM_M8Q::setMessageSendRate(uint8_t msgClass, uint8_t msgId, uint8_t sendRate = 0x01){
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_MSG;
  this->ubxmsg.length = 8;
  this->ubxmsg.payload = {msgClass, msgId, sendRate, 0x00, 0x00, 0x00, 0x00, 0x00};
  this->writeUbxMessage(this->ubxmsg);
}

/*measurementPeriodMillis:
    elapsed time between GNSS measurements, which defines the rate,
    e.g. 100ms => 10Hz, Measurement rate should be greater than or
    equal to 25 ms.\n
navigationRate :
    The ratio between the number of measurements and the number of
    navigation solutions, e.g. 5 means five measurements for
    every navigation solution. Maximum value is 127. \n
timeref :
    The time system to which measurements are aligned:
    UTC | GPS | GLONASS | BeiDou | Galileo */
int8_t SAM_M8Q::setMeasurementFrequency(uint16_t measurementPeriodMillis, uint8_t navigationRate, TimeRef timeref){
  this->ubxmsg.msgClass = CFG_CLASS;
  this->ubxmsg.msgId = CFG_RATE;
  this->ubxmsg.length = 6;
  this->ubxmsg.payload = {(measurementPeriodMillis >> 8 ), measurementPeriodMillis % (1 << 8),
    0x00, navigationRate, 0x00, timeref};
  this->writeUbxMessage(this->ubxmsg);
}

/*Updates the pvt data contained in the struct pvtData.
polling : if true the pvt message is polled, else waits for the next navigation solution
timeOutMillis : the maximum time to wait for the message
To reduce the time between pvt messages the frequency of the message can be
increased with setMessageSendRate and setMeasurementFrequency. */
int8_t SAM_M8Q::updatePVT(bool polling, uint16_t timeOutMillis){
  if (polling){ //send message without payload
    this->ubxmsg.msgClass = NAV_CLASS;
    this->ubxmsg.msgId = NAV_PVT;
    this->length = 0;
    this->writeUbxMessage(this->ubxmsg); //TODO : check status
  }

  //read response / wait for the next pvt message
  this->length = 92;
  Status status = this->waitForUbxMessage(this->ubxmsg, timeOutMillis); //TODO: check status
  if (status == Status::NoError){
    this->pvtData.itow = this->extractU4FromUbxMessage(this->ubxmsg, 0);
    this->pvtData.year = this->extractU2FromUbxMessage(this->ubxmsg, 4);
    this->pvtData.month = this->ubxmsg.payload[6];
    this->pvtData.day = this->ubxmsg.payload[7];
    this->pvtData.hour = this->ubxmsg.payload[8];
    this->pvtData.min = this->ubxmsg.payload[9];
    this->pvtData.sec = this->ubxmsg.payload[10];
    this->pvtData.validTimeFlag = this->ubxmsg.payload[11];
    
  }
}

uint32_t SAM_M8Q::extractU4FromUbxMessage(UbxMessage &msg, uint startIndex){
  if (startIndex + 3 >= msg.length)
    return Status::ArgumentError;

  uint32_t value = this->extractU2FromUbxMessage(msg, startIndex) << 16;
  value |= this->extractU2FromUbxMessage(msg, startIndex + 2);
  return value;
}

uint16_t SAM_M8Q::extractU2FromUbxMessage(UbxMessage &msg, uint startIndex){
  if (startIndex + 1 >= msg.length)
    return Status::ArgumentError;

  return (uint16_t) (msg.payload[startIndex] << 8) | msg.payload[startIndex + 1];
}

    def get_pvt(self, polling = True, time_out_s = 1):
        """updates and returns the pvt_data dictionary that contains the last received
        pvt data.\n
        polling :
            if true the pvt message is polled, else waits for the next navigation solution
        time_out_s :
            the maximum time to wait for the message

        To reduce the time between pvt messages the frequency of the message can be
        increased with set_message_frequency and set_measurement_freq
        """
        if polling:
            #send polling message
            message = ubx.compose_message(ubx.NAV_CLASS, ubx.NAV_PVT)
            self.write_message(message)

        #reads response
        read = self.wait_for_message(time_out_s = time_out_s, msg_cls = ubx.NAV_CLASS, msg_id = ubx.NAV_PVT)
        if read is not None:
            start_payload = 6
            #WARNING: POSITION_DOP AND ITOW ARE MISSING (NOT RETRIEVED)

            #Time solution
            year = ubx.u2_to_int(read[start_payload+4:start_payload+6])
            month = read[start_payload+6]
            day = read[start_payload+7]
            hour = read[start_payload+8]
            minutes = read[start_payload+9]
            sec = read[start_payload+10]
            valid_flag = read[start_payload+11]

            #clarifying flags
            valid_date = 0x01 & valid_flag == 0x01
            valid_time = 0x02 & valid_flag == 0x02
            fully_resolved = 0x04 & valid_flag == 0x04
            valid_mag = 0x08 & valid_flag == 0x08

            #GNSS fix and flags
            gnss_fix = ubx.get_gnss_fix_type(read[start_payload+20])
            fix_status_flags = read[start_payload+21:start_payload+23]
            num_satellites = read[start_payload+23]

            #longitude and latitude are in Degrees
            longitude = ubx.i4_to_int(read[start_payload+24: start_payload+28]) * 1e-07
            latitude = ubx.i4_to_int(read[start_payload+28: start_payload+32]) * 1e-07


            #height, mean sea level height in millimeters
            height = ubx.i4_to_int(read[start_payload+32: start_payload+36])
            height_MSL = ubx.i4_to_int(read[start_payload+36: start_payload+40])

            #horizontal and vertical accuracy estimates in millimeters
            h_acc = ubx.u4_to_int(read[start_payload+40: start_payload+44])
            v_acc = ubx.u4_to_int(read[start_payload+44: start_payload+48])

            #North East Down velocity in mm / s
            n_vel = ubx.i4_to_int(read[start_payload+48: start_payload+52])
            e_vel = ubx.i4_to_int(read[start_payload+52: start_payload+56])
            d_vel = ubx.i4_to_int(read[start_payload+56: start_payload+60])

            #Ground speed in mm / s and heading of motion in degrees + speed and heading accuracy estimates
            g_speed = ubx.i4_to_int(read[start_payload+60: start_payload+64])
            motion_heading = ubx.i4_to_int(read[start_payload+64: start_payload+68]) * 1e-05
            s_acc = ubx.u4_to_int(read[start_payload+68: start_payload+72])
            m_acc = ubx.u4_to_int(read[start_payload+72: start_payload+76]) * 1e-05

            #Heading of vehicle in degrees
            vehicle_heading = ubx.i4_to_int(read[start_payload+84: start_payload+88]) * 1e-05

            #Magnetic declination and magnetic declination accuracy both in degrees
            mag_deg = ubx.i2_to_int(read[start_payload+88: start_payload+90]) * 1e-02
            mag_deg_acc = ubx.u2_to_int(read[start_payload+90: start_payload+92]) * 1e-02

            #time
            self.pvt_data[self.YEAR_TAG] = year
            self.pvt_data[self.MONTH_TAG] = month
            self.pvt_data[self.DAY_TAG] = day
            self.pvt_data[self.HOUR_TAG] = hour
            self.pvt_data[self.MINUTE_TAG] = minutes
            self.pvt_data[self.SECOND_TAG] = sec

            #flags
            self.pvt_data[self.VALID_TIME_TAG] = valid_time
            self.pvt_data[self.VALID_DATE_TAG] = valid_date
            self.pvt_data[self.FULLY_RESOLVED_TAG] = fully_resolved
            self.pvt_data[self.VALID_MAG_DEC_TAG] = valid_mag

            #GNSS
            self.pvt_data[self.GNSS_FIX_TAG] = gnss_fix
            self.pvt_data[self.FIX_STATUS_FLAGS_TAG] = fix_status_flags
            self.pvt_data[self.NUM_SATELLITES_TAG] = num_satellites

            #Coordinates
            self.pvt_data[self.LONGITUDE_TAG] = longitude
            self.pvt_data[self.LATITUDE_TAG] = latitude
            self.pvt_data[self.ELLIPSOID_HEIGHT_TAG] = height
            self.pvt_data[self.MSL_HEIGHT_TAG] = height_MSL
            self.pvt_data[self.HORIZONTAL_ACCURACY_TAG] = h_acc
            self.pvt_data[self.VERTICAL_ACCURACY_TAG] = v_acc

            #Velocity and heading
            self.pvt_data[self.NED_VELOCITY_TAG] = (n_vel, e_vel, d_vel)
            self.pvt_data[self.GROUND_SPEED_TAG] = g_speed
            self.pvt_data[self.VEHICLE_HEADING_TAG] = vehicle_heading
            self.pvt_data[self.MOTION_HEADING_TAG] = motion_heading
            self.pvt_data[self.SPEED_ACCURACY_TAG] = s_acc
            self.pvt_data[self.HEADING_ACCURACY_TAG] = m_acc

            #Magnetic declination
            self.pvt_data[self.MAGNETIC_DECLINATION_TAG] = mag_deg
            self.pvt_data[self.MAG_DEC_ACCURACY_TAG] = mag_deg_acc

            return self.pvt_data

        return None
