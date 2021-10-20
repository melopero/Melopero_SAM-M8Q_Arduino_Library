#include <Melopero_SAM_M8Q.h>
#include <Wire.h>

Melopero_SAM_M8Q gps;
UbxMessage msg; // ubx message used to read and write ubx messages (we could also use gps.msg...)

int lastUpdate;

void setup() {
  Serial.begin(9600);
  while(!Serial);

  // init the i2c communication channel
  // by default it uses the SAM_M8Q_DEFAULT_I2C_ADDRESS and Wire (I2C-0)
  // You can specify a different address and/or bus:
  // gps.initI2C(address, bus);
  // For example for Wire1 it would be:
  // Wire1.begin();
  // gps.initI2C(SAM_M8Q_DEFAULT_I2C_ADDRESS, Wire1);
  Wire.begin();
  gps.initI2C();

  //First set up the gps to use only UBX messages
  //Many functions in the library return a Status
  //You can get a description of the status with:
  //gps.getStatusDescription(status);
  //(This function returns a String)
  Serial.println("Setting comunication to ubx only...");
  Status stat = gps.setCommunicationToUbxOnly();
  Serial.println(gps.getStatusDescription(stat));
  //Check if there was an error
  if (stat != Status::NoError){
    Serial.println("Something went wrong... (check connections and restart script)");
    while (true);
  }

  //After a configuration message is sent the device will
  //send an acknowledge or a not acknowledged message.
  //After having sent a configuration message you can
  //wait for the acknowledge with:
  //gps.waitForAcknowledge(MSG_CLASS, MSG_ID);
  //Where MSG_CLASS and MSG_ID are the class and id of the
  //message you are waiting an acknowledge for.
  bool ack = gps.waitForAcknowledge(CFG_CLASS, CFG_PRT);
  Serial.print("acknowledged : ");
  Serial.println(ack);

  // Let's configure the gps device as we need it!
  // For example we want to put the gps on an air balloon
  // that reaches altitudes higher than 12000m (altitude limit
  // for default configuration).
  // We want to change the dynamic platform model 
  // from the default portable model to the airborne<1g
  // model. This model allows a maximum altitude of 50000m.
  // If you are wondering, all this information and all available 
  // messages come from the ubx protocol documentation:
  // https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
  // 
  // Ubx messages can be written(set) and read(get/poll) to and from the device.
  // In this case we want to set some configuration and then poll
  // the configuration message to see if the device is setup correctly.
  
  // 1) set desired configuration
  //    to set the dynamic platform model we need to use the message 
  //    with class=CFG and id=0x24, the length of the payload is 36 bytes
  msg.msgClass = CFG_CLASS;
  msg.msgId = 0x24;
  msg.length = 36;
  //    set payload
  //    note: all numbers are in little endian format!
  //    payload[0:2] =  2 bytes mask that tells which parameters to set
  //                    we only want the dynamic platform model: 0x0001
  msg.payload[0] = 0x01;
  msg.payload[1] = 0x00;
  //    payload[2:3] =  1 byte specifying which dynamic platform model 
  //                    to use: 0x06 for airborne<1g 
  msg.payload[2] = 0x06;
  
  // Now we can send the messgae:
  stat = gps.writeUbxMessage(msg);
  Serial.println("Change dynamic model: " + gps.getStatusDescription(stat));
  ack = gps.waitForAcknowledge(CFG_CLASS, 0x24);
  Serial.print("acknowledged : ");
  Serial.println(ack);

  // 2) read the device configuration to see if all settings are correct.
  //    To poll a message we only set class and id and then we call pollUbxMessage.
  //    If there are no errors the message will be populated.
  msg.msgClass = CFG_CLASS;
  msg.msgId = 0x24; 
  resetPayload(msg);
  stat = gps.pollUbxMessage(msg);
  Serial.println(gps.getStatusDescription(stat));

  uint8_t dynamic_platform_model = msg.payload[2];
  if (dynamic_platform_model == 0x06)
    Serial.println("Dynamic platform model changed succesfully.");
  else 
    Serial.println("Error while changing dynamic platform model.");

  lastUpdate = millis();
}

void loop() {
  int curTime = millis();
  if (curTime - lastUpdate > 1000){
    lastUpdate = millis();

    //Update the PVT data contained in gps.pvtData
    //with gps.updatePVT(polling , timeoutMillis)
    //by default polling is true and timeoutMillis = 1000
    Status stat = gps.updatePVT();
    if (stat == Status::NoError){
      //Print out the data
      printDate();
      Serial.print("Gnss fix : ");
      Serial.println(getGNSSFixType(gps.pvtData.fixType));
      if (gps.pvtData.fixType != NO_FIX){
        printCoordinates();
        printHeight();
      }
    }
    else {
      Serial.println(gps.getStatusDescription(stat));
    }
  }

}

void printDate(){
  Serial.print("[");
  Serial.print(gps.pvtData.year);
  Serial.print(" - ");
  Serial.print(gps.pvtData.month);
  Serial.print(" - ");
  Serial.print(gps.pvtData.day);
  Serial.print("] ");

  Serial.print(gps.pvtData.hour);
    Serial.print(" : ");
  Serial.print(gps.pvtData.min);
    Serial.print(" : ");
  Serial.println(gps.pvtData.sec);
}

void printCoordinates(){
  Serial.print("Longitude : ");
  Serial.print(gps.pvtData.longitude);
  Serial.print(" Latitude : ");
  Serial.print(gps.pvtData.latitude);
  Serial.println(" Scale: 1e-7  Unit: degrees");
}

void printHeight(){
  Serial.print("Heigth: ");
  Serial.print(gps.pvtData.height);
  Serial.print(" Height MSL: ");
  Serial.print(gps.pvtData.hMSL);
  Serial.println(" Unit: millimeters");
}
