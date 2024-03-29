#include <Melopero_SAM_M8Q.h>
#include <Wire.h>

Melopero_SAM_M8Q gps;

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
  Serial.print("Setting comunication to ubx only: ");
  Status stat = gps.setCommunicationToUbxOnly();
  Serial.print(gps.getStatusDescription(stat));
  //Check if there was an error
  if (stat != Status::NoError){
    Serial.println(" Something went wrong... (check connections and restart script)");
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
  Serial.print(" acknowledged : ");
  Serial.println(ack);

  // set the PVT to be sent 10 times a second:
  // setMeasurementFrequency(measurementPeriod, measurementspersolution)
  // with these settings we will have a measurement every 50 milliseconds
  // and a navigation solution every 2 measurements which means every 100 
  // milliseconds. 
  stat = gps.setMeasurementFrequency(50, 2);
  Serial.print("Set measurement and navigation solution frequency: ");
  Serial.print(gps.getStatusDescription(stat));
  ack = gps.waitForAcknowledge(CFG_CLASS, CFG_RATE);
  Serial.print(" Acknowledged: ");
  Serial.println(ack);


  // set the pvt message send rate:
  // in this way the message will be sent every navigation solution
  stat = gps.setMessageSendRate(NAV_CLASS, NAV_PVT, 1);
  Serial.print("Set message send rate: ");
  Serial.print(gps.getStatusDescription(stat));
  ack = gps.waitForAcknowledge(CFG_CLASS, CFG_MSG);
  Serial.print(" Acknowledged: ");
  Serial.println(ack);
}

void loop() {
    //Update the PVT data contained in gps.pvtData
    //with gps.updatePVT(polling , timeoutMillis)
    //by default polling is false and timeoutMillis = 1000
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
