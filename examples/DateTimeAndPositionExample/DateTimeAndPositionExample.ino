#include <Melopero_SAM_M8Q.h>
#include <Wire.h>

Melopero_SAM_M8Q gps;
int lastUpdate;

void setup() {
  Serial.begin(9600);

  // init the i2c communication channel
  // by default it uses the DEFAULT_I2C_ADDRESS and I2C-0
  // You can specify a different address and/or bus:
  // gps.initI2C(address, bus);
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
