# Melopero_SAM-M8Q_Arduino_Library
A library for interfacing the <b>Melopero SAM-M8Q Multi GNSS breakout board</b> with Arduino.
<br> If you were looking for the Python3 library for the Raspberry Pi click [HERE](https://github.com/melopero/Melopero_SAM-M8Q)

# Melopero SAM-M8Q Multi GNSS breakout board
![melopero logo](images/Melopero-SAM-M8Q-diagonal.jpg?raw=true)


# Pinouts

<table style="width:100%">
  <tr>
    <th>Melopero SAM-M8Q</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>3V3</td>
    <td>Input power pin. Apply 3.3V to this pin</td>
  </tr>
  <tr>
    <td>SCL</td>
    <td>I2C Serial CLock pin</td>
  </tr>
  <tr>
    <td>SDA</td>
    <td>I2C Serial DAta pin</td>
  </tr>
  <tr>
    <td>GND</td>
    <td>Ground pin</td>
  </tr>
  <tr>
    <td>INT</td>
    <td>External Interrupt pin (INPUT)</td>
  </tr>
  <tr>
    <td>SAF</td>
    <td>SAFEBOOT_N pin, for future service, updates and reconfiguration </td>
  </tr>
  <tr>
    <td>RST</td>
    <td>RESET pin, INPUT, Active Low</td>
  </tr>
  <tr>
    <td>PPS</td>
    <td>Pulse Per Second pin, OUTPUT, connected to the Blue LED</td>
  </tr>
  <tr>
    <td>VBA</td>
    <td>V_BACKUP pin, INPUT. This pin accepts a voltage in the 3.3V-6V range. By applying a voltage to this pin, you will automatically disable the coin cell battery and avoid the installation of the optional CR1220 battery holder, while still allowing a warm start of the GNSS module.
      </td>
  </tr> 
</table>

## Getting Started
### Prerequisites
You will need:
- Arduino IDE, you can download it here: [download Arduino IDE](https://www.arduino.cc/en/main/software)
- the Melopero SAM-M8Q Multi GNSS breakout: [buy here](https://www.melopero.com/shop)

### Connect the sensor to Arduino <br>
Use <b>only 3.3V power and logic</b> Arduino boards, such as the <b>Arduino MKR</b>.<br> <b>DO NOT connect</b> this board directly to <b>5V</b>. You'll need a level converter to use it with an Arduino UNO.<br>This sensor communicates over I2C.
<br><b>I2C connections</b>:
<table style="width:100%">
  <tr>
    <th>Melopero SAM-M8Q</th>
    <th>Arduino</th>
  </tr>
  <tr>
    <td>3V3</td>
    <td>VCC</td>
  </tr>
  <tr>
    <td>SCL</td>
    <td>SCL</td>
  </tr>
  <tr>
    <td>SDA</td>
    <td>SDA</td>
  </tr>
  <tr>
    <td>GND</td>
    <td>GND</td>
  </tr>
</table>

<br><b>Alternatively, use the onboard Sparkfun's Qwiic compatible connectors to make a quick I2C connection without soldering</b>



### Install the library
This library can be installed directly from the Library manager of the Arduino IDE.
<br>Open the Arduino IDE, select <b>Sketch-->Include Library-->Manage Libraries</b>.
<br>Type "melopero sam-m8q", select the library and click on "Install".
<br>The installation will include some examples, available under <b>File-->Examples-->Melopero SAM-M8Q</b>.



### Attention:

This breakout board is compatible only with 3.3V power and logic board, such as the <b>Arduino MKR</b>.<br> You'll need a level converter to use this breakout board with an Arduino UNO.
