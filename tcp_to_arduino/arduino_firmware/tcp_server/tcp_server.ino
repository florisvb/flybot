// Ethernet client for Arduino, intended for use with ROS
// Floris van Breugel 2012

// Written for Arduino Uno with integrated Ethernet Shield

// Some useful info:
// from: http://arduino.cc/en/Main/ArduinoEthernetShield
// Arduino communicates with both the W5100 and SD card using the SPI bus 
// (through the ICSP header). This is on digital pins 11, 12, and 13 on the 
// Duemilanove and pins 50, 51, and 52 on the Mega. On both boards, pin 10 
// is used to select the W5100 and pin 4 for the SD card. These pins cannot 
// be used for general i/o. On the Mega, the hardware SS pin, 53, is not 
// used to select either the W5100 or the SD card, but it must be kept as an 
// output or the SPI interface won't work.

// Note that because the W5100 and SD card share the SPI bus, only one can 
// be active at a time. If you are using both peripherals in your program, 
// this should be taken care of by the corresponding libraries. If you're 
// not using one of the peripherals in your program, however, you'll need 
// to explicitly deselect it. To do this with the SD card, set pin 4 as an 
// output and write a high to it. For the W5100, set digital pin 10 as a 
// high output.

#include <SPI.h>
#include <Ethernet.h>

// for UNO: 3, 5, 6, 9, 10, and 11. (the pins with the ~ next to them) 
// Provide 8-bit PWM output with the analogWrite() function
int motor_speed_pwm_pin = 3;
int motor_steer_pwm_pin = 9;

// Enter a MAC address, IP address and Portnumber for your Server below.
// The IP address will be dependent on your local network:
byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x6B, 0xA0 };
IPAddress serverIP(192,168,0,104);
int serverPort=8888;

// Initialize the Ethernet server library
// with the IP address and port you want to use
EthernetServer server(serverPort);

// Variables used for decoding and delimiting TCP msg
boolean reading = false;
boolean readAction = false;
boolean readDataName = false;
boolean readDataVal = false;

void setup()
{
  // start the serial for debugging
  Serial.begin(9600);
  
  // fix pwm frequency
  // TCCR2B = (TCCR2B & 0xF8) | 5;
  // 31372.5      Hz    ->        1
  // 3921.57      Hz    ->        2
  // 980.392      Hz    ->        3
  // 490.196      Hz    ->        4    (default)
  // 245.098      Hz    ->        5
  // 122.549      Hz    ->        6
  // 30.6373      Hz    ->        7

  
  
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, serverIP);
  server.begin();
  Serial.println("Server started");//log
}

void loop()
{
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    String clientMsg;
    String action;
    String dataName;
    String dataVal;
    
    readAction = true;
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        clientMsg+=c;//store the recieved characters in a string
        
        // check for delimiters
        if (c==':') {
          readAction = false;
          readDataName = true;
        } else if (c=='=') {
          readDataName = false;
          readDataVal = true;
        } else if (c=='\n') { // if "end of line" run the appropriate function
          Serial.println("Message from Client:"+clientMsg);//print it to the serial, for debugging
          if (action=="s") setData(dataName, dataVal, client);
          else if (action=="g") getData(dataName, client);
          clientMsg = "";
          action = "";
          dataName = "";
          dataVal = "";
          readAction = true;
          readDataVal = false;
        }
        
        // save data into appropriate strings
        else if (readAction) {
          action+=c;
        } else if (readDataName) {
          dataName+=c;
        } else if (readDataVal) {
          dataVal+=c;
        }
        
       
      }
    }
    
    // give the Client time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
}

////////////////////////////////////////////////////////////////////////////////////
// functions to do stuff

void getData(String dataName, EthernetClient client) {
  
  if (dataName=="ax") {
    int sensorReading = analogRead(0);
    //String serial_msg = "Sensor2 reading: ";
    //serial_msg+=sensorReading;
    //Serial.println(serial_msg);
    client.println(sensorReading);
  }
  else if (dataName=="ay") {
    int sensorReading = analogRead(1);
    //String serial_msg = "Sensor2 reading: ";
    //serial_msg+=sensorReading;
    //Serial.println(serial_msg);
    client.println(sensorReading);
  }
  else if (dataName=="az") {
    int sensorReading = analogRead(2);
    //String serial_msg = "Sensor2 reading: ";
    //serial_msg+=sensorReading;
    //Serial.println(serial_msg);
    client.println(sensorReading);
  }
  
  else if (dataName=="s") {
    int sensorReading = analogRead(3);
    //String serial_msg = "Sensor2 reading: ";
    //serial_msg+=sensorReading;
    //Serial.println(serial_msg);
    client.println(sensorReading);
  }
  
  else {
    client.println(-1000);
  }
  
  
}
  
  
  
void setData(String dataName, String dataVal, EthernetClient client) {
  
  if (dataName=="test"){
    String serial_msg = "Setting motor value: ";
    serial_msg+=dataVal;
    Serial.println(serial_msg);
    
    // convert to an integer
    char buf[dataVal.length()+1];
    dataVal.toCharArray(buf,dataVal.length()+1);
    int dataValInt=atoi(buf);
    
    // send it back
    // client.println(dataValInt); 
  }
  
  if (dataName=="mspd"){
    // convert to an integer
    char buf[dataVal.length()+1];
    dataVal.toCharArray(buf,dataVal.length()+1);
    int dataValInt=atoi(buf);
    
    // set motor speed pwm pin
    analogWrite(motor_speed_pwm_pin,dataValInt);
    
    // send it back
    // client.println(dataValInt); 
  }
  
  if (dataName=="mstr"){
    // convert to an integer
    char buf[dataVal.length()+1];
    dataVal.toCharArray(buf,dataVal.length()+1);
    int dataValInt=atoi(buf);
    
    // set motor speed pwm pin
    analogWrite(motor_steer_pwm_pin,dataValInt);
    
    // send it back
    // client.println(dataValInt); 
  }
  
  
  
}
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
          
