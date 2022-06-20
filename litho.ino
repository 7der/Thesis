// latest latest
#include <SPI.h>
#include <DW1000Ranging.h>
#include <WiFi.h>
#include "link.h"

#include <Wire.h>//gy906
#include <Adafruit_MLX90614.h>//gy906
String tempState = "Pending";

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define PIN_RST 27
#define PIN_IRQ 34


const int xInput = 35;//adxl
const int yInput = 36;
const int zInput = 39;
int RawMin = 0;
int RawMax = 1023;
const int sampleSize = 10;
String xAccelState = "0G";
String yAccelState = "0G";
String zAccelState = "0G";
// Set web server port number to 80
WiFiServer server(80);


// Current time
unsigned long cTime = millis();
// Previous time
unsigned long pTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;


Adafruit_MLX90614 mlx = Adafruit_MLX90614();//gy906

//max30102 heart rate sensor
//#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
String bpmWeb = "0";
String irWeb = "0";

//const char* ssid = "HUAWEI-2.4G-Y7Bv";//Aries' House
//const char* password = "DxmwQ2w3";
const char* ssid = "HUAWEI-CJ2v";//Der's House
const char* password = "GE6z8NH5";
const char *host = "192.168.18.21";
WiFiClient client;
String header;//webserver

struct MyLink *uwb_data;
int index_num = 0;
long runtime = 0;
long runtime2 = 0;
String all_json = "";
/////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);




  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());
  server.begin();
  if (client.connect(host, 80))
  {
    Serial.println("Success");
    client.print(String("GET /") + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n" +
                 "\r\n");
  }

  delay(1000);

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //we start the module as a tag
  DW1000Ranging.startAsTag("7D:00:22  :EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  mlx.begin(0x5A);  //gy906
  uwb_data = init_link();
  // Initialize sensor
  /*if (!particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57)) //Use default I2C port, 400kHz speed
    {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
    }
    Serial.println("Place your index finger on the sensor with steady pressure.");
  */
  particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57);
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED



}

/////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{

  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  //Serial.print("IR=");
  //Serial.print(irValue);
  //Serial.print(", BPM=");
  //Serial.print(beatsPerMinute);
  //Serial.print(", Avg BPM=");
  //Serial.print(beatAvg);

  //bpmWeb = String(beatsPerMinute);
  //irWeb = String(irValue);
  //String bpmWeb = "0";
  //String irWeb = "0";
  /*
    if (irValue < 50000)
      Serial.print(" No finger?");

    //  Serial.println();


  */

  bpmWeb = String(beatsPerMinute);
  irWeb = String(irValue);
  DW1000Ranging.loop();
  if ((millis() - runtime) > 1000)
  {
    make_link_json(uwb_data, &all_json);
    send_udp(&all_json);
    runtime = millis();


    //Serial.print("IR=");
    //Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    //Serial.print(", Avg BPM=");
    //Serial.print(beatAvg);
    runtime2 = millis();

    printSensors();




    WiFiClient client = server.available();   // Listen for incoming clients

    if (client) {                             // If a new client connects,
      cTime = millis();
      pTime = cTime;
      Serial.println("New Client.");          // print a message out in the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
      while (client.connected() && cTime - pTime <= timeoutTime) {  // loop while the client's connected
        cTime = millis();
        if (client.available()) {             // if there's bytes to read from the client,
          char c = client.read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          header += c;
          if (c == '\n') {                    // if the byte is a newline character
            // if the current line is blank, you got two newline characters in a row.
            // that's the end of the client HTTP request, so send a response:
            if (currentLine.length() == 0) {
              // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
              // and a content-type so the client knows what's coming, then a blank line:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              // Display the HTML web page
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");
              // CSS to style the on/off buttons
              // Feel free to change the background-color and font-size attributes to fit your preferences
              client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
              client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
              client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
              client.println(".button2 {background-color: #555555;}</style></head>");

              // Web Page Heading
              client.println("<body><h1>ESP32 UWB Web Server</h1>");


              //Display data like from S monitor
              client.println("<p>IR reading:" +  irWeb  + "</p>");
              client.println("<p>Patient's Temperature:  " +  tempState  + "</p>");
              client.println("<p>Patient's Arm movement:  " +  xAccelState + "G," + yAccelState + "G," + zAccelState + "G "  + "</p>");
              client.println("<p>bpm: " +  bpmWeb  + "</p>");
              //Serial.print("IR=");
              //Serial.print(irValue);
              //Serial.print(", BPM=");
              //Serial.print(beatsPerMinute);
              //Serial.print(", Avg BPM=");
              //Serial.print(beatAvg);
              //String bpmWeb = "0";
              //String irWeb = "0";

              client.println("</body></html>");

              // The HTTP response ends with another blank line
              client.println();
              // Break out of the while loop
              break;
            } else { // if you got a newline, then clear currentLine
              currentLine = "";
            }
          } else if (c != '\r') {  // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }
        }
      }
      // Clear the header variable
      header = "";
      // Close the connection
      client.stop();
      Serial.println("Client disconnected.");
      Serial.println("");


    }
  }
  /*
    if ((millis() - runtime2) > 1000)
    { //Serial.print("IR=");
      //Serial.print(irValue);
      Serial.print(", BPM=");
      Serial.print(beatsPerMinute);
      //Serial.print(", Avg BPM=");
      //Serial.print(beatAvg);
      runtime2 = millis();

    }
  */
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
void newRange()
{
  char c[30];

  // Serial.print("from: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  // Serial.print("\t Range: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  // Serial.print(" m");
  //Serial.print("\t RX power: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  //Serial.println(" dBm");
  fresh_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(), DW1000Ranging.getDistantDevice()->getRange(), DW1000Ranging.getDistantDevice()->getRXPower());
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
void newDevice(DW1000Device * device)
{
  // Serial.print("ranging init; 1 device added ! -> ");
  //Serial.print(" short:");
  // Serial.println(device->getShortAddress(), HEX);

  add_link(uwb_data, device->getShortAddress());
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
void inactiveDevice(DW1000Device * device)
{
  // Serial.print("delete inactive device: ");
  //Serial.println(device->getShortAddress(), HEX);

  delete_link(uwb_data, device->getShortAddress());
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
void send_udp(String * msg_json)
{
  if (client.connected())
  {
    client.print(*msg_json);
    ///Serial.println("UDP send");
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/*void heartSensor() {

  }*/
/////////////////////////////////////////////////////////////////////////////////////////////////////
void printSensors() {

  //Serial.print("Ambient = "); Serial.print(.readAmbientTempC()); //gy906
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC() - 16.8); Serial.println("*C");

  //  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());
  // Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  int tp = mlx.readObjectTempC() - 0;// Temp Sensor calibration
  tempState = String(tp);




  int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);

  // Convert raw values to 'milli-Gs"
  long xScaled = map(xRaw, RawMin, RawMax, -3000, 3000);
  long yScaled = map(yRaw, RawMin, RawMax, -3000, 3000);
  long zScaled = map(zRaw, RawMin, RawMax, -3000, 3000);

  // re-scale to fractional Gs
  float xAccel = xScaled / 1000.0;
  float yAccel = yScaled / 1000.0;
  float zAccel = zScaled / 1000.0;
  int xAccel4 = xAccel;
  int yAccel4 = yAccel;
  int zAccel4 = zAccel;
  xAccelState = String(xAccel4);
  yAccelState = String(yAccel4);
  zAccelState = String(zAccel4);
  // Serial.print(xAccel, 0);
  // Serial.print("G, ");
  // Serial.print(yAccel, 0);
  // Serial.print("G, ");
  //  Serial.print(zAccel, 0);
  // Serial.println("G");
  // Serial. print('\n');



}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*void printBpm(){
     //Serial.print("IR=");
  //Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  //Serial.print(", Avg BPM=");
  //Serial.print(beatAvg);

  bpmWeb = String(beatsPerMinute);
  irWeb = String(irValue);
  }*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading / sampleSize;
}
