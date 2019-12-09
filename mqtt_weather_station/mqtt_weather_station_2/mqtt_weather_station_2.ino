#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include "TimerOne.h"
#include <math.h>

#include "cactus_io_DS18B20.h"
#include "cactus_io_BME280_I2C.h"
#include <BH1750FVI.h>

#define Bucket_Size 0.01 // bucket size to trigger tip count
#define RG11_Pin 3 // digital pin RG11 connected to
#define TX_Pin 8 // used to indicate web data tx
#define DS18B20_Pin 9 // DS18B20 Signal pin on digital 9

#define WindSensor_Pin (2) // digital pin for wind speed sensor
#define WindVane_Pin (A3) // analog pin for wind direction sensor
#define VaneOffset 0 // define the offset for caclulating wind direction

volatile unsigned long tipCount; // rain bucket tip counter used in interrupt routine
volatile unsigned long contactTime; // timer to manage any rain contact bounce in interrupt routine

volatile unsigned int timerCount; // used to count ticks for 2.5sec timer count
volatile unsigned long rotations; // cup rotation counter for wind speed calcs
volatile unsigned long contactBounceTime; // timer to avoid contact bounce in wind speed sensor

long lastTipcount; // keep track of bucket tips
float totalRainfall; // total amount of rainfall detected

volatile float windSpeed;
int vaneValue; // raw analog value from wind vane
int vaneDirection; // translated 0 - 360 wind direction
int calDirection; // calibrated direction after offset applied
int lastDirValue; // last recorded direction value

float minTemp; // keep track of minimum recorded temp
float maxTemp; // keep track of maximum recorded temp

// Create DS18B20, BME280 object
DS18B20 ds(DS18B20_Pin); // on digital pin 9
BME280_I2C bme; // I2C using address 0x76
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);                  // lux sensor address 0x23

// MQTT stuff
// Function prototypes

// Set your MAC address and IP address here
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 55, 100);

// Make sure to leave out the http and slashes!
const char* server = "192.168.55.119";
const char* MQTT_CLIENT_ID = "kazimier_weather";

// Ethernet and MQTT related objects
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

long i; // loop counter

void setup() {

// setup rain sensor values
lastTipcount = 0;
tipCount = 0;
totalRainfall = 0;

// setup anemometer values
lastDirValue = 0;
rotations = 0;

// setup timer values
timerCount = 0;

// disable the SD card by switching pin 4 High
pinMode(4, OUTPUT);
digitalWrite(4, HIGH);

// start the Ethernet connection and MQTT server
Ethernet.begin(mac, ip);
// Ethernet takes some time to boot!
delay(3000);  
Serial.begin(9600);

// Set the MQTT server to the server stated above ^
mqttClient.setServer(server, 1883);   

// Attempt to connect to the server with the ID "myClientID"
//if (mqttClient.connect(MQTT_CLIENT_ID)) 
//{
//  Serial.println("Connection has been established, well done");
//} 
//else 
//{
//  Serial.println("Looks like the server connection failed...");
//}

// start sensors
LightSensor.begin();  
bme.begin();

pinMode(TX_Pin, OUTPUT);
pinMode(RG11_Pin, INPUT);
pinMode(WindSensor_Pin, INPUT);
attachInterrupt(digitalPinToInterrupt(RG11_Pin), isr_rg, FALLING);
attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);

// setup the timer for 0.5 second
Timer1.initialize(500000);
Timer1.attachInterrupt(isr_timer);

sei();// Enable Interrupts
}

void loop() {
//reconnect();  // mqtt connection check...
// This is needed at the top of the loop!
mqttClient.loop();

bme.readSensor();

// update rainfall total if required
if(tipCount != lastTipcount) {
cli(); // disable interrupts
lastTipcount = tipCount;
totalRainfall = tipCount * Bucket_Size;
sei(); // enable interrupts
}
void sendMQTT();
Serial.println(i);
i=i+1;
delay(1);
}

void sendMQTT() {
 String mqttData;    // outgoing mqtt data
char charBuf[50];   // convert to char before sending mqtt
uint16_t lux = LightSensor.GetLightIntensity();
// Publish MQTT Data
mqttData = String(bme.getTemperature_C());
mqttData.toCharArray(charBuf, 50) ;
mqttClient.publish("/weathersensor/temperature", charBuf);

mqttData = String(totalRainfall);
mqttData.toCharArray(charBuf, 50) ;
mqttClient.publish("/weathersensor/rainfall", charBuf);

mqttData = String(bme.getHumidity());
mqttData.toCharArray(charBuf, 50) ;
mqttClient.publish("/weathersensor/humidity", charBuf);

mqttData = String(bme.getPressure_MB());
mqttData.toCharArray(charBuf, 50) ;
mqttClient.publish("/weathersensor/pressure", charBuf);

mqttData = String(windSpeed);
mqttData.toCharArray(charBuf, 50) ;
mqttClient.publish("/weathersensor/wind_speed", charBuf);

getWindDirection();
mqttData = String(calDirection);
mqttData.toCharArray(charBuf, 50) ;
mqttClient.publish("/weathersensor/wind_direction", charBuf);

mqttData = String(lux);
mqttData.toCharArray(charBuf, 50) ;
mqttClient.publish("/weathersensor/luminosity", charBuf);
}


// Interrupt handler routine for timer interrupt
void isr_timer() {

timerCount++;

if(timerCount == 5) {
// convert to mp/h using the formula V=P(2.25/T)
// V = P(2.25/2.5) = P * 0.9
windSpeed = rotations * 0.9;
rotations = 0;
timerCount = 0;
}
}

// Interrupt handler routine that is triggered when the rg-11 detects rain
void isr_rg() {

if((millis() - contactTime) > 15 ) { // debounce of sensor signal
tipCount++;
totalRainfall = tipCount * Bucket_Size;
contactTime = millis();
}
}

// Interrupt handler routine to increment the rotation count for wind speed
void isr_rotation() {

if((millis() - contactBounceTime) > 15 ) { // debounce the switch contact
rotations++;
contactBounceTime = millis();
}
}

// Get Wind Direction
void getWindDirection() {

vaneValue = analogRead(WindVane_Pin);
vaneDirection = map(vaneValue, 0, 1023, 0, 360);
calDirection = vaneDirection + VaneOffset;

if(calDirection > 360)
calDirection = calDirection - 360;

if(calDirection > 360)
calDirection = calDirection - 360;
}

//############################################
// MQTT reconnect routine
//############################################
void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
  Serial.print("MQTT reconnect...");

  // Attempt to connect
  if (mqttClient.connect(MQTT_CLIENT_ID))
  {
    Serial.println("connected!");
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 5 s");

    // Wait 5 seconds before retrying
    delay(5000);
  }
  }
}
