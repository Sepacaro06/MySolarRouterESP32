/*
 Solar router by PBU for ESP32
*/
#include <RBDdimmer.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include "config.h"

#define zerocross  1 // for boards with CHANGEBLE input pins
#define outputPin  2 //12 
#define BUILTIN_LED 2

#define STEP 1
#define MAX_POWER 100
#define MIN_POWER   0

#define RXD2 13 // For ESP32 - ESP8266 RXD2 = GPIO 13
#define TXD2 15 // For ESP32 -ESP8266 = GPIO 15

#define MK194_FACTORY_SPEED 4800
#define MK194_SPEED 38400

long baudRates[] = { 4800, 9600, 19200, 38400, 57600, 115200 };  // array of baud rates to test
int numBaudRates = 1;      

byte SensorData[62];

dimmerLamp WaterHeaterDimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
int outVal = 0;

const char* TopicPower                 = "Home/House/First/LivingRoom/Power";

const char* TopicHello                 = "Home/Workshop/WaterHeater/Hello";
const char* TopicSwitchForceOnCommand  = "Home/Workshop/WaterHeater/Switch/ForceON/command";
const char* TopicSwitchForceOnState    = "Home/Workshop/WaterHeater/Switch/ForceON/state";
const char* TopicSwitchForceOffCommand = "Home/Workshop/WaterHeater/Switch/ForceOFF/command";
const char* TopicSwitchForceOffState   = "Home/Workshop/WaterHeater/Switch/ForceOFF/state";
const char* TopicWaterHeaterAvailable  = "Home/Workshop/WaterHeater/available";
const char* TopicDimmerSliderCommand   = "Home/Workshop/WaterHeater/Dimmer/Slider/command";
const char* TopicDimmerSliderState     = "Home/Workshop/WaterHeater/Dimmer/Slider/state";

const char* TopicSensorPower           = "Home/Workshop/WaterHeater/Sensor/Power";
const char* TopicSensorWaterTemp       = "Home/Workshop/WaterHeater/Sensor/WaterTemp";
const char* TopicSensorDimmerTemp      = "Home/Workshop/WaterHeater/Sensor/DimmerTemp";

WiFiClient espClient;
PubSubClient client(espClient);


unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
int m_iExportedPower = 0;
uint8_t m_iDimmerPower = 0;
uint8_t m_iDimmerSlider = 0;
int8_t m_iDimmerStep = 0;
bool m_bForceON = false;
bool m_bForceOFF = false;

void setup() {

  // Initialize the BUILTIN_LED pin as an output
  pinMode(BUILTIN_LED, OUTPUT); 
  digitalWrite(BUILTIN_LED, HIGH);

  // Initialize the Serial USB    
  Serial.begin(115200);
  Serial2.begin(MK194_FACTORY_SPEED); 
  delay(500);

  // Initialize the Wifi
  setup_wifi();

  // Initialize Mqtt
  client.setServer(MQTT_BROKER_IP, 1883);
  client.setCallback(callback);

  OTASetup();

   // Turn the Dimmer on, power 0
  WaterHeaterDimmer.begin(NORMAL_MODE, OFF);
}

void setup_wifi() {

  delay(100);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.printf("Connecting to %s ", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.print(" WiFi connected on IP address: ");
  // Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void SerialMqttMessage(char* topic, byte* payload, unsigned int length){
  // print mqtt message
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
   Serial.print((char)payload[i]);
  }
  Serial.println();
}

bool MqttSwitchCommand(const char* topic, byte* payload, unsigned int length){
    char commandPayload[length + 1];
    memcpy(commandPayload, payload, length);
    commandPayload[length] = '\0';
    Serial.printf("Switch state : [%s] %s\n\r", topic, commandPayload); 
    client.publish(topic, commandPayload, true);
    return (strcmp(commandPayload, "ON") == 0);
}

void callback(char* topic, byte* payload, unsigned int length) {
  String sTopic = topic;

  SerialMqttMessage(topic, payload, length);
 
  if (sTopic == TopicSwitchForceOffCommand) {
    m_bForceOFF = MqttSwitchCommand(TopicSwitchForceOffState, payload, length);
  }

  if (sTopic == TopicSwitchForceOnCommand) {
    m_bForceON = MqttSwitchCommand(TopicSwitchForceOnState, payload, length);
  }

  if (sTopic == TopicDimmerSliderCommand) {
    char sSlider[length + 1];
    memcpy(sSlider, payload, length);
    sSlider[length] = '\0';
    m_iDimmerSlider = atoi(sSlider);
    //Serial.printf("m_iDimmerSlider : %s\n\r", sSlider); 
  }

  if (sTopic == TopicPower) {
    char sPower[length + 1];
    memcpy(sPower, payload, length);
    sPower[length] = '\0';
    // Serial.printf("m_iExportedPower : %s\n\r", sPower); 
    m_iExportedPower = atoi(sPower);

    // Power regulation
    if ( m_iExportedPower > 500 )     { m_iDimmerStep = -STEP * 2; }
    else if (m_iExportedPower > 0)    { m_iDimmerStep = -STEP;     } 
    else if (m_iExportedPower < -500) { m_iDimmerStep = STEP * 2;  } 
    else if (m_iExportedPower < -100) { m_iDimmerStep = STEP;      } 
    else                              { m_iDimmerStep = 0;         }
  }
}

bool ComputeDimmer() {

  uint8_t lPreviousDimmerPower = m_iDimmerPower;

  if (m_bForceOFF == true)         { m_iDimmerPower = MIN_POWER;      }
  else if ( m_bForceON == true)    { m_iDimmerPower = MAX_POWER;      }
  else if (m_iDimmerSlider != 0)   { m_iDimmerPower = m_iDimmerSlider;}
  else {
    int liDimmerPower = m_iDimmerPower + m_iDimmerStep ;
    if (liDimmerPower > MAX_POWER)      { m_iDimmerPower = MAX_POWER;     } 
    else if (liDimmerPower < MIN_POWER) { m_iDimmerPower = MIN_POWER;     }
    else                                { m_iDimmerPower = liDimmerPower; }
  }

  return (lPreviousDimmerPower != m_iDimmerPower);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_TOPIC_NAME, MQTT_USER, MQTT_PASSWORD, TopicWaterHeaterAvailable, 1, true, "offline")) {
      Serial.println("connected");
      client.publish(TopicWaterHeaterAvailable, "online", true);
      client.publish(TopicSwitchForceOffState, "OFF", true);
      client.publish(TopicSwitchForceOnState, "OFF", true);
      PublishDimmer();

      // ... and resubscribe
      client.subscribe(TopicSwitchForceOnCommand);
      client.subscribe(TopicSwitchForceOffCommand);
      client.subscribe(TopicDimmerSliderCommand);
      client.subscribe(TopicPower);
    } else {
      m_iDimmerStep = 0;
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  //SerialHelloMessage();

  //BlinkLED();
  DimmerLoop();

  OTALoop();

  // Read water heather power consumption
  ReadPowerMeterSensor();

  // 330ms
  delay(330);
}

void DimmerLoop() {

  if (ComputeDimmer()) {

    // Set Dimmer power
    if (m_iDimmerPower <= MIN_POWER + 10) { WaterHeaterDimmer.setState(OFF); }
    else                                  { WaterHeaterDimmer.setState(ON);  }
    WaterHeaterDimmer.setPower(m_iDimmerPower); 

    PublishDimmer();

    // Turn on the BUILTIN_LED if power is active
    if (WaterHeaterDimmer.getState() == ON) {
      //digitalWrite(BUILTIN_LED, LOW); 
    } else {
      //digitalWrite(BUILTIN_LED, HIGH);
    }
  }
}

void PublishDimmer() {
    // Publish Dimmer value in %
    char cDimmerPower[10];
    itoa(m_iDimmerPower, cDimmerPower, 10);
    Serial.printf("Message send : [%s] %s\n\r", TopicDimmerSliderState, cDimmerPower); 
    client.publish(TopicDimmerSliderState, cDimmerPower, true);
}

void BlinkLED() {
  if ( digitalRead(BUILTIN_LED) == LOW){
    digitalWrite(BUILTIN_LED, HIGH); 
  }
  else{
    digitalWrite(BUILTIN_LED, LOW); 
  }
}

void SerialHelloMessage() {
  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(TopicHello, msg);
  }
}

bool ReadPowerMeterSensor() {
  // Modbus RTU message to get the data
  byte msg[] = { 0x01, 0x03, 0x00, 0x48, 0x00, 0x0E, 0x44, 0x18 };
  int len = sizeof(msg);

  // Reset data
  for (int i = 0; i < sizeof(SensorData); i++) {
    SensorData[i] = 0x00;
  }

  // Send message
  for (int i = 0; i < len; i++) {
    Serial2.write(msg[i]);
  }
  //delay(500);

  // Get data from JSY-MK-194
  int a = 0;
  while (Serial2.available()) {
    SensorData[a] = Serial.read();
    a++;
  }
  // Sanity Checks
  if (SensorData[0] != 0x01) {
    char lenData[32];
    itoa(a, lenData, 10);
    Serial.println("JSY-MK-194 response error");
    return false;
  }

  if (a != 61) {
    Serial.println("JSY-MK-194 response size error (!= 61)");
    return false;
  }

  String resp; 
  char digit[10];
  for (int i = 0; i < a; i++) {

    itoa(SensorData[i], digit, 16);
    resp += digit;
    resp += " ";
  }
  client.publish(TopicSensorPower, resp.c_str());

  return true;
}

