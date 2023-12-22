/*
 Solar router by PBU for ESP32
*/

#include <U8g2lib.h>            // gestion affichage écran Oled  https://github.com/olikraus/U8g2_Arduino/ //
#include <RBDdimmer.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <NTPClient.h>          // gestion de l'heure https://github.com/arduino-libraries/NTPClient //
#include <OneWire.h>            // pour capteur de température DS18B20
#include <DallasTemperature.h>  // pour capteur de température DS18B20 https://github.com/milesburton/Arduino-Temperature-Control-Library
#include "config.h"

const int zeroCrossPin = 35;    // broche utilisée pour le zéro crossing
const int pulsePin = 25;        // broche impulsions routage 1

#define BUILTIN_LED 2

#define STEP 1
#define MAX_POWER 100
#define MIN_POWER   0

#define RXD2 16
#define TXD2 17

#define MK194_FACTORY_SPEED 4800
#define MK194_SPEED 38400

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // ESP32 Thing, HW I2C with pin remapping

long baudRates[] = { 4800, 9600, 19200, 38400, 57600, 115200 };  // array of baud rates to test
byte SensorData[62];

dimmerLamp WaterHeaterDimmer(pulsePin, zeroCrossPin);

int outVal = 0;

const char* TopicPower                 = "Home/House/First/LivingRoom/Power";

const char* TopicHello                 = "Home/Workshop/WaterHeaterPlus/Hello";
const char* TopicSwitchForceOnCommand  = "Home/Workshop/WaterHeaterPlus/Switch/ForceON/command";
const char* TopicSwitchForceOnState    = "Home/Workshop/WaterHeaterPlus/Switch/ForceON/state";
const char* TopicSwitchForceOffCommand = "Home/Workshop/WaterHeaterPlus/Switch/ForceOFF/command";
const char* TopicSwitchForceOffState   = "Home/Workshop/WaterHeaterPlus/Switch/ForceOFF/state";
const char* TopicWaterHeaterAvailable  = "Home/Workshop/WaterHeaterPlus/available";
const char* TopicDimmerSliderCommand   = "Home/Workshop/WaterHeaterPlus/Dimmer/Slider/command";
const char* TopicDimmerSliderState     = "Home/Workshop/WaterHeaterPlus/Dimmer/Slider/state";

const char* TopicSensorPower           = "Home/Workshop/WaterHeaterPlus/Sensor/Power";
const char* TopicSensorWaterTemp       = "Home/Workshop/WaterHeaterPlus/Sensor/WaterTemp";
const char* TopicSensorDimmerTemp      = "Home/Workshop/WaterHeaterPlus/Sensor/DimmerTemp";

WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
// Choix du serveur NTP pour récupérer l'heure, 3600 =1h est le fuseau horaire et 60000=60s est le * taux de rafraichissement
NTPClient timeNTP(ntpUDP, "fr.pool.ntp.org", 3600, 60000);

//int value = 0;
int m_iExportedPower = 0;
uint8_t m_iDimmerPower = 0;
uint8_t m_iDimmerSlider = 0;
int8_t m_iDimmerStep = 0;
bool m_bForceON = false;
bool m_bForceOFF = false;

// Modbus RTU message to get the data
byte JSKReadPower[] = { 0x01, 0x03, 0x00, 0x48, 0x00, 0x04, 0xC4, 0x1F };
float voltage          = 0;
float current          = 0;
float power            = 0;
float energy           = 0;
float temperatureC     = 0;    
float EnergySavedDaily = 0;  // énergie sauvées le jour J et remise à zéro tous les jours //
float EnergyInitDaily  = 0;  // Energie en debut de journee, lu a 00:00:00
int Start              = 1;  // variable de démarrage du programme //

const int oneWireBus = 32; // broche du capteur DS18B20 //
OneWire oneWire(oneWireBus); // instance de communication avec le capteur de température
DallasTemperature sensors(&oneWire); // correspondance entreoneWire et le capteur Dallas de température

// Multi-core
TaskHandle_t Task1, Task2, Task3;

void setup() {

  // ECRAN OLED
  u8g2.begin(); 
  u8g2.enableUTF8Print(); //nécessaire pour écrire des caractères accentués
  Display();

  // Initialize the BUILTIN_LED pin as an output
  pinMode(BUILTIN_LED, OUTPUT); 
  digitalWrite(BUILTIN_LED, LOW);

  // Initialize the Serial USB    
  Serial.begin(115200);

  // Initialize the Wifi
  setup_wifi();

  // Initialize Mqtt
  client.setServer(MQTT_BROKER_IP, 1883);
  client.setCallback(callback);
  
  //Intialisation du client NTP
  timeNTP.begin(); 

  delay(500);

  OTASetup();

  delay(500);

  // initialisation du capteur DS18B20
  sensors.begin(); 

   // initialisation du capteurJSY
  setup_JSY();

   // Turn the Dimmer on, power 0
  WaterHeaterDimmer.begin(NORMAL_MODE, OFF);

  Serial.println("Create Task_PowerMonitoring on Core 0");
  
  // Code pour créer un Task Core 0//
  xTaskCreatePinnedToCore(
    Task_PowerMonitoring, /* Task function. */
    "Task1",              /* name of task. */
    10000,                /* Stack size of task */
    NULL,                 /* parameter of the task */
    1,                    /* priority of the task */
    &Task1,               /* Task handle to keep track of created task */
    0);                   /* pin task to core 0 */


  Serial.println("Create Task_Screen on Core 1");
  xTaskCreatePinnedToCore(
    Task_Screen, /* Task function. */
    "Screen",    /* name of task. */
    40000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */

  Serial.println("Create Task_Communication on Core 1");
  xTaskCreatePinnedToCore(
    Task_Communication, /* Task function. */
    "Communication",    /* name of task. */
    40000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &Task3,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
}

void Task_PowerMonitoring(void *pvParameters) {
  Serial.println("Task_PowerMonitoring");
  const TickType_t taskPeriod = 330;  // In ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    //MQTT loop to read power send to network
    client.loop();

    //Calculate and set dimmer value
    DimmerLoop();

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void Task_Communication(void *pvParameters) {
  Serial.println("Task_Communication");
  const TickType_t taskPeriod = 1000;  // In ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    // Mqqt connection
    reconnect();

    timeNTP.update();

    // OTA loop
    OTALoop();

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void Task_Screen(void *pvParameters) {
  Serial.println("Task_Screen");
  const TickType_t taskPeriod = 1000;  // In ms
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {

     // Read water heather power consumption
    ReadPowerMeterSensor();

    // Read water Temperature
    ReadTemperature();

    CalculateEnergyDaily();

    Display();

    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
  }
}

void setup_JSY(){
  Serial2.begin(MK194_FACTORY_SPEED, SERIAL_8N1, RXD2, TXD2); //PORT DE CONNEXION AVEC LE CAPTEUR JSY-MK-194
    // Read water heather power consumption
  if (ReadPowerMeterSensor()) {
    Serial.printf("Sensor is working at speed : %d\n\r", MK194_FACTORY_SPEED);
  }
}

void setup_wifi() {
  // We start by connecting to a WiFi network
  // Serial.println();
  Serial.printf("Connecting to %s ", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    BlinkLED();
    Serial.print(".");
    delay(500);
  }

  randomSeed(micros());

  Serial.println();
  Serial.print("WiFi connected on IP address: ");
  Serial.println(WiFi.localIP());

  // Display blue LED : wifi connected
  digitalWrite(BUILTIN_LED, HIGH); 
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
      Serial.println(" try again in 1 seconds");
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}

// Everything is done in specific task
void loop() {}

void DimmerLoop() {

  if (ComputeDimmer()) {

    // Set Dimmer power
    if (m_iDimmerPower <= MIN_POWER + 10) { WaterHeaterDimmer.setState(OFF); }
    else                                  { WaterHeaterDimmer.setState(ON);  }
    WaterHeaterDimmer.setPower(m_iDimmerPower); 

    PublishDimmer();
  }
}

void PublishDimmer() {
    // Publish Dimmer value in %
    char cDimmerPower[10];
    itoa(m_iDimmerPower, cDimmerPower, 10);
    Serial.printf("Message send : [%s] %s\n\r", TopicDimmerSliderState, cDimmerPower); 
    client.publish(TopicDimmerSliderState, cDimmerPower, true);
}

void PublishTemperature() {
    // Publish temperature of water C
    char cTemperature[10];
    sprintf(cTemperature, "%f", temperatureC);
    Serial.printf("Message send : [%s] %s\n\r", TopicSensorWaterTemp, cTemperature); 
    client.publish(TopicSensorWaterTemp, cTemperature, true);
}

void BlinkLED() {
  if ( digitalRead(BUILTIN_LED) == LOW){
    digitalWrite(BUILTIN_LED, HIGH); 
  }
  else{
    digitalWrite(BUILTIN_LED, LOW); 
  }
}

bool ReadPowerMeterSensor() {
  

  // Reset data
  memset(SensorData, 0x00, sizeof(SensorData));

   Serial2.flush();
  // Send message
  for (int i = 0; i < sizeof(JSKReadPower); i++) {
    Serial2.write(JSKReadPower[i]);
  }

  delay(200);

  // Get data from JSY-MK-194
  int a = 0;
  while (Serial2.available()) {
    SensorData[a] = Serial2.read();
    a++;
  }

  // Display response
  for (int i = 0; i < a; i++) {
    Serial.printf("%x:", SensorData[i]);
  }
  Serial.println();

  // Sanity Checks
  if (SensorData[0] != 0x01) {
    Serial.printf("ERROR2 - ReadPowerMeterSensor() - Message received do not start with 0x01, message length is %d\n\r", a);
    return false;
  }

  voltage = ConvertByteArrayToFloat(&SensorData[3]);
  Serial.printf("voltage = %4.2f\n\r",voltage);
  current = ConvertByteArrayToFloat(&SensorData[7]);
  Serial.printf("current = %4.2f\n\r",current);
  power = ConvertByteArrayToFloat(&SensorData[11]);
  Serial.printf("power = %4.2f\n\r",power);
  energy = ConvertByteArrayToFloat(&SensorData[15]);
  Serial.printf("energy = %4.2f\n\r",energy);

  return true;
}

void Display() {
  u8g2.clearBuffer(); // on efface ce qui se trouve déjà dans le buffer

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(5, 10); // position du début du texte
  u8g2.print("ROUTEUR S"); // écriture de texte
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.drawGlyph(49, 13, 0x2600);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(58, 10); // position du début du texte
  u8g2.print("LAIREv1.0"); // écriture de texte
  u8g2.drawRFrame(5,16,120,22,11); // rectangle x et y haut gauche / longueur / hauteur / arrondi //
  u8g2.setCursor(75, 47);
  u8g2.print(WiFi.localIP()); // affichage adresse ip //

  if (m_bForceON || m_iDimmerSlider != 0) {
    u8g2.setFont(u8g2_font_streamline_all_t);
    u8g2.drawGlyph(5, 38, 0x00d9);
  } else if (power > 20){
    u8g2.setFont(u8g2_font_emoticons21_tr);
    u8g2.drawGlyph(5, 38, 0x0036);
  } else {
    u8g2.setFont(u8g2_font_emoticons21_tr);
    u8g2.drawGlyph(5, 38, 0x0026);
  }

  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.setCursor(30, 31);
  u8g2.printf("%4.0f W %2.1f °", power, temperatureC);  

  if (client.connected()) {
    u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
    u8g2.drawGlyph(58 + 51, 11, 0x0053);    
  }
  if (WiFi.status() == WL_CONNECTED) {
    u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
    u8g2.drawGlyph(58 + 62, 11, 0x0051);
  }

  // Display time
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(5, 47);
  u8g2.print(timeNTP.getFormattedTime());

  // Energy by day in WaterHeater
  u8g2.setCursor(5, 55);
  u8g2.printf("Energie/Jour : %2.1f KWh", EnergySavedDaily/1000);

  // Display power %
  u8g2.setCursor(5, 63);
  u8g2.printf("Puissance : %d %%", m_iDimmerPower);

  u8g2.sendBuffer();  // l'image qu'on vient de construire est affichée à l'écran
}

void ReadTemperature() {
  float newTemperatureC = 0;
  sensors.requestTemperatures();                          // demande de température au capteur //
  newTemperatureC = sensors.getTempCByIndex(0);           // température en degrés Celcius
  Serial.printf("Temperature °C : %f\n\r", temperatureC);
  if (newTemperatureC != temperatureC){
    temperatureC = newTemperatureC;
    PublishTemperature();
  }
 
}

float ConvertByteArrayToFloat(byte* bytes) {
  float value = (bytes[3]) | (bytes[2] << 8) | (bytes[1] << 16) | (bytes[0] << 24);
  return value * 0.0001;
}

void CalculateEnergyDaily() {

      if (timeNTP.getHours() == 23 & timeNTP.getMinutes() == 59 & timeNTP.getSeconds() == 59) {
      EnergySavedDaily = 0;
      Start = 1;
    }

    if (Start == 1) {
      EnergyInitDaily = energy;
      Start = 0;
    }

    EnergySavedDaily = energy - EnergyInitDaily;
}
