/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/

#include <WiFi.h>
#include <PubSubClient.h>
#include "EEPROM.h"
#include <ArduinoJson.h>
#include <stdio.h>
#include "driver/adc.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


//#define DEBUG 1  //IRM Enable/Disable as required

#define BUTTON KEY_BUILTIN


#define ENABLE_SENSOR_MEASUREMENT_PIN 17 //IRM Sensors' VCC Enable Pin (so no power is wasted while not measuring)
//IRM All the sensors' "VCC" must be connected to this pin. Max theoretical current with 6 sensors is 1mA

#define ENABLE_BATTERY_MEASUREMENT_PIN 16
#define BATTERY_ADC_PIN A11

/*
 * MAX VOLTAGE : 4.2 V = ADC 2606
 * ALARM       : 3.8 V = ADC 2358
 * MIN VOLTAGE : 3.7 V = ADC 2296
 * 
 * N = 12 bits 
 * VREF = 3.3 V
 */ 
#define MAX_BAT_VOLTAGE 2606
#define ALARM_BAT_VOLTAGE 2358
#define MIN_BAT_VOLTAGE 2296


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

#define WAKEUP_DELAY_FIX 3 //IRM Substract 3 seconds from sleep time to compensate for wake-up delay

#define EEPROM_SIZE 64

#define LED_BLINK_ON 25    //IRM LED On period between blink cycles
#define LED_BLINK_OFF 400  //IRM LED Off period between blink cycles

#define LONG_PRESS_MS 2000 //IRM Min time to determine whether the button press event was long or short
#define SHORT_PRESS_MS 20 //IRM Period between button-press checking try events
#define BUTTON_CHECK_TIMES 3 //IRM How many times the button must remain pressed to trigger a button-pressed event
#define PRESSED_STATE 0 //IRM Inverted-logic

#define NODE_ID_ADDR 0 //IRM Address where NodeID will be stored/retreived from
#define MAX_NODES 5

#define SECONDS 1000 //IRM Milisenconds to Seconds conversion

#define SAMPLING_RATE_REQUEST_RETRY_PERIOD 10*SECONDS //IRM Retry requesting sampling period after X seconds

#define DATA_TOPIC_PUBLISH "SensorNetwork/Nodes"
#define HELP_TOPIC_PUBLISH DATA_TOPIC_PUBLISH

#define SAMPLING_TOPIC_SUBSCRIBE "SensorNetwork/SamplingRate"

#define HELP_TOKEN "HELP!" //IRM Request Sampling Rate to the server

#define CONNECTED_SENSORS 6

// Update these with values suitable for your network.

const char* ssid = "Agricultura-Vertical";
const char* password = "eco>LITE";
const char* mqtt_server = "192.168.0.100";


//const char* mqtt_server = "192.168.0.102";

//IRM Analog pins where sensors may be plugged-in to. Sorted in ascendent order.
const int8_t ANALOG_PINS[] = {A4, A5, A6, A7, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19};
unsigned int sensorValues[MAX_NODES]; //IRM Sensor data will be stored here

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

bool samplingReceived = false;

byte nodeID = 0;

int timeToSleep = TIME_TO_SLEEP;


//IRM Custom function prototypes
void setupVREF(void);
void requestSamplingPeriod(void);
void showNodeID(void);
void setNodeID(void);
byte retrieveNodeIDFromEEPROM(void);
bool buttonPressed(unsigned int tries);
bool writeNodeIDToEEPROM(byte id);
void incrementNodeID(void);
void initSensorValues(void);
void sampleSensorValues(unsigned int *data);
unsigned int batteryLife(unsigned int adcChannel, unsigned int activationPin);
String toJSON(unsigned int node, unsigned int battery, unsigned int *data, uint8_t N);

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print(".");
    if(++i>5){ //IRM Reboot if WiFi connection isn't established in 10 seconds (Software Watchdog)
      esp_sleep_enable_timer_wakeup(1);
      esp_deep_sleep_start();
    }
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  char data[6];

  for(int j = 0; j < sizeof(data); j++){
    data[j] = 0;
  }
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");
  for (int i = 0; i < length; i++) { //IRM Convert to const char *
    data[i] = (char)payload[i];
  }
  const char * c = data;
  
  //IRM Parse text to integer
  timeToSleep = atoi(c) - WAKEUP_DELAY_FIX;

  #ifdef DEBUG
  Serial.print("Time to sleep: ");
  Serial.print(timeToSleep);
  Serial.println(" seconds");
  #endif

  digitalWrite(LED_BUILTIN, HIGH);
  samplingReceived = true; //IRM Go to deep sleep only if required token arrives
  delay(20);

}

void reconnect() {

  static boolean ledState = false;
  // Loop until we're reconnected
  while (!client.connected()) {
    ledState = ledState ? false : true;
    digitalWrite(LED_BUILTIN, ledState);
    Serial.print("Attempting MQTT connection...");

    // Create a random client ID
    String clientId = "ESP32-Node-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish(DATA_TOPIC_PUBLISH, "hello world");
      // ... and resubscribe
      client.subscribe(SAMPLING_TOPIC_SUBSCRIBE);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  digitalWrite(LED_BUILTIN, 0);
}

void setup() {
  pinMode(KEY_BUILTIN, INPUT_PULLUP); //Init dev-board integrated button. Inverted-logic
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(LED_BUILTIN, 0);
  
  pinMode(ENABLE_SENSOR_MEASUREMENT_PIN, OUTPUT); //IRM Disable Sensors' Power Source
  digitalWrite(ENABLE_SENSOR_MEASUREMENT_PIN, 0);

  pinMode(ENABLE_BATTERY_MEASUREMENT_PIN, INPUT); //IRM Disable battery measurement sink
  
  Serial.begin(115200); 

  digitalWrite(LED_BUILTIN, 1);
  delay(30);
  digitalWrite(LED_BUILTIN, 0);
  delay(2500);

  if(buttonPressed(BUTTON_CHECK_TIMES)){
    setNodeID();
    esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }else{

    //delay(1000);
    
    nodeID = retrieveNodeIDFromEEPROM();
    showNodeID();
  }


  //IRM Disable Brownout Detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  setup_wifi();
  //setupVREF(); //IRM Initialize Internal Voltage Reference

  Serial.print("Node ID: ");
  Serial.println(nodeID);
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  initSensorValues();
}

void loop() {

  String json;

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 200) {
    lastMsg = now;
    ++value;
    
    #ifdef DEBUG
    snprintf (msg, 75, "hw #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(DATA_TOPIC_PUBLISH, msg);
    #endif
    
    //IRM Gather data from sensors
    sampleSensorValues(sensorValues, CONNECTED_SENSORS);

    //IRM Generate JSON String from data and battery values
    unsigned int battery = batteryLife(BATTERY_ADC_PIN, ENABLE_BATTERY_MEASUREMENT_PIN);
    json = toJSON(nodeID, battery, sensorValues, CONNECTED_SENSORS);

    //IRM Publish json string to broker
    const char * c = json.c_str();
    client.publish(DATA_TOPIC_PUBLISH, c);
    
  }

  Serial.println("Waiting for server response");
  requestSamplingPeriod();
  Serial.println("Going to deep-sleep now");

  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

void requestSamplingPeriod(void){
  client.publish(HELP_TOPIC_PUBLISH, HELP_TOKEN);
  while(!samplingReceived){
    client.loop();
  }
}

byte retrieveNodeIDFromEEPROM(void){
  if (!EEPROM.begin(EEPROM_SIZE)){
    Serial.println("failed to initialise EEPROM"); return false;
  }

  nodeID = EEPROM.read(NODE_ID_ADDR);
  return nodeID;
}

bool writeNodeIDToEEPROM(byte id){
  if (!EEPROM.begin(EEPROM_SIZE)){
    Serial.println("failed to initialise EEPROM"); return false;
  }
  EEPROM.write(NODE_ID_ADDR, id);
  EEPROM.commit();
  return true;
}


void showNodeID(void){

  #ifdef DEBUG
    Serial.println("Show Node ID");
  #endif

  for(int i=0; i<nodeID; i++){
    digitalWrite(LED_BUILTIN, 1);
    delay(LED_BLINK_ON);
    digitalWrite(LED_BUILTIN, 0);
    delay(LED_BLINK_OFF);
  }
}


void incrementNodeID(void){
  nodeID = nodeID >= MAX_NODES ? 1 : nodeID + 1;
  writeNodeIDToEEPROM(nodeID);
  nodeID = retrieveNodeIDFromEEPROM();
  delay(100);
  showNodeID();
  
  #ifdef DEBUG
  Serial.println("Button Pressed!");
  Serial.print("Node ID: ");
  Serial.println(nodeID);
  #endif
}


void setNodeID(void){
  static boolean setOk = false;

  static long startTime;
  long deltaTime;

  digitalWrite(LED_BUILTIN, 1);

  delay(1500);

  digitalWrite(LED_BUILTIN, 0);

  #ifdef DEBUG
    Serial.println("Starting setNodeID");
  #endif

  while(!setOk){ //IRM Keep looping until ID has been succesfully stored in EEPROM

    if (buttonPressed(BUTTON_CHECK_TIMES)){
        incrementNodeID();
    }

  
    //IRM Long button press
    if(buttonPressed(BUTTON_CHECK_TIMES)){
      startTime = millis();
      deltaTime = 0;
      while((buttonPressed(BUTTON_CHECK_TIMES))&&(deltaTime < LONG_PRESS_MS)){
        deltaTime = millis() - startTime;
        delay(10);
        
        #ifdef DEBUG
          Serial.println("LP");
        #endif
        
      }
      if(deltaTime >= LONG_PRESS_MS){
        #ifdef DEBUG
          Serial.println("Long Press!");
        #endif
        setOk = writeNodeIDToEEPROM(nodeID);
      }else{
        incrementNodeID();
      }
    }
    
  }
  
}


bool buttonPressed(unsigned int tries){
  for(int i = 0; i < tries; i++){
    if(digitalRead(KEY_BUILTIN) != PRESSED_STATE)
      return false;
    delay(SHORT_PRESS_MS);
  }
  return true;
}

//IRM Clean Sensor Values Data Vector
void initSensorValues(void){ 
  for(int i = 0; i < MAX_NODES; i++){
    sensorValues[i] = 0;
  }
}

//IRM write sensor values into referenced data vector with N values
void sampleSensorValues(unsigned int *data, uint8_t N){

  //IRM Enable sensors' power supply
  digitalWrite(ENABLE_SENSOR_MEASUREMENT_PIN, 1);

  
  for(int i = 0; i < N; i++){
    //IRM Fixed nonlinearity in ESP32 ADC mearurements
    //IRM Coefficients obtained through experimentation with a calibrated power supply
    data[i] = (unsigned int)(analogRead(ANALOG_PINS[i])*1.0337 + 203.42);
  }

  //IRM Disable sensors' power supply
  digitalWrite(ENABLE_SENSOR_MEASUREMENT_PIN, 0);
}

//IRM return battery measurement within 100% to 0% range in unsigned integer format
unsigned int batteryLife(unsigned int adcChannel, unsigned int activationPin){

  //IRM use a 1/2 voltage divider without exceeding 12 mA
  pinMode(activationPin, OUTPUT);
  digitalWrite(activationPin, 0); //IRM Sink battery measurement current
  unsigned int batt = (unsigned int)(analogRead(adcChannel)*1.0337 + 203.42); //IRM Nonlinearity fixed
  pinMode(activationPin, INPUT); //IRM Stop sinking battery measurement current to save power

  batt = batt>MAX_BAT_VOLTAGE?MAX_BAT_VOLTAGE:batt; //IRM Limit battery voltage to a theoretical 100%

<<<<<<< HEAD
  batt = map(batt, MIN_BAT_VOLTAGE, MAX_BAT_VOLTAGE, 0, 100); //IRM (3.7V -> 4.2V) = (0% -> 100%)

=======
  map(batt, 2296, 2605, 0, 100); //IRM (3.7V -> 4.2V) = (0% -> 100%)
>>>>>>> 2560c69812219ea112cd14702f6767079ab73f59

  //IRM battery measurement in 0% to 100% scale (see #defines for more details)
  return batt; 
  
}


String toJSON(unsigned int node, unsigned int battery, unsigned int *data, uint8_t N){

  String jsonDataString;
  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["nodes"] = node;
  root["battery"] = battery;

  JsonArray& sensors = root.createNestedArray("measures");
  for(int i = 0; i < N; i++){
    sensors.add(data[i]);
  }

  root.printTo(jsonDataString);

  #ifdef DEBUG
    Serial.println(jsonDataString);
  #endif

  return jsonDataString;
  
}

//IRM Setup Internal 1000 mV VRef for Battery Measurment
//IRM VRef routed to GPIO25 (A12)
void setupVREF(void){
  pinMode(A12, INPUT);
  esp_err_t status = adc2_vref_to_gpio(GPIO_NUM_25);
  if(status == ESP_OK){
    printf("v_ref routed to GPIO25\n");
  }else{
    printf("failed to route v_ref\n");
    Serial.println(status);
    //IRM Reboot Now
    esp_sleep_enable_timer_wakeup(1);
    esp_deep_sleep_start();
  }
}

