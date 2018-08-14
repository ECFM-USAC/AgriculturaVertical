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
//#include "driver/adc.h"
//#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


//#define DEBUG  //IRM Enable/Disable as required


#define BUTTON KEY_BUILTIN


#define ENABLE_SENSOR_MEASUREMENT_PIN 17 //IRM Sensors' VCC Enable Pin (so no power is wasted while not measuring)
//IRM All the sensors' "VCC" must be connected to this pin. Max theoretical current with 6 sensors is 1mA

#define ENABLE_BATTERY_MEASUREMENT_PIN 15
#define BATTERY_ADC_PIN 33

/*
 * USING 1/2 V resistor divider
 * 
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

#define NODE_ID_ADDR 0 //IRM EEPROM Address where NodeID will be stored/retreived from
#define MAX_NODES 5

#define RESTART_FLAG_ADDR 1 //IRM EEPROM Address where Restart Flag will be stored/retreived from

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


// Analog Mux used for Analog Input PINS
const uint8_t MUX_SEL_PINS [] = {14, 12, 13}; //S0, S1, S2
const uint8_t SENSOR_PIN      = 32; // Analog Input Pin

const uint8_t ANALOG_MUX   [] = {3, 0, 1, 2, 4, 6}; //Order of sampled pins
const uint8_t MUX_INHIBIT_PIN = 25;



//IRM Analog pins where sensors may be plugged-in to. Sorted in ascendent order.
const int8_t ANALOG_PINS[] = {A4, A5, A18, A17, A16, A15, A14};
unsigned int sensorValues[CONNECTED_SENSORS]; //IRM Sensor data will be stored here
int battery; //IRM Battery monitor voltage

byte shouldIRestartMyself; //IRM Restart flag to fix ADC2<->WiFi compatibility

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

bool samplingReceived = false;

byte nodeID = 0;

int timeToSleep = TIME_TO_SLEEP;


//IRM Custom function prototypes
//void setupVREF(void);
void requestSamplingPeriod(void);
void showNodeID(void);
void setNodeID(void);
byte retrieveNodeIDFromEEPROM(void);
bool buttonPressed(unsigned int tries);
bool writeNodeIDToEEPROM(byte id);
void incrementNodeID(void);
void initSensorValues(void);
void setMuxChannel(unsigned int channel);
void sampleSensorValues(unsigned int *data);
int batteryLife(unsigned int adcChannel, unsigned int lowActivationPin);
String toJSON(unsigned int node, unsigned int battery, unsigned int *data, uint8_t N);
byte getRestartFlag(byte eepromAddress);
bool setRestartFlag(byte eepromAddress, byte flagValue);

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if(++i>5){ //IRM Reboot if WiFi connection isn't established in 5 seconds (Software Watchdog)
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

  Serial.begin(115200);

  if(getRestartFlag(RESTART_FLAG_ADDR) > 0){
    setRestartFlag(RESTART_FLAG_ADDR, 0);
    //ESP.restart();
    digitalWrite(36, 0);
    pinMode(36, OUTPUT);
    digitalWrite(36, 0);
  }else{
    setRestartFlag(RESTART_FLAG_ADDR, 1); //IRM Restart after next boot 
  }

  WiFi.mode( WIFI_MODE_NULL );
  
  pinMode(KEY_BUILTIN, INPUT_PULLUP); //Init dev-board integrated button. Inverted-logic
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  digitalWrite(LED_BUILTIN, 0);
  
  pinMode(ENABLE_SENSOR_MEASUREMENT_PIN, OUTPUT); //IRM Disable Sensors' Power Source
  pinMode(MUX_INHIBIT_PIN, OUTPUT); //IRM Enable MUX's Inhibit function
  #ifdef DEBUG
    digitalWrite(ENABLE_SENSOR_MEASUREMENT_PIN, 1);
    digitalWrite(MUX_INHIBIT_PIN, 0);  
  #else
    digitalWrite(ENABLE_SENSOR_MEASUREMENT_PIN, 0);
    digitalWrite(MUX_INHIBIT_PIN, 1);
  #endif
  
  #ifdef DEBUG
    pinMode(ENABLE_BATTERY_MEASUREMENT_PIN, OUTPUT);  
    digitalWrite(ENABLE_BATTERY_MEASUREMENT_PIN, 0);
  #else
    pinMode(ENABLE_BATTERY_MEASUREMENT_PIN, INPUT); //IRM Disable battery measurement sink
  #endif
  
  
   

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

  //IRM Gather data from sensors and battery monitor
  sampleSensorValues(sensorValues, CONNECTED_SENSORS);
  battery = batteryLife(BATTERY_ADC_PIN, ENABLE_BATTERY_MEASUREMENT_PIN);

  
  
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
    


    //IRM Generate JSON String from data and battery values
    json = toJSON(nodeID, battery, sensorValues, CONNECTED_SENSORS);

    //IRM Publish json string to broker
    const char * c = json.c_str();
    client.publish(DATA_TOPIC_PUBLISH, c);
    
  }

  Serial.println("Waiting for server response");
  requestSamplingPeriod();

  #ifdef DEBUG
    Serial.println("10 seconds awake before going to sleep");
    delay(10000);
  #endif

  if (WiFi.isConnected()){
    WiFi.disconnect(true);
  }
  
  
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


byte getRestartFlag(byte eepromAddress){

  byte currentFlag;
  
  if (!EEPROM.begin(EEPROM_SIZE)){
    Serial.println("failed to initialise EEPROM"); return false;
  }
  currentFlag = EEPROM.read(RESTART_FLAG_ADDR);

  #ifdef DEBUG
    Serial.print("Restart Flag Value: ");
    Serial.println(currentFlag);
    
  #endif
  
  return currentFlag;
}

bool setRestartFlag(byte eepromAddress, byte flagValue){
  if (!EEPROM.begin(EEPROM_SIZE)){
    Serial.println("failed to initialise EEPROM"); return false;
  }

  EEPROM.write(eepromAddress, flagValue);
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
  for(int i = 0; i < CONNECTED_SENSORS; i++){
    sensorValues[i] = 0;
  }
}

//IRM Select MUX Analog Channel according to PCB pinout
void setMuxChannel(unsigned int channel){
  
  uint8_t muxPins[3];
  switch(channel){
    case 0: //3
      muxPins[0] = 1; muxPins[1] = 1; muxPins[2] = 0;
      break;
    case 1: //0
      muxPins[0] = 0; muxPins[1] = 0; muxPins[2] = 0;
      break;
    case 2: //1
      muxPins[0] = 1; muxPins[1] = 0; muxPins[2] = 0;
      break;
    case 3: //2
      muxPins[0] = 0; muxPins[1] = 1; muxPins[2] = 0;
      break;
    case 4: //4
      muxPins[0] = 0; muxPins[1] = 0; muxPins[2] = 1;
      break;    
    case 5: //6
      muxPins[0] = 0; muxPins[1] = 1; muxPins[2] = 1;
      break;
    default:
      muxPins[0] = 1; muxPins[1] = 1; muxPins[2] = 1;
      break;
  }
  int i;
  for(i = 0; i < 3; i++){
    digitalWrite(MUX_SEL_PINS[i], muxPins[i]);
  }
}

//IRM write sensor values into referenced data vector with N values
void sampleSensorValues(unsigned int *data, uint8_t N){

  /*
   * ==================
   * MUX Sensors Order
   * ==================
   * 
   * 3, 0, 1, 2, 4, 5
   */


   /*
   // Analog Mux used for Analog Input PINS
    
    const uint8_t SENSOR_PIN      = 32; // Analog Input Pin
    
    */

  // During Setup(): Set corresponding pin directions for MUX_Si pins and Enable MUX Inhibit
  // Disable MUX Inhibit
  // for(0,5)
    // Set channel control pin (Select) in Mux
    // Read analog pin
  // end for
  // Enable MUX Inhibit


  //IRM Enable sensors' power supply
  digitalWrite(ENABLE_SENSOR_MEASUREMENT_PIN, 1);
  
  //IRM Disable MUX's Inhibit to enable analog channel
  digitalWrite(MUX_INHIBIT_PIN, 0);

  
  for(int i = 0; i < N; i++){

    setMuxChannel(i); // IRM Set Analog Mux Input Channel
    
    //IRM Fixed nonlinearity in ESP32 ADC mearurements
    //IRM Coefficients obtained through experimentation with a calibrated power supply
    //data[i] = (unsigned int)(analogRead(ANALOG_PINS[i])*1.0337 + 203.42);
    data[i] = (unsigned int)(analogRead(SENSOR_PIN)*1.0337 + 203.42));    
    
  }

  //IRM Disable sensors' power supply
  #ifndef DEBUG
    digitalWrite(ENABLE_SENSOR_MEASUREMENT_PIN, 0);  
    digitalWrite(MUX_INHIBIT_PIN, 1);
  #endif
  
}





//IRM return battery measurement within 100% to 0% range in unsigned integer format
int batteryLife(uint8_t adcPin, uint8_t lowActivationPin){

  unsigned int batt;

  //IRM use a 1/2 voltage divider without exceeding 12 mA
  pinMode(lowActivationPin, OUTPUT);
  digitalWrite(lowActivationPin, 0); //IRM Sink battery measurement current path

  adcStart(adcPin);
  adcAttachPin(adcPin);
  
  delay(1);
  
  batt = (unsigned int)((analogRead(adcPin)*1.0337 + 203.42)); //IRM Nonlinearity fix
  #ifndef DEBUG
    pinMode(lowActivationPin, INPUT); //IRM Stop sinking battery measurement current to save power
  #endif
  

  #ifdef DEBUG
    Serial.print("Batt ADC: ");
    Serial.println(batt);
  #endif
  
  //IRM battery representation in a 0% to 100% scale (see #defines for more details)
  batt = batt>MAX_BAT_VOLTAGE?MAX_BAT_VOLTAGE:batt; //IRM Limit battery voltage to a theoretical 100%
  batt = map(batt, MIN_BAT_VOLTAGE, MAX_BAT_VOLTAGE, 0, 100); //IRM (3.7V -> 4.2V) = (0% -> 100%)

  #ifdef DEBUG
    Serial.print("Batt %: ");
    Serial.println(batt);  
  #endif
  
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
//IRM VRef routed to GPIO25 (A18)

/*
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
*/
