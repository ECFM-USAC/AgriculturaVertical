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

//#define DEBUG 1

#define BUTTON KEY_BUILTIN

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

#define EEPROM_SIZE 64

#define LED_BLINK_ON 25    //IRM LED On period between blink cycles
#define LED_BLINK_OFF 400  //IRM LED Off period between blink cycles

#define LONG_PRESS_MS 2000 //IRM Min time to determine whether the button press event was long or short
#define SHORT_PRESS_MS 20 //IRM Period between button-press checking try events
#define BUTTON_CHECK_TIMES 3 //IRM How many times the button must remain pressed to trigger a button-pressed event
#define PRESSED_STATE 0 //IRM Inverted-logic

#define NODE_ID_ADDR 0 //IRM Address where NodeID will be stored/retreived from
#define MAX_NODES 10

#define SECONDS 1000 //IRM Milisenconds to Seconds conversion

#define SAMPLING_RATE_REQUEST_RETRY_PERIOD 10*SECONDS //IRM Retry requesting sampling period after X seconds

#define DATA_TOPIC_PUBLISH "SensorNetwork/Nodes"
#define HELP_TOPIC_PUBLISH DATA_TOPIC_PUBLISH

#define SAMPLING_TOPIC_SUBSCRIBE "SensorNetwork/SamplingRate"

#define HELP_TOKEN "HELP!" //IRM Request Sampling Rate to the server

// Update these with values suitable for your network.

const char* ssid = "Agricultura-Vertical";
const char* password = "eco>LITE";
const char* mqtt_server = "192.168.0.100";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

bool samplingReceived = false;

byte nodeID = 0;

//IRM Custom function prototypes
void requestSamplingPeriod(void);
void showNodeID(void);
void setNodeID(void);
byte retrieveNodeIDFromEEPROM(void);
bool buttonPressed(unsigned int tries);
bool writeNodeIDToEEPROM(byte id);
void incrementNodeID(void);

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)

    samplingReceived = true; //IRM Go to deep sleep only if required token arrives
    delay(20);
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage HIGH
  }
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
  Serial.begin(115200);

  digitalWrite(LED_BUILTIN, 1);
  delay(3000);
  digitalWrite(LED_BUILTIN, 0);

  if(buttonPressed(BUTTON_CHECK_TIMES)){
    setNodeID();
    esp_sleep_enable_timer_wakeup(1 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }else{

    delay(1000);
    
    nodeID = retrieveNodeIDFromEEPROM();
    showNodeID();
  }

  
  setup_wifi();

  Serial.print("Node ID: ");
  Serial.println(nodeID);
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 200) {
    lastMsg = now;
    ++value;
    snprintf (msg, 75, "hw #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(DATA_TOPIC_PUBLISH, msg);
  }

  Serial.println("Waiting for server response");
  requestSamplingPeriod();
  Serial.println("Going to deep-sleep now");

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
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







