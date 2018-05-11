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


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

#define SECONDS 1000 //IRM Milisenconds to Seconds conversion

#define SAMPLING_RATE_REQUEST_RETRY_PERIOD 10*SECONDS //IRM Retry requesting sampling period after X seconds

#define DATA_TOPIC_PUBLISH "SensorNetwork/Nodes"
#define HELP_TOPIC_PUBLISH DATA_TOPIC_PUBLISH

#define SAMPLING_TOPIC_SUBSCRIBE "SensorNetwork/SamplingRate"

#define HELP_TOKEN "HELP!" //IRM Request Sampling Rate to the server

// Update these with values suitable for your network.

const char* ssid = "ECFM-Instrumentacion";
const char* password = "ChicolazoPositivo";
const char* mqtt_server = "192.168.126.113";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

bool samplingReceived = false;


//IRM Custom function prototypes
void requestSamplingPeriod(void);

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
  // Loop until we're reconnected
  while (!client.connected()) {
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
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
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

