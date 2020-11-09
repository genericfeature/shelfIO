#define BLYNK_PRINT Serial

#include <Arduino.h>
#include "secrets.h" //AWS IoT Core
#include "HX711.h"
#include "BluetoothSerial.h"
#include "soc/rtc.h" //lib um CPU clock zu ändern
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h> //AWS IoT Core
#include <BlynkSimpleEsp32.h>
// #include <PubSubClient.h> //MQTT Client --> Node Red Aedes
#include <MQTTClient.h> //AWS IoT Core
#include <ArduinoJson.h> //AWS IoT Core

// HX711 circuit wiring
byte LOADCELL_DOUT_PIN = 22;
byte LOADCELL_SCK_PIN = 23;
byte LED_PIN = 2;

// Menu Commands
int serialcommand;
const byte buffSize = 31;
char userCommand[buffSize];
const unsigned long questionInterval = 5000;
unsigned long prevResponseMillis = 0;
unsigned long currentMillis = 0;
boolean waitingForCommand = false;
const char endMarker = '\r'; // serial connection end line marker
byte bytesRcvd = 0;

// Bluetooth
BluetoothSerial bluetooth;

// Scale parameters
HX711 scale;
float reading;
float offset; //Offset, Gewichtswert Nullpunkt
float calibration1_value; //Gewichtswert Justagepunkt 1
float displayweight; // Anzeigewert

char material[64] = "unspecified";
bool setpoint0;          // Justagegewicht 0 gültig; Blynk Taster
float setpoint1_val;      // Digitwert Justagepunkt 1 Blynk Übergabe
float setpoint1_dig;     // gemerkter Digitwert Justagepunkt 1
bool setpoint1_set;     // Justagegewicht 1 gültig
bool startset;          // calibration mode on/off

//Blynk credentials
char auth[] = "PCBz_5uKp0O-Qob5fB5c2_NxopWAm6ln";
//Wifi credentials
char ssid[] = "DO_NOT_ENTER";
char wifipass[] = "m00dd4f1gg4";

//Blynk; get data from app to device
/*
V1...Material in Scale
V2...Justagegewicht 0 setzen
V3...Gewichtswert Justagegewicht 1
V4...Justagegewicht 1 setzen
V5...Justagemodus aktiv/inaktiv
*/

// Blynk Zuordnung der virtuellen IOs
BLYNK_WRITE(V1) {
  strncpy(material, param.asStr(), sizeof(material));
  material[sizeof(material)-1] = 0;
}
BLYNK_WRITE(V2) {
  setpoint0 = param.asInt();
}
BLYNK_WRITE(V3) {
  setpoint1_val = param.asInt();
}
BLYNK_WRITE(V4) {
  setpoint1_set = param.asInt();
}
BLYNK_WRITE(V5) {
  startset = param.asInt();  
}
WidgetTerminal terminal(V99); // Blynk Serielles Terminal

/* MQTT PubSubClient -- das war der Node Red Aedes Teil
const char* mqtt_server = "192.168.178.187";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;*/

// AWS IoT Core
// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC   "protoScale/values"
#define AWS_IOT_SUBSCRIBE_TOPIC "protoScale/cfg"
WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

/* MQTT AEDES
// Verbindung Wifi herstellen
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, wifipass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
*/

/* Das ist der MQTT nach Node Red Teil von https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
// MQTT subsribe, function 
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  // Feel free to add more if statements to control more GPIOs with MQTT
  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  /*
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("MQTT client connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}*/

void setup() {
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M); //CPU clock auf 80MHz, HX711 verträgt höhere clk nicht
  Serial.begin(57600);
  setup_wifi(); // call WLAN connection function
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  offset = 5000; 
  calibration1_value = 10;
  setpoint1_dig = 15000;
  Blynk.begin(auth, ssid, wifipass);
  pinMode(LED_PIN, OUTPUT);
  terminal.clear(); // empties Blynk Terminal
  terminal.println("IOT Scale v0.00");
  /* MQTT AEDES
  client.setServer(mqtt_server, 1883); //MQTT server an client übergeben; AEDES
  client.setCallback(callback); //??
  */
}

void loop() {
  /* MQTT AEDES
  if (!client.connected()) {
    Serial.print("MQTT client not connected -- reconnecting");
    reconnect();
  }
  client.loop();*/
  Blynk.run(); // Blynk starten
  Serial.println("Blynk run durchlaufen");
  currentMillis = millis(); // Aktuellen Zeitstempel in ms 
  terminal.println(startset);
  Serial.println("openScale in operation");
  delay(1000);
  // Justagepunkt 1 setzen
  if (scale.is_ready() && startset == 1 && setpoint1_set == 1)  {
    digitalWrite(LED_PIN, HIGH);
    delay(125);
    digitalWrite(LED_PIN, LOW);
    delay(125);
    setpoint1_dig = scale.read(); //Digitwerte Justagegewicht 1 lesen
    calibration1_value = setpoint1_val; //Gewichtswert Justagegewicht 1 lesen
    digitalWrite(LED_PIN, HIGH);
    delay(125);
    digitalWrite(LED_PIN, LOW);
    delay(125);
    terminal.print("Set point     | ");
    terminal.println(calibration1_value);
  }
  
  // Justagegewicht 0 setzen
  if (scale.is_ready() && startset == 1 && setpoint0 == 1) {
    digitalWrite(LED_PIN, HIGH);
    delay(125);
    offset = scale.read(); //Offset lesen
    digitalWrite(LED_PIN, LOW);
    delay(125);
    terminal.print("Offset        | ");
    terminal.println(offset);
    delay(125);
  }

  // Operation
  if (scale.is_ready()) {
    reading = scale.read();
    Serial.println("raw value HX711: ");
    Serial.println(reading);
    displayweight = ((reading - offset)/(setpoint1_dig - offset) * calibration1_value);
    Blynk.virtualWrite(V0, displayweight);
    char displayvalue[8];
    /* MQTT AEDES
    dtostrf(displayweight, 1, 2, displayvalue);
    client.publish("esp32/display", displayvalue);
    */
    Serial.print("processed value: ");
    Serial.println(displayvalue);
    Serial.println("------------------------------");
  }
}