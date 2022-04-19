#include <HX711_ADC.h>
#include "WiFi.h"
#include "String.h"
#include <PubSubClient.h>

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell(21, 22);

// ---- WiFi Login Details ----
const char* ssid = "SSID HERE";
const char* password = "PASSWORD HERE";

// ----- MQTT Broker Setup -----
PubSubClient client;
const char *mqtt_broker = "test.mosquitto.org";
const char *topic = "esp32/test";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;

long t;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  Serial.println("Wait...");
  initWiFi();  //Connect to WiFi
  LoadCell.begin();  // Start Reading From HX711 Strain Gauge Amplifier
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  LoadCell.setCalFactor(10000.0); // user set calibration factor (float)
  Serial.println("Startup + tare is complete");
}

// Publish Message to MQTT Broker
void publishMessage() {
  client.setServer(mqtt_broker, mqtt_port);
 // client.setCallback(callback);
 // bool connection_established = client.connect("myESP32_1234");
  //if (connection_established) {
  //  Serial.println("Connected to MQTT Broker");
 // }
 // else {
 //   Serial.println("Connection Failed");
//  }
 // boolean rc = client.publish("test", "test message");
}



void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  delay(2000);
}

void loop() {

  //update() should be called at least as often as HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS
  //longer delay in scetch will reduce effective sample rate (be carefull with delay() in loop)
  LoadCell.update();

  //get smoothed value from data set + current calibration factor
  if (millis() > t + 250) {
    float i = fabs(LoadCell.getData());
    float v = LoadCell.getCalFactor();
    //  Serial.print("Load_cell output val: ");
    Serial.println(i);
    publishMessage();
    // Serial.print("      Load_cell calFactor: ");
    //  Serial.println(v);

    // Create a string which is the integer value of the weight times 10,
    //  to remove the decimal point.
    String weight = String(int(i * 10));
    Serial2.write(0x76); // Clear the display
    Serial2.print(weight); // Write out the weight value to the display

    // Identify which decimal point to set, and set it.
    int shiftBy = 5 - weight.length();
    int decimalPoint = 0x08 >> (shiftBy);
    Serial2.write(0x77);
    Serial2.write(decimalPoint & 0x0F);

    t = millis();
  }

  //receive from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 'l') i = -1.0;
    else if (inByte == 'L') i = -10.0;
    else if (inByte == 'h') i = 1.0;
    else if (inByte == 'H') i = 10.0;
    else if (inByte == 't') LoadCell.tareNoDelay();
    if (i != 't') {
      float v = LoadCell.getCalFactor() + i;
      LoadCell.setCalFactor(v);
    }
  }

  //check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}
