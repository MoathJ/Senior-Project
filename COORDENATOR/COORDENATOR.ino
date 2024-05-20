#include <LinkedList.h>
#include <XBee.h>
#include <Printers.h>
#include <SoftwareSerial.h>
#include "binary.h"
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <SPI.h>


#define ssRX D7
#define ssTX D6
SoftwareSerial xbee_ss(ssRX, ssTX);

const char *ssid = "AAUP-Guests";
// const char *ssid = "JAMAL";
const char *pass = "";

const char MQTT_SERVER[] = "mqtt.beebotte.com";
const char MQTT_CLIENTID[] = "";
const char MQTT_USERNAME[] = "1BBve2k2DsdqOmzAtlOBpVzi0uVH5qcW";
const char MQTT_PASSWORD[] = "";
const int MQTT_PORT = 1883;


LinkedList<XBeeAddress64> NodesList;
XBeeWithCallbacks xbee;
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_PORT,
                          MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);


float WindSpeed = NAN;
// float MaxWindSpeed = 0;
float setpoint = NAN;
bool motor_state = false;
bool inputs_changed = false;
bool sending = true;
bool WifiStates = false;
bool WifiPrevStatus = false;
bool MqttStatus = false;
bool MqttPrevStatus = false;
bool getRebort = false;

unsigned long int timer = millis();
unsigned long int interval = 10000;

const char HOUSE_SETPOINT[] = "Nodes_Statues/GetReport";
Adafruit_MQTT_Subscribe setpoint_subscription(&mqtt, HOUSE_SETPOINT);

const char CONTROLL_MOTOR[] = "13a2004213e103/Motor";
Adafruit_MQTT_Subscribe motor_subscription(&mqtt, CONTROLL_MOTOR);

const char CONTROLL_MOTOR2[] = "13a2004213e11c/Motor";
Adafruit_MQTT_Subscribe motor_subscription2(&mqtt, CONTROLL_MOTOR2);

const char CONTROLL_WindSpeedTH[] = "Thresholds/WindSpeedTH";
Adafruit_MQTT_Subscribe WindSpeedTH_subscription(&mqtt, CONTROLL_WindSpeedTH);

//------------------------------------------------------------------------------------------------------------------------------------------------------
void publish(const __FlashStringHelper *resource, float value) {

  const char *resourceStr = reinterpret_cast<const char *>(resource);
  String data;
  data += "{\"data\": ";
  data += value;
  data += ", \"write\": true}";

  Serial.print(F("Publishing "));
  Serial.print(data);
  Serial.print(F(" to "));
  Serial.println(resource);

  if (!mqtt.publish(resourceStr, data.c_str())) {
    Serial.println(F("Failed to publish, trying reconnect..."));
    connect();
  }
}

//------------------------------------------------------------------------------------------------------------------------------------
void publishStatues(const __FlashStringHelper *resource, String value) {

  const char *resourceStr = reinterpret_cast<const char *>(resource);
  String data;
  data += "{\"data\": ";
  data += "\"";
  data += value;
  data += "\"";
  data += ", \"write\": true}";

  Serial.print(F("Publishing "));
  Serial.print(data);
  Serial.print(F(" to "));
  Serial.println(resource);

  if (!mqtt.publish(resourceStr, data.c_str())) {
    Serial.println(F("Failed to publish, trying reconnect..."));
    connect();
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------
void publishLocation(const __FlashStringHelper *resource, float lon, float lat) {

  const char *resourceStr = reinterpret_cast<const char *>(resource);
  char message[50];
  snprintf(message, 50, "{\"latitude\":%f,\"longitude\":%f}", lat, lon);

  String data;
  data += "{\"data\": ";
  data += message;
  data += ", \"write\": true}";

  Serial.print(F("Publishing "));
  Serial.print(data);
  Serial.print(F(" to "));
  Serial.println(resource);

  if (!mqtt.publish(resourceStr, data.c_str())) {
    Serial.println(F("Failed to publish, trying reconnect..."));
    connect();
  }
}
//------------------------------------------------------------------------------------------------------------------------
int indexOf(XBeeAddress64 addr) {
  int index = -1;
  for (int i = 0; i < NodesList.size(); i++) {
    if (NodesList.get(i) == addr) {
      index = i;
      break;
    }
  }

  return index;
}

//------------------------------------------------------------------------------------------------------------------------
bool decideMotorState() {

  float corrected = motor_state ? (setpoint + 0.5) : setpoint;

  // if (corrected < MaxWindSpeed)
  if (corrected < WindSpeed)
    return true;

  // There is no Node above the threshold
  return false;
}
//------------------------------------------------------------------------------------------------------------------------
void ToggleMotors(bool state) {
  Serial.print(F("Motors should be "));
  Serial.println(state ? F("on") : F("off"));
  motor_state = state;
  ZBTxRequest txRequest;
  txRequest.setAddress64(0x000000000000FFFF);
  txRequest.setAddress16(0xFFFE);
  AllocBuffer<2> checkpacket;
  checkpacket.append<uint8_t>(6);
  checkpacket.append<bool>(state);
  txRequest.setPayload(checkpacket.head, checkpacket.len());
  xbee.send(txRequest);
}
//------------------------------------------------------------------------------------------------------------------------

void processRxPacket(ZBRxResponse &rx, uintptr_t) {
  
  XBeeAddress64 addr = rx.getRemoteAddress64();
  if(!sending) RecivingStatusforNode(false, addr );
  int index = indexOf(addr);
  String Node = String(addr, HEX);
  Buffer b(rx.getData(), rx.getDataLength());
  uint8_t type = b.remove<uint8_t>();

  if (index == -1) {
    NodesList.add(addr);
  }

  if (type == 1) {
    publish(FPSTR((Node + "/Temperature").c_str()), b.remove<float>());
    publish(FPSTR((Node + "/Humidity").c_str()), b.remove<float>());
    publish(FPSTR((Node + "/RainFall").c_str()), b.remove<float>());
    WindSpeed = b.remove<float>();
    // if (MaxWindSpeed < WindSpeed)
      // MaxWindSpeed = WindSpeed;
    inputs_changed = true;
    publish(FPSTR((Node + "/WindSpeed").c_str()), WindSpeed);
    publishLocation(FPSTR((Node + "/GPS").c_str()), b.remove<float>(), b.remove<float>());
    publish(FPSTR((Node + "/Location").c_str()), b.remove<char>());
    publish(FPSTR((Node + "/Battary").c_str()), b.remove<float>());
    return;
  }
  Serial.println(F("Unknown or invalid packet"));
  printResponse(rx, Serial);
}
//------------------------------------------------------------------------------------------------------------------------
void connect() {
  client.stop();  // Ensure any old connection is closed
  uint8_t ret = mqtt.connect();
  if (ret == 0) {
    Serial.println(F("MQTT connected"));
    MqttStatus = true;
  } else {
    Serial.println(mqtt.connectErrorString(ret));
    MqttStatus = false;
  }
}
//------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600); 
  xbee_ss.begin(9600);
  xbee.setSerial(xbee_ss); 

  delay(10);
  Serial.println(F("Starting..."));
  delay(10);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  WifiPrevStatus = true;
  mqtt.subscribe(&setpoint_subscription);
  mqtt.subscribe(&motor_subscription);
  mqtt.subscribe(&motor_subscription2);
  mqtt.subscribe(&WindSpeedTH_subscription);
  connect();
  MqttPrevStatus = MqttStatus;

  xbee.onPacketError(printErrorCb, (uintptr_t)(Print *)&Serial);
  xbee.onResponse(printErrorCb, (uintptr_t)(Print *)&Serial);
  xbee.onZBRxResponse(processRxPacket);
}

//------------------------------------------------------------------------------------------------------------------------

void loop() {

  //------//----------//--------//---------//----------//---------//----------//----------//-----------//------------//--------//-------
  if (WiFi.status() != WL_CONNECTED) WifiStates = false;
  else WifiStates = true;

  if (WifiStates == !WifiPrevStatus) {
    Serial.println("Somthing Happend to The Wifi");
    if (!WifiStates) {
      Serial.println("The Wi-Fi is not Connected.");
      Serial.println("Stop Sending.");
      RecivingStatus(false);
      sending = false;
    }
    if (WifiStates) {
      Serial.println("The Wi-Fi is Connected.");
      connect();
      if (WifiStates && MqttStatus && !sending) {
      Serial.println("Start To Send.");
      RecivingStatus(true);
      sending = true;
    }
    MqttPrevStatus = MqttStatus;
    }
  }
  if (!MqttStatus) {
    sending = false;
    connect();
    Serial.println("Mqtt is not Connected");
  }

  if(!MqttStatus && sending) RecivingStatus(false);

  if (MqttStatus == !MqttPrevStatus) {
    if (MqttStatus) {
      RecivingStatus(true);
      sending = true;
      Serial.println("Mqtt is Connected");
    } else {
      RecivingStatus(false);
      sending = false;
      Serial.println("Mqtt is not Connected");
    }
  }

  MqttPrevStatus = MqttStatus;
  WifiPrevStatus = WifiStates;
  //------//----------//--------//---------//----------//---------//----------//----------//-----------//------------//--------//-------

  Adafruit_MQTT_Subscribe *subscription = mqtt.readSubscription(0);
  if (subscription)
    CheckSub(subscription);
  if (inputs_changed) {
    Serial.println("The input Changed");
    if (decideMotorState() != motor_state)
      ToggleMotors(!motor_state);
    inputs_changed = false;
  }

  xbee.loop();
}
//-----------------------------------------------------------------------------------------------------------------------------
void CheckModules() {
  ZBTxRequest txRequest;
  AllocBuffer<1> checkpacket;
  checkpacket.append<uint8_t>(3);
  txRequest.setPayload(checkpacket.head, checkpacket.len());
  for (int i = 0; i < NodesList.size(); i++) {
    Serial.println(String(NodesList[i], HEX));
    txRequest.setAddress64(NodesList[i]);
    uint8_t deliveryStatus = xbee.sendAndWait(txRequest, 1000);
    publishStatues(F("Nodes_Statues/Node"), String(NodesList[i], HEX));
    if (deliveryStatus == XBEE_WAIT_TIMEOUT) {
      Serial.println("Time Out the Node is Not Connected");
      publishStatues(F("Nodes_Statues/Statues"), "Not Connected");
    } else {
      Serial.print("Delivery Status: 0x");
      Serial.println(deliveryStatus, HEX);
      publishStatues(F("Nodes_Statues/Statues"), "Connected");
    }
  }
}
//------------------------------------------------------------------------------------------------------------------------
bool getBoolValue(Adafruit_MQTT_Subscribe *subscription) {
  String str((const char *)subscription->lastread);
  const char *PREFIX = "{\"data\":";
  // Check for the prefix
  if (!str.startsWith(PREFIX)) {
    Serial.print(F("Unsupported value received: "));
    Serial.println(str);
    return false;
  }
  // Remove the prefix
  str.remove(0, strlen(PREFIX));
  if (str[0] == 'f')
    return false;
  else
    return true;
}
//------------------------------------------------------------------------------------------------------------------------
int getValue(Adafruit_MQTT_Subscribe *subscription) {
  String str((const char *)subscription->lastread);
  const char *PREFIX = "{\"data\":";
  // Check for the prefix
  if (!str.startsWith(PREFIX)) {
    Serial.print(F("Unsupported value received: "));
    Serial.println(str);
    return 0;
  }
  // Remove the prefix
  str.remove(0, strlen(PREFIX));
  // Convert the rest into int (as much as possible)
  return str.toInt();
}
//------------------------------------------------------------------------------------------------------------------------
float getFloatValue(Adafruit_MQTT_Subscribe *subscription) {
  String str((const char *)subscription->lastread);
  const char *PREFIX = "{\"data\":";
  // Check for the prefix
  if (!str.startsWith(PREFIX)) {
    Serial.print(F("Unsupported value received: "));
    Serial.println(str);
    return NAN;
  }
  // Remove the prefix
  str.remove(0, strlen(PREFIX));
  return str.toFloat();
}

//------------------------------------------------------------------------------------------------------------------------
void WriteToMotor(uint8_t value, XBeeAddress64 address) {
  ZBTxRequest txRequest;
  txRequest.setAddress64(address);
  AllocBuffer<2> speedPacket;
  speedPacket.append<uint8_t>(4);
  Serial.println(value);
  speedPacket.append<uint8_t>(value);
  txRequest.setPayload(speedPacket.head, speedPacket.len());
  xbee.send(txRequest);
}
//------------------------------------------------------------------------------------------------------------------------
void CheckSub(Adafruit_MQTT_Subscribe *subscription) {

  if (subscription == &setpoint_subscription) {
    bool value = getBoolValue(subscription);
    getRebort = value;
    if (getRebort == true)
      CheckModules();
  }
  if (subscription == &motor_subscription) {
    uint8_t value = getValue(subscription);
    Serial.println(value);
    XBeeAddress64 address = 0x0013A2004213E103;
    WriteToMotor(value, address);
  }

  if (subscription == &motor_subscription2) {
    uint8_t value = getValue(subscription);
    XBeeAddress64 address = 0x0013A2004213E11C;
    Serial.println(value);
    WriteToMotor(value, address);
  }

  if (subscription == &WindSpeedTH_subscription) {
    Serial.print("A value of Th is recived : ");
    float value = getFloatValue(subscription);
    Serial.println(value);
    setpoint = value;
    inputs_changed = true;
  }
}
//------------------------------------------------------------------------------------------------------------------------
void RecivingStatus(bool status) {
  Serial.print("The sending status is: ");
  Serial.println(status);
  ZBTxRequest txRequest;
  txRequest.setAddress64(0x000000000000FFFF);
  txRequest.setAddress16(0xFFFE);
  AllocBuffer<2> checkpacket;
  checkpacket.append<uint8_t>(5);
  if (status) checkpacket.append<uint8_t>(1);
  else checkpacket.append<uint8_t>(0);
  txRequest.setPayload(checkpacket.head, checkpacket.len());
  xbee.send(txRequest);
}
//------------------------------------------------------------------------------------------------------------------------
void RecivingStatusforNode(bool status, XBeeAddress64 addr) {
  Serial.print("The sending status is: ");
  Serial.println(status);
  ZBTxRequest txRequest;
  txRequest.setAddress64(addr);
  AllocBuffer<2> checkpacket;
  checkpacket.append<uint8_t>(5);
  if (status) checkpacket.append<uint8_t>(1);
  else checkpacket.append<uint8_t>(0);
  txRequest.setPayload(checkpacket.head, checkpacket.len());
  xbee.send(txRequest);
}
//------------------------------------------------------------------------------------------------------------------------
void AboveThreshold() {
  Serial.println("Sending To Nodes To turn the Motors on.");
  ZBTxRequest txRequest;
  txRequest.setAddress64(0x000000000000FFFF);
  txRequest.setAddress16(0xFFFE);
  AllocBuffer<1> checkpacket;
  checkpacket.append<uint8_t>(6);
  txRequest.setPayload(checkpacket.head, checkpacket.len());
  xbee.send(txRequest);
}