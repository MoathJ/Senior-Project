#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <XBee.h>
#include <Printers.h>
#include <SoftwareSerial.h>
#include "binary.h"
#include <DHT.h>
#include <math.h>
#include <LinkedList.h>

#define ssRX 9
#define ssTX 8
#define motorPin A0
#define photosensor 3

TinyGPSPlus gps;
XBeeWithCallbacks xbee;
LinkedList<ZBTxRequest> ErrorBackets;

DHT dht(4, DHT11);
SoftwareSerial xbee_ss(ssRX, ssTX);

float rainAmount = 0;
float motorSpeed;
float Area = PI * pow(3.75, 2);
float lon = 0.000;
float lat = 0.000;
float battaryAmount = 0;

int counter = 0;

unsigned long currentTime;
unsigned long lastResetTime = 0;
unsigned long resetInterval = 24 * 60 * 60 * 1000;  // 24 hours in milliseconds

unsigned long last_tx_time = 0;
unsigned long last_tx1_time = 0;

char g = '#';

bool sendingStatues = true;

uint8_t speed = 0;
//----------------------------------------------------------------------------------------------------------
void processRxPacket(ZBRxResponse& rx, uintptr_t) {

  Buffer b(rx.getData(), rx.getDataLength());
  uint8_t type = b.remove<uint8_t>();
  if (type == 4) {
    uint8_t x = b.remove<uint8_t>();
    speed = map(x, 0, 100, 0, 255);
  }
  if (type == 5) {
    if (b.remove<uint8_t>()) sendingStatues = true;
    else sendingStatues = false;
  }
  if (type == 6) {
    if(b.remove<bool>()) speed = 255;
    else speed = 0;
  }
}
//----------------------------------------------------------------------------------------------------------
void countnum() {
  counter++;
}
//----------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  xbee_ss.begin(9600);
  xbee.setSerial(xbee_ss);
  delay(10);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(photosensor), countnum, CHANGE);
  pinMode(6, OUTPUT);
  xbee.onZBRxResponse(processRxPacket);
  dht.begin();
  sendPacket();
}
//----------------------------------------------------------------------------------------------------------
void sendPacket() {
  ZBTxRequest txRequest;
  txRequest.setAddress64(0x0000000000000000);
  txRequest.setAddress16(0x0000);

  AllocBuffer<35> packet;

  packet.append<uint8_t>(1);
  packet.append<float>(dht.readTemperature());
  packet.append<float>(dht.readHumidity());

  rainAmount = counter * 1.6667;
  rainAmount = rainAmount / Area;

  packet.append<float>(rainAmount);

  float motorVoltage = analogRead(motorPin);
  motorSpeed = map(motorVoltage, 0, 1023, 0, 100);  //  must get an equation for animometer

  packet.append<float>(motorSpeed);
  packet.append<float>(lon);
  packet.append<float>(lat);
  packet.append<char>(g);

  battaryAmount = analogRead(A2);
  // battaryAmount = battaryAmount * 3 / 2;
  battaryAmount = map(battaryAmount, 0, 1023, 0, 100);

  packet.append<float>(battaryAmount);  // Vout = Vin(R2 / (R1+R2)) ==> Vin = Vout * (R2 + R1) / R2

  txRequest.setPayload(packet.head, packet.len());

  if (sendingStatues == false) ErrorBackets.add(txRequest);
  else if (xbee.sendAndWait(txRequest, 1000) != 0) ErrorBackets.add(txRequest);
}
//------------------------------------------------------------------------------------------------------------------------
void loop() {

  xbee.loop();

  while (Serial.available() > 0) {
    g = '$';
    gps.encode(Serial.read());
    if (gps.location.isValid()) {
      g = '?';
      lon = gps.location.lng();
      lat = gps.location.lat();
      break;
    } else {
      g = '!';
    }
  }

  analogWrite(6, speed);

  currentTime = millis();
  if (currentTime - lastResetTime >= 86400000) {
    counter = 0;
    lastResetTime = currentTime;
  }

  if (millis() - last_tx_time > 10000) {
    getLocation();
    sendPacket();
    last_tx_time = millis();
  }

  if (millis() - last_tx1_time > 2000) {
    if (ErrorBackets.size() && sendingStatues)
      if (xbee.sendAndWait(ErrorBackets[0], 1000) == 0) ErrorBackets.remove(0);
    last_tx1_time = millis();
  }
}
void getLocation() {
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
    if (gps.location.isValid()) {
      lon = gps.location.lng();
      lat = gps.location.lat();
      break;
    }
  }
}