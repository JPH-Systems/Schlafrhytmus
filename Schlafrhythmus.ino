#include "CurieIMU.h"
#include "CurieTimerOne.h"
#include "CurieBLE.h"

BLEPeripheral blePeripheral;
BLEService axis_read_Service("A815");
BLEUnsignedIntCharacteristic axisInt("A8150", BLERead | BLENotify);
BLEIntCharacteristic readInt("A8151", BLEWrite);



const byte ledPin = 13; //onboard, zeigt ob Programm laueft
const byte redLED = 8; //zeigt Kalibrierung an
const byte greenLED = 12; //zeigt ob Bewegungen stattfinden
const byte interruptPin_sub = 10; //Knopf um fuer negative kal
const byte interruptPin_add = 6; //Knopf um fuer positive kal
const byte interruptPin_kal = 4; //Knopf um fuer kal

int axisPos = -1;
int time_xp = -1;
int time_xn = -1;
int time_yp = -1;
int time_yn = -1;
int time_zp = -1;
int time_zn = -1;
int last_read = 0;
int last_kal_read = 0;
int auto_kal_wert = 1500;

int axis_count = 0;
int count_xp = 0;
int count_xn = 0;
int count_yp = 0;
int count_yn = 0;
int count_zp = 0;
int count_zn = 0;


boolean kal_running = false;

volatile byte state = LOW;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;
int ledState = LOW; //state zum Blinken

//aufleuchten für 2. LED
unsigned long previousMillis_2 = 0;
const long interval_2 = 500;


void setup() {
  Serial.begin(9600); // initialize Serial communication
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventShock);
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 5000); // 1.5g = 1500 mg
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50);   // 50ms
  CurieIMU.interrupts(CURIE_IMU_SHOCK);

  blePeripheral.setLocalName("Schlafrhytmus 101");
  blePeripheral.setAdvertisedServiceUuid(axis_read_Service.uuid());
  blePeripheral.addAttribute(axis_read_Service);
  blePeripheral.addAttribute(axisInt);
  blePeripheral.addAttribute(readInt);
  readInt.setEventHandler(BLEWritten, bleread);
  readInt.setValue(0);
  axisInt.setValue(axis_count);
  blePeripheral.begin();

  pinMode(ledPin, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(interruptPin_sub, INPUT_PULLUP);
  pinMode(interruptPin_add, INPUT_PULLUP);
  attachInterrupt(interruptPin_sub, button_sub, FALLING);
  attachInterrupt(interruptPin_add, button_add, FALLING);
  attachInterrupt(interruptPin_kal, auto_kal, FALLING);
  digitalWrite(ledPin, HIGH);
  auto_kal();

}

void bleread(BLECentral& central, BLECharacteristic& characteristic) {
  if (readInt.value() == 1) {
    readInt.setValue(0);
    axis_count = 0;
    axisInt.setValue(axis_count);

  }
}

void auto_kal() {
  Serial.println("Auto Kal laeuft!");
  auto_kal_wert = 2000;
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, auto_kal_wert);
  CurieTimerOne.start(5000, &autokal);
  kal_running = true;

}

void button_sub() {
  Serial.println("Button Sub");
  auto_kal_wert -= 1;
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, auto_kal_wert);
}

void button_add() {
  Serial.println("Button Add");
  auto_kal_wert += 1;
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, auto_kal_wert);
}

void autokal() {
  digitalWrite(redLED, state);
  if (time_xp > last_kal_read || time_yp > last_kal_read || time_zp > last_kal_read || time_xn > last_kal_read || time_yn > last_kal_read || time_zn > last_kal_read) {
    auto_kal_wert += 10;
    CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, auto_kal_wert);
    Serial.println("Detection Wert:");
    Serial.println(auto_kal_wert);
    kal_running = false;
    state = LOW;
    digitalWrite(redLED, state);
    CurieTimerOne.kill();
  }
  else {
    auto_kal_wert--;
    CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, auto_kal_wert);
    Serial.println(auto_kal_wert);
    last_read = millis();
  }

}



void eventShock() {
  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
    if (CurieIMU.shockDetected(X_AXIS, POSITIVE)) {
      axisPos = 1;
      count_xp++;
      axis_count++;
      axisInt.setValue(axis_count);
      time_xp = millis();
      axisInt.setValue(axisPos);
      Serial.println(axisPos);
    }
    if (CurieIMU.shockDetected(X_AXIS, NEGATIVE)) {
      axisPos = 2;
      count_xn++;
      axis_count++;
      axisInt.setValue(axis_count);
      time_xn = millis();
      axisInt.setValue(axisPos);
      Serial.println(axisPos);
    }
    if (CurieIMU.shockDetected(Y_AXIS, POSITIVE)) {
      axisPos = 3;
      count_yp++;
      axis_count++;
      axisInt.setValue(axis_count);
      time_yp = millis();
      axisInt.setValue(axisPos);
      Serial.println(axisPos);
    }
    if (CurieIMU.shockDetected(Y_AXIS, NEGATIVE)) {
      axisPos = 4;
      count_yn++;
      axis_count++;
      axisInt.setValue(axis_count);
      time_yn = millis();
      axisInt.setValue(axisPos);
      Serial.println(axisPos);
    }
    if (CurieIMU.shockDetected(Z_AXIS, POSITIVE)) {
      axisPos = 5;
      count_zp++;
      axis_count++;
      axisInt.setValue(axis_count);
      time_zp = millis();
      axisInt.setValue(axisPos);
      Serial.println(axisPos);
    }
    if (CurieIMU.shockDetected(Z_AXIS, NEGATIVE)) {
      axisPos = 6;
      count_zn++;
      axis_count++;
      axisInt.setValue(axis_count);
      time_zn = millis();
      axisInt.setValue(axisPos);
      Serial.println(axisPos);
    }
  }
}

void loop() { //waehrend der laufzeit wird dies immer wieder durchgefuehrt

  BLECentral central = blePeripheral.central();

  if (central) {
    digitalWrite(redLED, HIGH);
    Serial.print("BT Device Verbunden: ");
    Serial.println(central.address());

    // while the central is still connected to peripheral:

    while (central.connected()) {
      
      }
       

    // when the central disconnects, print it out:
    digitalWrite(redLED, LOW);
    Serial.print(F("BT Device getrennt: "));
    Serial.println(central.address());
  }


unsigned long currentMillis_2 = millis();
if (time_xp > last_read || time_yp > last_read || time_zp > last_read || time_xn > last_read || time_yn > last_read || time_zn > last_read) {
  digitalWrite(greenLED, HIGH);
  last_read = millis();
}
if (currentMillis_2 - previousMillis_2 >= interval_2) {
  previousMillis_2 = currentMillis_2;
  digitalWrite(greenLED, LOW);
}

unsigned long currentMillis = millis();
if (kal_running == true) {
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
  }

  // set the LED with the ledState of the variable:
  digitalWrite(redLED, ledState);
}
}


