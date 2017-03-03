#include <Arduino.h>
#include <SoftwareSerial.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

// Factory Reset. Define it if reset is needed.
// #define FACTORY_RESET 1

// Pin Settings
#define RESETPIN  2
#define SENSORPIN 4
#define READYPIN  6
#define TONEPIN   7
#define LEDPIN    13

// Piezo Settings
#define PITCH 1250
#define PITCH_DURATION_MS 50

// BLE Events
#define BLE_EVENT_NONE     0
#define BLE_EVENT_RESET    1
#define BLE_EVENT_COUNT_UP 2

// BlueFruit LED Mode Settings
#define MODE_LED_BEHAVIOUR          "MODE"

// Sensor States
int sensorState = 0, lastState=0;

// Reset Button States
int resetState = 0, lastResetState = 0;

int bleEvent = BLE_EVENT_NONE;

SoftwareSerial bluefruitSS =
  SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                              BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

void setup() {
  pinMode(LEDPIN, OUTPUT);
  pinMode(READYPIN, OUTPUT);  
  pinMode(SENSORPIN, INPUT);
  
  digitalWrite(READYPIN, LOW);
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
  
  while (!Serial) {}
  delay(500);
  Serial.begin(115200);

  if (!ble.begin(VERBOSE_MODE))
  {
    Serial.println("Couldn't find Bluefruit!");
    while(1);
  }

#ifdef FACTORY_RESET
  if (!ble.factoryReset())
  {
    Serial.println("Couldn't factory reset");
    while(1);
  }
#endif

  // System ready to be connected!
  ble.echo(false);
  ble.info();
  ble.verbose(false);
  digitalWrite(READYPIN, HIGH); 

  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println("Connected");
  ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
}

void playTone() {
  tone (TONEPIN, PITCH, PITCH_DURATION_MS);
  delay(PITCH_DURATION_MS/2 );
}


void bleSendEvent(int event) {
  ble.print("AT+BLEUARTTX=");
  ble.println(event);
  if (!ble.waitForOK() ) {
    Serial.println("Failed to send?");
  }
  Serial.println(event);
}

void bleSendReset() {
  bleSendEvent(BLE_EVENT_RESET);
}

void bleSendCountUp() {
  bleSendEvent(BLE_EVENT_COUNT_UP);
}
  
void loop(){
  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);
  resetState = digitalRead(RESETPIN);
  
  if (resetState == HIGH && lastResetState == LOW) {
    Serial.println("Reset");
    bleSendReset();
  }

  digitalWrite(LEDPIN, sensorState == LOW ? HIGH : LOW);  
  
  if (sensorState == LOW && lastState == HIGH) {
    bleSendCountUp();
    playTone(); 
  }
  
  lastState = sensorState;
  lastResetState = resetState;
}
