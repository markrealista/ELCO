//Libs for espnow e wifi
#include <esp_now.h>
#include <WiFi.h>

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

uint8_t gpios[] = {25};
int gpioCount;
int counter = 0;

float tiempo1 = 0;
float tiempo2 = 0;
float diferenciaTiempo = 0;

void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  Serial.print("Mac Address in Station: "); 
  Serial.println(WiFi.macAddress());

  SerialBT.begin("ESP32_LED"); //Name of your Bluetooth Signal
  
  gpioCount = sizeof(gpios)/sizeof(uint8_t);

  InitESPNow();

  esp_now_register_recv_cb(OnDataRecv);

  //For each gpio on gpios array
   for(int i=0; i<gpioCount; i++){
    //We put in read mode
    pinMode(gpios[i], OUTPUT);
    digitalWrite(gpios[i], LOW);
  }
}

void InitESPNow() {
  //If the initialization was successful
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  //If there was an error
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Received from: "); 
  Serial.println(macStr);
  Serial.println("");

  //For each gpio
  for(int i=0; i<gpioCount; i++){
    digitalWrite(gpios[i], data[i]);
  }
}

void loop() {
  tiempo1 = millis();
  if (digitalRead(25) == HIGH){
    diferenciaTiempo = (tiempo1 - tiempo2)/1000;
    String cuenta = String(counter);
    String buff = String(diferenciaTiempo);
    SerialBT.print(cuenta); SerialBT.print(": ");
    SerialBT.print(buff);
    SerialBT.println(" segundos");
    tiempo2 = tiempo1;
    counter++;
    digitalWrite(25, LOW);
  }
}
