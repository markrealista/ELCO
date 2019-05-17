//Libs for espnow e wifi
#include <esp_now.h>
#include <WiFi.h>
#define CHANNEL 1

//LEDS
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define LED_PIN 13
#define LED_COUNT 12
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint8_t gpios[] = {25};
int gpioCount;

long randNumber; 
int led = 0; //Led inicialmente encendido

//Variables para el altavoz
int freq = 2000;
int channel = 0;
int resolution = 8;

uint8_t macSlaves[][6] = {
  //Envio a direcciones especificas. Poner las MACs asociadas a los esp32. Por ejemplo:
  //{0x3C, 0x71, 0xBF, 0xA9, 0x45, 0x3C},
  //{0x3C, 0x71, 0xBF, 0xA9, 0x43, 0xF0},
  //{0x3C, 0x71, 0xBF, 0xA9, 0x47, 0x08}
};

//Inicializacion del protocolo
void InitESPNow() {
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed"); 
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("Mac Address in Station: "); 
  Serial.println(WiFi.macAddress());
  
  InitESPNow();

  //Altavoz
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(12, channel);

  //Leds
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  gpioCount = sizeof(gpios)/sizeof(uint8_t);
  int slavesCount = sizeof(macSlaves)/6/sizeof(uint8_t);
  
  //For each Slave
  for(int i=0; i<slavesCount; i++){
    esp_now_peer_info_t slave;
    slave.channel = CHANNEL;
    slave.encrypt = 0;
    memcpy(slave.peer_addr, macSlaves[i], sizeof(macSlaves[i]));
    esp_now_add_peer(&slave);
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  //For each GPIO pin in array
  for(int i=0; i<gpioCount; i++){
    //We put in read mode
    pinMode(gpios[i], OUTPUT);
    digitalWrite(gpios[i], HIGH); //Este modulo se inicializa con el led encendido
    colorWipe(strip.Color(0, 0, 0), 1); // Red
  }
}

//Function that will read the gpios and send
//the read values to the others ESPs
void send(){
  uint8_t values[gpioCount];

  for(int i=0; i<gpioCount; i++){
    digitalWrite(values[i], HIGH);
  }

  uint8_t macAddr1[] = {0x3C, 0x71, 0xBF, 0xA9, 0x45, 0x3C};
  uint8_t macAddr2[] = {0x3C, 0x71, 0xBF, 0xA9, 0x43, 0xF0};
  uint8_t macAddr3[] = {0x3C, 0x71, 0xBF, 0xA9, 0x47, 0x08};

  //Siempre se envia la información a esta direccion MAC
  esp_err_t result1 = esp_now_send(macAddr3, (uint8_t*) &values, sizeof(values));
  Serial.print("Send Status: ");
  if (result1 == ESP_OK) {Serial.println("Success");}
  else {Serial.println("Error");}
  
  randNumber = random(1, 3); //numero aleatorio del 1 al 2
  if (randNumber == 1) {
    esp_err_t result = esp_now_send(macAddr1, (uint8_t*) &values, sizeof(values));
    Serial.print("Send Status: ");
    if (result == ESP_OK) {Serial.println("Success");}
    else {Serial.println("Error");} 
    
  } else {
    esp_err_t result = esp_now_send(macAddr2, (uint8_t*) &values, sizeof(values));
    Serial.print("Send Status: ");
    if (result == ESP_OK) {Serial.println("Success");}
    else {Serial.println("Error");}
  }
}

//Callback function that gives us feedback about the sent data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  //Copies the receiver Mac Address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Prints it on Serial Monitor
  Serial.print("Sent to: "); 
  Serial.println(macStr);
  //Prints if it was successful or not
  Serial.print("Status: "); 
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

//Callback function that tells us when data from Master is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  //Copies the sender Mac Address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Prints it on Serial Monitor
  Serial.print("Received from: "); 
  Serial.println(macStr);
  Serial.println("");

  //For each gpio
  for(int i=0; i<gpioCount; i++){
    digitalWrite(gpios[i], data[i]);
  }

  led = 0;
} 

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void loop() {
  int sharp = analogRead(37);
  Serial.println(sharp);
  
  if (digitalRead(25) == HIGH){
    if (led == 0) {
      colorWipe(strip.Color(255, 0, 0), 0); // Red
      led = 1;
      
      ledcWriteTone(channel, 900);
      ledcWrite(channel, 125);
      delay(500);
      ledcWriteTone(channel, 0);
    }
    if (sharp > 3800){
      digitalWrite(25, LOW);
      colorWipe(strip.Color(0, 0, 0), 5); 
      send();
    }
  }
  
}
