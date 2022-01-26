#include <Arduino.h>
#include<stdlib.h>
#include <esp_now.h>
#include "esp_task_wdt.h"

#ifdef ESP32
  #include <WiFi.h>
#endif



#define debug
#define test

#define FordCargo
// #define MercedesArocs
// #define FusoCanter
bool SendOK=false;




uint8_t receiverMAC[] = {0x9c, 0x9c, 0x1f, 0xd8, 0x17, 0x7c}; // Master MAC Adress

typedef struct message {
  int V1 = -1;
  int V2 = -1;
  int V3 = -1;
  int TotalDistance = -1;
  int FuelTank = -1;
  int TotalHours = -1;
}message;
message bus; // créer une structure message nommé bus

void sendData() {
  esp_now_send(receiverMAC,(uint8_t *) &bus, sizeof(bus)); // NULL means send to all peers
}
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    #ifdef debug
      Serial.println("points sents");
    #endif
    SendOK=true;
    }else{
    #ifdef debug
      Serial.println("points not sents");
    #endif
    SendOK=false;
    }
}



unsigned long VDHR = 0; //High Resolution Vehicle Distance
unsigned long FuelLVL = 0; //High Resolution Vehicle Distance
unsigned long TotalHours=0;
// String trame="18FEC1EE0A4D2E006AC21500";
String trame="";





void setup() {

  delay(500);
  // put your setup code here, to run once:
  #ifdef debug
  Serial.begin(115200);
  while(!Serial);
  #endif
  Serial2.begin(9600);
  while(!Serial2);
  delay(1000);

  Serial2.println("atz");
  delay(1500);

  #ifdef FordCargo
  Serial2.println("atsp9");
  #endif

  // esp_sleep_enable_timer_wakeup(10000000); // 10 s
  esp_sleep_enable_timer_wakeup(300000000); // 5 min

  #ifndef test
  WiFi.mode(WIFI_STA); // set the wifi mode as Station
  if (esp_now_init() != ESP_OK) {
    #ifdef debug 
    Serial.println("ESP_Now init failed...");
    #endif
    delay(2000);
    ESP.restart();
  }
  

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info  receiverinfo;
    memcpy(receiverinfo.peer_addr, receiverMAC, 6);
    receiverinfo.channel=0;
    receiverinfo.encrypt = false;

    // add the receiver module 
    if( esp_now_add_peer(&receiverinfo) != ESP_OK){
      #ifdef debug
      Serial.println("Failed to add the receiver module");
      #endif
      esp_restart();
    }else{
      #ifdef debug
      Serial.println("Add the receiver module OK...");
      #endif
    }
  #endif
  

  Serial2.setTimeout(4000);
}


#ifndef test
void loop() {
  esp_task_wdt_init(60, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  delay(1000);
  Serial2.println("ath1");
  delay(1000);
  Serial2.println("ats0");
  delay(1000);
  #ifdef FordCargo
  Serial2.println("atcra 98fec1ee");
  #endif
  #ifdef MercedesArocs
  Serial2.println("atcra 98fec117");
  #endif
  #ifdef FusoCanter
  Serial2.println("atcra 98fec117"); // A Verifier
  #endif
  
  delay(1000);
  Serial2.println("atma");
  while (Serial2.available()){
    char c =Serial2.read();
    delay(5);
  }
  delay(500);
  esp_task_wdt_reset();
  unsigned long premillis=millis();

  while ((millis()-premillis)<15000)
  {
    if (Serial2.available()){
      char c = Serial2.read();
      if (c == '\r'){
        trame= Serial2.readStringUntil('\r');
        if (trame[2]=='F' && trame[3]=='E' && trame[4]=='C' && trame[5]=='1'){
          #ifdef debug
          Serial.println("Good PID VDHR");
          Serial.println(trame);
          #endif
          unsigned x1 = strtol(String(trame[8]).c_str(), NULL, 16);
          unsigned x2 = strtol(String(trame[9]).c_str(), NULL, 16)<<4;
          unsigned x3 = strtol(String(trame[11]).c_str(), NULL, 16)<<8;
          unsigned x4 = strtol(String(trame[10]).c_str(), NULL, 16)<<12;
          unsigned x5 = strtol(String(trame[13]).c_str(), NULL, 16)<<16;
          unsigned x6 = strtol(String(trame[12]).c_str(), NULL, 16)<<20;
          unsigned x7 = strtol(String(trame[15]).c_str(), NULL, 16)<<24;
          unsigned x8 = strtol(String(trame[14]).c_str(), NULL, 16)<<28;
          VDHR = x1 | x2 | x3 | x4 | x5 | x6 | x7 | x8;
          VDHR=VDHR*5;
          VDHR=VDHR/1000;
          bus.TotalDistance=int(VDHR);
          #ifdef debug
          Serial.println(VDHR);
          Serial.println(bus.TotalDistance);
          #endif
          delay(1000);
          esp_task_wdt_reset();          
          Serial2.println("ath1");
          delay(1000);
          Serial2.println("ats0");
          delay(1000);
          // Fuel LVL
          #ifdef FordCargo
          Serial2.println("atcra 98fefc17");
          #endif
          #ifdef MercedesArocs
          Serial2.println("atcra 98fefcfc");
          #endif
          #ifdef FusoCanter
          Serial2.println("atcra 98fefcfc"); // A Verifier 
          #endif
          delay(1000);
          Serial2.println("atma");
          while (Serial2.available())
          {
            char c =Serial2.read();
            delay(5);
          }
          
          delay(500);
          unsigned long premillis=millis();
          while ((millis()-premillis)<15000){
            if (Serial2.available()){
              char c = Serial2.read();
              if (c == '\r'){
                trame= Serial2.readStringUntil('\r');
                if (trame[2]=='F' && trame[3]=='E' && trame[4]=='F' && trame[5]=='C'){
                  #ifdef debug
                  Serial.println(trame);
                  Serial.println("Good Fuel LV ID");
                  #endif
                  unsigned x1 = strtol(String(trame[11]).c_str(), NULL, 16);
                  unsigned x2 = strtol(String(trame[10]).c_str(), NULL, 16)<<4;
                  FuelLVL = x1 | x2 ;
                  FuelLVL=FuelLVL*0.4;
                  bus.FuelTank=int(FuelLVL);

                  #ifdef debug
                  Serial.println(FuelLVL);
                  Serial.println(bus.FuelTank);
                  #endif

                  delay(1000);
                    
                  for(int i=0;i<5;i++){
                    sendData();
                    delay(1500);
                    if(SendOK){
                      esp_deep_sleep_start();
                      break;
                    }
                  }
                }else {
                  #ifdef debug
                  Serial.println("Bad Fuel LVL ID");
                  #endif
                }
              }
            }
          }
          for(int i=0;i<5;i++){
            sendData();
            delay(1500);
            if(SendOK){
              esp_deep_sleep_start();
              break;
            }
          }

        }else {
          #ifdef debug
          Serial.println("Bad VDHR");
          #endif
        }
      }
    }
  }

  #ifdef debug
  Serial.println("Going to Sleep");
  #endif
  esp_deep_sleep_start();
}

#endif

void getVDHR(){

  Serial2.println("atz");
  delay(1500);
  Serial2.println("ath1");
  delay(1000);
  Serial2.println("ats0");
  delay(1000);
  #ifdef FordCargo
  Serial2.println("atcra 98fec1ee");
  #endif
  #ifdef MercedesArocs
  Serial2.println("atcra 98fec117");
  #endif
  #ifdef FusoCanter
  Serial2.println("atcra 98fec117"); // A Verifier
  #endif
  delay(1000);
  Serial2.println("atma");
  delay(500);
  esp_task_wdt_reset();
  unsigned long premillis=millis();
  while ((millis()-premillis)<15000){
    if (Serial2.available()){
      char c = Serial2.read();
      if (c == '\r'){
        trame= Serial2.readStringUntil('\r');
        if (trame[2]=='F' && trame[3]=='E' && trame[4]=='C' && trame[5]=='1'){
          #ifdef debug
          Serial.println("Good PID VDHR");
          Serial.println(trame);
          #endif
          unsigned x1 = strtol(String(trame[8]).c_str(), NULL, 16);
          unsigned x2 = strtol(String(trame[9]).c_str(), NULL, 16)<<4;
          unsigned x3 = strtol(String(trame[11]).c_str(), NULL, 16)<<8;
          unsigned x4 = strtol(String(trame[10]).c_str(), NULL, 16)<<12;
          unsigned x5 = strtol(String(trame[13]).c_str(), NULL, 16)<<16;
          unsigned x6 = strtol(String(trame[12]).c_str(), NULL, 16)<<20;
          unsigned x7 = strtol(String(trame[15]).c_str(), NULL, 16)<<24;
          unsigned x8 = strtol(String(trame[14]).c_str(), NULL, 16)<<28;
          VDHR = x1 | x2 | x3 | x4 | x5 | x6 | x7 | x8;
          VDHR=VDHR*5;
          VDHR=VDHR/1000;
          bus.TotalDistance=int(VDHR);
          #ifdef debug
          Serial.println(VDHR);
          Serial.println(bus.TotalDistance);
          #endif
        }else{
          #ifdef debug
          Serial.println("Bad VDHR");
          #endif
        }
      }
    }
  }
}

void getFuelTank(){
  Serial2.println("ath1");
  delay(1000);
  Serial2.println("ats0");
  delay(1000);
  // Fuel LVL
  #ifdef FordCargo
  Serial2.println("atcra 98fefc17");
  #endif
  #ifdef MercedesArocs
  Serial2.println("atcra 98fefcfc");
  #endif
  #ifdef FusoCanter
  Serial2.println("atcra 98fefcfc"); // A Verifier 
  #endif
  delay(1000);
  Serial2.println("atma");
  while (Serial2.available())
  {
    char c =Serial2.read();
    delay(5);
  }
  
  delay(500);
  unsigned long premillis=millis();
  while ((millis()-premillis)<15000){
    if (Serial2.available()){
      char c = Serial2.read();
      if (c == '\r'){
        trame= Serial2.readStringUntil('\r');
        if (trame[2]=='F' && trame[3]=='E' && trame[4]=='F' && trame[5]=='C'){
          #ifdef debug
          Serial.println(trame);
          Serial.println("Good Fuel LV ID");
          #endif
          unsigned x1 = strtol(String(trame[11]).c_str(), NULL, 16);
          unsigned x2 = strtol(String(trame[10]).c_str(), NULL, 16)<<4;
          FuelLVL = x1 | x2 ;
          FuelLVL=FuelLVL*0.4;
          bus.FuelTank=int(FuelLVL);

          #ifdef debug
          Serial.println(FuelLVL);
          Serial.println(bus.FuelTank);
          #endif

          delay(1000);
            
          for(int i=0;i<5;i++){
            sendData();
            delay(1500);
            if(SendOK){
              esp_deep_sleep_start();
              break;
            }
          }
        }else {
          #ifdef debug
          Serial.println("Bad Fuel LVL ID");
          #endif
        }
      }
    }
  }
}


#ifdef test


void loop(){
  if(Serial2.available()){
    char c =Serial2.read();
    Serial.print(c);
  }
  if(Serial.available()){
    char c =Serial.read();
    Serial2.print(c);
  }
}
#endif
