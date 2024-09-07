#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
//#include "esp_task_wdt.h"

//-----------------------------definitions-------------------------
const byte MAX_OUTPUT_LENGTH   = 100;
const byte MAX_ATTEMPTS        = 5;
const byte PWM_RESOLUTION      = 8;
const byte PWM_CHANEL          = 0;
uint16_t MAXIMUN_FREQUENCY     = 10000;
const float  VCC               = 3.3;
uint32_t R                     = 100000;
float B_1                      = 4007.499; //25  -> 100
float B2                       = 4258.308; //101 -> 176
float B3                       = 4594.546; //177 -> 252
float B4                       = 4668.355; //253 -> 300
const uint32_t R25             = 100000;
const uint32_t R101            = 6526;
const uint16_t R177            = 955; //0.9550
const float    R253            = 218.6; //0.2186
//-----------------------------PIN---------------------------------
const byte FHeater = 32; //Filament heater
const byte FCooler = 13; //Filament Cooler
const byte FS      = 36; //Filament sensor
const byte STOP    = 15; //Emergency Stop
const byte Bled    = 2;  //Correct operation
const byte Rled    = 15; //Error notifier
const byte DIAG    = 39; //Motor DIAG
const byte StepMotor = 25;
const byte DirMotor  = 26;
const byte EN        = 33;
const byte MS1       = 27;   
const byte MS2       = 14;
//-----------------------------Tasks-------------------------------
//Core0
TaskHandle_t Reciver;
TaskHandle_t Control;
TaskHandle_t MControl;  
TaskHandle_t HControl;
//Core1  --> Reciver & Sender
//-----------------------------Objects-----------------------------
Adafruit_ADS1115 ads;
WiFiClient       client;
//-----------------------------variables---------------------------
uint8_t Mvel = 0;   //Motor velocity
uint8_t Cvel = 0;   //Cooler velocity
uint8_t Hum  = 0;   //Humidity
uint16_t SP  = 0;   //SetPoint

double Heat       = 0.0; //Temperature
double Vsense     = 0.0; 
const float multiplier = 0.1875F;

volatile bool FlagForTemp    = false; //Temperature calculation
volatile bool FlagForReciver = false; //Receive data
volatile bool FlagForSender  = false; //Send data
volatile bool FlagForADC     = false; //Read differential voltage
volatile bool FlagForMotor   = false; //Activate motor
volatile bool FlagForHeater  = false; //Filament heater
//-----------------------------------------------------------------

void IRAM_ATTR alarmTrigger(void){
  digitalWrite(Rled, LOW);
  FlagForReciver = false;
  FlagForMotor   = false;
  FlagForADC     = false;
  FlagForTemp    = false;
  FlagForSender  = false;
  /*
  WiFi.disconnect();
  WiFi.reconnect();
  ESP.restart();
  */
}

void IRAM_ATTR motorTrigger(void){
  digitalWrite(Rled, LOW);
  FlagForMotor = false;
}

void IRAM_ATTR EmergencyStop(void){
  digitalWrite(Rled, LOW);
  FlagForReciver = false;
  FlagForMotor   = false;
  FlagForADC     = false;
  FlagForTemp    = false;
  FlagForSender  = false;
}

void loop0(void *parameter){ //RECIVER
  String data  = "";
  for(;;){
    if(FlagForReciver){
      while(!serverInit());
      unsigned long startTime = millis();
      while(millis() - startTime < 1000){
        if(client.available() > 0){
          String input = client.readStringUntil('\n');
          String output = "";
          byte ILength = input.length(); //Input Length
          byte Index   = 0;              //Output Index
          
          for (int i = 0; i < ILength; i++) {// Funcion se mejorar
            char c = input.charAt(i);
            if (isAlphaNumeric(c) || c == ' ' || c == '-') {
              output += c;
              if (output.length() >= MAX_OUTPUT_LENGTH) {
                  break;
              }
            }
          }
          if (output.length() >= 2){
            char         X = output.charAt(0);
            uint16_t value = output.substring(1).toInt();
            Serial.print("Dato del suich: "+ X);
            switch (X) {
              case 'O': //ON
                Serial.println("ON");
                FlagForMotor   = true;
                FlagForHeater  = true;
                FlagForADC     = true;
                FlagForSender  = true;
                FlagForHeater  = true;
                break;
              case 'F': //OFF
                Serial.println("OFF");
                FlagForMotor   = false;
                FlagForADC     = false;
                FlagForTemp    = false;
                FlagForSender  = false;
                FlagForHeater  = false;
                break;
              case 'H': //Heater Block
                Serial.println("Heater Block: "    +String(value));
                Hum = value;
                break;
              case 'C': //Cooler velocity
                if(value == 1){
                  Serial.println("Cooler ON");
                  digitalWrite(FCooler , HIGH);

                }
                else{
                  Serial.println("Cooler OFF");
                  digitalWrite(FCooler , LOW);
                }
                break;
              case 'M': //Motor velocity
                Serial.println("Motor velocity: "  +String(value));
                Mvel = value;
                break;
              case 'S': //SetPoint
                Serial.println("SetPoint: "        +String(value));
                SP   = value;
                break;
            }
            X = '\0';
            value = 0;
          }
        }
      }
      if(FlagForSender){
        data = String(Heat);
        client.print(data);
      }
      client.stop();
    }
    vTaskDelay(10);
  }
}

void loop1(void *parameter){ //PID, NTC & Hblock
  float multiplier = 0.1875F;
  for(;;){
    float B = 0, Rlow = 0.0;
    double temp = 0.0, Vdc = 0.0, res = 0.0;
    if(FlagForTemp){ 
      for (int i=0; i<800; i++) {
      temp += Vsense;
      }
      temp = temp/800;
      Vdc  = temp*multiplier/1000;        //Differential Voltage
      res  = R*(2*Vdc-VCC)/(-VCC-2*Vdc);
      Serial.println("Resistor: " + String(res));
      if(res >= 6710  && res <= R25){
        HeatC(B_1, R25, res);
      }
      if(res >= 975.7 && res <= R101){
        HeatC(B2, R101, res);
      }
      if(res >= 222.3 && res <= R177){
        HeatC(B3, R177, res);
      }
      if(res >= 105.6 && res <= R253){
        HeatC(B4, R253, res);
      }
      FlagForTemp = false; 
      FlagForADC = true;
    }
    vTaskDelay(100);
  }
}

void loop2(void *parameter){ //MControl
  for(;;){
    while(FlagForMotor){
      if(Mvel==0){
        digitalWrite(DirMotor , LOW);
        digitalWrite(StepMotor, LOW);
      }
      else{
        delayMicroseconds(map(Mvel , 0, 100, 4000, 0));
        digitalWrite(DirMotor , HIGH);
        digitalWrite(StepMotor, LOW);
        delayMicroseconds(map(Mvel , 0, 100, 4000, 0));
        digitalWrite(DirMotor , LOW);
        digitalWrite(StepMotor, HIGH);
      }
    }
    //vTaskDelay(10);
  }
}

void loop3(void *parameter){ //HControl
  for(;;){
    while(FlagForHeater){
      if(SP==0){
        digitalWrite(FHeater , LOW);
      }
      else{
        delayMicroseconds(map(SP , 0, 400, 10000, 0));
        digitalWrite(FHeater, LOW);
        delayMicroseconds(map(SP , 0, 400, 10000, 0));
        digitalWrite(FHeater, HIGH);
      }
    }
    //vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);
  //Pin initialization
  pinMode(FS,   INPUT_PULLDOWN);
  pinMode(DIAG, INPUT);
  pinMode(STOP, INPUT_PULLDOWN);
  pinMode(StepMotor, OUTPUT);
  pinMode(DirMotor , OUTPUT);
  pinMode(FHeater  , OUTPUT);
  pinMode(FCooler  , OUTPUT);
  pinMode(Bled     , OUTPUT);
  pinMode(Rled     , OUTPUT);
  pinMode(EN       , OUTPUT);
  pinMode(MS1      , OUTPUT);
  pinMode(MS2      , OUTPUT);
  //External interruptions
  attachInterrupt(digitalPinToInterrupt(FS),   alarmTrigger, HIGH); 
  //attachInterrupt(digitalPinToInterrupt(DIAG), motorTrigger, HIGH); 
  //attachInterrupt(digitalPinToInterrupt(STOP), motorTrigger, HIGH);
  //Core0
  xTaskCreatePinnedToCore(loop0, "Reciver" , 1024*2, NULL, 1, &Reciver, 0);
  xTaskCreatePinnedToCore(loop1, "Control" , 1024*2, NULL, 1, &Control, 0);
  //Core1
  xTaskCreatePinnedToCore(loop2, "MControl", 1024*1, NULL, 1, &MControl,1);
  xTaskCreatePinnedToCore(loop3, "HControl", 1024*1, NULL, 1, &HControl,1);

  while(!Init_main());
}

void loop() {
  ReadNTC();
}

void HeatC(float B, float Rlow, float res){
  Heat = (B/(B/298.15+log(res/R)))-273.15;
  Serial.println(String(Heat));
}

String filtro(const String& input){
  String output = "";
  for (int i=0; i<input.length(); i++){
    char X = input.charAt(i);
    if (!isspace(X) && X != '\0'){
      output += X;
    }
  }
  return output;
}

void ReadNTC(void){
  if(FlagForADC){
    Vsense = ads.readADC_Differential_0_3();
    FlagForADC  = false;
    FlagForTemp = true;
  }
}

bool adsInit(void){
  if(ads.begin()) {
    ads.setGain(GAIN_TWOTHIRDS);
    Serial.println("ADS LISTO");
    return true;
  }
  else{
    Serial.println("Failed to initialize ADS...");
    return false;
  }
}

bool initWifi(void){
  const char* SSID     = "Emmanuel";   //Emmanuel     //Docentes   //iPhone de Juan Jose //sc-ea40
  const char* PW       = "01161996";   //01161996     //KIZqBsUf8m //Juanjo0205          //QDKX9VQPM37C

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PW);
  while(WiFi.status() != WL_CONNECTED){
    Serial.println("Wifi...");
    delay(500);
  }
  Serial.println(WiFi.localIP());
  return true;
}

bool serverInit(void){
  const char* HOST          = "192.168.144.206";
  const uint16_t serverPort = 5000;       // Puerto del servidor Java
  byte attempts = 0;
  while((attempts <= MAX_ATTEMPTS) && (WiFi.status()==WL_CONNECTED)){
    if(client.connect(HOST,serverPort)){ //Connection established
      return true;
    }
    else if(attempts >= MAX_ATTEMPTS){   //Fail
      return false;
    }
    else{
      attempts++;
      continue;
    }
  }
}

bool Init_main(void){
//-----------------------------ads-----------------------------------
  while(!adsInit());
//-----------------------------wifi----------------------------------
  while(!initWifi());
//-------------------------------------------------------------------
  digitalWrite(EN,  LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(Bled, HIGH);
  digitalWrite(Rled, LOW);
  FlagForReciver = true;
  return true;
}