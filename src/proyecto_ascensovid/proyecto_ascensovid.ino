#include "ThingsBoard.h"


#include <LiquidCrystal_I2C.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include "SparkFun_SGP30_Arduino_Library.h"

#include <Wire.h>
#include "paj7620.h" //gesture sensor

#define GES_REACTION_TIME    500       // You can adjust the reaction time according to the actual circumstance.
#define GES_ENTRY_TIME      800       // When you want to recognize the Forward/Backward gestures, your gestures' reaction time must less than GES_ENTRY_TIME(0.8s). 
#define GES_QUIT_TIME     1000


SGP30 mySensor; //SGP30 sensor
long t1, t2;
SCD30 airSensor;//SVM30 sensor

const int ledRojoPIN = 33;
const int ledAmarilloPIN = 26;
const int ledVerdePIN = 27;
const int ledWifiPIN = 25;
const int ledThingsBoardPIN = 32;

int LEDpin = 13;
int entradapin = 16;

int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

int piso = 0;
int contadorMediciones = 0;


#define WIFI_AP             "MOVISTAR_2D54"
#define WIFI_PASSWORD       "UuQZR7W9uh8TEB6osc2X"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
#define TOKEN               "jxEobjqTLBcKQs3pzK53"//"psfR3NWLbMHRtwzyWTGS"
#define THINGSBOARD_SERVER    "demo.thingsboard.io"//"iot.etsisi.upm.es"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD   115200

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;

void setup() {

  // initialize serial for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  InitWiFi();
  ///////////////////////////////////LEDS///////////////////////////////////////////
  pinMode(ledRojoPIN , OUTPUT); 
  pinMode(ledAmarilloPIN , OUTPUT); 
  pinMode(ledVerdePIN , OUTPUT); 
  pinMode(ledWifiPIN , OUTPUT);
  pinMode(ledThingsBoardPIN , OUTPUT);
  //////////////////////////////PIR///////////////////////////////////////////////
  pinMode(LEDpin,OUTPUT);
  pinMode(entradapin,INPUT);
/////////////////////////////////////DISPLAY//////////////////////////////////////

  // initialize LCD
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();//activar led
  /////////////////////////////////GESTOS//////////////////////////////////////////
  uint8_t error = 0;

    Serial.begin(115200);
    Serial.println("\nPAJ7620U2 TEST DEMO: Recognize 9 gestures.");

    error = paj7620Init();      // initialize Paj7620 registers
    if (error) {
        Serial.print("INIT ERROR,CODE:");
        Serial.println(error);
    } else {
        Serial.println("INIT OK");
    }
    Serial.println("Please input your gestures:\n");
  ///////////////////////////////////////////CO2///////////////////////////////////
  Serial.println("SCD30 Example");
  Wire.begin();

  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  //The SCD30 has data ready every two seconds
/////////////////////////////////////////TVOC///////////////////////////////////////////////////
  Wire.setClock(400000);
    //Initialize sensor
    if (mySensor.begin() == false) {
      Serial.println("No SGP30 Detected. Check connections.");
      while (1);
    }
    //Initializes sensor for air quality readings
    //measureAirQuality should be called in one second increments after a call to initAirQuality
    mySensor.initAirQuality();
    t1 = millis();
////////////////////////////////////////////////////////////////////////////////////////////
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledWifiPIN , LOW);
    reconnect();
  }

  if (!tb.connected()) {
    digitalWrite(ledThingsBoardPIN , LOW); 
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
    digitalWrite(ledThingsBoardPIN , HIGH); 
  }

  Serial.println("Sending data...");

  // Uploads new telemetry to ThingsBoard using MQTT.
  // See https://thingsboard.io/docs/reference/mqtt-api/#telemetry-upload-api
  // for more details

  if(contadorMediciones ==60){
  ///////////////////////////////////CO2///////////////////////////////////////////////////
    if (airSensor.dataAvailable())
    {
      Serial.print("co2(ppm):");
      Serial.print(airSensor.getCO2());
      tb.sendTelemetryFloat("CO2", airSensor.getCO2());
      while(airSensor.getCO2()>=2000){
        digitalWrite(ledRojoPIN , HIGH); 
        digitalWrite(ledAmarilloPIN , HIGH); 
        digitalWrite(ledVerdePIN , HIGH); 
        delay(200);
        digitalWrite(ledRojoPIN , LOW); 
        digitalWrite(ledAmarilloPIN , LOW); 
        digitalWrite(ledVerdePIN , LOW); 
        delay(200);
      }
      if(airSensor.getCO2()>=1500 && airSensor.getCO2()<2000){
        digitalWrite(ledRojoPIN , HIGH); 
        digitalWrite(ledAmarilloPIN , LOW); 
        digitalWrite(ledVerdePIN , LOW); 
      }
      if(airSensor.getCO2()>=1000 && airSensor.getCO2()<1500){
        digitalWrite(ledRojoPIN , LOW); 
        digitalWrite(ledAmarilloPIN , HIGH); 
        digitalWrite(ledVerdePIN , LOW); 
      }
      if(airSensor.getCO2()<1000){
        digitalWrite(ledRojoPIN , LOW); 
        digitalWrite(ledAmarilloPIN , LOW); 
        digitalWrite(ledVerdePIN , HIGH); 
      }
      
      Serial.print(" temp(C):");
      Serial.print(airSensor.getTemperature(), 1);
      tb.sendTelemetryFloat("Temperatura", airSensor.getTemperature());
  
      Serial.print(" humidity(%):");
      Serial.print(airSensor.getHumidity(), 1);
      tb.sendTelemetryFloat("Humedad", airSensor.getHumidity());
  
      Serial.println();
    }
    else
      Serial.println("Waiting for new data");
  


  //////////////////////////////////////TVOC/////////////////////////////////////////////////////
      //First fifteen readings will be
    //CO2: 400 ppm  TVOC: 0 ppb
    t2 = millis();
    if ( t2 >= t1 + 1000) //only will occur if 1 second has passed
    {
      t1 = t2;
      //measure CO2 and TVOC levels
      mySensor.measureAirQuality();
      Serial.print("\nTVOC: ");
      Serial.print(mySensor.TVOC);
      Serial.println(" ppb");
      tb.sendTelemetryFloat("Tvoc", mySensor.TVOC);
      while(mySensor.TVOC>=220){
        digitalWrite(ledRojoPIN , HIGH); 
        digitalWrite(ledAmarilloPIN , HIGH); 
        digitalWrite(ledVerdePIN , HIGH); 
        delay(200);
        digitalWrite(ledRojoPIN , LOW); 
        digitalWrite(ledAmarilloPIN , LOW); 
        digitalWrite(ledVerdePIN , LOW); 
        delay(200);
      }
      
      //get raw values for H2 and Ethanol
      mySensor.measureRawSignals();
      Serial.print("Raw H2: ");
      Serial.print(mySensor.H2);
      tb.sendTelemetryFloat("H2", mySensor.H2);
      Serial.print(" \tRaw Ethanol: ");
      Serial.println(mySensor.ethanol);
      tb.sendTelemetryFloat("Etanol", mySensor.ethanol);
      Serial.println();
    }
    contadorMediciones = 0;
  }

   ////////////////////////////////////////PIR///////////////////////////////////////
    int value= digitalRead(entradapin);
   
    if (value == HIGH)
    {
      Serial.println("DETECTADO");
      tb.sendTelemetryFloat("Pir", 1);
    }
    else
    {
      Serial.println("NO DETECTADO");
      tb.sendTelemetryFloat("Pir", 0);
    }
////////////////////////////////////GESTOS////////////////////////////////////////////////

  uint8_t data = 0, error;

    error = paj7620ReadReg(0x43, 1, &data);        // Read Bank_0_Reg_0x43/0x44 for gesture result.
    if (!error) {
        switch (data) {               // When different gestures be detected, the variable 'data' will be set to different values by paj7620ReadReg(0x43, 1, &data).
            case GES_UP_FLAG:
                Serial.println("Up");
                if(piso < 3){
                  lcd.clear();
                  lcd.setCursor(0, 0);
                  lcd.print("SUBIR PISO:");
                  lcd.setCursor(0, 1);
                  lcd.print(piso);
                  lcd.print(" --->");
                  piso++;
                  lcd.print(piso);
                }else {
                  lcd.clear();
                  lcd.setCursor(0, 0);
                  lcd.print("NO SE PUEDE");
                  lcd.setCursor(0, 1);
                  lcd.print ("SUBIR MAS");
                }
                break;
            case GES_DOWN_FLAG:
                Serial.println("Down");
                if(piso > 0){
                  lcd.clear();
                  lcd.setCursor(0, 0);
                  lcd.print("BAJAR PISO:");
                  lcd.setCursor(0, 1);
                  lcd.print(piso);
                  lcd.print(" --->");
                  piso--;
                  lcd.print(piso);
                }else{
                  lcd.clear();
                  lcd.setCursor(0, 0);
                  lcd.print("NO SE PUEDE");
                  lcd.setCursor(0, 1);
                  lcd.print ("BAJAR MAS");
                }
                break;
            default:
                break;
        }
         tb.sendTelemetryFloat("Piso", piso);
    }
////////////////////DISPLAY//////////////////////////////////////////////
   if (contadorMediciones %20  == 0){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CO2: ");
    lcd.print(airSensor.getCO2());
    lcd.print(" PPM");
  
    lcd.setCursor(0,1);
    lcd.print("TVOC: ");
    lcd.print(mySensor.TVOC);
    lcd.print(" PPB");
  }
  else if (contadorMediciones %20 == 13) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TEMP:");
    lcd.print((int)airSensor.getTemperature());
    lcd.print("C ");
    lcd.print("PISO:");
    lcd.print(piso);
    
    lcd.setCursor(0,1);
    lcd.print("HUMEDAD: ");
    lcd.print(airSensor.getHumidity());
    lcd.print("%");
  }

  delay(1000);
  contadorMediciones++;
  tb.loop();
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  digitalWrite(ledWifiPIN , HIGH);
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
    digitalWrite(ledWifiPIN , HIGH);
  }
}
