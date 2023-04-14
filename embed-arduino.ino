//#include <WiFiClientSecure.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESPPubSubClientWrapper.h>
#include <Adafruit_BMP280.h>
#include <sps30.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define LED_RED 13
#define LED_GREEN_1 27
#define LED_GREEN_2 33

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define MQTT_SERVER "---------"
#define MQTT_PORT 1883
#define MQTT_NAME "ESP32"
#define MQTT_USERNAME "-----"
#define MQTT_PASSWORD "-----"

#define HTTP_URL "-----------"
#define HTTP_NAME "ESP32"

#define SP30_COMMS Serial1
#define DEBUG 0

#define MODE_GPS 0

WiFiClient client;
const char* ssid = "----------";
const char* pass = "----------";

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

HTTPClient http;
PubSubClient mqtt(client);
Adafruit_SSD1306 OLED(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BMP280 bmp;
SPS30 sps30;

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);

  LED_init();
  OLED_init();
  //SPS30_init();
  BMP280_init();
  MQTT_init();
  HTTP_init();

  
  // for (int i = 0; i < 5; i++) {
  //   Serial.println(SPS30_get());
  // }
  // while (1) {} /* Stop */
  

}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    //Serial.println(BMP280_get());
    //HTTP_PostAPI();

    if(MODE_GPS == 0){
      smartDelay(4000);
      if(gps.location.isValid()) {
        LED_ON(1);
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        double dust = 0;
        double bmp = BMP280_get();

        OLED_PrintData(lat, lng, dust, bmp);

        Serial.print("Lat: ");
        Serial.print(lat);
        Serial.print(F("\t"));
        Serial.print("Lng: ");
        Serial.print(lng);
        Serial.print(F("\t"));
        Serial.print("Dust: ");
        Serial.print(dust);
        Serial.print(F("\t"));
        Serial.print("Temp: ");
        Serial.println(bmp);

        HTTP_PostAPI(HTTP_NAME, lat, lng, dust, bmp);
      }
      else {
        LED_OFF(1);
        Serial.println("GPS Not Valid");
      }
    }else{
      if (mqtt.connected() == false) {
        Serial.print("MQTT connection... ");
        if (mqtt.connect(MQTT_NAME)) {
          Serial.println("connected");
          mqtt.subscribe("embed");
        } else {
          Serial.println("Failed");
          delay(3000);
        }
      } else {
        mqtt.loop();
      }
    }

  } else {  //Reconnecting to Router
    LED_OFF(0);
    Serial.print("Connecting to : [");
    Serial.print(ssid);
    Serial.print("] ->> ");
    WiFi.begin(ssid, pass);                //Connect ESP32 to home network
    while (WiFi.status() != WL_CONNECTED)  //Wait until Connection is complete
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    LED_ON(0);
  }
}

// =========================================================================
//
//  GPS Sensor : Neo-6M
//
// =========================================================================

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available()) {
      //print()
      //ss.read()
      //Serial.print(ss.read());
      gps.encode(ss.read());
    }

  } while (millis() - start < ms);
}

// =========================================================================
//
//  PM2.5 Sensor : SPS30
//
// =========================================================================

void SPS30_init() {
  sps30.EnableDebugging(DEBUG);
  SP30_COMMS.begin(115200, SERIAL_8N1, 25, 26);

  // Initialize SPS30 library
  if (sps30.begin(&SP30_COMMS)) {
    Serial.println("[SPS30] communication channel Success.");
  } else {
    Serial.println("Could not set serial communication channel.");
    while (1) {} /* Stop */
  }

  // check for SPS30 connection
  if (sps30.probe()) {
    Serial.println(F("[SPS30] Detected SPS30."));
    sps30.reset();
  } else {
    Serial.println(F("[SPS30] could not probe / connect with SPS30."));
    while (1) {} /* Stop */
  }

  if (sps30.start()) {
    Serial.println(F("[SPS30] Measurement started"));
  } else {
    Serial.println(F("[SPS30] Measurement can't start!"));
    while (1) {} /* Stop */
  }
}

double SPS30_get() {
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == SPS30_ERR_DATALENGTH) {

      if (error_cnt++ > 3) {
        Serial.println(F("[SPS30] Error during reading values:"));
      }
      delay(1000);
    }

    // if other error
    else if (ret != SPS30_ERR_OK) {
      Serial.println(F("[SPS30] Other error"));
    }

  } while (ret != SPS30_ERR_OK);

  return (val.MassPM2);
}

// =========================================================================
//
//  Temperature Sensor : BMP280
//
// =========================================================================

void BMP280_init() {
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

double BMP280_get() {
  int intValue = (int)(bmp.readTemperature() * 100);
  double result = (double)intValue / 100;
  return result;
}

// =========================================================================
//
//  Protocol : HTTP
//
// =========================================================================

void HTTP_init() {
  http.begin(HTTP_URL);
  http.addHeader("Content-Type", "application/json");
}

void HTTP_PostAPI(String name_input, double lat_input, double lng_input, int dust_input, double temp_input) {
  LED_ONT(2, 100);
  Serial.println("HTTP Send");
  // Data to send with HTTP POST
  StaticJsonDocument<200> jsonDocument;
  jsonDocument["Device"] = name_input;
  jsonDocument["Lat"] = lat_input;
  jsonDocument["Lng"] = lng_input;
  jsonDocument["Dust"] = dust_input;

  jsonDocument["Temp"] = temp_input;
  String jsonString;
  serializeJson(jsonDocument, jsonString);

  // Send HTTP POST request
  int httpResponseCode = http.POST(jsonString);
  if (httpResponseCode > 0) {
    Serial.println("Channel update successful.");
    String content = http.getString();
    Serial.println("Content ---------");
    Serial.println(content);
    Serial.println("-----------------");
  } else {
    Serial.println("Channel update failed.");
  }
  // Free resources
  http.end();
}

// =========================================================================
//
//  Protocol : MQTT
//
// =========================================================================

void callback(char* topic, byte* payload, unsigned int length) {
  LED_ONT(2, 100);
  payload[length] = '\0';
  String topic_str = topic, payload_str = (char*)payload;
  Serial.println("[" + topic_str + "]: " + payload_str);

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, payload, length);
  double lat = doc["latitude"];
  double lng = doc["longtitude"];
  double dust = SPS30_get();
  double bmp = BMP280_get();

  OLED_PrintData(lat, lng, dust, bmp);

  Serial.print("Lat: ");
  Serial.print(lat);
  Serial.print(F("\t"));
  Serial.print("Lng: ");
  Serial.print(lng);
  Serial.print(F("\t"));
  Serial.print("Dust: ");
  Serial.print(dust);
  Serial.print(F("\t"));
  Serial.print("Temp: ");
  Serial.println(bmp);


  //HTTP_PostAPI(HTTP_NAME, lat, lng, dust, bmp);
  //LED_OFF(2);
}

void MQTT_init() {
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(callback);
}

// =========================================================================
//
//  Display : OLED
//
// =========================================================================

void OLED_init() {
  OLED.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
}

void OLED_PrintData(double raw_lat, double raw_lng, double raw_dust, double raw_temp) {
  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0, 0);
  OLED.println("PM2.5 Detector");
  OLED.println("");

  OLED.print("Lat: ");
  OLED.println(raw_lat);

  OLED.print("Lng: ");
  OLED.println(raw_lng);

  OLED.print("Dust: ");
  OLED.print(raw_dust);
  OLED.println(" ppm");

  OLED.print("Temp: ");
  OLED.print(raw_temp);
  OLED.println(" c");

  // OLED.print("Temp: ");
  // OLED.print("30");
  // OLED.println(" c");
  OLED.display();
}

void OLED_TESTGPS(double raw_lat, double raw_lng) {
  OLED.clearDisplay();
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0, 0);
  OLED.println("GPS TESTER ");

  String lat = "";
  lat = String(raw_lat, 6);
  OLED.setCursor(0, 8);
  OLED.print("Lat : ");
  OLED.println(lat);

  String lng = "";
  lng = String(raw_lng, 6);
  OLED.setCursor(0, 16);
  OLED.print("Lng : ");
  OLED.println(lng);

  OLED.display();
}

// =========================================================================
//
//  Display : LED
//
// =========================================================================

void LED_init() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
}

void LED_ON(int led_select) {
  if (led_select == 0) {
    digitalWrite(LED_RED, HIGH);
  } else if (led_select == 1) {
    digitalWrite(LED_GREEN_1, HIGH);
  } else if (led_select == 2) {
    digitalWrite(LED_GREEN_2, HIGH);
  }
}

void LED_OFF(int led_select) {
  if (led_select == 0) {
    digitalWrite(LED_RED, LOW);
  } else if (led_select == 1) {
    digitalWrite(LED_GREEN_1, LOW);
  } else if (led_select == 2) {
    digitalWrite(LED_GREEN_2, LOW);
  }
}

void LED_ONT(int led_select, int time_select) {
  if (led_select == 0) {
    digitalWrite(LED_RED, HIGH);
  } else if (led_select == 1) {
    digitalWrite(LED_GREEN_1, HIGH);
  } else if (led_select == 2) {
    digitalWrite(LED_GREEN_2, HIGH);
  }
  delay(time_select);
  if (led_select == 0) {
    digitalWrite(LED_RED, LOW);
  } else if (led_select == 1) {
    digitalWrite(LED_GREEN_1, LOW);
  } else if (led_select == 2) {
    digitalWrite(LED_GREEN_2, LOW);
  }
}
