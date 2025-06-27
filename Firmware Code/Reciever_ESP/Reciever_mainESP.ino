#include <HardwareSerial.h>

#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <UbidotsEsp32Mqtt.h>
#include <WebServer.h>

// Static IP
IPAddress local_IP(192, 168, 86, 184);    
IPAddress gateway(192, 168, 1, 1);       /
IPAddress subnet(255, 255, 255, 0);     


// WiFi credentials
const char* WIFI_SSID = "Asim";
const char* WIFI_PASS = "972003Aa";


// Ubidots configuration
const char* UBIDOTS_TOKEN = "BBUS-UDgv0Gpv89FSpEPJf77jRerOtP5emL";
const char *VARIABLE_LABEL4 = "left-detected";
const char *VARIABLE_LABEL5 = "middle-detected"; 
const char *VARIABLE_LABEL6 = "right-detected";
const char *VARIABLE_LABEL_FLAME = "Flame Sensor";
const char *VARIABLE_LABEL_GAS = "MQ-2";
const char *VARIABLE_LABEL_speed = "Speed";
const char *DEVICE_LABEL = "demo-machine";

const char* server1 = "http://industrial.api.ubidots.com";
const char* endpoint = "/api/v1.6/devices/";

unsigned long lastTime = 0;
unsigned long timerDelay = 50;
const int PUBLISH_FREQUENCY = 150;
unsigned long timer;


#define MQ2_SENSOR_PIN 34

// Pump control pin
#define PUMP_PIN 18


TinyGPSPlus gps;
HardwareSerial GPSSerial(1); // UART1 for GPS
WebServer server(80);

String htmlPage;

#define SERIAL_TX_PIN 17  // UART2 TX
#define SERIAL_RX_PIN 16  // UART2 RX

int LeftIR = 0;
int MiddleIR = 0;
int RightIR = 0;



HardwareSerial irSerial(2);  // UART2 for IR sensors


Ubidots ubidots(UBIDOTS_TOKEN);



void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}



void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, 5, 4);
  Serial.println("GPS Module is starting...");
  

pinMode(MQ2_SENSOR_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);


  // Initialize IR sensor UART
  irSerial.begin(115200, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);




WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  Serial.println("Connected to Ubidots");

  timer = millis();

// Serve map page
  server.on("/", []() {
    htmlPage = "<!DOCTYPE html><html><head><title>GPS Tracker</title>";
    htmlPage += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    htmlPage += "<link rel='stylesheet' href='https://unpkg.com/leaflet/dist/leaflet.css' />";
    htmlPage += "<script src='https://unpkg.com/leaflet/dist/leaflet.js'></script></head><body>";
    htmlPage += "<h2>ESP32 GPS Location</h2>";
    htmlPage += "<div id='coordinates' style='font-size: 18px; margin-bottom: 10px;'>";
    htmlPage += "Latitude: <span id='lat'>0</span>, Longitude: <span id='lon'>0</span>";
    htmlPage += "</div>";
    htmlPage += "<div id='map' style='height: 80vh;'></div>";
    htmlPage += "<script>";
    htmlPage += "var map = L.map('map').setView([0, 0], 2);";
    htmlPage += "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);";
    htmlPage += "var marker = L.marker([0, 0]).addTo(map);";
    htmlPage += "async function updateMarker() {";
    htmlPage += "const res = await fetch('/location'); const json = await res.json();";
    htmlPage += "marker.setLatLng([json.lat, json.lon]); map.setView([json.lat, json.lon], 15);";
    htmlPage += "document.getElementById('lat').textContent = json.lat.toFixed(6);";
    htmlPage += "document.getElementById('lon').textContent = json.lon.toFixed(6);";
    htmlPage += "}";
    htmlPage += "setInterval(updateMarker, 3000);";
    htmlPage += "</script></body></html>";
    server.send(200, "text/html", htmlPage);
  });

  // Serve current location as JSON
  server.on("/location", []() {
    if (gps.location.isValid()) {
      String data = "{\"lat\":" + String(gps.location.lat(), 6) +
                    ",\"lon\":" + String(gps.location.lng(), 6) + "}";
      server.send(200, "application/json", data);
    } else {
      server.send(200, "application/json", "{\"lat\":0,\"lon\":0}");
    }
  });

  server.begin();
}





void loop() {
  // Process GPS data
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }
  server.handleClient();
  
  // Process IR sensor data
  while (irSerial.available()) {
    String data = irSerial.readStringUntil('\n');
    data.trim();
    
    int comma1 = data.indexOf(',');
    int comma2 = data.lastIndexOf(',');
    
    if (comma1 > 0 && comma2 > comma1) {
       LeftIR = data.substring(0, comma1).toInt();
       MiddleIR = data.substring(comma1 + 1, comma2).toInt();
       RightIR = data.substring(comma2 + 1).toInt();
      
      Serial.printf("Left: %d | Mid: %d | Right: %d\n", LeftIR, MiddleIR, RightIR);
     
      
    }
  }

if (!ubidots.connected()) {
    ubidots.reconnect();
  }
if ((millis() - timer) > PUBLISH_FREQUENCY) {

    int gasValue = analogRead(MQ2_SENSOR_PIN);
    ubidots.add(VARIABLE_LABEL_GAS, gasValue);

    if ((millis() - lastTime) > timerDelay) {
      if (WiFi.status() == WL_CONNECTED) {
           
        int speed = getUbidotsValue(VARIABLE_LABEL_speed);
        int flameValue = getUbidotsValue(VARIABLE_LABEL_FLAME);
        ubidots.add(VARIABLE_LABEL4,LeftIR);
      ubidots.add(VARIABLE_LABEL5,MiddleIR);
      ubidots.add(VARIABLE_LABEL6,RightIR);
      
      
      
        Serial.println(flameValue);

        // Pump control logic based on flame sensor
        if (flameValue == 1) {
          digitalWrite(PUMP_PIN, HIGH);
          Serial.println("Flame detected! Pump ON");
        } else {
          digitalWrite(PUMP_PIN, LOW);
          Serial.println("No flame. Pump OFF");
        }

        

        Serial.printf("Left: %d | Mid: %d | Right: %d | Speed: %d | Flame: %.2f\n", 
                      LeftIR, MiddleIR, RightIR, speed, flameValue);

        
      } else {
        Serial.println("WiFi Disconnected");
      }
      lastTime = millis();
    }

    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }

  ubidots.loop();
}

float getUbidotsValue(const char* variableLabel) {
  HTTPClient http;
  String url = String(server1) + String(endpoint) + String(DEVICE_LABEL) + "/" + String(variableLabel) + "/lv?token=" + String(UBIDOTS_TOKEN);
  http.begin(url.c_str());
  int httpResponseCode = http.GET();
  float value = NAN;

  if (httpResponseCode > 0) { 
    String payload = http.getString();
    value = payload.toFloat();
  } else {
    Serial.print("Error in HTTP request: ");
    Serial.println(httpResponseCode);
  }

  http.end();
  return value;
  delay(100);
}


  
  