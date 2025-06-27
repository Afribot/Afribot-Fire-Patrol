#include <UbidotsEsp32Mqtt.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <DHT.h>


#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"



// WiFi and Ubidots credentials
const char *UBIDOTS_TOKEN = "BBUS-UDgv0Gpv89FSpEPJf77jRerOtP5emL";
const char *ssid = "Asim";
const char *password = "972003Aa";
const char *DEVICE_LABEL = "demo-machine";
const char *VARIABLE_LABEL1 = "temperature"; 
const char *VARIABLE_LABEL2 = "humidity"; 
const char *VARIABLE_LABEL_FLAME = "Flame Sensor";
const char *VARIABLE_LABEL7 = "ultrasonic-distance";  


//Flame Sensor pins
#define FLAME_SENSOR_1 2  
#define FLAME_SENSOR_2 14 
#define FLAME_SENSOR_3 12 
#define FLAME_SENSOR_4 4 


//Temperature and Humidity pins
#define DHTPIN 15
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

  int f1 = 0;
    int f2 = 0;
    int f3 = 0;
    int f4 = 0;
const int TRIG_PIN = 0;  // Ultrasonic
const int ECHO_PIN = 13;  // Ultrasonic

long duration;
float distance;




const int PUBLISH_FREQUENCY = 1000;
unsigned long timer;

Ubidots ubidots(UBIDOTS_TOKEN);

void startCameraServer();
void setupLedFlash(int pin);



void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

float getDistance() {
  // Send a short LOW pulse to ensure a clean HIGH pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond HIGH pulse to trigger the sensor
 digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the time it takes for the echo to return
 long duration = pulseIn(ECHO_PIN, HIGH);
  
  
  
  float distance = duration * 0.0343 / 2;
  
  return distance;
}


void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(2000);
  Serial.println("ESP32 Booting...");

  pinMode(FLAME_SENSOR_1, INPUT);
  pinMode(FLAME_SENSOR_2, INPUT);
  pinMode(FLAME_SENSOR_3, INPUT);
  pinMode(FLAME_SENSOR_4, INPUT);
  

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

dht.begin();


 
  

  ubidots.connectToWifi(ssid, password);
  Serial.println("Connecting to WiFi...");
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
 Serial.println("Connected to Ubidots.");
  
  timer = millis();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  if (!ubidots.connected()) {
    ubidots.reconnect();
  }







  if ((millis() - timer) > PUBLISH_FREQUENCY) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    

    if (!isnan(temperature) && !isnan(humidity)) {
      Serial.printf("Temp: %.2f °C, Humidity: %.2f %%", temperature, humidity);
      ubidots.add(VARIABLE_LABEL1, temperature);
      ubidots.add(VARIABLE_LABEL2, humidity);
    }
ubidots.add(VARIABLE_LABEL7, distance);
    int f1 = 0;
    int f2 = 0;
    int f3 = 0;
    int f4 = 0;
    // Flame detection
    f1 = digitalRead(FLAME_SENSOR_1);
    f2 = digitalRead(FLAME_SENSOR_2);
    f3 = digitalRead(FLAME_SENSOR_3);
    f4 = digitalRead(FLAME_SENSOR_4);
    
    bool flameDetected = (f1 == HIGH || f2 == HIGH || f3 == HIGH || f4 == HIGH);
    Serial.printf("Flame Sensors: [%d, %d, %d, %d] => Fire: %s\n", f1, f2, f3, f4, flameDetected ? "YES" : "NO");
    ubidots.add(VARIABLE_LABEL_FLAME, flameDetected );


    
    
  

    
    
    
    float distance = getDistance();
    
     Serial.printf("Distance: %.2f cm\n", distance);


    ubidots.add(VARIABLE_LABEL7, distance);


    ubidots.publish(DEVICE_LABEL);

    timer = millis();


    
  }

  ubidots.loop();


   delay(1000);
}