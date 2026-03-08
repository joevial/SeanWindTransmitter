#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "driver/pulse_cnt.h"
//#include <ArduinoOTA.h>
#include <Adafruit_BMP280.h>
#include <AHTxx.h>
#include <Wire.h>
#include <ADS1115_WE.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <FastLED.h>
#include <Average.h>

// Reserve space for 10 entries in the average bucket.
// Change the type between < and > to change the entire way the library works.
Average<float> winddirAvg(60);
float transmitterAvgWindow = 60.0;   // Default 60 seconds
float transmitterGustWindow = 1.0;   // Default 1 second
// How many leds in your strip?
#define NUM_LEDS 1
#define PIN_DATA 48
float getInstantWindSpeed();
float calculateAverage(float* buffer, int size);
AHTxx aht(AHTXX_ADDRESS_X38, AHT2x_SENSOR);
const uint8_t I2C_ADDR = 0x76;
Adafruit_BMP280 bmp;
CRGB leds[NUM_LEDS];
float temp, hum, presread, volts0, abshum;

static uint8_t espnow_pmk[16] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                                  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
static uint8_t espnow_lmk[16] = {0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA,
                                  0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11, 0x22};
uint8_t receiverMAC[6] = {0x98, 0xA3, 0x16, 0xE4, 0x06, 0xD4}; // Your seantempLVGL MAC
bool credentialRequestPending = false;
bool receivedCredentials = false;
// GPIO pins
const int BUTTON_PIN = 1;

// Broadcast MAC for ESP-NOW
uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// WiFi credential message structure
typedef struct {
  char ssid[32];
  char password[64];
  uint8_t magic[4];
} wifi_credentials_t;

// Credential request message structure
typedef struct {
  uint8_t magic[4];  // Magic bytes: 0xDE, 0xAD, 0xBE, 0xEF
} credential_request_t;

// UDP slider values from receiver
typedef struct {
  float sliderAvg;
  float sliderGust;
  uint8_t magic[4];  // Magic bytes: 0xAB, 0xCD, 0xEF, 0x12
} slider_values_t;

// Calibration command from receiver
// magic: 0xCA, 0x1B, 0xCA, 0x1B  ("CALIB")
typedef struct {
  uint8_t mode;       // 1 = enter calibration, 0 = exit calibration
  uint8_t magic[4];
} calib_command_t;

// Calibration mode state
bool calibMode = false;

// Preferences for storing WiFi credentials
Preferences preferences;

// WiFi UDP for sending/receiving data
WiFiUDP udp;
IPAddress receiverIP;
const uint16_t UDP_PORT = 4210;

// State variables
bool hasWiFiCredentials = false;
bool wifiConnected = false;
bool listeningMode = false;
unsigned long ledBlinkTime = 0;
bool ledState = false;
unsigned long lastCredentialRequest = 0;
const unsigned long CREDENTIAL_REQUEST_INTERVAL = 5000; // Request every 5 seconds
unsigned long greenLedTime = 0;
bool showingGreenLed = false;

// Slider values received from base station
float sliderAvg = 0.0;
float sliderGust = 0.0;

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

// Global variables
pcnt_unit_handle_t pcnt_unit = NULL;
const int ANEMOMETER_PIN = 6;
const int WINDVANE_PIN = 8;
const float CALIBRATION_FACTOR = 2.4;
const int SAMPLE_INTERVAL_MS = 1000;
const int GLITCH_FILTER_NS = 4000;
float windVaneV = 0.0;

// ADS1115 for wind vane
ADS1115_WE ads1115 = ADS1115_WE(0x48);

const int AVG_WINDOW_SIZE = 60;
const int GUST_WINDOW_SIZE = 1;
float windSamples[AVG_WINDOW_SIZE];
int sampleIndex = 0;
int sampleCount = 0;

float currentWindSpeed = 0.0;
float averageWindSpeed = 0.0;
float windGust = 0.0;

unsigned long lastSampleTime = 0;
uint16_t readWindADC();
uint16_t readWindADC() {
    ads1115.setMeasureMode(ADS1115_SINGLE);
    ads1115.setCompareChannels(ADS1115_COMP_0_GND);
    ads1115.startSingleMeasurement();
    while (ads1115.isBusy()) {}
    return (uint16_t)ads1115.getRawResult();
}
// ESP-NOW callback when transmission is complete
void OnDataSent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
  // Callback for transmission status
}

const uint8_t SENSOR_ID = 0;  // Wind sensor is ID 0

// UPDATE the payload structure - ADD avgWindow and gustWindow fields
typedef struct {
  uint8_t sensor_id;
  float temp;
  float windgust;
  float avgwind;
  float instwind;
  float winddirV;
  float pres;
  float hum;
  float avgWindow;      // ADD THIS
  float gustWindow;     // ADD THIS
  uint8_t magic[4];     // Magic bytes: 0x57, 0x49, 0x4E, 0x44 ("WIND")
} esp_now_payload_wind_t;


void onESPNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(wifi_credentials_t)) {
    wifi_credentials_t creds;
    memcpy(&creds, data, sizeof(creds));
    
    // Verify magic bytes
    if (creds.magic[0] == 0xCA && creds.magic[1] == 0xFE &&
        creds.magic[2] == 0xBA && creds.magic[3] == 0xBE) {
      
      Serial.println("Received WiFi credentials!");
      Serial.printf("SSID: %s\n", creds.ssid);
      
      // Save to preferences
      preferences.begin("wifi", false);
      preferences.putString("ssid", String(creds.ssid));
      preferences.putString("password", String(creds.password));
      preferences.putBool("hasCredentials", true);
      preferences.end();
      
      // Store receiver MAC for future communication
      memcpy(receiverMAC, info->src_addr, 6);
      
      receivedCredentials = true;
      hasWiFiCredentials = true;
      listeningMode = false;
      leds[0] = CRGB::Black;
      FastLED.show();
      
      Serial.println("Credentials saved, restarting...");
      delay(1000);
      ESP.restart();
    }
  }
}

bool connectToWiFi() {
  preferences.begin("wifi", true);
  hasWiFiCredentials = preferences.getBool("hasCredentials", false);
  
  if (!hasWiFiCredentials) {
    preferences.end();
    return false;
  }
  
  String savedSSID = preferences.getString("ssid", "");
  String savedPassword = preferences.getString("password", "");
  preferences.end();
  
  if (savedSSID.length() == 0) {
    return false;
  }
  
  Serial.printf("Connecting to WiFi: %s\n", savedSSID.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(savedSSID.c_str(), savedPassword.c_str());
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    wifiConnected = true;
    
    // Start UDP listener
    udp.begin(UDP_PORT);
    Serial.printf("UDP listener started on port %d\n", UDP_PORT);
    
    return true;
  } else {
    Serial.println("\nWiFi connection failed!");
    // Clear bad credentials
    preferences.begin("wifi", false);
    preferences.clear();
    preferences.end();
    return false;
  }
}


void sendCredentialRequest() {
  credential_request_t request;
  request.magic[0] = 0xDE;
  request.magic[1] = 0xAD;
  request.magic[2] = 0xBE;
  request.magic[3] = 0xEF;
  
  Serial.println("Sending credential request across all channels...");
  
  // Scan all channels to find the receiver
  for (int channel = 1; channel <= 11; channel++) {
    WiFi.setChannel(channel);
    delay(50);
    
    if (esp_now_is_peer_exist(receiverMAC)) {
      esp_now_del_peer(receiverMAC);
    }
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = channel;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      for (int i = 0; i < 3; i++) {
        esp_now_send(receiverMAC, (uint8_t *)&request, sizeof(request));
        delay(50);
      }
    }
    delay(50);
  }
  
  Serial.println("Credential request sent");
}

void enterListeningMode() {
  listeningMode = true;
  Serial.println("Entering ESP-NOW listening mode...");
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(onESPNowRecv);
  
  Serial.println("Listening for WiFi credentials...");
  credentialRequestPending = true;
}

// UPDATE the handleUDPData() function to receive and store slider values:
void handleUDPData() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    uint8_t buffer[256];
    int len = udp.read(buffer, sizeof(buffer));
    
    if (len == sizeof(slider_values_t)) {
      slider_values_t sliders;
      memcpy(&sliders, buffer, sizeof(sliders));
      
      // Verify magic bytes
      if (sliders.magic[0] == 0xAB && sliders.magic[1] == 0xCD &&
          sliders.magic[2] == 0xEF && sliders.magic[3] == 0x12) {
        
        // UPDATE: Store the received values
        transmitterAvgWindow = sliders.sliderAvg;
        transmitterGustWindow = sliders.sliderGust;
        
        // Flash green LED for 200ms
        leds[0] = CRGB::Green;
        FastLED.show();
        showingGreenLed = true;
        greenLedTime = millis();
        
        Serial.printf("Received slider values - Avg: %.1f, Gust: %.1f\n", 
                      transmitterAvgWindow, transmitterGustWindow);
      }
    } else if (len == sizeof(calib_command_t)) {
      calib_command_t cmd;
      memcpy(&cmd, buffer, sizeof(cmd));
      if (cmd.magic[0] == 0xCA && cmd.magic[1] == 0x1B &&
          cmd.magic[2] == 0xCA && cmd.magic[3] == 0x1B) {
        calibMode = (cmd.mode == 1);
        Serial.printf("Calibration mode: %s\n", calibMode ? "ON" : "OFF");
        // Flash LED cyan briefly to acknowledge
        leds[0] = calibMode ? CRGB::Cyan : CRGB::Green;
        FastLED.show();
        showingGreenLed = true;
        greenLedTime = millis();
      }
    }
  }
}

// UPDATE the sendDataViaUDP() function to include window settings:
void sendDataViaUDP() {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  esp_now_payload_wind_t payload;
  payload.sensor_id = SENSOR_ID;
  payload.temp = temp;
  payload.windgust = windGust;
  payload.avgwind = averageWindSpeed;
  payload.instwind = currentWindSpeed;
  payload.winddirV = windVaneV;
  payload.pres = presread;
  payload.hum = hum;
  payload.avgWindow = transmitterAvgWindow;    // ADD THIS
  payload.gustWindow = transmitterGustWindow;  // ADD THIS
  payload.magic[0] = 0x57;  // 'W'
  payload.magic[1] = 0x49;  // 'I'
  payload.magic[2] = 0x4E;  // 'N'
  payload.magic[3] = 0x44;  // 'D'
  
  // Broadcast to subnet
  IPAddress broadcastIP = WiFi.localIP();
  broadcastIP[3] = 255;
  
  for (int i = 0; i < 3; i++) {
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write((uint8_t *)&payload, sizeof(payload));
    udp.endPacket();
    delay(50);  // Small delay between packets
  }
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  
  for (int i = 0; i < 3; i++) {
    udp.beginPacket(broadcastIP, UDP_PORT);
    udp.write((uint8_t *)&payload, sizeof(payload));
    udp.endPacket();
    delay(50);  // Small delay between packets
  }
  Serial.printf("Sensor %d: Wind data sent via UDP (Avg Window: %.1f, Gust Window: %.1f)\n", 
                SENSOR_ID, transmitterAvgWindow, transmitterGustWindow);
}
void initPulseCounter() {
  pcnt_unit_config_t unit_config = {
    .low_limit = -32768,
    .high_limit = 32767,
  };
  
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
  
  pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = GLITCH_FILTER_NS,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
  
  pcnt_chan_config_t chan_config = {
    .edge_gpio_num = ANEMOMETER_PIN,
    .level_gpio_num = -1,
  };
  
  pcnt_channel_handle_t pcnt_chan = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));
  
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, 
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_HOLD));
  
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP));
  
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void updateWindData() {
  currentWindSpeed = getInstantWindSpeed();
  
  windSamples[sampleIndex] = currentWindSpeed;
  sampleIndex = (sampleIndex + 1) % AVG_WINDOW_SIZE;
  if (sampleCount < AVG_WINDOW_SIZE) {
    sampleCount++;
  }
  
  averageWindSpeed = calculateAverage(windSamples, sampleCount);
  
  windGust = 0.0;
  if (sampleCount >= GUST_WINDOW_SIZE) {
    for (int i = 0; i <= sampleCount - GUST_WINDOW_SIZE; i++) {
      float sum = 0.0;
      for (int j = 0; j < GUST_WINDOW_SIZE; j++) {
        int idx = (sampleIndex - sampleCount + i + j + AVG_WINDOW_SIZE) % AVG_WINDOW_SIZE;
        sum += windSamples[idx];
      }
      float gustAvg = sum / GUST_WINDOW_SIZE;
      if (gustAvg > windGust) {
        windGust = gustAvg;
      }
    }
  } else {
    windGust = currentWindSpeed;
  }
}

float getInstantWindSpeed() {
  int pulse_count = 0;
  ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  
  float pulses_per_second = (float)pulse_count / (SAMPLE_INTERVAL_MS / 1000.0);
  float wind_speed = pulses_per_second * CALIBRATION_FACTOR;
  
  return wind_speed;
}



float calculateAverage(float* buffer, int size) {
  if (size == 0) return 0.0;
  
  float sum = 0.0;
  for (int i = 0; i < size; i++) {
    sum += buffer[i];
  }
  return sum / size;
}

bool connected = false;

void setup() {
  //Serial.begin(115200);
  //delay(1000);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize FastLED
  FastLED.addLeds<NEOPIXEL, PIN_DATA>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();
  
  Wire.begin(9, 10);
  
  ads1115.init();
  ads1115.setVoltageRange_mV(ADS1115_RANGE_4096);  // +/-4.096V range for 3.3V signal
  ads1115.setConvRate(ADS1115_8_SPS);
  ads1115.setMeasureMode(ADS1115_SINGLE);
  
  aht.begin();
  temp = aht.readTemperature();
  hum = aht.readHumidity(AHTXX_USE_READ_DATA);
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  bmp.takeForcedMeasurement();
  presread = bmp.readPressure() / 100.0F;
  
  Serial.println("ESP32-S3 Weather Transmitter");
  Serial.println("Initializing pulse counter...");
  
  initPulseCounter();
  
  Serial.println("Pulse counter initialized!");
  
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  
  bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);
  
  if (buttonPressed) {
    Serial.println("Button pressed - clearing credentials and entering listening mode");
    preferences.begin("wifi", false);
    preferences.clear();
    preferences.end();
    enterListeningMode();
  } else {
    if (!connectToWiFi()) {
      Serial.println("No valid WiFi credentials, entering listening mode");
      enterListeningMode();
    }
  }
}




void loop() {
  if (!connected && WiFi.status() == WL_CONNECTED) {
    connected = true;

  }

  // Handle green LED timeout
  if (showingGreenLed && (millis() - greenLedTime > 200)) {
    showingGreenLed = false;
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  
  // Handle listening mode
  if (listeningMode) {
    // LED blinking white
    if (millis() - ledBlinkTime > 500) {
      ledBlinkTime = millis();
      ledState = !ledState;
      leds[0] = ledState ? CRGB::White : CRGB::Black;
      FastLED.show();
    }
    
    if (credentialRequestPending) {
      sendCredentialRequest();
      credentialRequestPending = false;
      lastCredentialRequest = millis();
    }
    
    if (millis() - lastCredentialRequest > CREDENTIAL_REQUEST_INTERVAL) {
      sendCredentialRequest();
      lastCredentialRequest = millis();
    }
  }

  every(1000) {
    updateWindData();

    // Read raw ADC from wind vane and store it. The receiver is responsible
    // for snapping to calibrated directions and taking the median of 20
    // readings before committing to history.
    windVaneV = readWindADC();

    if (wifiConnected) {
      // Check for incoming UDP data
      handleUDPData();
    }
  }

  // In calibration mode: also sample wind vane at 500ms and transmit rapidly
  static uint32_t __calibTx__ = 0;
  if (calibMode && wifiConnected && (millis() - __calibTx__ >= 500)) {
    __calibTx__ = millis();
    // Take a fresh ADC reading for the wind vane (raw, no median, for live calib)
    windVaneV = readWindADC();
    sendDataViaUDP();
  }

  // Sample temp/hum/pressure every 10 seconds (prevent self-heating)
  every(10000) {
    temp = aht.readTemperature();
    hum = aht.readHumidity(AHTXX_USE_READ_DATA);
    bmp.begin();
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmp.takeForcedMeasurement();
    presread = bmp.readPressure() / 100.0F;
  }

  // Transmit every 3 seconds using the most recently saved sensor values
  every(3000) {
    if (wifiConnected && !calibMode) {
      sendDataViaUDP();
    }
  }
  
  // Check WiFi connection status
  if (wifiConnected && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Entering listening mode...");
    wifiConnected = false;
    preferences.begin("wifi", false);
    preferences.clear();
    preferences.end();
    enterListeningMode();
  }
}