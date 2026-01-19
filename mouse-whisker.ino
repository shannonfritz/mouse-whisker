  /*
    ESP32 Mouse Whisker
    - USB HID mouse (ESP32-S3/S2 only)
    - BLE HID mouse via NimBLE (ESP32, ESP32-C3, ESP32-S3)
    - WiFi connectivity (for WebUI and/or MQTT)
    - MQTT + Home Assistant auto-discovery (optional)
    - Local WebUI for browser-based configuration
    - Randomized stealthy movements
    - Persistent settings via Preferences
    
    Supported Boards:
    - ESP32-C3 Super Mini: BLE + WiFi (no USB HID)
    - ESP32-S3 Super Mini: BLE + USB HID + WiFi
    - ESP32-S2 Super Mini: USB HID + WiFi (no Bluetooth)
    - ESP32 NodeMCU: BLE + WiFi
    - ESP32 D1 Mini: BLE + WiFi
  
    Dependencies (install via Arduino Library Manager):
    - NimBLE-Arduino by h2zero (for BLE HID)
    - PubSubClient by Nick O'Leary (for MQTT)
    - Adafruit NeoPixel (for RGB LED on S3/S2 boards)
  */

  // Firmware version (Semantic Versioning: MAJOR.MINOR.PATCH)
  #define FIRMWARE_VERSION "1.5.0"

  // WiFi connection (required for WebUI on your network or MQTT)
  #define ENABLE_WIFI true

  // MQTT + Home Assistant integration (requires ENABLE_WIFI)
  #define ENABLE_MQTT true

  // Set to false to disable LED status indicator (works with ESP32 D1mini and ESP32-C3/S3/S2 SuperMini boards)
  #define ENABLE_LED true
  
  // Set to false to disable BLE mouse emulation (works on ESP32 boards with BLE support (ex: ESP32 D1mini and ESP32-C3/S3/S2 SuperMini boards)
  #define ENABLE_BLE true

  // Set to false to disable USB mouse emulation (works on ESP32 Boards with HID (ex: ESP32-S3/S2 SuperMini boards)
  #define ENABLE_USB false

  // Set to true to enable local WebUI for configuration (works with or without MQTT)
  // If ENABLE_WIFI is true, WebUI runs on your WiFi network
  // If ENABLE_WIFI is false, WebUI creates its own AP hotspot
  #define ENABLE_WEBUI true

  // Validate: MQTT requires WiFi
  #if ENABLE_MQTT && !ENABLE_WIFI
    #error "ENABLE_MQTT requires ENABLE_WIFI to be true"
  #endif

  #if ENABLE_WIFI
    #include <WiFi.h>
  #endif

  #if ENABLE_MQTT
    #include <PubSubClient.h>
  #endif

  #if ENABLE_WEBUI
    #if !ENABLE_WIFI
      #include <WiFi.h>  // Need WiFi for AP mode even without MQTT
    #endif
    #include <WebServer.h>
    #include <DNSServer.h>  // For captive portal in AP mode
    // AP mode settings
    // AP_PASS: Leave empty for open network, or set 8+ chars for WPA2
    const char* AP_PASS = "";  // Default: open network (no password)
    const byte DNS_PORT = 53;
  #endif
  
  #if ENABLE_USB
    // Check for incompatible board selection
    #if defined(CONFIG_IDF_TARGET_ESP32C3)
      #error "USB HID not supported on ESP32-C3. Set ENABLE_USB to false and use ENABLE_BLE instead, or select ESP32-S3 board."
    #endif
  
    #include "USB.h"
    #include "USBHID.h"
  #endif
  
  #if ENABLE_BLE
    // Using NimBLE for smaller binary size (~400KB savings vs Bluedroid)
    // Install via: Arduino Library Manager -> "NimBLE-Arduino" by h2zero
    #include <NimBLEDevice.h>
    #include <NimBLEHIDDevice.h>
  #endif
  
  // Include NeoPixel for S3/S2 boards (needed before LED_TYPE_RGB is defined)
  #if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2)
    #include <Adafruit_NeoPixel.h>
  #endif
  
  #include <Preferences.h>

  // ----- CONFIG: WiFi credentials -----
  // Option 1: Set at compile time (replace these values)
  // Option 2: Leave as "SET_IN_WEBUI" and configure via WebUI AP mode
  #if ENABLE_WIFI
    const char* DEFAULT_WIFI_SSID = "SET_IN_WEBUI";
    const char* DEFAULT_WIFI_PASS = "SET_IN_WEBUI";
  #endif
  // ----- CONFIG: MQTT credentials -----
  // Option 1: Set at compile time (replace these values)
  // Option 2: Leave as "SET_IN_WEBUI" and configure via WebUI
  #if ENABLE_MQTT
    const char* DEFAULT_MQTT_SERVER = "SET_IN_WEBUI";
    const uint16_t DEFAULT_MQTT_PORT = 1883;
    const char* DEFAULT_MQTT_USER = "SET_IN_WEBUI";
    const char* DEFAULT_MQTT_PASS = "SET_IN_WEBUI";
  #endif

  // default settings (can be persisted)
  // BLE advertising name limit: 29 chars max (31 bytes minus 2 for flags/type)
  // With unique ID suffix (e.g. " 8857"): max 24 chars for name
  // Without unique ID suffix: max 29 chars for name
  const char* DEFAULT_DEVICE_NAME = "Mouse Whisker";
  const bool DEFAULT_APPEND_UNIQUE_ID = true;  // Default: append unique ID to device name
  const bool DEFAULT_LED_ENABLED = true;       // Default: LED status indicators enabled
  const int DEFAULT_X_RANGE = 5;
  const int DEFAULT_Y_RANGE = 5;
  const int DEFAULT_MIN_MOVE_INTERVAL = 60;   // seconds
  const int DEFAULT_MAX_MOVE_INTERVAL = 240;  // seconds

  // Allowed bounds for min/max move interval (compile-time editable)
  const int ALLOWED_MIN_MOVE_SECONDS = 1;    // smallest allowed (seconds)
  const int ALLOWED_MAX_MOVE_SECONDS = 600;  // largest allowed (seconds)

  #if ENABLE_LED
  // LED configuration per chip type
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3 boards often have RGB LED on GPIO48 or single LED on GPIO38/47
    // Uncomment ONE of these based on your board:
    #define LED_TYPE_RGB      // WS2812/Neopixel RGB LED on GPIO48
    // #define LED_TYPE_SINGLE   // Single LED (uncomment and set pin below)
    
    #ifdef LED_TYPE_RGB
      const int LED_PIN = 48;       // GPIO48 for most S3 boards with RGB
      const bool LED_ACTIVE_LOW = false;
    #else
      const int LED_PIN = 38;       // Common single LED pin on S3 boards
      const bool LED_ACTIVE_LOW = false;  // Usually active-high on S3
    #endif
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)
    // ESP32-S2 Super Mini has WS2812 RGB LED on GPIO18
    #define LED_TYPE_RGB
    const int LED_PIN = 18;
    const bool LED_ACTIVE_LOW = false;
  #elif defined(CONFIG_IDF_TARGET_ESP32C3)
    // ESP32-C3 SuperMini has single LED on GPIO8
    #define LED_TYPE_SINGLE
    const int LED_PIN = 8;
    const bool LED_ACTIVE_LOW = true;  // Active-low on C3 SuperMini
  #else
    // Default/fallback for other ESP32 variants
    #define LED_TYPE_SINGLE
    const int LED_PIN = 2;            // GPIO2 common on ESP32 classic
    const bool LED_ACTIVE_LOW = false;
  #endif

  const int LED_PWM_CHANNEL = 0;
  const int LED_PWM_FREQ = 5000;
  const int LED_PWM_RESOLUTION = 8;  // 8-bit (0-255)

  // Forward declaration for LED function (needed by BLE callbacks)
  void ledBlink(int times, int delayMs = 100);
  #endif // ENABLE_LED

  // Shared HID Mouse Report Descriptor (used by both USB and BLE)
  #if ENABLE_USB || ENABLE_BLE
  const uint8_t mouseHidReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1) - Required for BLE HID
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Buttons)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x03,        //     Usage Maximum (Button 3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data, Variable, Absolute)
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x01,        //     Input (Constant) - Padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x09, 0x38,        //     Usage (Wheel)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x03,        //     Report Count (3)
    0x81, 0x06,        //     Input (Data, Variable, Relative)
    0xC0,              //   End Collection
    0xC0               // End Collection
  };
  #endif

  #if ENABLE_USB
    // Custom HID class for mouse
    class USBHIDMouse : public USBHIDDevice {
    public:
      USBHID hid;
      
      USBHIDMouse() {
        static bool initialized = false;
        if (!initialized) {
          initialized = true;
          hid.addDevice(this, sizeof(mouseHidReportDescriptor));
        }
      }
      
      void begin() {
        hid.begin();
      }
      
      uint16_t _onGetDescriptor(uint8_t* buffer) {
        memcpy(buffer, mouseHidReportDescriptor, sizeof(mouseHidReportDescriptor));
        return sizeof(mouseHidReportDescriptor);
      }
      
      void sendReport(uint8_t* report, size_t len) {
        hid.SendReport(1, report, len);
      }
    };
    
    USBHIDMouse HID;
  #endif
  
  #if ENABLE_BLE
  NimBLEHIDDevice* hid;
  NimBLECharacteristic* input;
  NimBLEServer* pServer = nullptr;
  bool bleConnected = false;
  String bleClientAddress = "";
  
  class BleConnectionCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
      Serial.println(">>> BLE onConnect callback start");
      bleConnected = true;
      bleClientAddress = connInfo.getAddress().toString().c_str();
      Serial.println("BLE Client connected");
      Serial.printf("  Host address: %s\n", connInfo.getAddress().toString().c_str());
      Serial.printf("  Address type: %s\n", connInfo.getAddress().isPublic() ? "Public" : "Random");
      Serial.printf("  Connection handle: %d\n", connInfo.getConnHandle());
      Serial.printf("  Connection interval: %.2f ms\n", connInfo.getConnInterval() * 1.25);
      Serial.printf("  Connection latency: %d\n", connInfo.getConnLatency());
      Serial.printf("  Supervision timeout: %d ms\n", connInfo.getConnTimeout() * 10);
      Serial.printf("  MTU: %d\n", pServer->getPeerMTU(connInfo.getConnHandle()));
      // Note: MQTT publish and LED blink moved to loop() to avoid blocking BLE stack
      Serial.println(">>> BLE onConnect callback end");
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      Serial.printf(">>> BLE onDisconnect callback (reason: %d)\n", reason);
      bleConnected = false;
      bleClientAddress = "";
      Serial.println("BLE Client disconnected");
      // Note: MQTT publish moved to loop() to avoid blocking BLE stack
      // Restart advertising
      NimBLEDevice::startAdvertising();
      Serial.println("BLE advertising restarted");
    }
  };
  #endif
  
  Preferences preferences;

  #ifdef LED_TYPE_RGB
    Adafruit_NeoPixel strip(1, LED_PIN, NEO_GRB + NEO_KHZ800);
  #endif

  #if ENABLE_MQTT
    // MQTT base topics (will include unique id)
    String MQTT_TOPIC_CMD;
    String MQTT_TOPIC_STATE;
    String MQTT_TOPIC_CONFIG;
    String MQTT_TOPIC_AVAILABILITY;
    String MQTT_TOPIC_EVENT;

    String MQTT_TOPIC_SET_ENABLE;
    String MQTT_TOPIC_SET_XRANGE;
    String MQTT_TOPIC_SET_YRANGE;
    String MQTT_TOPIC_SET_MINMOVE;
    String MQTT_TOPIC_SET_MAXMOVE;
    String MQTT_TOPIC_SET_NAME;
    String MQTT_TOPIC_SET_APPENDID;
    String MQTT_TOPIC_SET_LED;
    String MQTT_TOPIC_CMD_WHISK;
    String MQTT_TOPIC_CMD_REBOOT;
    String MQTT_TOPIC_CMD_FACTORY_RESET;
    String MQTT_TOPIC_WHISKING;
    String MQTT_TOPIC_BLE_CONNECTED;
    String MQTT_TOPIC_BLE_HOST;
    String MQTT_TOPIC_USB_MOUNTED;
    String MQTT_TOPIC_WIFI_RSSI;

    WiFiClient wifiClient;
    PubSubClient mqttClient(wifiClient);
  #endif

  #if ENABLE_WEBUI
    WebServer webServer(80);
    DNSServer dnsServer;  // For captive portal in AP mode
  #endif

  // Unique ID and device name
  char uniqueId[12];
  String deviceName;
  bool appendUniqueId = DEFAULT_APPEND_UNIQUE_ID;
  #if ENABLE_LED
    bool ledEnabled = DEFAULT_LED_ENABLED;
  #endif

  #if ENABLE_WIFI
    // Runtime WiFi credentials (loaded from preferences or compiled defaults)
    String wifiSSID;
    String wifiPassword;
    bool wifiAPMode = false;  // True if running in AP mode
    bool portalPageSeen = false;  // Track if user has seen portal landing (iOS CNA workaround)
  #endif

  #if ENABLE_MQTT
    // Runtime MQTT settings (loaded from preferences or compiled defaults)
    String mqttServer;
    uint16_t mqttPort;
    String mqttUser;
    String mqttPassword;
  #endif

  // movement settings
  int xRange = DEFAULT_X_RANGE;
  int yRange = DEFAULT_Y_RANGE;
  int minMoveInterval = DEFAULT_MIN_MOVE_INTERVAL;
  int maxMoveInterval = DEFAULT_MAX_MOVE_INTERVAL;

  // state
  bool periodicOn = true;
  unsigned long lastMove = 0;
  unsigned long moveInterval = 0; // ms (randomized)

  #if ENABLE_LED
  // LED state for non-blocking pulse effect
  unsigned long ledLastUpdate = 0;
  int ledBrightness = 5;             // Current brightness (0-255)
  int ledFadeAmount = 1;             // Brightness change per step (1-5 typical; larger = faster/choppier)
  const int LED_FADE_INTERVAL = 15;  // Milliseconds between brightness changes
  const int LED_MIN_BRIGHTNESS = 5;  // Minimum brightness (0-255)
  const int LED_MAX_BRIGHTNESS = 80; // Maximum brightness for pulse (0-255)
  
  // LED event blink state
  bool ledBlinking = false;
  int ledBlinkCount = 0;
  int ledBlinkTarget = 0;
  unsigned long ledBlinkLastChange = 0;
  int ledBlinkDelay = 100;
  bool ledBlinkState = false;
  #endif // ENABLE_LED

  // Forward declarations
  #if ENABLE_WIFI
    void connectWiFi();
  #endif
  #if ENABLE_MQTT
    void connectMQTT();
    void publishState(const char* state);
    void publishConfigJSON();
    void publishEventJSON(const char* event, const String &value);
    void publishDiscovery();
    void mqttCallback(char* topic, byte* payload, unsigned int length);
  #endif
  
  void sendMouseMove(int8_t x, int8_t y);
  void moveMouse();
  void printSettingsToSerial();
  String sanitizeDeviceName(const String &in, size_t maxLen = 24);
  #if ENABLE_LED
    void updateLED();
  #endif
  #if ENABLE_WEBUI
    void setupWebUI();
    void handleWebRoot();
    void handleWebUpdate();
    void handleWebWhisk();
    void handleWebWiFi();
    #if ENABLE_MQTT
      void handleWebMQTT();
    #endif
    void handleCaptivePortal();
    void handleCaptiveDetect();
    String generateWebPage();
  #endif

  // ---------- Implementation ----------

  // Print current settings to serial (can be called anytime)
  void printSettingsToSerial() {
    Serial.printf("Device Name: %s\n", deviceName.c_str());
    Serial.printf("Unique ID: %s\n", uniqueId);
    Serial.printf("Append ID to Name: %s\n", appendUniqueId ? "Yes" : "No");
    Serial.printf("Random Whisk: %s\n", periodicOn ? "Running" : "Stopped");
    Serial.printf("Movement Range: X=%d, Y=%d pixels\n", xRange, yRange);
    Serial.printf("Move Interval: %d-%d seconds\n", minMoveInterval, maxMoveInterval);
    #if ENABLE_LED
      Serial.printf("Status LED: %s\n", ledEnabled ? "Enabled" : "Disabled");
    #endif
    #if ENABLE_BLE
      Serial.printf("BLE Mouse: Enabled (connected: %s)\n", bleConnected ? "Yes" : "No");
    #endif
    #if ENABLE_USB
      Serial.println("USB Mouse: Enabled");
    #endif
    #if ENABLE_WIFI
      Serial.printf("WiFi: %s (RSSI: %d dBm)\n", 
                    WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                    WiFi.RSSI());
    #endif
    Serial.println("------------------------");
  }
  
  // Pick a new random move interval and reset the timer
  void pickNextMoveInterval() {
    moveInterval = (unsigned long)random(minMoveInterval, maxMoveInterval + 1) * 1000UL;
    lastMove = millis();
    if (periodicOn) {
      Serial.printf("Next whisk in %lu seconds\n", moveInterval / 1000UL);
    } else {
      Serial.println("Random whisk is paused");
    }
  }
  
  #if ENABLE_LED
  // Helper function to write LED value (handles active-low logic)
  void ledWrite(uint8_t brightness) {
    #ifdef LED_TYPE_RGB
      // RGB LED: use color based on brightness (cyan pulse)
      uint8_t r = brightness / 4;
      uint8_t g = brightness / 2;
      uint8_t b = brightness;
      strip.setPixelColor(0, strip.Color(r, g, b));
      strip.show();
    #else
      // Single LED: apply active-low inversion if needed
      uint8_t value = LED_ACTIVE_LOW ? (255 - brightness) : brightness;
      ledcWrite(LED_PIN, value);
    #endif
  }
  
  // Non-blocking LED update function - call this from loop()
  void updateLED() {
    if (!ledEnabled) {
      // Ensure LED is off when disabled
      static bool wasDisabled = false;
      if (!wasDisabled) {
        ledWrite(0);
        wasDisabled = true;
      }
      return;
    }
    
    unsigned long now = millis();
    
    // Handle event blinks (higher priority)
    if (ledBlinking) {
      if (now - ledBlinkLastChange >= ledBlinkDelay) {
        ledBlinkLastChange = now;
        ledBlinkState = !ledBlinkState;
        
        if (ledBlinkState) {
          ledWrite(255);  // Full brightness for blink
        } else {
          ledWrite(0);    // Off
          ledBlinkCount++;
          if (ledBlinkCount >= ledBlinkTarget) {
            ledBlinking = false;  // Done blinking, return to pulse
          }
        }
      }
      return;  // Skip pulse effect while blinking
    }
    
    // Simple linear fade effect (breathing)
    if (now - ledLastUpdate >= LED_FADE_INTERVAL) {
      ledLastUpdate = now;
      
      // Change brightness
      ledBrightness += ledFadeAmount;
      
      // Reverse direction at the limits
      if (ledBrightness <= LED_MIN_BRIGHTNESS || ledBrightness >= LED_MAX_BRIGHTNESS) {
        ledFadeAmount = -ledFadeAmount;
      }
      
      ledWrite(ledBrightness);
    }
  }
  
  // Trigger a non-blocking blink sequence
  void ledBlink(int times, int delayMs) {
    if (!ledEnabled) return;  // Skip if LED disabled
    ledBlinking = true;
    ledBlinkCount = 0;
    ledBlinkTarget = times;
    ledBlinkDelay = delayMs;
    ledBlinkState = false;
    ledBlinkLastChange = millis();
  }
  #endif // ENABLE_LED
  
  void publishState(const char* state) {
    #if ENABLE_MQTT
    mqttClient.publish(MQTT_TOPIC_STATE.c_str(), state, true);
    #endif
  }

  void publishConfigJSON() {
    #if ENABLE_MQTT
    String payload = "{";
    payload += "\"state\": \"" + String(periodicOn ? "running" : "stopped") + "\",";
    payload += "\"deviceName\": \"" + deviceName + "\",";
    payload += "\"xRange\": " + String(xRange) + ",";
    payload += "\"yRange\": " + String(yRange) + ",";
    payload += "\"minMoveInterval\": " + String(minMoveInterval) + ",";
    payload += "\"maxMoveInterval\": " + String(maxMoveInterval) + ",";
    payload += "\"appendUniqueId\": " + String(appendUniqueId ? "true" : "false") + ",";
    int rssi = WiFi.RSSI();
    String quality = (rssi >= -50) ? "Excellent" : (rssi >= -60) ? "Good" : (rssi >= -70) ? "Fair" : "Weak";
    payload += "\"wifiRssi\": " + String(rssi) + ",";
    payload += "\"wifiQuality\": \"" + quality + "\",";
    #if ENABLE_LED
      payload += "\"ledEnabled\": " + String(ledEnabled ? "true" : "false") + "}";
    #else
      payload += "\"ledEnabled\": false}";
    #endif
    mqttClient.publish(MQTT_TOPIC_CONFIG.c_str(), payload.c_str(), true);
    #endif
  }

  void publishEventJSON(const char* event, const String &value) {
    #if ENABLE_MQTT
    String payload = "{";
    payload += "\"event\": \"" + String(event) + "\",";
    payload += "\"value\": \"" + value + "\"}";
    mqttClient.publish(MQTT_TOPIC_EVENT.c_str(), payload.c_str(), false);
    #endif
  }

  void sendMouseMove(int8_t x, int8_t y) {
    uint8_t report[4] = {0};  // buttons, x, y, wheel
    report[0] = 0;            // no buttons pressed
    report[1] = x;            // x movement
    report[2] = y;            // y movement
    report[3] = 0;            // no wheel movement
    
    #if ENABLE_USB
      HID.sendReport(report, sizeof(report));
    #endif
    
    #if ENABLE_BLE
      if (bleConnected && input != nullptr) {
        input->setValue(report, sizeof(report));
        input->notify();
      }
    #endif
    
    #if ENABLE_LED
      // Single quick flash for mouse movement
      ledBlink(1, 50);
    #endif
  }

  void moveMouse() {
    #if ENABLE_MQTT
      mqttClient.publish(MQTT_TOPIC_WHISKING.c_str(), "ON", false);
    #endif
    
    int x = random(-xRange, xRange + 1);
    int y = random(-yRange, yRange + 1);
    
    // Move out
    sendMouseMove((int8_t)x, (int8_t)y);
    delay(50);  // Brief pause to ensure host registers the movement
    // Move back to original position
    sendMouseMove((int8_t)-x, (int8_t)-y);
    
    Serial.printf("Whisked mouse: ±%d, ±%d\n", x, y);
    publishEventJSON("whisked", String("dx=") + String(x) + ",dy=" + String(y));
    
    #if ENABLE_MQTT
      mqttClient.publish(MQTT_TOPIC_WHISKING.c_str(), "OFF", false);
    #endif
  }

  #if ENABLE_MQTT
  void publishDiscovery() {
    Serial.println("Publishing Home Assistant discovery...");
    String baseId = String("mousewhisker_") + String(uniqueId);
    Serial.printf("Device ID: %s\n", baseId.c_str());
    
    // Helper lambda to yield and process MQTT between publishes
    // This prevents watchdog timeout and WiFi stack starvation
    auto yieldAndProcess = [](const char* step) {
      yield();
      mqttClient.loop();
      delay(50);  // Let WiFi/BLE stacks breathe
    };
    
    // Build full device info (used in first discovery payload)
    String deviceFull = "{\"identifiers\": [\"" + baseId + "\"],\"name\": \"Mouse Whisker " + String(uniqueId) + "\",\"sw_version\": \"" + String(FIRMWARE_VERSION) + "\",\"manufacturer\": \"Shannon Fritz\",\"model\": \"ESP32 Mouse Whisker\"";
    #if ENABLE_WEBUI
      deviceFull += ",\"configuration_url\": \"http://" + WiFi.localIP().toString() + "/\"";
    #endif
    deviceFull += "}";
    
    // Abbreviated device info (used in subsequent payloads)
    String deviceShort = "{\"identifiers\": [\"" + baseId + "\"],\"name\": \"Mouse Whisker " + String(uniqueId) + "\"}";
    
    // Random whisk enable switch discovery payload
    String swTopic = String("homeassistant/switch/mousewhisker_") + String(uniqueId) + "/random/config";
    String sw = "{";
    sw += "\"name\": \"Whisk Randomly\",";
    sw += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_random\",";
    sw += "\"command_topic\": \"" + MQTT_TOPIC_SET_ENABLE + "\",";
    sw += "\"state_topic\": \"" + MQTT_TOPIC_STATE + "\",";
    sw += "\"state_on\": \"running\",";
    sw += "\"state_off\": \"stopped\",";
    sw += "\"payload_on\": \"1\",";
    sw += "\"payload_off\": \"0\",";
    sw += "\"optimistic\": false,";
    sw += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    sw += "\"icon\": \"mdi:dice-multiple\",";
    sw += "\"unique_id\": \"" + baseId + "_random\",";
    sw += "\"device\": " + deviceFull + "}";
    
    bool result = mqttClient.publish(swTopic.c_str(), sw.c_str(), true);
    Serial.printf("Discovery publish: %s\n", result ? "OK" : "FAILED");
    yieldAndProcess("switch");

    // Whisking binary sensor
    String whiskingTopic = String("homeassistant/binary_sensor/mousewhisker_") + String(uniqueId) + "/whisking/config";
    String whisking = "{";
    whisking += "\"name\": \"Whisking\",";
    whisking += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_whisking\",";
    whisking += "\"state_topic\": \"" + MQTT_TOPIC_WHISKING + "\",";
    whisking += "\"payload_on\": \"ON\",";
    whisking += "\"payload_off\": \"OFF\",";
    whisking += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    whisking += "\"icon\": \"mdi:cursor-move\",";
    whisking += "\"unique_id\": \"" + baseId + "_whisking\",";
    whisking += "\"device\": " + deviceShort + "}";
    mqttClient.publish(whiskingTopic.c_str(), whisking.c_str(), true);
    yieldAndProcess("whisking");

    // Whisk button
    String btnTopic = String("homeassistant/button/mousewhisker_") + String(uniqueId) + "/whisk/config";
    String btn = "{";
    btn += "\"name\": \"Whisk Now\",";
    btn += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_whisk\",";
    btn += "\"command_topic\": \"" + MQTT_TOPIC_CMD_WHISK + "\",";
    btn += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    btn += "\"icon\": \"mdi:cursor-move\",";
    btn += "\"unique_id\": \"" + baseId + "_whisk\",";
    btn += "\"device\": " + deviceShort + "}";
    mqttClient.publish(btnTopic.c_str(), btn.c_str(), true);
    yieldAndProcess("button");

    // Append Unique ID switch
    String appendIdTopic = String("homeassistant/switch/mousewhisker_") + String(uniqueId) + "/appendid/config";
    String appendIdSw = "{";
    appendIdSw += "\"name\": \"Append ID (" + String(uniqueId) + ")\",";
    appendIdSw += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_appendid\",";
    appendIdSw += "\"command_topic\": \"" + MQTT_TOPIC_SET_APPENDID + "\",";
    appendIdSw += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    appendIdSw += "\"value_template\": \"{{ 'ON' if value_json.appendUniqueId else 'OFF' }}\",";
    appendIdSw += "\"json_attributes_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    appendIdSw += "\"json_attributes_template\": \"{ \\\"hint\\\": \\\"Reboot required to apply. Appends '" + String(uniqueId) + "' to mouse name.\\\" }\",";
    appendIdSw += "\"state_on\": \"ON\",";
    appendIdSw += "\"state_off\": \"OFF\",";
    appendIdSw += "\"payload_on\": \"1\",";
    appendIdSw += "\"payload_off\": \"0\",";
    appendIdSw += "\"optimistic\": false,";
    appendIdSw += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    appendIdSw += "\"icon\": \"mdi:identifier\",";
    appendIdSw += "\"entity_category\": \"config\",";
    appendIdSw += "\"unique_id\": \"" + baseId + "_appendid\",";
    appendIdSw += "\"device\": " + deviceShort + "}";
    mqttClient.publish(appendIdTopic.c_str(), appendIdSw.c_str(), true);
    yieldAndProcess("appendid");

    #if ENABLE_LED
    // LED enable switch
    String ledTopic = String("homeassistant/switch/mousewhisker_") + String(uniqueId) + "/led/config";
    String ledSw = "{";
    ledSw += "\"name\": \"Status LED\",";
    ledSw += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_led\",";
    ledSw += "\"command_topic\": \"" + MQTT_TOPIC_SET_LED + "\",";
    ledSw += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    ledSw += "\"value_template\": \"{{ 'ON' if value_json.ledEnabled else 'OFF' }}\",";
    ledSw += "\"state_on\": \"ON\",";
    ledSw += "\"state_off\": \"OFF\",";
    ledSw += "\"payload_on\": \"1\",";
    ledSw += "\"payload_off\": \"0\",";
    ledSw += "\"optimistic\": false,";
    ledSw += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    ledSw += "\"icon\": \"mdi:led-on\",";
    ledSw += "\"unique_id\": \"" + baseId + "_led\",";
    ledSw += "\"device\": " + deviceShort + "}";
    mqttClient.publish(ledTopic.c_str(), ledSw.c_str(), true);
    yieldAndProcess("led");
    #endif

    // Reboot button
    String rebootTopic = String("homeassistant/button/mousewhisker_") + String(uniqueId) + "/reboot/config";
    String rebootBtn = "{";
    rebootBtn += "\"name\": \"Reboot\",";
    rebootBtn += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_reboot\",";
    rebootBtn += "\"command_topic\": \"" + MQTT_TOPIC_CMD_REBOOT + "\",";
    rebootBtn += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    rebootBtn += "\"icon\": \"mdi:restart\",";
    rebootBtn += "\"entity_category\": \"config\",";
    rebootBtn += "\"unique_id\": \"" + baseId + "_reboot\",";
    rebootBtn += "\"device\": " + deviceShort + "}";
    mqttClient.publish(rebootTopic.c_str(), rebootBtn.c_str(), true);
    yieldAndProcess("reboot");

    // Factory Reset button
    String factoryTopic = String("homeassistant/button/mousewhisker_") + String(uniqueId) + "/factory_reset/config";
    String factoryBtn = "{";
    factoryBtn += "\"name\": \"Factory Reset\",";
    factoryBtn += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_factory_reset\",";
    factoryBtn += "\"command_topic\": \"" + MQTT_TOPIC_CMD_FACTORY_RESET + "\",";
    factoryBtn += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    factoryBtn += "\"icon\": \"mdi:file-undo\",";
    factoryBtn += "\"entity_category\": \"config\",";
    factoryBtn += "\"unique_id\": \"" + baseId + "_factory_reset\",";
    factoryBtn += "\"device\": " + deviceShort + "}";
    mqttClient.publish(factoryTopic.c_str(), factoryBtn.c_str(), true);
    yieldAndProcess("factory");

    // Min Move Interval number
    String minTopic = String("homeassistant/number/mousewhisker_") + String(uniqueId) + "/mininterval/config";
    String minNum = "{";
    minNum += "\"name\": \"Min Interval\",";
    minNum += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_mininterval\",";
    minNum += "\"command_topic\": \"" + MQTT_TOPIC_SET_MINMOVE + "\",";
    minNum += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    minNum += "\"value_template\": \"{{ value_json.minMoveInterval }}\",";
    minNum += "\"min\": " + String(ALLOWED_MIN_MOVE_SECONDS) + ",";
    minNum += "\"max\": " + String(ALLOWED_MAX_MOVE_SECONDS) + ",";
    minNum += "\"step\": 1,";
    minNum += "\"unit_of_measurement\": \"s\",";
    minNum += "\"mode\": \"box\",";
    minNum += "\"optimistic\": false,";
    minNum += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    minNum += "\"icon\": \"mdi:timer-outline\",";
    minNum += "\"entity_category\": \"config\",";
    minNum += "\"unique_id\": \"" + baseId + "_mininterval\",";
    minNum += "\"device\": " + deviceShort + "}";
    mqttClient.publish(minTopic.c_str(), minNum.c_str(), true);
    yieldAndProcess("minInterval");

    // Max Move Interval number
    String maxTopic = String("homeassistant/number/mousewhisker_") + String(uniqueId) + "/maxinterval/config";
    String maxNum = "{";
    maxNum += "\"name\": \"Max Interval\",";
    maxNum += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_maxinterval\",";
    maxNum += "\"command_topic\": \"" + MQTT_TOPIC_SET_MAXMOVE + "\",";
    maxNum += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    maxNum += "\"value_template\": \"{{ value_json.maxMoveInterval }}\",";
    maxNum += "\"min\": " + String(ALLOWED_MIN_MOVE_SECONDS) + ",";
    maxNum += "\"max\": " + String(ALLOWED_MAX_MOVE_SECONDS) + ",";
    maxNum += "\"step\": 1,";
    maxNum += "\"unit_of_measurement\": \"s\",";
    maxNum += "\"mode\": \"box\",";
    maxNum += "\"optimistic\": false,";
    maxNum += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    maxNum += "\"icon\": \"mdi:timer\",";
    maxNum += "\"entity_category\": \"config\",";
    maxNum += "\"unique_id\": \"" + baseId + "_maxinterval\",";
    maxNum += "\"device\": " + deviceShort + "}";
    mqttClient.publish(maxTopic.c_str(), maxNum.c_str(), true);
    yieldAndProcess("maxInterval");

    // X Range number
    String xTopic = String("homeassistant/number/mousewhisker_") + String(uniqueId) + "/xrange/config";
    String xNum = "{";
    xNum += "\"name\": \"Range X\",";
    xNum += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_xrange\",";
    xNum += "\"command_topic\": \"" + MQTT_TOPIC_SET_XRANGE + "\",";
    xNum += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    xNum += "\"value_template\": \"{{ value_json.xRange }}\",";
    xNum += "\"min\": 0,";
    xNum += "\"max\": 127,";
    xNum += "\"step\": 1,";
    xNum += "\"unit_of_measurement\": \"px\",";
    xNum += "\"mode\": \"box\",";
    xNum += "\"optimistic\": false,";
    xNum += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    xNum += "\"icon\": \"mdi:arrow-left-right\",";
    xNum += "\"entity_category\": \"config\",";
    xNum += "\"unique_id\": \"" + baseId + "_xrange\",";
    xNum += "\"device\": " + deviceShort + "}";
    mqttClient.publish(xTopic.c_str(), xNum.c_str(), true);
    yieldAndProcess("xRange");

    // Y Range number
    String yTopic = String("homeassistant/number/mousewhisker_") + String(uniqueId) + "/yrange/config";
    String yNum = "{";
    yNum += "\"name\": \"Range Y\",";
    yNum += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_yrange\",";
    yNum += "\"command_topic\": \"" + MQTT_TOPIC_SET_YRANGE + "\",";
    yNum += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    yNum += "\"value_template\": \"{{ value_json.yRange }}\",";
    yNum += "\"min\": 0,";
    yNum += "\"max\": 127,";
    yNum += "\"step\": 1,";
    yNum += "\"unit_of_measurement\": \"px\",";
    yNum += "\"mode\": \"box\",";
    yNum += "\"optimistic\": false,";
    yNum += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    yNum += "\"icon\": \"mdi:arrow-up-down\",";
    yNum += "\"entity_category\": \"config\",";
    yNum += "\"unique_id\": \"" + baseId + "_yrange\",";
    yNum += "\"device\": " + deviceShort + "}";
    mqttClient.publish(yTopic.c_str(), yNum.c_str(), true);
    yieldAndProcess("yRange");

    // BLE Connected binary sensor
    #if ENABLE_BLE
    String bleTopic = String("homeassistant/binary_sensor/mousewhisker_") + String(uniqueId) + "/ble_connected/config";
    String bleSensor = "{";
    bleSensor += "\"name\": \"BLE Connected\",";
    bleSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_ble_connected\",";
    bleSensor += "\"state_topic\": \"" + MQTT_TOPIC_BLE_CONNECTED + "\",";
    bleSensor += "\"payload_on\": \"ON\",";
    bleSensor += "\"payload_off\": \"OFF\",";
    bleSensor += "\"device_class\": \"connectivity\",";
    bleSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    bleSensor += "\"icon\": \"mdi:bluetooth-connect\",";
    bleSensor += "\"unique_id\": \"" + baseId + "_ble_connected\",";
    bleSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(bleTopic.c_str(), bleSensor.c_str(), true);
    yieldAndProcess("bleConnected");
    
    // BLE Host Address sensor
    String bleHostTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/ble_host/config";
    String bleHostSensor = "{";
    bleHostSensor += "\"name\": \"BLE Host\",";
    bleHostSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_ble_host\",";
    bleHostSensor += "\"state_topic\": \"" + MQTT_TOPIC_BLE_HOST + "\",";
    bleHostSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    bleHostSensor += "\"icon\": \"mdi:bluetooth\",";
    bleHostSensor += "\"unique_id\": \"" + baseId + "_ble_host\",";
    bleHostSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(bleHostTopic.c_str(), bleHostSensor.c_str(), true);
    yieldAndProcess("bleHost");
    #endif

    // USB Mounted binary sensor
    #if ENABLE_USB
    String usbTopic = String("homeassistant/binary_sensor/mousewhisker_") + String(uniqueId) + "/usb_mounted/config";
    String usbSensor = "{";
    usbSensor += "\"name\": \"USB Mounted\",";
    usbSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_usb_mounted\",";
    usbSensor += "\"state_topic\": \"" + MQTT_TOPIC_USB_MOUNTED + "\",";
    usbSensor += "\"payload_on\": \"ON\",";
    usbSensor += "\"payload_off\": \"OFF\",";
    usbSensor += "\"device_class\": \"connectivity\",";
    usbSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    usbSensor += "\"icon\": \"mdi:usb\",";
    usbSensor += "\"unique_id\": \"" + baseId + "_usb_mounted\",";
    usbSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(usbTopic.c_str(), usbSensor.c_str(), true);
    yieldAndProcess("usbMounted");
    #endif

    // Mouse Name text entity (the name shown to BLE/USB host)
    // Allow longer input (50 chars) - ESP32 will truncate to BLE limit (24 or 29 chars)
    String nameTopic = String("homeassistant/text/mousewhisker_") + String(uniqueId) + "/name/config";
    String nameText = "{";
    nameText += "\"name\": \"Mouse Name\",";
    nameText += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_name\",";
    nameText += "\"command_topic\": \"" + MQTT_TOPIC_SET_NAME + "\",";
    nameText += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    nameText += "\"value_template\": \"{{ value_json.deviceName }}\",";
    nameText += "\"json_attributes_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    nameText += "\"json_attributes_template\": \"{ \\\"hint\\\": \\\"Reboot required to apply. Max 24 chars with ID suffix, 29 without.\\\", \\\"append_id_enabled\\\": {{ value_json.appendUniqueId | lower }} }\",";
    nameText += "\"min\": 1,";
    nameText += "\"max\": 50,";
    nameText += "\"mode\": \"text\",";
    nameText += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    nameText += "\"icon\": \"mdi:mouse\",";
    nameText += "\"entity_category\": \"config\",";
    nameText += "\"unique_id\": \"" + baseId + "_name\",";
    nameText += "\"device\": " + deviceShort + "}";
    mqttClient.publish(nameTopic.c_str(), nameText.c_str(), true);
    yieldAndProcess("mouseName");

    // WiFi Signal Strength sensor
    String rssiTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/wifi_rssi/config";
    String rssiSensor = "{";
    rssiSensor += "\"name\": \"WiFi Signal\",";
    rssiSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_wifi_rssi\",";
    rssiSensor += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    rssiSensor += "\"value_template\": \"{{ value_json.wifiRssi }}\",";
    rssiSensor += "\"device_class\": \"signal_strength\",";
    rssiSensor += "\"unit_of_measurement\": \"dBm\",";
    rssiSensor += "\"entity_category\": \"diagnostic\",";
    rssiSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    rssiSensor += "\"icon\": \"mdi:wifi\",";
    rssiSensor += "\"unique_id\": \"" + baseId + "_wifi_rssi\",";
    rssiSensor += "\"device\": {\"identifiers\": [\"" + baseId + "\"],\"name\": \"Mouse Whisker " + String(uniqueId) + "\"}}";
    mqttClient.publish(rssiTopic.c_str(), rssiSensor.c_str(), true);
    yieldAndProcess("wifiRssi");

    // WiFi Quality sensor (Excellent/Good/Fair/Weak)
    String qualityTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/wifi_quality/config";
    String qualitySensor = "{";
    qualitySensor += "\"name\": \"WiFi Quality\",";
    qualitySensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_wifi_quality\",";
    qualitySensor += "\"state_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    qualitySensor += "\"value_template\": \"{{ value_json.wifiQuality }}\",";
    qualitySensor += "\"json_attributes_topic\": \"" + MQTT_TOPIC_CONFIG + "\",";
    qualitySensor += "\"json_attributes_template\": \"{ \\\"thresholds\\\": \\\"Excellent: ≥-50dBm, Good: ≥-60dBm, Fair: ≥-70dBm, Weak: <-70dBm\\\", \\\"rssi\\\": {{ value_json.wifiRssi }} }\",";
    qualitySensor += "\"entity_category\": \"diagnostic\",";
    qualitySensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    qualitySensor += "\"icon\": \"mdi:wifi-strength-3\",";
    qualitySensor += "\"unique_id\": \"" + baseId + "_wifi_quality\",";
    qualitySensor += "\"device\": {\"identifiers\": [\"" + baseId + "\"],\"name\": \"Mouse Whisker " + String(uniqueId) + "\"}}";
    mqttClient.publish(qualityTopic.c_str(), qualitySensor.c_str(), true);
    yieldAndProcess("wifiQuality");
    
    Serial.println("Home Assistant discovery complete");
  }

  void handleSetCommand(const String &msg) {
    // commands: whisk | start | stop | setname <name> | status
    if (msg.equalsIgnoreCase("whisk")) {
      moveMouse();
      publishConfigJSON();
    } else if (msg.startsWith("start")) {
      periodicOn = true;
      publishState("running");
      publishConfigJSON();
    } else if (msg.equalsIgnoreCase("stop")) {
      periodicOn = false;
      publishState("stopped");
      publishConfigJSON();
    } else if (msg.startsWith("setname ")) {
      String newName = msg.substring(8);
      newName.trim();
      if (newName.length() > 0) {
        String clean = sanitizeDeviceName(newName);
        if (clean != deviceName) {
          deviceName = clean;
          preferences.putString("devname", deviceName);
          #if ENABLE_USB
            USB.productName(deviceName.c_str());
          #endif
          mqttClient.publish(MQTT_TOPIC_EVENT.c_str(), "name_set", false);
          publishConfigJSON();
        }
      }
    } else if (msg.equalsIgnoreCase("status")) {
      publishState(periodicOn ? "running" : "stopped");
    }
  }

  void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String t = String(topic);
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
    msg.trim();
    Serial.printf("MQTT [%s] => %s\n", t.c_str(), msg.c_str());

    // Dedicated topics
    if (t == MQTT_TOPIC_SET_ENABLE) {
      bool newVal = (msg == "1" || msg.equalsIgnoreCase("on") || msg.equalsIgnoreCase("start"));
      if (newVal != periodicOn) {
        Serial.printf("[MQTT] Whisk state: %s -> %s\n", periodicOn ? "running" : "stopped", newVal ? "running" : "stopped");
        periodicOn = newVal;
        if (periodicOn) {
          // Pick new interval and log when next whisk will occur
          pickNextMoveInterval();
        }
      }
      publishState(periodicOn ? "running" : "stopped");
      publishConfigJSON();
      return;
    }

    if (t == MQTT_TOPIC_CMD_WHISK) {
      moveMouse();
      publishConfigJSON();
      return;
    }

    if (t == MQTT_TOPIC_CMD_REBOOT) {
      Serial.println("Reboot command received via MQTT");
      publishEventJSON("reboot", "rebooting");
      delay(500);  // Brief delay to allow MQTT message to send
      ESP.restart();
      return;
    }

    if (t == MQTT_TOPIC_CMD_FACTORY_RESET) {
      Serial.println("\n!!! FACTORY RESET command received via MQTT !!!");
      Serial.println("Clearing all preferences...");
      publishEventJSON("factory_reset", "resetting");
      mqttClient.loop();  // Process outgoing MQTT
      delay(100);
      mqttClient.loop();  // Ensure it's sent
      preferences.clear();
      preferences.end();  // Properly close preferences to flush to flash
      Serial.println("Preferences cleared. Rebooting...");
      Serial.flush();  // Ensure serial output completes
      delay(200);
      ESP.restart();
      return;
    }

    if (t == MQTT_TOPIC_SET_XRANGE) {
      int v = msg.toInt();
      if (v >= 0 && v <= 127) {
        if (v != xRange) {
          Serial.printf("[MQTT] X Range: %d -> %d (saved to preferences)\n", xRange, v);
          xRange = v;
          preferences.putInt("xrange", xRange);
        }
        publishEventJSON("xrange_set", String(xRange));
        publishConfigJSON();
      }
      return;
    }

    if (t == MQTT_TOPIC_SET_YRANGE) {
      int v = msg.toInt();
      if (v >= 0 && v <= 127) {
        if (v != yRange) {
          Serial.printf("[MQTT] Y Range: %d -> %d (saved to preferences)\n", yRange, v);
          yRange = v;
          preferences.putInt("yrange", yRange);
        }
        publishEventJSON("yrange_set", String(yRange));
        publishConfigJSON();
      }
      return;
    }

    if (t == MQTT_TOPIC_SET_MINMOVE) {
      int v = msg.toInt();
      if (v >= ALLOWED_MIN_MOVE_SECONDS && v <= ALLOWED_MAX_MOVE_SECONDS) {
        if (v != minMoveInterval) {
          Serial.printf("[MQTT] Min Interval: %d -> %d (saved to preferences)\n", minMoveInterval, v);
          minMoveInterval = v;
          preferences.putInt("minmove", minMoveInterval);
        }
        if (minMoveInterval > maxMoveInterval) {
          Serial.printf("[MQTT] Max Interval adjusted: %d -> %d (min > max)\n", maxMoveInterval, minMoveInterval);
          maxMoveInterval = minMoveInterval;
          preferences.putInt("maxmove", maxMoveInterval);
        }
        // Recalculate next move interval with new bounds
        pickNextMoveInterval();
        publishEventJSON("minmove_set", String(minMoveInterval));
        publishConfigJSON();
      }
      return;
    }

    if (t == MQTT_TOPIC_SET_MAXMOVE) {
      int v = msg.toInt();
      if (v >= ALLOWED_MIN_MOVE_SECONDS && v <= ALLOWED_MAX_MOVE_SECONDS) {
        if (v != maxMoveInterval) {
          Serial.printf("[MQTT] Max Interval: %d -> %d (saved to preferences)\n", maxMoveInterval, v);
          maxMoveInterval = v;
          preferences.putInt("maxmove", maxMoveInterval);
        }
        if (maxMoveInterval < minMoveInterval) {
          Serial.printf("[MQTT] Min Interval adjusted: %d -> %d (max < min)\n", minMoveInterval, maxMoveInterval);
          minMoveInterval = maxMoveInterval;
          preferences.putInt("minmove", minMoveInterval);
        }
        // Recalculate next move interval with new bounds
        pickNextMoveInterval();
        publishEventJSON("maxmove_set", String(maxMoveInterval));
        publishConfigJSON();
      }
      return;
    }

    if (t == MQTT_TOPIC_SET_NAME) {
      if (msg.length() > 0) {
        // Dynamic limit based on appendUniqueId setting
        size_t nameMaxLen = appendUniqueId ? 24 : 29;
        String clean = sanitizeDeviceName(msg, nameMaxLen);
        
        // Check if name was truncated (compare sanitized length to input length after removing invalid chars)
        String inputClean = sanitizeDeviceName(msg, 255);  // Get full sanitized version
        bool wasTruncated = (inputClean.length() > clean.length());
        
        if (wasTruncated) {
          // Log truncation to serial (name was too long for BLE advertising limit)
          Serial.printf("Device name truncated: '%s' -> '%s' (max %d chars%s)\n", 
                        inputClean.c_str(), clean.c_str(), nameMaxLen,
                        appendUniqueId ? " with ID suffix" : "");
        }
        
        if (clean != deviceName) {
          String oldName = deviceName;  // Store old name for logging
          deviceName = clean;
          preferences.putString("devname", deviceName);
          #if ENABLE_USB
            USB.productName(deviceName.c_str());
          #endif
          
          publishEventJSON("name_set", deviceName);
          publishConfigJSON();
          // Note: BLE name is set at init time and requires reboot to update
          Serial.printf("Device name changed: '%s' -> '%s'. Reboot required to apply to BLE advertising.\n",
                        oldName.c_str(), deviceName.c_str());
        } else if (wasTruncated) {
          // Name was truncated but result equals current name - still publish to update HA text box
          Serial.printf("Device name unchanged (truncated input matches current name '%s')\n", deviceName.c_str());
          publishConfigJSON();  // Send truncated value back to HA
        }
      }
      return;
    }

    if (t == MQTT_TOPIC_SET_APPENDID) {
      bool newVal = (msg == "1" || msg.equalsIgnoreCase("on"));
      if (newVal != appendUniqueId) {
        bool oldVal = appendUniqueId;  // Store old value for logging
        appendUniqueId = newVal;
        preferences.putBool("appendid", appendUniqueId);
        publishEventJSON("appendid_set", String(appendUniqueId ? "true" : "false"));
        publishConfigJSON();
        // Note: Changing this requires reboot to update USB/BLE mouse name
        Serial.printf("Append Unique ID changed: %s -> %s. Reboot required to apply to mouse name.\n",
                      oldVal ? "true" : "false", appendUniqueId ? "true" : "false");
      }
      return;
    }

    #if ENABLE_LED
    if (t == MQTT_TOPIC_SET_LED) {
      bool newVal = (msg == "1" || msg.equalsIgnoreCase("on"));
      if (newVal != ledEnabled) {
        Serial.printf("[MQTT] LED Enabled: %s -> %s (saved to preferences)\n", ledEnabled ? "true" : "false", newVal ? "true" : "false");
        ledEnabled = newVal;
        preferences.putBool("ledEnabled", ledEnabled);
        publishEventJSON("led_set", String(ledEnabled ? "true" : "false"));
        publishConfigJSON();
      }
      return;
    }
    #endif

    // fallback: legacy single cmd topic
    if (t == MQTT_TOPIC_CMD) {
      handleSetCommand(msg);
    }
  }
  #endif // ENABLE_MQTT

  #if ENABLE_WIFI
  void startAPMode() {
    Serial.println("Starting WiFi AP mode for configuration...");
    WiFi.mode(WIFI_AP);
    
    // Use fixed AP SSID based on device ID (consistent, doesn't change with device name)
    String apSSID = "Mouse Whisker " + String(uniqueId);
    
    // Start AP with or without password
    if (strlen(AP_PASS) >= 8) {
      WiFi.softAP(apSSID.c_str(), AP_PASS);
      Serial.printf("  AP SSID: %s\n", apSSID.c_str());
      Serial.printf("  AP Password: %s\n", AP_PASS);
    } else {
      WiFi.softAP(apSSID.c_str());  // Open network
      Serial.printf("  AP SSID: %s\n", apSSID.c_str());
      Serial.println("  AP Password: (none - open network)");
    }
    wifiAPMode = true;
    
    #if ENABLE_WEBUI
    // Start DNS server for captive portal - redirect all domains to our IP
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    Serial.println("  Captive portal DNS started");
    #endif
    
    Serial.printf("  WebUI: http://%s/\n", WiFi.softAPIP().toString().c_str());
    Serial.println("  Connect to AP and configuration page will open automatically.");
    #if ENABLE_LED
      ledBlink(5, 100); // Rapid blinks to indicate AP mode
    #endif
  }

  void connectWiFi() {
    // If no credentials configured, go straight to AP mode
    if (wifiSSID.length() == 0) {
      startAPMode();
      return;
    }
    
    WiFi.mode(WIFI_STA);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);  // Helps with hidden networks
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
    
    // Debug: show credentials info
    Serial.printf("WiFi credentials: SSID='%s' (len=%d), Pass len=%d\n", 
                  wifiSSID.c_str(), wifiSSID.length(), wifiPassword.length());
    
    // Try up to 3 times before giving up
    for (int attempt = 1; attempt <= 3; attempt++) {
      // Use extended begin() with channel=0, bssid=NULL, connect=true for hidden SSID support
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str(), 0, NULL, true);
      Serial.printf("Connecting to WiFi '%s' (attempt %d/3)", wifiSSID.c_str(), attempt);
      
      unsigned long start = millis();
      unsigned long lastDot = 0;
      while (WiFi.status() != WL_CONNECTED) {
        #if ENABLE_LED
          updateLED();  // Keep LED pulsing during connection
        #endif
        if (millis() - lastDot >= 500) {
          lastDot = millis();
          Serial.print(".");
        }
        if (millis() - start > 10000) break; // timeout (10s per attempt)
      }
      Serial.println();
      
      if (WiFi.status() == WL_CONNECTED) {
        wifiAPMode = false;
        int rssi = WiFi.RSSI();
        Serial.println("WiFi connected");
        Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("  Signal: %d dBm (%s)\n", rssi, 
                      rssi >= -50 ? "Excellent" : rssi >= -60 ? "Good" : rssi >= -70 ? "Fair" : "Weak");
        #if ENABLE_LED
          ledBlink(1, 500); // 1 long pulse for WiFi connected
        #endif
        return;  // Success!
      }
      
      // Log the failure reason
      wl_status_t status = WiFi.status();
      Serial.printf("  Failed (status=%d: %s)\n", status,
        status == WL_IDLE_STATUS ? "IDLE" :
        status == WL_NO_SSID_AVAIL ? "NO_SSID_AVAIL" :
        status == WL_CONNECT_FAILED ? "CONNECT_FAILED" :
        status == WL_DISCONNECTED ? "DISCONNECTED" :
        status == WL_CONNECTION_LOST ? "CONNECTION_LOST" :
        "UNKNOWN");
      
      // On first failure, scan for networks to help diagnose
      if (attempt == 1) {
        Serial.println("Scanning for networks to diagnose...");
        int n = WiFi.scanNetworks(false, true);  // false=async off, true=show hidden
        Serial.printf("  Found %d networks:\n", n);
        bool foundTarget = false;
        for (int i = 0; i < n && i < 10; i++) {
          String ssid = WiFi.SSID(i);
          bool hidden = ssid.length() == 0;
          Serial.printf("    %d: '%s' (%d dBm) ch%d %s%s\n", 
                        i+1, 
                        hidden ? "(hidden)" : ssid.c_str(),
                        WiFi.RSSI(i),
                        WiFi.channel(i),
                        WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "encrypted",
                        (ssid == wifiSSID) ? " <-- TARGET" : "");
          if (ssid == wifiSSID) foundTarget = true;
        }
        WiFi.scanDelete();
        if (!foundTarget) {
          Serial.println("  WARNING: Target SSID not found in scan (may be hidden or out of range)");
        }
      }
      
      // Brief pause before retry
      if (attempt < 3) {
        Serial.println("  Retrying...");
        WiFi.disconnect();
        delay(1000);
      }
    }
    
    // All attempts failed
    Serial.println("WiFi connect failed after 3 attempts - starting AP mode");
    startAPMode();
  }
  
  // Try to connect to WiFi while in AP mode (returns true if successful)
  bool tryWiFiFromAPMode() {
    if (wifiSSID.length() == 0) return false;
    
    Serial.printf("Attempting WiFi connection to '%s' from AP mode...\n", wifiSSID.c_str());
    
    // Temporarily switch to STA mode to try connecting
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str(), 0, NULL, true);  // Hidden SSID support
    
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(100);
      #if ENABLE_LED
        updateLED();
      #endif
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      wifiAPMode = false;
      Serial.println("WiFi connected! Switching from AP mode.");
      Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
      #if ENABLE_LED
        ledBlink(2, 300);
      #endif
      return true;
    }
    
    // Failed - go back to AP mode
    Serial.println("  WiFi still unavailable, staying in AP mode");
    WiFi.disconnect();
    startAPMode();
    return false;
  }
  #endif // ENABLE_WIFI

  #if ENABLE_MQTT
  void connectMQTT() {
    // Don't attempt MQTT if WiFi isn't connected
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("MQTT skipped - WiFi not connected");
      return;
    }
    
    // Don't attempt MQTT if server not configured
    if (mqttServer.length() == 0) {
      Serial.println("MQTT skipped - server not configured");
      return;
    }
    
    mqttClient.setServer(mqttServer.c_str(), mqttPort);
    mqttClient.setBufferSize(1024);  // Increase buffer for HA discovery payloads
    mqttClient.setSocketTimeout(5);  // 5 second socket timeout to prevent blocking
    mqttClient.setKeepAlive(60);     // 60 second keepalive
    mqttClient.setCallback(mqttCallback);
    Serial.printf("Connecting to MQTT %s:%d...", mqttServer.c_str(), mqttPort);
    String clientId = "esp32c3-mouse-" + String(uniqueId);
    // connect with Last Will (availability offline)
    if (mqttClient.connect(clientId.c_str(), mqttUser.c_str(), mqttPassword.c_str(),
                           MQTT_TOPIC_AVAILABILITY.c_str(), 1, true, "offline")) {
      Serial.println("connected");
      mqttClient.subscribe(MQTT_TOPIC_CMD.c_str());
      mqttClient.subscribe(MQTT_TOPIC_SET_ENABLE.c_str());
      mqttClient.subscribe(MQTT_TOPIC_SET_XRANGE.c_str());
      mqttClient.subscribe(MQTT_TOPIC_SET_YRANGE.c_str());
      mqttClient.subscribe(MQTT_TOPIC_SET_MINMOVE.c_str());
      mqttClient.subscribe(MQTT_TOPIC_SET_MAXMOVE.c_str());
      mqttClient.subscribe(MQTT_TOPIC_SET_NAME.c_str());
      mqttClient.subscribe(MQTT_TOPIC_SET_APPENDID.c_str());
      #if ENABLE_LED
        mqttClient.subscribe(MQTT_TOPIC_SET_LED.c_str());
      #endif
      mqttClient.subscribe(MQTT_TOPIC_CMD_WHISK.c_str());
      mqttClient.subscribe(MQTT_TOPIC_CMD_REBOOT.c_str());
      mqttClient.subscribe(MQTT_TOPIC_CMD_FACTORY_RESET.c_str());
      // publish availability online
      mqttClient.publish(MQTT_TOPIC_AVAILABILITY.c_str(), "online", true);
      publishDiscovery();
      // publish current config snapshot to config topic
      publishConfigJSON();
      // publish default running state on connect so device defaults to running after power-up
      publishState("running");
      #if ENABLE_BLE
        // Publish initial BLE connected state and host address
        mqttClient.publish(MQTT_TOPIC_BLE_CONNECTED.c_str(), bleConnected ? "ON" : "OFF", true);
        mqttClient.publish(MQTT_TOPIC_BLE_HOST.c_str(), bleClientAddress.c_str(), true);
      #endif
      #if ENABLE_USB
        // Publish initial USB mounted state
        mqttClient.publish(MQTT_TOPIC_USB_MOUNTED.c_str(), USB ? "ON" : "OFF", true);
      #endif
      #if ENABLE_LED
        ledBlink(3, 80); // 3 rapid taps for MQTT connected
      #endif
      // Re-print settings to serial now that connection is stable
      Serial.println("\n--- Settings (post-connect) ---");
      printSettingsToSerial();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("; will retry in loop");
    }
  }
  #endif

  // ========== WebUI Implementation ==========
  #if ENABLE_WEBUI
  String generateWebPage() {
    String html = F("<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Mouse Whisker</title>"
      "<style>body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:10px;font-size:16px;}"
      "h2{color:#333;}h3{margin-top:20px;margin-bottom:10px;}"
      "input[type=text],input[type=password],input[type=number]{font-size:16px;padding:10px;border:1px solid #ccc;border-radius:4px;}"
      "input[type=text],input[type=password]{width:100%;max-width:280px;box-sizing:border-box;}"
      "input[type=number]{width:80px;}"
      "input[type=checkbox]{width:20px;height:20px;margin-right:8px;vertical-align:middle;}"
      "label{display:block;margin:8px 0;}"
      ".btn{display:inline-block;padding:14px 28px;margin:5px;background:#007bff;color:#fff;text-decoration:none;border-radius:6px;font-size:16px;border:none;cursor:pointer;}"
      ".btn:hover{background:#0056b3;}.status{padding:10px;margin:10px 0;border-radius:4px;}"
      ".on{background:#d4edda;color:#155724;}.off{background:#f8d7da;color:#721c24;}"
      ".wait{background:#fff3cd;color:#856404;}.disabled{background:#e2e3e5;color:#6c757d;}"
      ".tight{margin:4px 0;}"
      ".group{background:#f8f9fa;border:1px solid #dee2e6;border-radius:6px;padding:12px;margin:15px 0;}"
      ".group-title{font-weight:bold;margin-bottom:8px;}"
      "details{margin:10px 0;}summary{cursor:pointer;padding:8px 0;font-weight:bold;color:#007bff;}"
      "summary:hover{text-decoration:underline;}details[open] summary{margin-bottom:10px;}"
      "</style></head><body>");
    
    // Show full name (with ID suffix if enabled)
    String displayName = deviceName;
    if (appendUniqueId) {
      displayName += " " + String(uniqueId);
    }
    html += "<h2>" + displayName + "</h2>";
    // Always show ID separately for reference
    html += "<p><b>Device ID:</b> " + String(uniqueId) + "</p>";
    
    // Whisk Status (with id for dynamic updates)
    html += "<div id='status' class='status " + String(periodicOn ? "on" : "off") + "'>";
    html += "<b>Whisk:</b> <span id='runState'>" + String(periodicOn ? "Running" : "Stopped") + "</span>";
    html += "</div>";
    
    // BLE status box
    #if ENABLE_BLE
    html += "<div id='bleStatus' class='status " + String(bleConnected ? "on" : "wait") + "'>";
    html += "<b>BLE:</b> <span id='ble'>" + String(bleConnected ? "Connected to " + bleClientAddress : "Waiting...") + "</span>";
    html += "</div>";
    #else
    html += F("<div class='status disabled'><b>BLE:</b> Disabled</div>");
    #endif
    
    // USB status box
    #if ENABLE_USB
    html += "<div id='usbStatus' class='status " + String(USB ? "on" : "wait") + "'>";
    html += "<b>USB:</b> <span id='usb'>" + String(USB ? "Mounted" : "Not mounted") + "</span>";
    html += "</div>";
    #else
    html += F("<div class='status disabled'><b>USB:</b> Disabled</div>");
    #endif
    
    // Whisk Now button
    html += "<p><a class='btn' href='/whisk'>Whisk Now</a></p>";
    
    // Settings form
    html += "<hr><form method='POST' action='/update'>";
    html += "<p><label><input type='checkbox' name='enabled' " + String(periodicOn ? "checked" : "") + "> Enable Random Whisking</label></p>";
    html += F("<div class='group'>");
    html += F("<div class='group-title'>Mouse Name</div>");
    html += "<p class='tight'><label><input type='text' name='name' id='nameInput' value='" + deviceName + "' maxlength='" + String(appendUniqueId ? 24 : 29) + "'></label></p>";
    html += "<p class='tight'><label><input type='checkbox' name='appendid' id='appendId' " + String(appendUniqueId ? "checked" : "") + "> Append ID</label></p>";
    html += "<p class='tight'><small>Preview: <b id='namePreview'></b></small></p>";
    html += F("<p class='tight'><small><i>Changing the name requires a reboot.</i></small></p>");
    html += F("</div>");
    html += "<p><label>Min Interval (sec): <input type='number' name='minint' id='minint' value='" + String(minMoveInterval) + "' min='" + String(ALLOWED_MIN_MOVE_SECONDS) + "' max='" + String(ALLOWED_MAX_MOVE_SECONDS) + "'></label></p>";
    html += "<p><label>Max Interval (sec): <input type='number' name='maxint' id='maxint' value='" + String(maxMoveInterval) + "' min='" + String(ALLOWED_MIN_MOVE_SECONDS) + "' max='" + String(ALLOWED_MAX_MOVE_SECONDS) + "'></label></p>";
    html += "<p><label>X Range (px): <input type='number' name='xrange' id='xrange' value='" + String(xRange) + "' min='0' max='127'></label></p>";
    html += "<p><label>Y Range (px): <input type='number' name='yrange' id='yrange' value='" + String(yRange) + "' min='0' max='127'></label></p>";
    #if ENABLE_LED
    html += "<p><label><input type='checkbox' name='led' " + String(ledEnabled ? "checked" : "") + "> Status LED</label></p>";
    #endif
    html += F("<p><input type='submit' class='btn' value='Save Settings'></p>");
    html += "</form>";
    
    // WiFi Configuration section (separate form)
    #if ENABLE_WIFI
    html += F("<hr><h3>WiFi</h3>");
    if (wifiAPMode) {
      html += F("<div class='status wait'><b>Mode:</b> AP (Configuration Mode)</div>");
    } else {
      html += "<div class='status on'><b>Connected to:</b> " + wifiSSID + "</div>";
    }
    html += F("<details><summary>Configure WiFi</summary>");
    html += F("<form method='POST' action='/wifi'>");
    html += "<p><label>SSID: <input type='text' name='ssid' value='" + wifiSSID + "' maxlength='32'></label></p>";
    html += F("<p><label>Password: <input type='password' name='pass' id='wifiPass' maxlength='64'></label>");
    html += F(" <label class='tight'><input type='checkbox' onclick='togglePass(\"wifiPass\")'> Show</label></p>");
    html += F("<p><input type='submit' class='btn' value='Save WiFi'></p>");
    html += F("</form></details>");
    #endif
    
    // MQTT Configuration section (separate form)
    #if ENABLE_MQTT
    html += F("<hr><h3>MQTT</h3>");
    if (mqttServer.length() == 0) {
      html += F("<div class='status wait'><b>Status:</b> Not configured</div>");
    } else if (mqttClient.connected()) {
      html += "<div class='status on'><b>Connected to:</b> " + mqttServer + ":" + String(mqttPort) + "</div>";
    } else {
      html += "<div class='status off'><b>Disconnected:</b> " + mqttServer + ":" + String(mqttPort) + "</div>";
    }
    html += F("<details><summary>Configure MQTT</summary>");
    html += F("<form method='POST' action='/mqtt'>");
    html += "<p><label>Server: <input type='text' name='server' value='" + mqttServer + "' maxlength='64' placeholder='192.168.1.100 or hostname'></label></p>";
    html += "<p><label>Port: <input type='number' name='port' value='" + String(mqttPort) + "' min='1' max='65535'></label></p>";
    html += "<p><label>Username: <input type='text' name='user' value='" + mqttUser + "' maxlength='32'></label></p>";
    html += F("<p><label>Password: <input type='password' name='pass' id='mqttPass' maxlength='64'></label>");
    html += F(" <label class='tight'><input type='checkbox' onclick='togglePass(\"mqttPass\")'> Show</label></p>");
    html += F("<p><input type='submit' class='btn' value='Save MQTT'></p>");
    html += F("</form></details>");
    #endif
    
    html += "<hr><p><small>Mouse Whiskerv" + String(FIRMWARE_VERSION) + " by Shannon Fritz | <a href='/reboot'>Reboot</a></small></p>";
    
    // JavaScript for dynamic status updates (polls every 3 seconds)
    html += F("<script>"
      "function togglePass(id){var p=document.getElementById(id);p.type=p.type=='password'?'text':'password';}"
      "setInterval(function(){"
        "fetch('/status').then(r=>r.json()).then(d=>{"
          "document.getElementById('runState').textContent=d.running?'Running':'Stopped';"
          "document.getElementById('status').className='status '+(d.running?'on':'off');"
          "var b=document.getElementById('ble');if(b){"
            "b.textContent=d.ble?'Connected to '+d.bleAddr:'Waiting...';"
            "document.getElementById('bleStatus').className='status '+(d.ble?'on':'wait');"
          "}"
          "var u=document.getElementById('usb');if(u){"
            "u.textContent=d.usb?'Mounted':'Not mounted';"
            "document.getElementById('usbStatus').className='status '+(d.usb?'on':'wait');"
          "}"
          "document.querySelector('[name=minint]').value=d.minInt;"
          "document.querySelector('[name=maxint]').value=d.maxInt;"
          "document.querySelector('[name=xrange]').value=d.xRange;"
          "document.querySelector('[name=yrange]').value=d.yRange;"
        "}).catch(()=>{});"
      "},3000);");
    // Live preview for mouse name (separate to inject uniqueId)
    html += "var uid='" + String(uniqueId) + "';";
    html += F("function updatePreview(){"
        "var n=document.getElementById('nameInput');"
        "var a=document.getElementById('appendId').checked;"
        "var maxLen=a?24:29;"
        "n.maxLength=maxLen;"
        "var s=n.value.replace(/[^A-Za-z0-9 ]/g,'').substring(0,maxLen);"
        "if(s.length==0)s='Mouse Whisker';"
        "var preview=a?(s+' '+uid):s;"
        "document.getElementById('namePreview').textContent=preview;"
      "}"
      "document.getElementById('nameInput').addEventListener('input',updatePreview);"
      "document.getElementById('appendId').addEventListener('change',updatePreview);"
      "updatePreview();"
    "</script>");
    
    html += F("</body></html>");
    return html;
  }

  // Lightweight JSON endpoint for status polling
  void handleWebStatus() {
    String json = "{\"running\":" + String(periodicOn ? "true" : "false");
    json += ",\"minInt\":" + String(minMoveInterval);
    json += ",\"maxInt\":" + String(maxMoveInterval);
    json += ",\"xRange\":" + String(xRange);
    json += ",\"yRange\":" + String(yRange);
    #if ENABLE_BLE
    json += ",\"ble\":" + String(bleConnected ? "true" : "false");
    json += ",\"bleAddr\":\"" + bleClientAddress + "\"";
    #endif
    #if ENABLE_USB
    json += ",\"usb\":" + String(USB ? "true" : "false");
    #endif
    json += "}";
    webServer.send(200, "application/json", json);
  }

  void handleWebRoot() {
    if (wifiAPMode) {
      // In AP mode, show simple captive portal landing page
      Serial.printf("[Captive] Landing page accessed from %s\n", webServer.client().remoteIP().toString().c_str());
      if (!portalPageSeen) {
        portalPageSeen = true;
        Serial.println("[Captive] Portal landing seen - next captive check will return Success");
      }
      // Simple landing page - just a button to open setup in Safari
      webServer.send(200, "text/html", F(
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Mouse Whisker Setup</title>"
        "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;'>"
        "<h2 style='font-size:28px;'>Mouse Whisker</h2>"
        "<p style='font-size:18px;'>Welcome! Tap the button below to configure your device.</p>"
        "<p style='margin-top:40px;'><a href='http://192.168.4.1/setup' style='"
        "display:block;width:80%;max-width:280px;margin:0 auto;padding:24px 20px;"
        "background:#007bff;color:#fff;text-decoration:none;border-radius:12px;font-size:24px;"
        "'>Open Setup</a></p>"
        "</body></html>"));
    } else {
      // In station mode, redirect to /setup
      webServer.sendHeader("Location", "/setup");
      webServer.send(302);
    }
  }

  void handleWebSetup() {
    Serial.printf("[WebUI] Setup page accessed from %s\n", webServer.client().remoteIP().toString().c_str());
    webServer.send(200, "text/html", generateWebPage());
  }

  void handleWebWhisk() {
    Serial.printf("[WebUI] Whisk triggered from %s\n", webServer.client().remoteIP().toString().c_str());
    moveMouse();
    webServer.sendHeader("Location", "/setup");
    webServer.send(303);
  }

  void handleWebUpdate() {
    Serial.printf("[WebUI] Settings updated from %s\n", webServer.client().remoteIP().toString().c_str());
    bool needsReboot = false;
    
    // Mouse Name (requires reboot)
    if (webServer.hasArg("name")) {
      String newName = webServer.arg("name");
      size_t maxLen = appendUniqueId ? 24 : 29;
      newName = sanitizeDeviceName(newName, maxLen);
      if (newName != deviceName) {
        Serial.printf("[WebUI] Mouse Name: '%s' -> '%s' (reboot required)\n", deviceName.c_str(), newName.c_str());
        deviceName = newName;
        preferences.putString("devname", deviceName);
        needsReboot = true;
      }
    }
    
    // Append ID (requires reboot)
    bool newAppendId = webServer.hasArg("appendid");
    if (newAppendId != appendUniqueId) {
      Serial.printf("[WebUI] Append ID: %s -> %s (reboot required)\n", appendUniqueId ? "on" : "off", newAppendId ? "on" : "off");
      appendUniqueId = newAppendId;
      preferences.putBool("appendid", appendUniqueId);
      needsReboot = true;
    }
    
    // Enable/disable whisking
    bool newEnabled = webServer.hasArg("enabled");
    if (newEnabled != periodicOn) {
      periodicOn = newEnabled;
      if (periodicOn) pickNextMoveInterval();
      Serial.printf("[WebUI] Whisking: %s\n", periodicOn ? "enabled" : "disabled");
      #if ENABLE_MQTT
        publishState(periodicOn ? "running" : "stopped");
        publishConfigJSON();
      #endif
    }
    
    // Min interval
    if (webServer.hasArg("minint")) {
      int v = webServer.arg("minint").toInt();
      if (v >= ALLOWED_MIN_MOVE_SECONDS && v <= ALLOWED_MAX_MOVE_SECONDS && v != minMoveInterval) {
        Serial.printf("[WebUI] Min Interval: %d -> %d\n", minMoveInterval, v);
        minMoveInterval = v;
        preferences.putInt("minmove", minMoveInterval);
        if (minMoveInterval > maxMoveInterval) {
          maxMoveInterval = minMoveInterval;
          preferences.putInt("maxmove", maxMoveInterval);
        }
        pickNextMoveInterval();
      }
    }
    
    // Max interval
    if (webServer.hasArg("maxint")) {
      int v = webServer.arg("maxint").toInt();
      if (v >= ALLOWED_MIN_MOVE_SECONDS && v <= ALLOWED_MAX_MOVE_SECONDS && v != maxMoveInterval) {
        Serial.printf("[WebUI] Max Interval: %d -> %d\n", maxMoveInterval, v);
        maxMoveInterval = v;
        preferences.putInt("maxmove", maxMoveInterval);
        if (maxMoveInterval < minMoveInterval) {
          minMoveInterval = maxMoveInterval;
          preferences.putInt("minmove", minMoveInterval);
        }
        pickNextMoveInterval();
      }
    }
    
    // X Range
    if (webServer.hasArg("xrange")) {
      int v = webServer.arg("xrange").toInt();
      if (v >= 0 && v <= 127 && v != xRange) {
        Serial.printf("[WebUI] X Range: %d -> %d\n", xRange, v);
        xRange = v;
        preferences.putInt("xrange", xRange);
      }
    }
    
    // Y Range
    if (webServer.hasArg("yrange")) {
      int v = webServer.arg("yrange").toInt();
      if (v >= 0 && v <= 127 && v != yRange) {
        Serial.printf("[WebUI] Y Range: %d -> %d\n", yRange, v);
        yRange = v;
        preferences.putInt("yrange", yRange);
      }
    }
    
    #if ENABLE_LED
    // LED enable
    bool newLed = webServer.hasArg("led");
    if (newLed != ledEnabled) {
      Serial.printf("[WebUI] LED: %s -> %s\n", ledEnabled ? "on" : "off", newLed ? "on" : "off");
      ledEnabled = newLed;
      preferences.putBool("ledEnabled", ledEnabled);
    }
    #endif
    
    #if ENABLE_MQTT
      publishConfigJSON();
    #endif
    
    if (needsReboot) {
      webServer.send(200, "text/html", F(
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='5;url=/setup'>"
        "<title>Settings Saved</title>"
        "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
        "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
        "<h2 style='color:#28a745;margin-top:0;'>&#10003; Settings Saved</h2>"
        "<p style='font-size:18px;'>Mouse name changed.</p>"
        "<p style='font-size:16px;color:#666;'><b>Rebooting...</b></p>"
        "<div style='margin-top:20px;'><span style='display:inline-block;width:40px;height:40px;border:4px solid #007bff;border-top-color:transparent;border-radius:50%;animation:spin 1s linear infinite;'></span></div>"
        "<style>@keyframes spin{to{transform:rotate(360deg);}}</style>"
        "</div></body></html>"));
      delay(500);
      ESP.restart();
    }
    
    webServer.sendHeader("Location", "/setup");
    webServer.send(303);
  }

  void handleWebReboot() {
    Serial.printf("[WebUI] Reboot requested from %s\n", webServer.client().remoteIP().toString().c_str());
    webServer.send(200, "text/html", F(
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<meta http-equiv='refresh' content='5;url=/setup'>"
      "<title>Rebooting</title>"
      "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
      "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
      "<h2 style='color:#007bff;margin-top:0;'>Rebooting...</h2>"
      "<p style='font-size:18px;'>Please wait while the device restarts.</p>"
      "<div style='margin-top:20px;'><span style='display:inline-block;width:40px;height:40px;border:4px solid #007bff;border-top-color:transparent;border-radius:50%;animation:spin 1s linear infinite;'></span></div>"
      "<style>@keyframes spin{to{transform:rotate(360deg);}}</style>"
      "</div></body></html>"));
    delay(500);
    ESP.restart();
  }

  void handleWebWiFi() {
    Serial.printf("[WebUI] WiFi config update from %s\n", webServer.client().remoteIP().toString().c_str());
    
    String newSSID = webServer.arg("ssid");
    String newPass = webServer.arg("pass");
    
    Serial.printf("[WebUI] Received SSID: '%s' (len=%d)\n", newSSID.c_str(), newSSID.length());
    Serial.printf("[WebUI] Received Pass length: %d\n", newPass.length());
    
    bool changed = false;
    
    // Handle clearing WiFi config (empty SSID = revert to AP mode)
    if (newSSID.length() == 0 && wifiSSID.length() > 0) {
      Serial.println("[WebUI] WiFi config cleared - will use AP mode");
      wifiSSID = "";
      wifiPassword = "";
      preferences.remove("wifiSSID");
      preferences.remove("wifiPass");
      changed = true;
    }
    // Update SSID if provided and different
    else if (newSSID.length() > 0 && newSSID != wifiSSID) {
      Serial.printf("[WebUI] WiFi SSID: '%s' -> '%s'\n", wifiSSID.c_str(), newSSID.c_str());
      wifiSSID = newSSID;
      preferences.putString("wifiSSID", wifiSSID);
      changed = true;
    }
    
    // Update password (blank = clear it)
    if (newSSID.length() > 0 && newPass != wifiPassword) {
      Serial.printf("[WebUI] WiFi password %s\n", newPass.length() > 0 ? "updated" : "cleared");
      wifiPassword = newPass;
      if (newPass.length() > 0) {
        preferences.putString("wifiPass", wifiPassword);
      } else {
        preferences.remove("wifiPass");
      }
      changed = true;
    }
    
    if (changed) {
      webServer.send(200, "text/html", F(
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='5;url=/setup'>"
        "<title>WiFi Saved</title>"
        "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
        "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
        "<h2 style='color:#28a745;margin-top:0;'>&#10003; WiFi Settings Saved</h2>"
        "<p style='font-size:18px;'>Applying new WiFi configuration.</p>"
        "<p style='font-size:16px;color:#666;'><b>Rebooting...</b></p>"
        "<div style='margin-top:20px;'><span style='display:inline-block;width:40px;height:40px;border:4px solid #007bff;border-top-color:transparent;border-radius:50%;animation:spin 1s linear infinite;'></span></div>"
        "<style>@keyframes spin{to{transform:rotate(360deg);}}</style>"
        "</div></body></html>"));
      delay(500);
      ESP.restart();
    } else {
      webServer.sendHeader("Location", "/setup");
      webServer.send(303);
    }
  }

  #if ENABLE_MQTT
  void handleWebMQTT() {
    Serial.printf("[WebUI] MQTT config update from %s\n", webServer.client().remoteIP().toString().c_str());
    
    String newServer = webServer.arg("server");
    String newPortStr = webServer.arg("port");
    String newUser = webServer.arg("user");
    String newPass = webServer.arg("pass");
    
    bool changed = false;
    
    // Update server if provided
    if (newServer != mqttServer) {
      Serial.printf("[WebUI] MQTT Server: '%s' -> '%s'\n", mqttServer.c_str(), newServer.c_str());
      mqttServer = newServer;
      preferences.putString("mqttServer", mqttServer);
      changed = true;
    }
    
    // Update port
    uint16_t newPort = newPortStr.toInt();
    if (newPort > 0 && newPort != mqttPort) {
      Serial.printf("[WebUI] MQTT Port: %d -> %d\n", mqttPort, newPort);
      mqttPort = newPort;
      preferences.putUShort("mqttPort", mqttPort);
      changed = true;
    }
    
    // Update username
    if (newUser != mqttUser) {
      Serial.printf("[WebUI] MQTT User: '%s' -> '%s'\n", mqttUser.c_str(), newUser.c_str());
      mqttUser = newUser;
      preferences.putString("mqttUser", mqttUser);
      changed = true;
    }
    
    // Update password (blank = clear it)
    if (newPass != mqttPassword) {
      Serial.printf("[WebUI] MQTT password %s\n", newPass.length() > 0 ? "updated" : "cleared");
      mqttPassword = newPass;
      if (newPass.length() > 0) {
        preferences.putString("mqttPass", mqttPassword);
      } else {
        preferences.remove("mqttPass");
      }
      changed = true;
    }
    
    if (changed) {
      // Disconnect and reconfigure MQTT client with new settings
      if (mqttClient.connected()) {
        mqttClient.disconnect();
      }
      mqttClient.setServer(mqttServer.c_str(), mqttPort);
      Serial.println("[WebUI] MQTT reconfigured - will reconnect with new settings");
      
      webServer.send(200, "text/html", F(
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='3;url=/setup'>"
        "<title>MQTT Saved</title>"
        "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
        "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
        "<h2 style='color:#28a745;margin-top:0;'>&#10003; MQTT Settings Saved</h2>"
        "<p style='font-size:18px;'>Reconnecting to MQTT...</p>"
        "<div style='margin-top:20px;'><span style='display:inline-block;width:40px;height:40px;border:4px solid #007bff;border-top-color:transparent;border-radius:50%;animation:spin 1s linear infinite;'></span></div>"
        "<style>@keyframes spin{to{transform:rotate(360deg);}}</style>"
        "</div></body></html>"));
    } else {
      webServer.sendHeader("Location", "/setup");
      webServer.send(303);
    }
  }
  #endif

  // Captive portal handler - redirects all unknown requests to main page
  void handleCaptivePortal() {
    String host = webServer.hostHeader();
    // If request is for our IP, serve normally (let onNotFound show 404)
    // Otherwise redirect to our IP (captive portal behavior)
    if (host == WiFi.softAPIP().toString() || host == "192.168.4.1") {
      webServer.send(404, "text/plain", "Not Found");
    } else {
      Serial.printf("[Captive] Redirecting %s to portal\n", host.c_str());
      webServer.sendHeader("Location", "http://192.168.4.1/", true);
      webServer.send(302, "text/plain", "");
    }
  }

  // Handler for common captive portal detection endpoints
  void handleCaptiveDetect() {
    String endpoint = webServer.uri();
    String clientIP = webServer.client().remoteIP().toString();
    
    // iOS CNA workaround: After user has seen the portal page, return "Success"
    // so iOS thinks we're connected. Then links will open in Safari!
    if (portalPageSeen) {
      Serial.printf("[Captive] %s requested %s -> Success page sent\n", clientIP.c_str(), endpoint.c_str());
      webServer.send(200, "text/html", F(
        "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>"
        "<p>Connected!</p>"
        "<p><a href='http://192.168.4.1/setup' style='"
        "display:inline-block;padding:20px 50px;background:#007bff;color:#fff;"
        "text-decoration:none;border-radius:12px;font-family:sans-serif;font-size:22px;"
        "'>Open Setup</a></p>"
        "</BODY></HTML>"));
    } else {
      // First time - redirect to portal landing page
      Serial.printf("[Captive] %s requested %s -> Redirect to portal\n", clientIP.c_str(), endpoint.c_str());
      webServer.sendHeader("Location", "http://192.168.4.1/", true);
      webServer.send(302, "text/plain", "");
    }
  }
  void setupWebUI() {
    webServer.on("/", handleWebRoot);
    webServer.on("/setup", handleWebSetup);
    webServer.on("/status", handleWebStatus);  // Lightweight JSON for polling
    webServer.on("/update", HTTP_POST, handleWebUpdate);
    webServer.on("/wifi", HTTP_POST, handleWebWiFi);
    #if ENABLE_MQTT
    webServer.on("/mqtt", HTTP_POST, handleWebMQTT);
    #endif
    webServer.on("/whisk", handleWebWhisk);
    webServer.on("/reboot", handleWebReboot);
    
    // Captive portal detection endpoints (various OS/browser checks)
    if (wifiAPMode) {
      // Android
      webServer.on("/generate_204", handleCaptiveDetect);
      webServer.on("/gen_204", handleCaptiveDetect);
      // Windows
      webServer.on("/ncsi.txt", handleCaptiveDetect);
      webServer.on("/connecttest.txt", handleCaptiveDetect);
      webServer.on("/redirect", handleCaptiveDetect);
      // Apple
      webServer.on("/hotspot-detect.html", handleCaptiveDetect);
      webServer.on("/library/test/success.html", handleCaptiveDetect);
      // Firefox
      webServer.on("/canonical.html", handleCaptiveDetect);
      webServer.on("/success.txt", handleCaptiveDetect);
      // Catch-all for any other requests
      webServer.onNotFound(handleCaptivePortal);
    }
    
    webServer.begin();
    if (wifiAPMode) {
      Serial.println("WebUI started: http://192.168.4.1/setup (Captive Portal enabled)");
    } else {
      Serial.printf("WebUI started: http://%s/setup\n", WiFi.localIP().toString().c_str());
    }
  }
  #endif // ENABLE_WEBUI

  void setup() {
    Serial.begin(115200);
    
    // Wait for serial monitor connection (with 2 second timeout)
    // This helps ensure startup messages are captured
    unsigned long serialWait = millis();
    while (!Serial && millis() - serialWait < 2000) {
      delay(10);
    }
    delay(100);  // Brief additional delay for stability
    
    Serial.println("\n\n=== ESP32 Mouse Whisker Starting ===");
    Serial.printf("Firmware: v%s\n", FIRMWARE_VERSION);
    Serial.printf("Chip: %s, Rev: %d\n", ESP.getChipModel(), ESP.getChipRevision());

    #if ENABLE_LED
    // Initialize LED
    #ifdef LED_TYPE_RGB
      strip.begin();
      strip.setBrightness(50);  // Moderate brightness for RGB
      strip.show();  // Initialize to off
      Serial.println("RGB LED initialized");
    #else
      ledcAttach(LED_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);
      ledWrite(5);  // Start dim for initial pulse
      Serial.printf("Single LED initialized (GPIO%d, %s)\n", LED_PIN, LED_ACTIVE_LOW ? "active-low" : "active-high");
    #endif
    ledLastUpdate = millis();
    #endif // ENABLE_LED

    // Unique ID from chip MAC (short form kept as-is)
    uint64_t chipid = ESP.getEfuseMac();
    sprintf(uniqueId, "%04X", (uint16_t)(chipid >> 32));

    // Preferences
    preferences.begin("whisker", false);
    Serial.println("\n--- Loading Preferences ---");
    
    // Load appendUniqueId first - needed to determine BLE name length limit
    bool hasAppendId = preferences.isKey("appendid");
    appendUniqueId = preferences.getBool("appendid", DEFAULT_APPEND_UNIQUE_ID);
    Serial.printf("Append ID: %s [%s]\n", appendUniqueId ? "Yes" : "No", hasAppendId ? "from flash" : "DEFAULT");
    
    size_t nameMaxLen = appendUniqueId ? 24 : 29;  // BLE limit: 29 total, minus 5 for " XXXX" suffix
    bool hasDevName = preferences.isKey("devname");
    deviceName = sanitizeDeviceName(preferences.getString("devname", DEFAULT_DEVICE_NAME), nameMaxLen);
    Serial.printf("Device Name: '%s' [%s]\n", deviceName.c_str(), hasDevName ? "from flash" : "DEFAULT");
    
    bool hasXRange = preferences.isKey("xrange");
    xRange = preferences.getInt("xrange", DEFAULT_X_RANGE);
    Serial.printf("X Range: %d [%s]\n", xRange, hasXRange ? "from flash" : "DEFAULT");
    
    bool hasYRange = preferences.isKey("yrange");
    yRange = preferences.getInt("yrange", DEFAULT_Y_RANGE);
    Serial.printf("Y Range: %d [%s]\n", yRange, hasYRange ? "from flash" : "DEFAULT");
    
    bool hasMinMove = preferences.isKey("minmove");
    minMoveInterval = preferences.getInt("minmove", DEFAULT_MIN_MOVE_INTERVAL);
    Serial.printf("Min Interval: %d [%s]\n", minMoveInterval, hasMinMove ? "from flash" : "DEFAULT");
    
    bool hasMaxMove = preferences.isKey("maxmove");
    maxMoveInterval = preferences.getInt("maxmove", DEFAULT_MAX_MOVE_INTERVAL);
    Serial.printf("Max Interval: %d [%s]\n", maxMoveInterval, hasMaxMove ? "from flash" : "DEFAULT");
    
    #if ENABLE_LED
      bool hasLedEnabled = preferences.isKey("ledEnabled");
      ledEnabled = preferences.getBool("ledEnabled", DEFAULT_LED_ENABLED);
      Serial.printf("LED Enabled: %s [%s]\n", ledEnabled ? "Yes" : "No", hasLedEnabled ? "from flash" : "DEFAULT");
    #endif
    
    #if ENABLE_WIFI
      // Load WiFi credentials: prefer stored, fall back to default
      bool hasWifiSSID = preferences.isKey("wifiSSID");
      bool hasWifiPass = preferences.isKey("wifiPass");
      if (hasWifiSSID) {
        wifiSSID = preferences.getString("wifiSSID", "");
        wifiPassword = preferences.getString("wifiPass", "");
        Serial.printf("WiFi SSID: '%s' [from flash]\n", wifiSSID.c_str());
        Serial.printf("WiFi Pass: [from flash] (len=%d)\n", wifiPassword.length());
      } else if (String(DEFAULT_WIFI_SSID) != "SET_IN_WEBUI") {
        wifiSSID = String(DEFAULT_WIFI_SSID);
        wifiPassword = String(DEFAULT_WIFI_PASS);
        Serial.printf("WiFi SSID: '%s' [DEFAULT]\n", wifiSSID.c_str());
        Serial.println("WiFi Pass: [DEFAULT]");
      } else {
        wifiSSID = "";
        wifiPassword = "";
        Serial.println("WiFi: Not configured (will start AP mode)");
      }
    #endif
    
    #if ENABLE_MQTT
      // Load MQTT settings: prefer stored, fall back to default
      bool hasMqttServer = preferences.isKey("mqttServer");
      if (hasMqttServer) {
        mqttServer = preferences.getString("mqttServer", "");
        mqttPort = preferences.getUShort("mqttPort", DEFAULT_MQTT_PORT);
        mqttUser = preferences.getString("mqttUser", "");
        mqttPassword = preferences.getString("mqttPass", "");
        Serial.printf("MQTT Server: '%s:%d' [from flash]\n", mqttServer.c_str(), mqttPort);
        Serial.println("MQTT User/Pass: [from flash]");
      } else if (String(DEFAULT_MQTT_SERVER) != "SET_IN_WEBUI") {
        mqttServer = String(DEFAULT_MQTT_SERVER);
        mqttPort = DEFAULT_MQTT_PORT;
        mqttUser = String(DEFAULT_MQTT_USER);
        mqttPassword = String(DEFAULT_MQTT_PASS);
        Serial.printf("MQTT Server: '%s:%d' [DEFAULT]\n", mqttServer.c_str(), mqttPort);
        Serial.println("MQTT User/Pass: [DEFAULT]");
      } else {
        mqttServer = "";
        mqttPort = DEFAULT_MQTT_PORT;
        mqttUser = "";
        mqttPassword = "";
        Serial.println("MQTT: Not configured (will skip MQTT)");
      }
    #endif
    Serial.println("------------------------");

    // Print current settings
    Serial.println("\n--- Active Configuration ---");
    Serial.printf("Device Name: %s\n", deviceName.c_str());
    Serial.printf("Unique ID: %s\n", uniqueId);
    Serial.printf("Append ID to Name: %s\n", appendUniqueId ? "Yes" : "No");
    Serial.printf("Movement Range: X=%d, Y=%d pixels\n", xRange, yRange);
    Serial.printf("Move Interval: %d-%d seconds\n", minMoveInterval, maxMoveInterval);
    #if ENABLE_LED
      Serial.printf("Status LED: %s\n", ledEnabled ? "Enabled" : "Disabled");
    #endif
    #if ENABLE_BLE
      Serial.println("BLE Mouse: Enabled");
    #endif
    #if ENABLE_USB
      Serial.println("USB Mouse: Enabled");
    #endif
    #if ENABLE_WIFI
      Serial.println("WiFi: Enabled");
    #else
      Serial.println("WiFi: Disabled");
    #endif
    #if ENABLE_MQTT
      Serial.println("MQTT: Enabled");
    #else
      Serial.println("MQTT: Disabled");
    #endif
    Serial.println("------------------------\n");

    #if ENABLE_MQTT
    // Build MQTT topics
    MQTT_TOPIC_CMD = String("mousewhisker/") + String(uniqueId) + "/cmd";
    MQTT_TOPIC_STATE = String("mousewhisker/") + String(uniqueId) + "/state";
    MQTT_TOPIC_CONFIG = String("mousewhisker/") + String(uniqueId) + "/config";
    MQTT_TOPIC_AVAILABILITY = String("mousewhisker/") + String(uniqueId) + "/availability";
    MQTT_TOPIC_EVENT = String("mousewhisker/") + String(uniqueId) + "/event";
    MQTT_TOPIC_SET_ENABLE = String("mousewhisker/") + String(uniqueId) + "/set/enable";
    MQTT_TOPIC_SET_XRANGE = String("mousewhisker/") + String(uniqueId) + "/set/xrange";
    MQTT_TOPIC_SET_YRANGE = String("mousewhisker/") + String(uniqueId) + "/set/yrange";
    MQTT_TOPIC_SET_MINMOVE = String("mousewhisker/") + String(uniqueId) + "/set/minmove";
    MQTT_TOPIC_SET_MAXMOVE = String("mousewhisker/") + String(uniqueId) + "/set/maxmove";
    MQTT_TOPIC_SET_NAME = String("mousewhisker/") + String(uniqueId) + "/set/name";
    MQTT_TOPIC_SET_APPENDID = String("mousewhisker/") + String(uniqueId) + "/set/appendid";
    #if ENABLE_LED
      MQTT_TOPIC_SET_LED = String("mousewhisker/") + String(uniqueId) + "/set/led";
    #endif
    MQTT_TOPIC_CMD_WHISK = String("mousewhisker/") + String(uniqueId) + "/cmd/whisk";
    MQTT_TOPIC_CMD_REBOOT = String("mousewhisker/") + String(uniqueId) + "/cmd/reboot";
    MQTT_TOPIC_CMD_FACTORY_RESET = String("mousewhisker/") + String(uniqueId) + "/cmd/factory_reset";
    MQTT_TOPIC_WHISKING = String("mousewhisker/") + String(uniqueId) + "/whisking";
    MQTT_TOPIC_BLE_CONNECTED = String("mousewhisker/") + String(uniqueId) + "/ble_connected";
    MQTT_TOPIC_BLE_HOST = String("mousewhisker/") + String(uniqueId) + "/ble_host";
    MQTT_TOPIC_USB_MOUNTED = String("mousewhisker/") + String(uniqueId) + "/usb_mounted";
    MQTT_TOPIC_WIFI_RSSI = String("mousewhisker/") + String(uniqueId) + "/wifi_rssi";
    #endif

    #if ENABLE_USB
      // Set USB device info and initialize
      if (appendUniqueId) {
        String usbName = deviceName + " " + String(uniqueId);
        USB.productName(usbName.c_str());
      } else {
        USB.productName(deviceName.c_str());
      }
      USB.manufacturerName("Generic");
      HID.begin();
      USB.begin();

      Serial.println("Waiting for USB host (5s timeout)...");
    unsigned long usbStart = millis();
    while (!USB && millis() - usbStart < 5000) delay(10);
    if (USB) {
      Serial.println("USB mounted");
    } else {
      Serial.println("USB host not detected; continuing without USB mount");
    }
    #else
    Serial.println("USB mouse disabled");
    #endif

    #if ENABLE_BLE
    // Initialize BLE with NimBLE
    String bleName = appendUniqueId ? (deviceName + " " + String(uniqueId)) : deviceName;
    Serial.printf("BLE device name: '%s'\n", bleName.c_str());
    NimBLEDevice::init(bleName.c_str());
    
    // Enable bonding with "Just Works" pairing (no PIN required)
    // This allows the ESP32 to remember paired hosts across reboots
    NimBLEDevice::setSecurityAuth(true, false, false);  // bonding=true, MITM=false, SC=false
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);  // No input/output = Just Works
    
    // Create BLE Server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new BleConnectionCallbacks());
    
    // Create HID Device
    hid = new NimBLEHIDDevice(pServer);
    
    // Set HID info and report map
    hid->setManufacturer("Generic");
    // PnP ID: Use 0x02 (BT SIG), generic vendor 0x0000, product 0x0000 to avoid OS database lookup
    // Previously used Apple IDs (0x05ac, 0x820a) which caused OS to display "USB Optical Mouse"
    hid->setPnp(0x02, 0x0000, 0x0000, 0x0100);
    hid->setHidInfo(0x00, 0x01);  // HID info (country, flags)
    hid->setReportMap((uint8_t*)mouseHidReportDescriptor, sizeof(mouseHidReportDescriptor));
    
    // Get input characteristic for mouse reports
    input = hid->getInputReport(1); // Report ID 1
    
    // Start services
    hid->startServices();
    
    // Start advertising
    // Strategy: Put NAME in primary advertising packet for maximum visibility
    // Put appearance and service UUID in scan response (requested by most scanners)
    NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->enableScanResponse(true);  // Enable scan response packet
    pAdvertising->setName(bleName.c_str()); // Primary adv: device name (highest priority)
    pAdvertising->addServiceUUID(hid->getHidService()->getUUID()); // Scan response: HID service
    pAdvertising->setAppearance(HID_MOUSE); // Scan response: Mouse appearance (0x03C2)
    pAdvertising->start();
    
    Serial.printf("BLE Mouse initialized and advertising as '%s' (NimBLE)\n", bleName.c_str());
    #if ENABLE_LED
      ledBlink(3, 80); // 3 rapid taps for BLE start
    #endif
    #else
    Serial.println("BLE mouse disabled");
    #endif

    #if ENABLE_WIFI
    // WiFi connection
    connectWiFi();
    #else
      #if ENABLE_WEBUI
      // Start AP mode for WebUI when WiFi is disabled
      Serial.println("Starting WiFi AP for WebUI...");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(AP_SSID, AP_PASS);
      Serial.printf("AP SSID: %s\n", AP_SSID);
      Serial.printf("AP Password: %s\n", AP_PASS);
      Serial.printf("WebUI: http://%s\n", WiFi.softAPIP().toString().c_str());
      #else
      Serial.println("WiFi disabled - running in standalone mode");
      #endif
    #endif

    #if ENABLE_MQTT
    // MQTT connection (non-blocking on failure)
    connectMQTT();
    #endif

    #if ENABLE_WEBUI
    Serial.println("[Setup] Starting WebUI...");
    setupWebUI();
    Serial.println("[Setup] WebUI started");
    #endif

    // seed randomness
    Serial.println("[Setup] Seeding random...");
    randomSeed(esp_random());

    // initial randomized interval
    Serial.println("[Setup] Picking first interval...");
    pickNextMoveInterval();
    
    Serial.println("[Setup] Setup complete, entering loop");
    #if ENABLE_LED
      // Rapid flutter to indicate setup complete
      delay(500);
      ledBlink(5, 40);
    #endif
  }

  void loop() {
    #if ENABLE_LED
      // Update LED (non-blocking pulse/blink)
      updateLED();
    #endif

    #if ENABLE_WEBUI
      // Process DNS requests for captive portal (only in AP mode)
      if (wifiAPMode) {
        dnsServer.processNextRequest();
      }
      webServer.handleClient();
    #endif
    
    #if ENABLE_BLE
    // Track BLE connection state changes (moved from callback to avoid blocking BLE stack)
    static bool lastBleState = false;
    if (bleConnected != lastBleState) {
      lastBleState = bleConnected;
      #if ENABLE_MQTT
        if (mqttClient.connected()) {
          mqttClient.publish(MQTT_TOPIC_BLE_CONNECTED.c_str(), bleConnected ? "ON" : "OFF", true);
          mqttClient.publish(MQTT_TOPIC_BLE_HOST.c_str(), bleClientAddress.c_str(), true);
        }
      #endif
      #if ENABLE_LED
        if (bleConnected) {
          ledBlink(2, 300); // 2 slow pulses for BLE connect
        }
      #endif
    }
    #endif
    
    #if ENABLE_WIFI
    // WiFi reconnect logic
    static unsigned long lastWifiAttempt = 0;
    if (!wifiAPMode && WiFi.status() != WL_CONNECTED && millis() - lastWifiAttempt > 60000) {
      // Not in AP mode but lost connection - try to reconnect every 60s
      lastWifiAttempt = millis();
      Serial.println("WiFi disconnected, attempting reconnect...");
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str(), 0, NULL, true);  // Hidden SSID support
    }
    // In AP mode with configured credentials - try WiFi every 15 minutes
    if (wifiAPMode && wifiSSID.length() > 0 && millis() - lastWifiAttempt > 900000) {
      lastWifiAttempt = millis();
      tryWiFiFromAPMode();
    }
    #endif

    #if ENABLE_MQTT
    static unsigned long lastMqttAttempt = 0;
    static unsigned long lastRssiPublish = 0;
    #if ENABLE_USB
    static bool lastUsbState = false;
    #endif
    unsigned long now = millis();
    // Only try MQTT if WiFi is connected and MQTT is configured
    if (WiFi.status() == WL_CONNECTED && mqttServer.length() > 0) {
      if (!mqttClient.connected()) {
        if (now - lastMqttAttempt > 15000) {  // 15s between MQTT reconnect attempts
          connectMQTT();
          lastMqttAttempt = now;
        }
      } else {
        mqttClient.loop();
        // Publish RSSI every 60 seconds
        if (now - lastRssiPublish >= 60000) {
          lastRssiPublish = now;
          publishConfigJSON();  // Config includes RSSI
        }
        #if ENABLE_USB
        // Publish USB state changes
        // Note: If device is powered via USB, unplugging powers it off, so this mostly
        // detects: USB host sleep/wake, data-only cable swaps, or separate power scenarios
        bool currentUsbState = USB;
        if (currentUsbState != lastUsbState) {
          lastUsbState = currentUsbState;
          mqttClient.publish(MQTT_TOPIC_USB_MOUNTED.c_str(), currentUsbState ? "ON" : "OFF", true);
          Serial.printf("USB %s\n", currentUsbState ? "mounted" : "unmounted");
        }
        #endif
      }
    }
    #endif

    if (periodicOn) {
      unsigned long now2 = millis();
      if (now2 - lastMove >= moveInterval) {
        lastMove = now2;
        moveMouse();
        // pick next randomized interval
        pickNextMoveInterval();
      }
    }
  }

  // sanitize deviceName: allow only A-Z a-z 0-9 and space, limit length
  // BLE name limit: 29 chars max (31 bytes - 2 for type/length)
  // With unique ID suffix (" XXXX"): 24 chars for base name + 5 for suffix = 29 total
  // maxLen parameter: 24 if appendUniqueId enabled, 29 if disabled
  String sanitizeDeviceName(const String &in, size_t maxLen) {
    String out;
    for (size_t i = 0; i < in.length() && out.length() < maxLen; ++i) {
      char c = in.charAt(i);
      if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == ' ') {
        out += c;
      }
    }
    if (out.length() == 0) return String(DEFAULT_DEVICE_NAME);
    return out;
  }
