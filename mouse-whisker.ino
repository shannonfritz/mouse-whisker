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
    - ESP32-S2 Mini: USB HID + WiFi (no Bluetooth)
    - ESP32 NodeMCU: BLE + WiFi
    - ESP32 D1 Mini: BLE + WiFi
  
    Dependencies (install via Arduino Library Manager):
    - NimBLE-Arduino by h2zero (for BLE HID)
    - PubSubClient by Nick O'Leary (for MQTT)
    - Adafruit NeoPixel (for RGB LED on S3/S2 boards)
  */

  // Firmware version (Semantic Versioning: MAJOR.MINOR.PATCH)
  #define FIRMWARE_VERSION "1.6.3"

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
    #include <Update.h>     // For OTA firmware updates
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
    // ESP32-S2 Mini has WS2812 RGB LED on GPIO18
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
      bleConnected = true;
      bleClientAddress = connInfo.getAddress().toString().c_str();
      Serial.printf("BLE connected: %s\n", bleClientAddress.c_str());
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      bleConnected = false;
      bleClientAddress = "";
      Serial.println("BLE disconnected");
      NimBLEDevice::startAdvertising();
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
    
    // Diagnostic topics
    String MQTT_TOPIC_DIAG;           // JSON payload with all diagnostics
    String MQTT_TOPIC_SET_DIAG;       // Enable/disable diagnostics

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
    #include <map>
    std::map<String, unsigned long> captivePortalFirstSeen;  // Track when each client first connected
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

  // Diagnostic settings and counters
  // When disabled: no flash writes for boot count, no periodic MQTT publishes
  bool diagnosticsEnabled = false;         // Off by default to minimize flash wear
  unsigned long bootCount = 0;             // Persisted boot counter (only when diagnostics enabled)
  unsigned long wifiDisconnectCount = 0;   // WiFi reconnect count this session
  unsigned long mqttDisconnectCount = 0;   // MQTT reconnect count this session
  unsigned long lastWifiConnectTime = 0;   // millis() when WiFi last connected

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
    void publishDiagnosticsJSON();
    void publishEventJSON(const char* event, const String &value);
    void publishDiscovery();
    void mqttCallback(char* topic, byte* payload, unsigned int length);
  #endif
  String getResetReason();
  
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
      void handleWebDiag();
    #endif
    void handleFirmwarePage();
    void handleFirmwareUpload();
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

  // Get human-readable reset reason from ESP32
  String getResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
      case ESP_RST_POWERON:    return "Power-on";
      case ESP_RST_EXT:        return "External reset";
      case ESP_RST_SW:         return "Software reset";
      case ESP_RST_PANIC:      return "Exception/Panic";
      case ESP_RST_INT_WDT:    return "Interrupt watchdog";
      case ESP_RST_TASK_WDT:   return "Task watchdog";
      case ESP_RST_WDT:        return "Other watchdog";
      case ESP_RST_DEEPSLEEP:  return "Deep sleep wake";
      case ESP_RST_BROWNOUT:   return "Brownout";
      case ESP_RST_SDIO:       return "SDIO";
      default:                 return "Unknown (" + String((int)reason) + ")";
    }
  }

  // Helper function to get formatted uptime string
  String getUptimeString() {
    unsigned long uptimeMs = millis();
    unsigned long uptimeSec = uptimeMs / 1000;
    unsigned long uptimeMin = uptimeSec / 60;
    unsigned long uptimeHr = uptimeMin / 60;
    unsigned long uptimeDay = uptimeHr / 24;
    
    if (uptimeDay > 0) {
      return String(uptimeDay) + "d " + String(uptimeHr % 24) + "h " + String(uptimeMin % 60) + "m";
    } else if (uptimeHr > 0) {
      return String(uptimeHr) + "h " + String(uptimeMin % 60) + "m " + String(uptimeSec % 60) + "s";
    } else if (uptimeMin > 0) {
      return String(uptimeMin) + "m " + String(uptimeSec % 60) + "s";
    } else {
      return String(uptimeSec) + "s";
    }
  }

  #if ENABLE_MQTT
  // Publish diagnostic information (useful for debugging connectivity issues)
  // Always publishes the enabled state; other fields only meaningful when enabled
  void publishDiagnosticsJSON() {
    unsigned long uptimeSec = millis() / 1000;
    String uptimeStr = getUptimeString();
    
    String payload = "{";
    payload += "\"enabled\": " + String(diagnosticsEnabled ? "true" : "false") + ",";
    payload += "\"uptime_sec\": " + String(uptimeSec) + ",";
    payload += "\"uptime\": \"" + uptimeStr + "\",";
    payload += "\"boot_count\": " + String(bootCount) + ",";
    payload += "\"reset_reason\": \"" + getResetReason() + "\",";
    payload += "\"wifi_disconnects\": " + String(wifiDisconnectCount) + ",";
    payload += "\"mqtt_disconnects\": " + String(mqttDisconnectCount) + ",";
    payload += "\"free_heap\": " + String(ESP.getFreeHeap()) + ",";
    payload += "\"min_free_heap\": " + String(ESP.getMinFreeHeap()) + ",";
    payload += "\"wifi_rssi\": " + String(WiFi.RSSI()) + "}";
    
    mqttClient.publish(MQTT_TOPIC_DIAG.c_str(), payload.c_str(), true);
  }
  #endif

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

    // ========== DIAGNOSTIC SENSORS ==========
    // These help diagnose connectivity issues (WiFi vs device reboots/crashes)
    // Diagnostics are disabled by default to minimize flash wear (boot count writes)
    
    // Diagnostics enable switch (controls boot count writes and periodic publishing)
    String diagSwTopic = String("homeassistant/switch/mousewhisker_") + String(uniqueId) + "/diagnostics/config";
    String diagSw = "{";
    diagSw += "\"name\": \"Diagnostics\",";
    diagSw += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_diagnostics\",";
    diagSw += "\"command_topic\": \"" + MQTT_TOPIC_SET_DIAG + "\",";
    diagSw += "\"state_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    diagSw += "\"value_template\": \"{{ 'ON' if value_json.enabled else 'OFF' }}\",";
    diagSw += "\"json_attributes_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    diagSw += "\"json_attributes_template\": \"{ \\\"hint\\\": \\\"Enable to track boot count, uptime, disconnects. Writes to flash on each boot when enabled.\\\" }\",";
    diagSw += "\"state_on\": \"ON\",";
    diagSw += "\"state_off\": \"OFF\",";
    diagSw += "\"payload_on\": \"1\",";
    diagSw += "\"payload_off\": \"0\",";
    diagSw += "\"optimistic\": false,";
    diagSw += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    diagSw += "\"icon\": \"mdi:bug-outline\",";
    diagSw += "\"entity_category\": \"diagnostic\",";
    diagSw += "\"unique_id\": \"" + baseId + "_diagnostics\",";
    diagSw += "\"device\": " + deviceShort + "}";
    mqttClient.publish(diagSwTopic.c_str(), diagSw.c_str(), true);
    yieldAndProcess("diagSwitch");
    
    // Uptime sensor (human-readable format)
    String uptimeTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/uptime/config";
    String uptimeSensor = "{";
    uptimeSensor += "\"name\": \"Uptime\",";
    uptimeSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_uptime\",";
    uptimeSensor += "\"state_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    uptimeSensor += "\"value_template\": \"{{ value_json.uptime }}\",";
    uptimeSensor += "\"json_attributes_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    uptimeSensor += "\"json_attributes_template\": \"{ \\\"seconds\\\": {{ value_json.uptime_sec }} }\",";
    uptimeSensor += "\"entity_category\": \"diagnostic\",";
    uptimeSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    uptimeSensor += "\"icon\": \"mdi:timer-outline\",";
    uptimeSensor += "\"unique_id\": \"" + baseId + "_uptime\",";
    uptimeSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(uptimeTopic.c_str(), uptimeSensor.c_str(), true);
    yieldAndProcess("uptime");

    // Boot count sensor (persisted across reboots)
    String bootTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/boot_count/config";
    String bootSensor = "{";
    bootSensor += "\"name\": \"Boot Count\",";
    bootSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_boot_count\",";
    bootSensor += "\"state_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    bootSensor += "\"value_template\": \"{{ value_json.boot_count }}\",";
    bootSensor += "\"state_class\": \"total_increasing\",";
    bootSensor += "\"entity_category\": \"diagnostic\",";
    bootSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    bootSensor += "\"icon\": \"mdi:counter\",";
    bootSensor += "\"unique_id\": \"" + baseId + "_boot_count\",";
    bootSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(bootTopic.c_str(), bootSensor.c_str(), true);
    yieldAndProcess("bootCount");

    // Last reset reason sensor
    String resetTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/reset_reason/config";
    String resetSensor = "{";
    resetSensor += "\"name\": \"Last Reset Reason\",";
    resetSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_reset_reason\",";
    resetSensor += "\"state_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    resetSensor += "\"value_template\": \"{{ value_json.reset_reason }}\",";
    resetSensor += "\"entity_category\": \"diagnostic\",";
    resetSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    resetSensor += "\"icon\": \"mdi:restart-alert\",";
    resetSensor += "\"unique_id\": \"" + baseId + "_reset_reason\",";
    resetSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(resetTopic.c_str(), resetSensor.c_str(), true);
    yieldAndProcess("resetReason");

    // WiFi disconnect count sensor (this session)
    String wifiDiscTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/wifi_disconnects/config";
    String wifiDiscSensor = "{";
    wifiDiscSensor += "\"name\": \"WiFi Disconnects\",";
    wifiDiscSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_wifi_disconnects\",";
    wifiDiscSensor += "\"state_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    wifiDiscSensor += "\"value_template\": \"{{ value_json.wifi_disconnects }}\",";
    wifiDiscSensor += "\"state_class\": \"total_increasing\",";
    wifiDiscSensor += "\"entity_category\": \"diagnostic\",";
    wifiDiscSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    wifiDiscSensor += "\"icon\": \"mdi:wifi-off\",";
    wifiDiscSensor += "\"unique_id\": \"" + baseId + "_wifi_disconnects\",";
    wifiDiscSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(wifiDiscTopic.c_str(), wifiDiscSensor.c_str(), true);
    yieldAndProcess("wifiDisconnects");

    // MQTT disconnect count sensor (this session)
    String mqttDiscTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/mqtt_disconnects/config";
    String mqttDiscSensor = "{";
    mqttDiscSensor += "\"name\": \"MQTT Disconnects\",";
    mqttDiscSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_mqtt_disconnects\",";
    mqttDiscSensor += "\"state_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    mqttDiscSensor += "\"value_template\": \"{{ value_json.mqtt_disconnects }}\",";
    mqttDiscSensor += "\"state_class\": \"total_increasing\",";
    mqttDiscSensor += "\"entity_category\": \"diagnostic\",";
    mqttDiscSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    mqttDiscSensor += "\"icon\": \"mdi:connection\",";
    mqttDiscSensor += "\"unique_id\": \"" + baseId + "_mqtt_disconnects\",";
    mqttDiscSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(mqttDiscTopic.c_str(), mqttDiscSensor.c_str(), true);
    yieldAndProcess("mqttDisconnects");

    // Free heap memory sensor
    String heapTopic = String("homeassistant/sensor/mousewhisker_") + String(uniqueId) + "/free_heap/config";
    String heapSensor = "{";
    heapSensor += "\"name\": \"Free Heap\",";
    heapSensor += "\"object_id\": \"mousewhisker_" + String(uniqueId) + "_free_heap\",";
    heapSensor += "\"state_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    heapSensor += "\"value_template\": \"{{ value_json.free_heap }}\",";
    heapSensor += "\"json_attributes_topic\": \"" + MQTT_TOPIC_DIAG + "\",";
    heapSensor += "\"json_attributes_template\": \"{ \\\"min_free_heap\\\": {{ value_json.min_free_heap }} }\",";
    heapSensor += "\"unit_of_measurement\": \"B\",";
    heapSensor += "\"entity_category\": \"diagnostic\",";
    heapSensor += "\"availability_topic\": \"" + MQTT_TOPIC_AVAILABILITY + "\",";
    heapSensor += "\"icon\": \"mdi:memory\",";
    heapSensor += "\"unique_id\": \"" + baseId + "_free_heap\",";
    heapSensor += "\"device\": " + deviceShort + "}";
    mqttClient.publish(heapTopic.c_str(), heapSensor.c_str(), true);
    yieldAndProcess("freeHeap");
    
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
      Serial.println("Reboot via MQTT");
      publishEventJSON("reboot", "rebooting");
      delay(500);
      ESP.restart();
      return;
    }

    if (t == MQTT_TOPIC_CMD_FACTORY_RESET) {
      Serial.println("FACTORY RESET");
      publishEventJSON("factory_reset", "resetting");
      mqttClient.loop();
      delay(100);
      mqttClient.loop();
      preferences.clear();
      preferences.end();
      delay(200);
      ESP.restart();
      return;
    }

    if (t == MQTT_TOPIC_SET_XRANGE) {
      int v = msg.toInt();
      if (v >= 0 && v <= 127) {
        if (v != xRange) {
          Serial.printf("[MQTT] xRange: %d->%d\n", xRange, v);
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
          Serial.printf("[MQTT] yRange: %d->%d\n", yRange, v);
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
          Serial.printf("[MQTT] minInt: %d->%d\n", minMoveInterval, v);
          minMoveInterval = v;
          preferences.putInt("minmove", minMoveInterval);
        }
        if (minMoveInterval > maxMoveInterval) {
          Serial.printf("[MQTT] maxInt adjusted: %d->%d\n", maxMoveInterval, minMoveInterval);
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
          Serial.printf("[MQTT] maxInt: %d->%d\n", maxMoveInterval, v);
          maxMoveInterval = v;
          preferences.putInt("maxmove", maxMoveInterval);
        }
        if (maxMoveInterval < minMoveInterval) {
          Serial.printf("[MQTT] minInt adjusted: %d->%d\n", minMoveInterval, maxMoveInterval);
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
        
        if (clean != deviceName) {
          Serial.printf("Name: '%s'->'%s'\n", deviceName.c_str(), clean.c_str());
          deviceName = clean;
          preferences.putString("devname", deviceName);
          #if ENABLE_USB
            USB.productName(deviceName.c_str());
          #endif
          publishEventJSON("name_set", deviceName);
          publishConfigJSON();
        } else if (wasTruncated) {
          publishConfigJSON();
        }
      }
      return;
    }

    if (t == MQTT_TOPIC_SET_APPENDID) {
      bool newVal = (msg == "1" || msg.equalsIgnoreCase("on"));
      if (newVal != appendUniqueId) {
        Serial.printf("AppendID: %d->%d\n", appendUniqueId, newVal);
        appendUniqueId = newVal;
        preferences.putBool("appendid", appendUniqueId);
        publishEventJSON("appendid_set", String(appendUniqueId ? "true" : "false"));
        publishConfigJSON();
      }
      return;
    }

    #if ENABLE_LED
    if (t == MQTT_TOPIC_SET_LED) {
      bool newVal = (msg == "1" || msg.equalsIgnoreCase("on"));
      if (newVal != ledEnabled) {
        Serial.printf("[MQTT] LED: %d->%d\n", ledEnabled, newVal);
        ledEnabled = newVal;
        preferences.putBool("ledEnabled", ledEnabled);
        publishEventJSON("led_set", String(ledEnabled ? "true" : "false"));
        publishConfigJSON();
      }
      return;
    }
    #endif

    // Diagnostics enable/disable
    if (t == MQTT_TOPIC_SET_DIAG) {
      bool newVal = (msg == "1" || msg.equalsIgnoreCase("on"));
      if (newVal != diagnosticsEnabled) {
        Serial.printf("[MQTT] Diag: %d->%d\n", diagnosticsEnabled, newVal);
        diagnosticsEnabled = newVal;
        preferences.putBool("diagEnabled", diagnosticsEnabled);
        
        // If just enabled, increment boot count now (since we skipped it at boot)
        if (diagnosticsEnabled && bootCount == preferences.getULong("bootCount", 0)) {
          bootCount++;
          preferences.putULong("bootCount", bootCount);
          Serial.printf("  Boot count incremented to %lu\n", bootCount);
        }
        
        publishEventJSON("diagnostics_set", String(diagnosticsEnabled ? "true" : "false"));
        publishDiagnosticsJSON();  // Publish immediately so HA updates
      }
      return;
    }

    // fallback: legacy single cmd topic
    if (t == MQTT_TOPIC_CMD) {
      handleSetCommand(msg);
    }
  }
  #endif // ENABLE_MQTT

  #if ENABLE_WIFI
  // Event handler when client connects to AP
  void onWiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("[WiFi] Client connected to AP (MAC: %02X:%02X:%02X:%02X:%02X:%02X)\n",
      info.wifi_ap_staconnected.mac[0], info.wifi_ap_staconnected.mac[1],
      info.wifi_ap_staconnected.mac[2], info.wifi_ap_staconnected.mac[3],
      info.wifi_ap_staconnected.mac[4], info.wifi_ap_staconnected.mac[5]);
  }
  
  // Event handler to reset captive portal when client disconnects
  void onWiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (wifiAPMode) {
      // Convert MAC to IP isn't straightforward, so just clear all tracked IPs
      // This is fine since disconnects are infrequent
      captivePortalFirstSeen.clear();
      Serial.printf("[WiFi] Client disconnected from AP (MAC: %02X:%02X:%02X:%02X:%02X:%02X)\n",
        info.wifi_ap_stadisconnected.mac[0], info.wifi_ap_stadisconnected.mac[1],
        info.wifi_ap_stadisconnected.mac[2], info.wifi_ap_stadisconnected.mac[3],
        info.wifi_ap_stadisconnected.mac[4], info.wifi_ap_stadisconnected.mac[5]);
      Serial.println("[Captive] Portal reset for next connection");
    }
  }

  void startAPMode() {
    Serial.println("Starting WiFi AP mode for configuration...");
    WiFi.mode(WIFI_AP);
    
    // Register event handlers for client connect/disconnect
    WiFi.onEvent(onWiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent(onWiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
    
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
    WiFi.persistent(false);  // Don't save to flash during connection attempts
    WiFi.setAutoReconnect(false);  // We handle reconnection ourselves in loop()
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);  // Helps with hidden networks
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
    
    // Debug: show credentials info
    Serial.printf("WiFi credentials: SSID='%s' (len=%d), Pass len=%d\n", 
                  wifiSSID.c_str(), wifiSSID.length(), wifiPassword.length());
    
    // Start with normal attempts, may switch to hidden mode if detected
    bool hiddenMode = false;
    int maxAttempts = 3;
    int timeout = 15000;
    int retryDelay = 100;
    
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
      // Clean disconnect before each attempt (helps with hidden SSIDs)
      WiFi.disconnect(true);  // true = also clear stored credentials
      delay(hiddenMode ? 500 : 100);  // Longer delay for hidden SSIDs
      
      // Use extended begin() with channel=0, bssid=NULL, connect=true for hidden SSID support
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str(), 0, NULL, true);
      Serial.printf("Connecting to WiFi '%s' (attempt %d/%d)%s", wifiSSID.c_str(), attempt, maxAttempts, hiddenMode ? " [hidden mode]" : "");
      
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
        if (millis() - start > timeout) break;
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
      
      // On first failure, scan for networks to help diagnose and detect hidden SSID situation
      if (attempt == 1) {
        Serial.println("Scanning for networks to diagnose...");
        WiFi.disconnect();  // Ensure radio is idle before scanning
        delay(100);
        int n = WiFi.scanNetworks(false, true);  // false=async off, true=show hidden
        if (n < 0) {
          // Scan failed, retry once after brief delay
          Serial.printf("  Scan failed (error %d), retrying...\n", n);
          delay(500);
          n = WiFi.scanNetworks(false, true);
        }
        if (n < 0) {
          Serial.printf("  Scan failed (error %d)\n", n);
        } else if (n == 0) {
          Serial.println("  No networks found");
        } else {
          Serial.printf("  Found %d networks:\n", n);
          bool foundTarget = false;
          int hiddenCount = 0;
          for (int i = 0; i < n && i < 10; i++) {
            String ssid = WiFi.SSID(i);
            bool hidden = ssid.length() == 0;
            if (hidden) hiddenCount++;
            Serial.printf("    %d: '%s' (%d dBm) ch%d %s%s\n", 
                          i+1, 
                          hidden ? "(hidden)" : ssid.c_str(),
                          WiFi.RSSI(i),
                          WiFi.channel(i),
                          WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "open" : "encrypted",
                          (ssid == wifiSSID) ? " <-- TARGET" : "");
            if (ssid == wifiSSID) foundTarget = true;
          }
          
          // Auto-detect hidden SSID: target not found in scan but hidden networks exist
          if (!foundTarget && hiddenCount > 0 && status == WL_NO_SSID_AVAIL && !hiddenMode) {
            Serial.printf("  Auto-detected hidden SSID scenario (found %d hidden network%s)\n", 
                          hiddenCount, hiddenCount > 1 ? "s" : "");
            Serial.println("  Switching to hidden SSID mode with extended timeouts...");
            hiddenMode = true;
            maxAttempts = 5;  // More attempts for hidden networks
            timeout = 20000;  // Longer timeout
          } else if (!foundTarget) {
            Serial.println("  WARNING: Target SSID not found in scan (may be hidden or out of range)");
          }
        }
        WiFi.scanDelete();
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
    
    // Use AP+STA mode to keep AP running while attempting connection
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str(), 0, NULL, true);  // Hidden SSID support
    
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(100);
      #if ENABLE_LED
        updateLED();
      #endif
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      // Success! Switch to STA-only mode (stop AP)
      wifiAPMode = false;
      WiFi.mode(WIFI_STA);
      Serial.println("WiFi connected! Switching from AP mode.");
      Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
      #if ENABLE_LED
        ledBlink(2, 300);
      #endif
      return true;
    }
    
    // Failed - go back to AP-only mode
    Serial.println("  WiFi still unavailable, staying in AP mode");
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
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
      mqttClient.subscribe(MQTT_TOPIC_SET_DIAG.c_str());  // Diagnostics enable/disable
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
      // Publish initial diagnostics
      publishDiagnosticsJSON();
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
      "<style>body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:10px;}"
      "h2{color:#333;}h3{margin-top:20px;margin-bottom:10px;}"
      "input[type=text],input[type=password],input[type=number]{padding:10px;border:1px solid #ccc;border-radius:4px;font-size:16px;}"
      "input[type=text],input[type=password]{width:100%;max-width:280px;box-sizing:border-box;}"
      "input[type=number]{width:80px;}input[type=checkbox]{width:20px;height:20px;margin-right:8px;}"
      "label{display:block;margin:8px 0;}"
      ".btn{display:inline-block;padding:14px 28px;margin:5px;background:#007bff;color:#fff;text-decoration:none;border-radius:6px;border:none;cursor:pointer;font-size:16px;}"
      ".btn:hover{background:#0056b3;}.status{padding:10px;margin:10px 0;border-radius:4px;}"
      ".on{background:#d4edda;color:#155724;}.off{background:#f8d7da;color:#721c24;}"
      ".wait{background:#fff3cd;color:#856404;}.disabled{background:#e2e3e5;color:#6c757d;}"
      ".tight{margin:4px 0;}.group{background:#f8f9fa;border:1px solid #dee2e6;border-radius:6px;padding:12px;margin:15px 0;}"
      ".group-title{font-weight:bold;margin-bottom:8px;display:flex;justify-content:space-between;align-items:center;}"
      ".group-status{font-size:13px;font-weight:normal;padding:4px 10px;border-radius:4px;}"
      ".group-status.on{color:#155724;background:#d4edda;}.group-status.off{color:#721c24;background:#f8d7da;}.group-status.wait{color:#856404;background:#fff3cd;}"
      "details{margin:10px 0;}summary{cursor:pointer;padding:8px 0;color:#007bff;}"
      "summary:hover{text-decoration:underline;}"
      ".diag-table{width:100%;border-collapse:collapse;margin:10px 0;}.diag-table td{padding:6px 8px;border-bottom:1px solid #dee2e6;}"
      ".diag-table td:first-child{color:#666;width:45%;}"
      ".help-box{margin:10px 0;background:#f0f7ff;border:1px solid #cce5ff;border-radius:6px;padding:0 12px;}"
      ".help-box summary{font-size:13px;color:#004085;}.help-content{font-size:12px;color:#444;}"
      ".help-content p{margin:6px 0;}"
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
    html += "</form><hr>";
    
    // WiFi Configuration section
    #if ENABLE_WIFI
    html += F("<div class='group'>");
    html += F("<div class='group-title'>WiFi <span id='wifiStatusText' class='group-status ");
    if (wifiAPMode) {
      html += F("wait'>AP Mode</span></div>");
    } else {
      html += "on'>Connected to " + wifiSSID + "</span></div>";
    }
    html += F("<details><summary>Configure</summary>");
    html += F("<form method='POST' action='/wifi'>");
    html += "<p><label>SSID: <input type='text' name='ssid' value='" + wifiSSID + "' maxlength='32'></label></p>";
    html += F("<p><label>Password: <input type='password' name='pass' id='wifiPass' maxlength='64'></label>");
    html += F(" <label class='tight'><input type='checkbox' onclick='togglePass(\"wifiPass\")'> Show</label></p>");
    html += F("<p><input type='submit' class='btn' value='Save WiFi'></p>");
    html += F("</form></details></div>");
    #endif
    
    // MQTT Configuration section
    #if ENABLE_MQTT
    html += F("<div class='group'>");
    html += F("<div class='group-title'>MQTT <span id='mqttStatusText' class='group-status ");
    if (mqttServer.length() == 0) {
      html += F("wait'>Not configured</span></div>");
    } else if (mqttClient.connected()) {
      html += "on'>Connected to " + mqttServer + "</span></div>";
    } else {
      html += "off'>Disconnected</span></div>";
    }
    html += F("<details><summary>Configure</summary>");
    html += F("<form method='POST' action='/mqtt'>");
    html += "<p><label>Server: <input type='text' name='server' value='" + mqttServer + "' maxlength='64' placeholder='192.168.1.100 or hostname'></label></p>";
    html += "<p><label>Port: <input type='number' name='port' value='" + String(mqttPort) + "' min='1' max='65535'></label></p>";
    html += "<p><label>Username: <input type='text' name='user' value='" + mqttUser + "' maxlength='32'></label></p>";
    html += F("<p><label>Password: <input type='password' name='pass' id='mqttPass' maxlength='64'></label>");
    html += F(" <label class='tight'><input type='checkbox' onclick='togglePass(\"mqttPass\")'> Show</label></p>");
    html += F("<p><input type='submit' class='btn' value='Save MQTT'></p>");
    html += F("</form></details></div>");
    
    // Diagnostics section
    html += F("<div class='group'>");
    html += F("<div class='group-title'>Diagnostics <span id='diagStatusText' class='group-status ");
    html += String(diagnosticsEnabled ? "on'>Enabled" : "wait'>Disabled") + "</span></div>";
    html += F("<details><summary>View &amp; Configure</summary>");
    html += F("<form method='POST' action='/diag'>");
    html += "<p><label><input type='checkbox' name='enabled' id='diagEnabled' " + String(diagnosticsEnabled ? "checked" : "") + "> Enable Diagnostics</label></p>";
    html += F("<p><small><i>When enabled, boot count is persisted to flash. Disable to minimize flash wear.</i></small></p>");
    html += F("<p><input type='submit' class='btn' value='Save'></p>");
    html += F("</form>");
    html += F("<table class='diag-table'>");
    html += "<tr><td>Uptime:</td><td id='diagUptime'>" + getUptimeString() + "</td></tr>";
    html += "<tr><td>Boot Count:</td><td id='diagBootCount'>" + String(bootCount) + "</td></tr>";
    html += "<tr><td>Reset Reason:</td><td id='diagResetReason'>" + getResetReason() + "</td></tr>";
    html += "<tr><td>WiFi Disconnects:</td><td id='diagWifiDisc'>" + String(wifiDisconnectCount) + "</td></tr>";
    html += "<tr><td>MQTT Disconnects:</td><td id='diagMqttDisc'>" + String(mqttDisconnectCount) + "</td></tr>";
    int rssi = WiFi.RSSI();
    String quality = (rssi >= -50) ? "Excellent" : (rssi >= -60) ? "Good" : (rssi >= -70) ? "Fair" : "Weak";
    html += "<tr><td>WiFi Signal:</td><td id='diagRssi'>" + String(rssi) + " dBm (" + quality + ")</td></tr>";
    html += "<tr><td>Free Heap:</td><td id='diagHeap'>" + String(ESP.getFreeHeap() / 1024) + " KB</td></tr>";
    html += F("</table>");
    html += F("<details class='help-box'><summary>How to interpret these values</summary>"
      "<div class='help-content'>"
      "<p><b>Uptime</b> resets on every reboot. If Home Assistant showed unavailable but uptime is high, "
      "the issue was likely network-related, not a crash.</p>"
      "<p><b>Boot Count</b> increments each start. Unexpected increases indicate crashes or power issues.</p>"
      "<p><b>Reset Reason</b> shows why the last reboot occurred. "
      "<i>Watchdog</i> or <i>Panic</i> = crash. <i>Brownout</i> = power issue.</p>"
      "<p><b>WiFi/MQTT Disconnects</b> count reconnections this session. High counts suggest network instability.</p>"
      "<p><b>WiFi Signal</b> (RSSI): -30 to -50 Excellent, -50 to -65 Good, -65 to -80 Weak, below -80 Poor.</p>"
      "<p><b>Free Heap</b> is available memory. Below 20 KB may cause instability.</p>"
      "</div></details></details></div>");
    #endif
    
    html += "<hr><p><small>Mouse Whisker v" + String(FIRMWARE_VERSION) + " by Shannon Fritz | <a href='/reboot'>Reboot</a> | <a href='/reset'>Reset</a> | <a href='/firmware'>Update</a></small></p>";
    
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
          "var ds=document.getElementById('diagStatusText');if(ds){"
            "ds.textContent=d.diagEnabled?'Enabled':'Disabled';"
            "ds.className='group-status '+(d.diagEnabled?'on':'wait');"
            "document.getElementById('diagEnabled').checked=d.diagEnabled;"
            "document.getElementById('diagUptime').textContent=d.uptime;"
            "document.getElementById('diagBootCount').textContent=d.bootCount;"
            "document.getElementById('diagWifiDisc').textContent=d.wifiDisc;"
            "document.getElementById('diagMqttDisc').textContent=d.mqttDisc;"
            "document.getElementById('diagRssi').textContent=d.rssi+' dBm ('+d.wifiQuality+')';"
            "document.getElementById('diagHeap').textContent=d.heap+' KB';"
          "}"
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
    #if ENABLE_MQTT
    // Diagnostics data
    json += ",\"diagEnabled\":" + String(diagnosticsEnabled ? "true" : "false");
    json += ",\"uptime\":\"" + getUptimeString() + "\"";
    json += ",\"bootCount\":" + String(bootCount);
    json += ",\"resetReason\":\"" + getResetReason() + "\"";
    json += ",\"wifiDisc\":" + String(wifiDisconnectCount);
    json += ",\"mqttDisc\":" + String(mqttDisconnectCount);
    int statusRssi = WiFi.RSSI();
    String statusQuality = (statusRssi >= -50) ? "Excellent" : (statusRssi >= -60) ? "Good" : (statusRssi >= -70) ? "Fair" : "Weak";
    json += ",\"rssi\":" + String(statusRssi);
    json += ",\"wifiQuality\":\"" + statusQuality + "\"";
    json += ",\"heap\":" + String(ESP.getFreeHeap() / 1024);
    #endif
    json += "}";
    webServer.send(200, "application/json", json);
  }

  void handleWebRoot() {
    if (wifiAPMode) {
      // In AP mode, show captive portal landing page
      String clientIP = webServer.client().remoteIP().toString();
      Serial.printf("[Captive] Landing page accessed from %s\n", clientIP.c_str());
      captivePortalFirstSeen[clientIP] = 0;  // Mark complete
      webServer.send(200, "text/html", F(
        "<HTML><HEAD><TITLE>Mouse Whisker</TITLE></HEAD>"
        "<BODY style='font-family:-apple-system,sans-serif;text-align:center;padding:40px 20px;margin:0;background:#f8f9fa;'>"
        "<div style='max-width:320px;margin:0 auto;background:#fff;padding:30px;border-radius:16px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
        "<h2 style='font-size:24px;margin:0 0 16px;color:#333;'>&#128433; Mouse Whisker</h2>"
        "<p style='font-size:16px;color:#666;margin:0 0 24px;'>Connected! Tap below to configure.</p>"
        "<a style='display:block;padding:16px 24px;background:#007bff;color:#fff;text-decoration:none;border-radius:10px;font-size:18px;font-weight:500;' href='http://192.168.4.1/setup'>Open Setup</a>"
        "</div>"
        "</BODY></HTML>"));
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
    Serial.println("[Web] Settings update");
    bool needsReboot = false;
    
    // Mouse Name (requires reboot)
    if (webServer.hasArg("name")) {
      String newName = webServer.arg("name");
      size_t maxLen = appendUniqueId ? 24 : 29;
      newName = sanitizeDeviceName(newName, maxLen);
      if (newName != deviceName) {
        Serial.printf("[Web] Name: '%s'->'%s'\n", deviceName.c_str(), newName.c_str());
        deviceName = newName;
        preferences.putString("devname", deviceName);
        needsReboot = true;
      }
    }
    
    // Append ID (requires reboot)
    bool newAppendId = webServer.hasArg("appendid");
    if (newAppendId != appendUniqueId) {
      Serial.printf("[Web] AppendID: %d->%d\n", appendUniqueId, newAppendId);
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
        Serial.printf("[Web] minInt: %d->%d\n", minMoveInterval, v);
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
        Serial.printf("[Web] maxInt: %d->%d\n", maxMoveInterval, v);
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
        Serial.printf("[Web] xRange: %d->%d\n", xRange, v);
        xRange = v;
        preferences.putInt("xrange", xRange);
      }
    }
    
    // Y Range
    if (webServer.hasArg("yrange")) {
      int v = webServer.arg("yrange").toInt();
      if (v >= 0 && v <= 127 && v != yRange) {
        Serial.printf("[Web] yRange: %d->%d\n", yRange, v);
        yRange = v;
        preferences.putInt("yrange", yRange);
      }
    }
    
    #if ENABLE_LED
    // LED enable
    bool newLed = webServer.hasArg("led");
    if (newLed != ledEnabled) {
      Serial.printf("[Web] LED: %d->%d\n", ledEnabled, newLed);
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

  // Factory reset confirmation page
  void handleWebReset() {
    Serial.printf("[WebUI] Reset page accessed from %s\n", webServer.client().remoteIP().toString().c_str());
    webServer.send(200, "text/html", F(
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Factory Reset</title>"
      "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
      "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
      "<h2 style='color:#dc3545;margin-top:0;'>Factory Reset</h2>"
      "<p style='font-size:16px;'>This will erase all settings including WiFi, MQTT, and device configuration.</p>"
      "<p style='font-size:14px;color:#666;'>The device will reboot in AP mode for reconfiguration.</p>"
      "<form method='POST' action='/reset-confirm' style='margin-top:20px;'>"
      "<button type='submit' style='padding:14px 28px;background:#dc3545;color:#fff;border:none;border-radius:6px;font-size:16px;cursor:pointer;'>Confirm Reset</button>"
      "</form>"
      "<p style='margin-top:20px;'><a href='/setup'>&larr; Cancel</a></p>"
      "</div></body></html>"));
  }

  // Factory reset execute handler
  void handleWebResetConfirm() {
    Serial.printf("[WebUI] Factory reset confirmed from %s\n", webServer.client().remoteIP().toString().c_str());
    webServer.send(200, "text/html", F(
      "<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Resetting</title>"
      "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
      "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
      "<h2 style='color:#dc3545;margin-top:0;'>Resetting...</h2>"
      "<p style='font-size:18px;'>All settings have been erased.</p>"
      "<p style='font-size:14px;color:#666;'>The device will reboot in AP mode.</p>"
      "<div style='margin-top:20px;'><span style='display:inline-block;width:40px;height:40px;border:4px solid #dc3545;border-top-color:transparent;border-radius:50%;animation:spin 1s linear infinite;'></span></div>"
      "<style>@keyframes spin{to{transform:rotate(360deg);}}</style>"
      "</div></body></html>"));
    Serial.println("FACTORY RESET via WebUI");
    delay(500);
    preferences.begin("mousewhisker", false);
    preferences.clear();
    preferences.end();
    delay(100);
    ESP.restart();
  }

  // Firmware update page
  void handleFirmwarePage() {
    String html = F("<!DOCTYPE html><html><head>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Firmware Update</title>"
      "<style>body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:10px;}"
      ".btn{display:inline-block;padding:14px 28px;margin:5px;background:#007bff;color:#fff;text-decoration:none;border-radius:6px;border:none;cursor:pointer;font-size:16px;}"
      ".btn:hover{background:#0056b3;}.btn-warn{background:#dc3545;}.btn-warn:hover{background:#c82333;}"
      ".box{background:#f8f9fa;border:1px solid #dee2e6;border-radius:6px;padding:15px;margin:15px 0;}"
      "input[type=file]{margin:10px 0;}"
      "</style></head><body>");
    html += F("<h2>Firmware Update</h2>");
    html += "<p>Current version: <b>" + String(FIRMWARE_VERSION) + "</b></p>";
    html += F("<div class='box'>"
      "<p><b>Instructions:</b></p>"
      "<ol>"
      "<li>Download the *_ota.bin file for your board from <a href='https://github.com/ShannonFritz/mouse-whisker/releases' target='_blank'>GitHub Releases</a></li>"
      "<li>Select the file below and click Update</li>"
      "<li>Wait for the update to complete (~30 seconds)</li>"
      "</ol>"
      "</div>");
    html += F("<form method='POST' action='/firmware' enctype='multipart/form-data'>"
      "<p><input type='file' name='firmware' accept='.bin'></p>"
      "<p><input type='submit' class='btn btn-warn' value='Update Firmware'></p>"
      "</form>");
    html += F("<p><a href='/setup'>&larr; Back to Settings</a></p>");
    html += F("</body></html>");
    webServer.send(200, "text/html", html);
  }

  // Handle firmware upload chunks
  void handleFirmwareUpload() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("[OTA] Update starting: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.printf("[OTA] Begin failed: %s\n", Update.errorString());
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Serial.printf("[OTA] Write failed: %s\n", Update.errorString());
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[OTA] Update success: %u bytes\n", upload.totalSize);
      } else {
        Serial.printf("[OTA] End failed: %s\n", Update.errorString());
      }
    }
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
  
  void handleWebDiag() {
    Serial.printf("[WebUI] Diagnostics config update from %s\n", webServer.client().remoteIP().toString().c_str());
    
    bool newEnabled = webServer.hasArg("enabled");
    
    if (newEnabled != diagnosticsEnabled) {
      Serial.printf("[WebUI] Diagnostics: %s -> %s\n", 
                    diagnosticsEnabled ? "enabled" : "disabled",
                    newEnabled ? "enabled" : "disabled");
      diagnosticsEnabled = newEnabled;
      preferences.putBool("diagEnabled", diagnosticsEnabled);
      
      // If just enabled, increment boot count now (since we missed it at boot)
      if (diagnosticsEnabled && bootCount == 0) {
        bootCount = preferences.getULong("bootCount", 0) + 1;
        preferences.putULong("bootCount", bootCount);
        Serial.printf("[Diag] Boot count initialized: %lu\n", bootCount);
      }
      
      webServer.send(200, "text/html", F(
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta http-equiv='refresh' content='2;url=/setup'>"
        "<title>Diagnostics Saved</title>"
        "</head><body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
        "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
        "<h2 style='color:#28a745;margin-top:0;'>&#10003; Diagnostics Saved</h2>"
        "<p style='font-size:18px;'>Settings updated.</p>"
        "</div></body></html>"));
    } else {
      webServer.sendHeader("Location", "/setup");
      webServer.send(303);
    }
  }
  #endif

  // Captive portal handler - redirects all unknown requests to main page
  void handleCaptivePortal() {
    // Only act as captive portal in AP mode
    if (!wifiAPMode) {
      webServer.send(404, "text/plain", "Not Found");
      return;
    }
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
  // iOS CNA requires <TITLE>Success</TITLE> to show blue checkmark
  void handleCaptiveDetect() {
    // Only handle captive portal detection in AP mode
    if (!wifiAPMode) {
      webServer.send(404, "text/plain", "Not Found");
      return;
    }
    
    String clientIP = webServer.client().remoteIP().toString();
    String endpoint = webServer.uri();
    
    // Check if this client has been marked complete
    auto it = captivePortalFirstSeen.find(clientIP);
    bool isComplete = (it != captivePortalFirstSeen.end() && it->second == 0);
    
    if (isComplete) {
      // Client has completed portal - return Success with setup link
      Serial.printf("[Captive] %s requested %s -> Success (complete)\n", clientIP.c_str(), endpoint.c_str());
      // Includes hidden Success text to make iOS CNA happy
      webServer.send(200, "text/html", F(
        "<HTML><HEAD><TITLE>Success</TITLE></HEAD>"
        "<BODY style='font-family:-apple-system,sans-serif;text-align:center;padding:40px 20px;margin:0;background:#f8f9fa;'>"
        "<p style='visibility:hidden;height:0;margin:0;'>Success</p>"
        "<div style='max-width:320px;margin:0 auto;background:#fff;padding:30px;border-radius:16px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
        "<h2 style='font-size:24px;margin:0 0 16px;color:#333;'>&#128433; Mouse Whisker</h2>"
        "<p style='font-size:16px;color:#666;margin:0 0 24px;'>Connected! Tap below to configure.</p>"
        "<a style='display:block;padding:16px 24px;background:#007bff;color:#fff;text-decoration:none;border-radius:10px;font-size:18px;font-weight:500;' href='http://192.168.4.1/setup'>Open Setup</a>"
        "</div>"
        "</BODY></HTML>"));
        } else {
      // First visit - serve landing page with Success title (triggers CNA popup)
      Serial.printf("[Captive] %s requested %s -> Success+Landing\n", clientIP.c_str(), endpoint.c_str());
      captivePortalFirstSeen[clientIP] = 0;  // Mark complete immediately
      webServer.send(200, "text/html", F(
        "<HTML><HEAD><TITLE>Mouse Whisker</TITLE></HEAD>"
        "<BODY style='font-family:-apple-system,sans-serif;text-align:center;padding:40px 20px;margin:0;background:#f8f9fa;'>"
        "<div style='max-width:320px;margin:0 auto;background:#fff;padding:30px;border-radius:16px;box-shadow:0 2px 10px rgba(0,0,0,0.1);'>"
        "<h2 style='font-size:24px;margin:0 0 16px;color:#333;'>&#128433; Mouse Whisker</h2>"
        "<p style='font-size:16px;color:#666;margin:0 0 24px;'>Connected! Tap below to configure.</p>"
        "<a style='display:block;padding:16px 24px;background:#007bff;color:#fff;text-decoration:none;border-radius:10px;font-size:18px;font-weight:500;' href='http://192.168.4.1/setup'>Open Setup</a>"
        "</div>"
        "</BODY></HTML>"));
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
    webServer.on("/diag", HTTP_POST, handleWebDiag);
    #endif
    webServer.on("/whisk", handleWebWhisk);
    webServer.on("/reboot", handleWebReboot);
    webServer.on("/reset", handleWebReset);
    webServer.on("/reset-confirm", HTTP_POST, handleWebResetConfirm);
    webServer.on("/firmware", HTTP_GET, handleFirmwarePage);
    webServer.on("/firmware", HTTP_POST, [](){
      webServer.sendHeader("Connection", "close");
      if (Update.hasError()) {
        webServer.send(500, "text/html", F(
          "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
          "<title>Update Failed</title></head>"
          "<body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
          "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;'>"
          "<h2 style='color:#dc3545;'>&#10007; Update Failed</h2>"
          "<p>Please try again.</p><p><a href='/firmware'>Back</a></p>"
          "</div></body></html>"));
      } else {
        webServer.send(200, "text/html", F(
          "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
          "<meta http-equiv='refresh' content='10;url=/setup'><title>Update Success</title></head>"
          "<body style='font-family:sans-serif;text-align:center;padding:40px 20px;background:#f8f9fa;'>"
          "<div style='max-width:400px;margin:0 auto;background:#fff;padding:30px;border-radius:12px;'>"
          "<h2 style='color:#28a745;'>&#10003; Update Successful!</h2>"
          "<p>Rebooting with new firmware...</p>"
          "<div style='margin-top:20px;'><span style='display:inline-block;width:40px;height:40px;border:4px solid #007bff;border-top-color:transparent;border-radius:50%;animation:spin 1s linear infinite;'></span></div>"
          "<style>@keyframes spin{to{transform:rotate(360deg);}}</style>"
          "</div></body></html>"));
        delay(500);
        ESP.restart();
      }
    }, handleFirmwareUpload);
    
    // Captive portal detection endpoints - always registered, handlers check wifiAPMode
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
    // Catch-all for any other requests (handles captive portal redirect)
    webServer.onNotFound(handleCaptivePortal);
    
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
    
    // Load diagnostics enabled setting (off by default to minimize flash wear)
    bool hasDiagEnabled = preferences.isKey("diagEnabled");
    diagnosticsEnabled = preferences.getBool("diagEnabled", false);
    Serial.printf("Diagnostics: %s [%s]\n", diagnosticsEnabled ? "Enabled" : "Disabled", hasDiagEnabled ? "from flash" : "DEFAULT");
    
    // Load boot counter (always read, but only increment/write if diagnostics enabled)
    bootCount = preferences.getULong("bootCount", 0);
    if (diagnosticsEnabled) {
      bootCount++;
      preferences.putULong("bootCount", bootCount);
    }
    Serial.printf("Boot Count: %lu (reset reason: %s)%s\n", 
                  bootCount, getResetReason().c_str(),
                  diagnosticsEnabled ? "" : " [not incremented - diagnostics off]");
    
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
    MQTT_TOPIC_DIAG = String("mousewhisker/") + String(uniqueId) + "/diagnostics";
    MQTT_TOPIC_SET_DIAG = String("mousewhisker/") + String(uniqueId) + "/set/diagnostics";
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
    // WiFi reconnect logic with disconnect tracking
    static unsigned long lastWifiAttempt = 0;
    static unsigned long wifiDisconnectTime = 0;  // When WiFi was first lost
    static bool wasWifiConnected = false;
    bool isWifiConnected = (WiFi.status() == WL_CONNECTED);
    
    // Track WiFi disconnect events
    if (wasWifiConnected && !isWifiConnected && !wifiAPMode) {
      wifiDisconnectCount++;
      if (wifiDisconnectTime == 0) {
        wifiDisconnectTime = millis();  // Mark when we first lost connection
      }
      Serial.printf("WiFi disconnected (total disconnects this session: %lu)\n", wifiDisconnectCount);
    }
    // Reset disconnect timer when reconnected
    if (isWifiConnected) {
      wifiDisconnectTime = 0;
    }
    wasWifiConnected = isWifiConnected;
    
    if (!wifiAPMode && !isWifiConnected && millis() - lastWifiAttempt > 60000) {
      // Not in AP mode but lost connection - try to reconnect every 60s
      lastWifiAttempt = millis();
      
      // After 10 minutes of failed reconnects, fall back to AP mode
      if (wifiDisconnectTime > 0 && millis() - wifiDisconnectTime > 600000) {
        Serial.println("WiFi reconnect failed for 10 minutes, enabling AP mode...");
        wifiDisconnectTime = 0;  // Reset for next time
        startAPMode();
      } else {
        Serial.println("WiFi disconnected, attempting reconnect...");
        WiFi.disconnect(true);  // Stop any in-progress connection attempt
        delay(100);
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str(), 0, NULL, true);  // Hidden SSID support
      }
    }
    // In AP mode with configured credentials - try WiFi every 3 minutes (only if no clients connected)
    if (wifiAPMode && wifiSSID.length() > 0 && millis() - lastWifiAttempt > 180000) {
      if (WiFi.softAPgetStationNum() == 0) {
        lastWifiAttempt = millis();
        tryWiFiFromAPMode();
      }
    }
    #endif

    #if ENABLE_MQTT
    static unsigned long lastMqttAttempt = 0;
    static unsigned long lastRssiPublish = 0;
    static unsigned long lastDiagPublish = 0;
    static bool wasMqttConnected = false;
    #if ENABLE_USB
    static bool lastUsbState = false;
    #endif
    unsigned long now = millis();
    
    // Track MQTT disconnect events
    bool isMqttConnected = mqttClient.connected();
    if (wasMqttConnected && !isMqttConnected) {
      mqttDisconnectCount++;
      Serial.printf("MQTT disconnected (total disconnects this session: %lu)\n", mqttDisconnectCount);
    }
    wasMqttConnected = isMqttConnected;
    
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
        // Publish diagnostics every 60 seconds (only when enabled to reduce MQTT traffic)
        if (diagnosticsEnabled && now - lastDiagPublish >= 60000) {
          lastDiagPublish = now;
          publishDiagnosticsJSON();
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
