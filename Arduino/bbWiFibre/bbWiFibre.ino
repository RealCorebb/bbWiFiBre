#include "bbWiFiBre.h"
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// BLE objects
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Mode definitions
#define MODE_MONITOR_AP 1
#define MODE_SCAN_CHANNEL 2
#define MODE_RSSI 3

// Global variables
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile int currentMode = MODE_MONITOR_AP;
TaskHandle_t modeTaskHandle = NULL;
TaskHandle_t channelTaskHandle = NULL;  // Added for mode2's second task
int currentBrightness = 200; // Default brightness

Preferences preferences;

// Forward declaration
void processSerialCommand(String command);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("BLE Client connected");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("BLE Client disconnected");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.println("BLE Command received: " + rxValue);
            processSerialCommand(rxValue);
        }
    }
};

void initBLE() {
    BLEDevice::init("bbWiFibre-BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
                         CHARACTERISTIC_UUID,
                         BLECharacteristic::PROPERTY_READ   |
                         BLECharacteristic::PROPERTY_WRITE  |
                         BLECharacteristic::PROPERTY_NOTIFY |
                         BLECharacteristic::PROPERTY_INDICATE
                       );

    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    
    Serial.println("BLE service started, waiting for connections...");
}

void sendBLEMessage(String message) {
    if (deviceConnected && pCharacteristic) {
        pCharacteristic->setValue(message.c_str());
        pCharacteristic->notify();
    }
}

void saveCurrentMode() {
    preferences.begin("appState", false);
    preferences.putInt("lastMode", currentMode);
    preferences.end();
    Serial.printf("Saved mode %d to NVS.\n", currentMode);
}

void saveBrightness() {
    preferences.begin("appState", false);
    preferences.putInt("brightness", currentBrightness);
    preferences.end();
    Serial.printf("Saved brightness %d to NVS.\n", currentBrightness);
}

void loadBrightness() {
    preferences.begin("appState", true); // Read-only
    currentBrightness = preferences.getInt("brightness", 200); // Default to 200 if not found
    preferences.end();
    Serial.printf("Loaded brightness %d from NVS.\n", currentBrightness);
    
    // Apply the loaded brightness
    strip.setBrightness(currentBrightness);
    strip.show();
}

void setBrightness(int brightness) {
    if (brightness >= 10 && brightness <= 255) {
        currentBrightness = brightness;
        strip.setBrightness(currentBrightness);
        strip.show();
        saveBrightness();
        Serial.printf("Brightness set to: %d\n", currentBrightness);
    } else {
        Serial.printf("Invalid brightness value: %d. Must be between 10-255.\n", brightness);
    }
}

void stopCurrentMode() {
    Serial.println("Stopping current mode...");
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(NULL);
    Serial.println("WiFi promiscuous mode stopped.");

    int modeToClean = currentMode; // Capture the mode that is being stopped

    if (modeTaskHandle != NULL) {
        Serial.printf("Deleting modeTaskHandle (was for mode %d)\n", modeToClean);
        vTaskDelete(modeTaskHandle);
        modeTaskHandle = NULL;
    }
    if (channelTaskHandle != NULL) { // Specific to MODE_SCAN_CHANNEL
        Serial.printf("Deleting channelTaskHandle (was for mode %d)\n", modeToClean);
        vTaskDelete(channelTaskHandle);
        channelTaskHandle = NULL;
    }

    Serial.println("Delaying for task cleanup (500ms)...");
    vTaskDelay(pdMS_TO_TICKS(250));

    strip.clear();
    strip.show();
    Serial.println("LEDs cleared.");

    if (modeToClean == MODE_MONITOR_AP && deviceDataMutex != NULL) {
        Serial.println("Deleting deviceDataMutex");
        vSemaphoreDelete(deviceDataMutex);
        deviceDataMutex = NULL;
    } else if (modeToClean == MODE_SCAN_CHANNEL && xSemaphore != NULL) {
        Serial.println("Deleting xSemaphore for channel scan");
        vSemaphoreDelete(xSemaphore);
        xSemaphore = NULL;
    } else if (modeToClean == MODE_RSSI && g_rssiDataMutex != NULL) {
        Serial.println("Deleting g_rssiDataMutex");
        vSemaphoreDelete(g_rssiDataMutex);
        g_rssiDataMutex = NULL;
    }
    Serial.println("Semaphores deleted (if any). Mode stopping sequence complete.");
}

void startMonitorAPMode() {
    Serial.println("Initializing Monitor AP Mode...");
    stopCurrentMode(); // Ensure any previous mode is fully stopped
    currentMode = MODE_MONITOR_AP;

    deviceDataMutex = xSemaphoreCreateMutex();
    if (deviceDataMutex == NULL) {
        Serial.println("Failed to create deviceDataMutex");
        return;
    }

    g_esp_boot_time_ms = millis();
    g_intruder_detection_armed = false;
    g_intruder_alert_active = false;
    for (int i = 0; i < MAX_CLIENT_DEVICES; i++) {
        trackedClientDevices[i].isActive = false;
        trackedClientDevices[i].activityScoreToAP = 0;
        trackedClientDevices[i].activityScoreFromAP = 0;
        memset(trackedClientDevices[i].mac, 0, 6);
    }

    esp_err_t err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) { Serial.printf("Failed to set promiscuous mode: %s\n", esp_err_to_name(err)); vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL; return; }
    err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) { Serial.printf("Failed to set channel: %s\n", esp_err_to_name(err)); vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL; return; }
    err = esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    if (err != ESP_OK) { Serial.printf("Failed to set callback: %s\n", esp_err_to_name(err)); vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL; return; }

    BaseType_t xReturned = xTaskCreatePinnedToCore(led_update_task, "LEDUpdateTask", 4096, NULL, 2, &modeTaskHandle, tskNO_AFFINITY);
    if (xReturned != pdPASS) {
        Serial.println("Failed to create LED update task for Monitor AP");
        vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL;
        return;
    }

    Serial.println("Monitor AP Mode Started Successfully");
    Serial.printf("Monitoring on Channel: %d\n", WIFI_CHANNEL);
    saveCurrentMode();
}

void startScanChannelMode() {
    Serial.println("Initializing Channel Scan Mode...");
    stopCurrentMode();
    currentMode = MODE_SCAN_CHANNEL;

    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
        Serial.println("Failed to create xSemaphore for Channel Scan");
        return;
    }

    currentScanningChannel = 1;
    for (int i = 0; i < CHANNELS; i++) {
        pendingActivityIncrements[i] = 0;
        channelActivity[i] = 0;
        channelBrightness[i] = 1.0f;
        pendingColorForDelayedLed[i] = OFF_COLOR;
        framesToDelayForLed[i] = 0;
    }

    esp_err_t err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) { Serial.printf("Failed to set promiscuous mode: %s\n", esp_err_to_name(err)); vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return; }
    err = esp_wifi_set_promiscuous_rx_cb(onWiFiPacket);
    if (err != ESP_OK) { Serial.printf("Failed to set callback: %s\n", esp_err_to_name(err)); vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return; }
    err = esp_wifi_set_channel(currentScanningChannel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) { Serial.printf("Failed to set channel: %s\n", esp_err_to_name(err)); vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return; }

    BaseType_t xReturnedLed = xTaskCreatePinnedToCore(LEDUpdateTask, "LEDTask", 4096, NULL, 2, &modeTaskHandle, tskNO_AFFINITY);
    if (xReturnedLed != pdPASS) {
        Serial.println("Failed to create LED update task for Channel Scan");
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        return;
    }

    BaseType_t xReturnedChannel = xTaskCreatePinnedToCore(ChannelTask, "ChannelTask", 4096, NULL, 1, &channelTaskHandle, tskNO_AFFINITY);
    if (xReturnedChannel != pdPASS) {
        Serial.println("Failed to create channel task for Channel Scan");
        if(modeTaskHandle != NULL) vTaskDelete(modeTaskHandle); modeTaskHandle = NULL;
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        return;
    }

    Serial.println("Channel Scan Mode Started Successfully");
    saveCurrentMode();
}

void startRSSIMode() {
    Serial.println("Initializing RSSI Mode...");
    stopCurrentMode();
    currentMode = MODE_RSSI;

    g_rssiDataMutex = xSemaphoreCreateMutex();
    if (g_rssiDataMutex == NULL) {
        Serial.println("Failed to create RSSI mutex");
        return;
    }

    g_latestRSSI = -100;
    g_routerIsVisible = false;
    g_smoothedRSSI = -100.0f; // Assuming float

    BaseType_t xReturned = xTaskCreatePinnedToCore(RSSIUpdateTask, "RSSITask", 4096, NULL, 2, &modeTaskHandle, tskNO_AFFINITY);
    if (xReturned != pdPASS) {
        Serial.println("Failed to create RSSI update task");
        vSemaphoreDelete(g_rssiDataMutex); g_rssiDataMutex = NULL;
        return;
    }

    Serial.println("RSSI Mode Started Successfully");
    saveCurrentMode();
}

void processSerialCommand(String command) {
    command.trim();

    if (command == "mode1") {
        if (currentMode != MODE_MONITOR_AP) {
            startMonitorAPMode();
        } else {
            Serial.println("Already in Monitor AP Mode");
            sendBLEMessage("Already in Monitor AP Mode");
        }
    } else if (command == "mode2") {
        if (currentMode != MODE_SCAN_CHANNEL) {
            startScanChannelMode();
        } else {
            Serial.println("Already in Channel Scan Mode");
            sendBLEMessage("Already in Channel Scan Mode");
        }
    } else if (command == "mode3") {
        if (currentMode != MODE_RSSI) {
            startRSSIMode();
        } else {
            Serial.println("Already in RSSI Mode");
            sendBLEMessage("Already in RSSI Mode");
        }
    } else if (command.startsWith("brightness:")) {
        int brightness = command.substring(11).toInt();
        setBrightness(brightness);
    } else if (command == "status") {
        // Additional command to check current settings
        String statusMsg = "Current Mode: " + String(currentMode) + ", Brightness: " + String(currentBrightness);
        Serial.println(statusMsg);
        sendBLEMessage(statusMsg);
    } else {
        String helpMsg = "Invalid command. Use 'mode1', 'mode2', 'mode3', 'brightness:XXX', or 'status'";
        Serial.println(helpMsg);
        sendBLEMessage(helpMsg);
    }
}

void serialCommandTask(void* parameter) {
    while (true) {
        // Check USB Serial
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            processSerialCommand(command);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void bleConnectionTask(void* parameter) {
    while (true) {
        // Handle BLE disconnection/reconnection
        if (!deviceConnected && oldDeviceConnected) {
            vTaskDelay(pdMS_TO_TICKS(500)); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            Serial.println("Start advertising");
            oldDeviceConnected = deviceConnected;
        }
        
        // Handle new BLE connection
        if (deviceConnected && !oldDeviceConnected) {
            oldDeviceConnected = deviceConnected;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("bbWiFiBre - Multi-mode WiFi Monitor with BLE");
    Serial.println("Commands:");
    Serial.println("  mode1 - Monitor AP Mode");
    Serial.println("  mode2 - Channel Scan Mode");
    Serial.println("  mode3 - RSSI Mode");
    Serial.println("  brightness:XXX - Set brightness (10-255)");
    Serial.println("  status - Show current settings");

    // Initialize BLE
    initBLE();

    strip.begin();
    
    // Load saved brightness before setting it
    loadBrightness();

    WiFi.mode(WIFI_STA); // Initialize WiFi stack
    WiFi.disconnect();   // Disconnect from any previous network

    preferences.begin("appState", true); // Read-only to check
    int savedMode = preferences.getInt("lastMode", MODE_MONITOR_AP);
    preferences.end();
    Serial.printf("Loaded mode %d from NVS.\n", savedMode);

    // Create a task for Serial command monitoring
    xTaskCreate(serialCommandTask, "SerialCommand", 4096, NULL, 1, NULL);
    
    // Create a task for BLE connection management
    xTaskCreate(bleConnectionTask, "BLEConnection", 4096, NULL, 1, NULL);

    delay(1000); // Allow serial task to initialize, etc.

    if (savedMode == MODE_MONITOR_AP) {
        startMonitorAPMode();
    } else if (savedMode == MODE_SCAN_CHANNEL) {
        startScanChannelMode();
    } else if (savedMode == MODE_RSSI) {
        startRSSIMode();
    } else {
        Serial.println("Invalid mode from NVS, defaulting to Monitor AP Mode.");
        startMonitorAPMode();
    }
}

void loop() {
    // Main loop is not used as we're using FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}