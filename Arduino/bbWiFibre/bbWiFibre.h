#ifndef BB_WIFI_BRE_H
#define BB_WIFI_BRE_H

#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// --- Common definitions ---
#define LED_PIN 10
#define NUM_LEDS 26
const TickType_t LED_UPDATE_INTERVAL_MS = 20;

// --- Network Configuration ---
#define WIFI_CHANNEL 13
const uint8_t ROUTER_MAC[6] = {0xDC, 0xD8, 0x7C, 0x5C, 0x32, 0x7D};

// --- Mode 1 (Monitor AP) specific declarations ---
#define MAX_CLIENT_DEVICES 13
extern SemaphoreHandle_t deviceDataMutex;
extern unsigned long g_esp_boot_time_ms;
extern volatile bool g_intruder_detection_armed;
extern volatile bool g_intruder_alert_active;

struct DeviceInfo {
    uint8_t mac[6];
    volatile float activityScoreToAP;
    volatile float activityScoreFromAP;
    unsigned long lastSeenTime;
    bool isActive;
};

extern DeviceInfo trackedClientDevices[];
void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type);
void led_update_task(void *pvParameters);

// --- Mode 2 (Channel Scan) specific declarations ---
#define CHANNELS 13
extern SemaphoreHandle_t xSemaphore;
extern volatile uint16_t pendingActivityIncrements[];
extern uint16_t channelActivity[];
extern float channelBrightness[];
extern uint8_t currentScanningChannel;
extern uint32_t pendingColorForDelayedLed[];
extern uint8_t framesToDelayForLed[];
extern const uint32_t OFF_COLOR;

void onWiFiPacket(void* buf, wifi_promiscuous_pkt_type_t type);
void LEDUpdateTask(void *pvParameters);
void ChannelTask(void *pvParameters);

#endif // BB_WIFI_BRE_H