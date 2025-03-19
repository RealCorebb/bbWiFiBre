#include <WiFi.h>
#include <esp_wifi.h> 
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#define LED_PIN 10
#define NUM_LEDS 26
#define CHANNELS 13

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Shared resources protection
SemaphoreHandle_t xSemaphore = xSemaphoreCreateMutex();

// Channel activity storage
uint16_t channelActivity[CHANNELS] = {0};
float channelBrightness[CHANNELS] = {1.0};
uint8_t currentChannel = 1;

// Adjusted decay factors for 20ms intervals
const float ACTIVITY_DECAY = 0.995;  // ≈0.977 per 20ms
const float BRIGHTNESS_DECAY = 0.980; // ≈0.887 per 20ms
const float BRIGHTNESS_RECOVERY = 1.0 - BRIGHTNESS_DECAY; // ≈0.113

// Color definitions
const uint32_t LOW_COLOR = strip.Color(0, 0, 10);
const uint32_t MID_COLOR = strip.Color(0, 255, 0);
const uint32_t HIGH_COLOR = strip.Color(255, 0, 0);
const uint8_t LOW_ACTIVITY_THRESHOLD = 5;

void onWiFiPacket(void* buf, wifi_promiscuous_pkt_type_t type) {
  if(xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
    channelActivity[currentChannel - 1] += 5;
    xSemaphoreGive(xSemaphore);
  }
}

void LEDUpdateTask(void *pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS; // 50Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if(xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      // Apply continuous decay
      for(int i = 0; i < CHANNELS; i++) {
        channelActivity[i] *= ACTIVITY_DECAY;
        channelBrightness[i] = channelBrightness[i] * BRIGHTNESS_DECAY + BRIGHTNESS_RECOVERY;
      }

      // Update LEDs
      for(int i = 0; i < CHANNELS; i++) {
        float intensity = fmin(channelActivity[i] / 50.0f, 1.0f);
        uint32_t color;

        if(intensity <= 0.5f) {
          color = interpolateColor(LOW_COLOR, MID_COLOR, intensity * 2.0f);
        } else {
          color = interpolateColor(MID_COLOR, HIGH_COLOR, (intensity - 0.5f) * 2.0f);
        }

        // Apply brightness
        uint8_t r = (color >> 16) & 0xFF;
        uint8_t g = (color >> 8) & 0xFF;
        uint8_t b = color & 0xFF;
        r *= channelBrightness[i];
        g *= channelBrightness[i];
        b *= channelBrightness[i];
        
        strip.setPixelColor(i, strip.Color(r, g, b));
        strip.setPixelColor(i + CHANNELS, strip.Color(r, g, b));
      }
      strip.show();
      xSemaphoreGive(xSemaphore);
    }
  }
}

void ChannelTask(void *pvParameters) {
  const TickType_t xFrequency = 200 / portTICK_PERIOD_MS; // 5Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if(xSemaphoreTake(xSemaphore, portMAX_DELAY)) {
      // Darken current channel if needed
      if(channelActivity[currentChannel - 1] < LOW_ACTIVITY_THRESHOLD) {
        channelBrightness[currentChannel - 1] = 0.3;
      }

      // Switch channel
      currentChannel = (currentChannel % CHANNELS) + 1;
      esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
      
      xSemaphoreGive(xSemaphore);
    }
  }
}

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();

  WiFi.mode(WIFI_STA);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&onWiFiPacket);

  xTaskCreatePinnedToCore(
    LEDUpdateTask,
    "LEDTask",
    4096,
    NULL,
    2,  // Higher priority
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    ChannelTask,
    "ChannelTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  vTaskDelete(NULL);
}

void loop() {}

uint32_t interpolateColor(uint32_t color1, uint32_t color2, float ratio) {
  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;
  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;

  return strip.Color(
    r1 + (r2 - r1) * ratio,
    g1 + (g2 - g1) * ratio,
    b1 + (b2 - b1) * ratio
  );
}