// --- Configuration ---
const float SMOOTHING_FACTOR = 0.35f;      // How quickly colors change (0.0 to 1.0). Higher is faster.
const uint8_t ROW_DELAY_FRAMES = 6;        // Delay for the second row in update frames (5 frames * 20ms/frame = 100ms delay)

// Shared resources protection
SemaphoreHandle_t xSemaphore = NULL; // Initialize to NULL

// Channel activity storage
volatile uint16_t pendingActivityIncrements[CHANNELS] = {0}; // Incremented by WiFi callback
uint16_t channelActivity[CHANNELS] = {0};                    // Processed activity
float channelBrightness[CHANNELS] = {1.0};                   // Brightness modifier per channel
uint8_t currentScanningChannel = 1;                          // Current channel being scanned

// Adjusted decay factors
const float ACTIVITY_DECAY = 0.990;    // Decay per LED update interval
const float BRIGHTNESS_DECAY = 0.985;  // Decay per LED update interval
const float BRIGHTNESS_RECOVERY = 1.0f - BRIGHTNESS_DECAY;

// Color definitions
const uint32_t OFF_COLOR = strip.Color(0, 0, 0);
const uint32_t LOW_COLOR = strip.Color(0, 0, 20);    // Dim blue for low activity
const uint32_t MID_COLOR = strip.Color(0, 255, 0);   // Green for medium activity
const uint32_t HIGH_COLOR = strip.Color(255, 0, 0);  // Red for high activity
const uint8_t LOW_ACTIVITY_THRESHOLD = 5;      // Threshold for dimming an inactive channel

// LED state variables for smoothing and delay
uint32_t currentLedStripColors[NUM_LEDS]; // Actual colors sent to strip.show()
uint32_t targetLedStripColors[NUM_LEDS];  // Target colors for smoothing

uint32_t pendingColorForDelayedLed[CHANNELS]; // Stores the color that the delayed LED for channel i will eventually target
uint8_t framesToDelayForLed[CHANNELS];      // Countdown for the delay for each channel's delayed LED

// WiFi Packet Handler - Called for each received packet in promiscuous mode
void onWiFiPacket(void* buf, wifi_promiscuous_pkt_type_t type) {
  // This callback must be very fast. Avoid blocking operations or lengthy computations.
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) { // Focus on relevant packet types
    return;
  }

  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  int receivedChannel = pkt->rx_ctrl.channel;

  if (receivedChannel >= 1 && receivedChannel <= CHANNELS) {
    // Use atomic increment if available/needed, for now, direct increment.
    // This is generally safe for counters if occasional miscounts under extreme load are acceptable.
    // For higher precision, consider ESP-IDF atomic operations or a FreeRTOS queue from ISR.
    pendingActivityIncrements[receivedChannel - 1] += 5; // Add some activity score
    // Cap pending activity to prevent overflow if LED task is slow, though unlikely with current setup
    if (pendingActivityIncrements[receivedChannel - 1] > 1000) {
        pendingActivityIncrements[receivedChannel - 1] = 1000;
    }
  }
}

uint32_t interpolateColor(uint32_t color1, uint32_t color2, float ratio) {
  ratio = fmax(0.0f, fmin(1.0f, ratio)); // Clamp ratio to [0,1]

  uint8_t r1 = (color1 >> 16) & 0xFF;
  uint8_t g1 = (color1 >> 8) & 0xFF;
  uint8_t b1 = color1 & 0xFF;

  uint8_t r2 = (color2 >> 16) & 0xFF;
  uint8_t g2 = (color2 >> 8) & 0xFF;
  uint8_t b2 = color2 & 0xFF;

  uint8_t r = r1 + (r2 - r1) * ratio;
  uint8_t g = g1 + (g2 - g1) * ratio;
  uint8_t b = b1 + (b2 - b1) * ratio;

  return strip.Color(r, g, b);
}

// Task to update LED colors and effects
void LEDUpdateTask(void *pvParameters) {
  const TickType_t xFrequency = LED_UPDATE_INTERVAL_MS / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    uint16_t localChannelActivity[CHANNELS];
    float localChannelBrightness[CHANNELS];

    // Section 1: Update shared data (activity, brightness) under semaphore
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < CHANNELS; i++) {
        // Transfer pending activity
        if (pendingActivityIncrements[i] > 0) {
          uint16_t activity_to_add = pendingActivityIncrements[i];
          pendingActivityIncrements[i] = 0; // Clear pending
          channelActivity[i] += activity_to_add;
          if (channelActivity[i] > 2000) channelActivity[i] = 2000; // Cap activity
        }

        // Decay activity
        channelActivity[i] *= ACTIVITY_DECAY;
        if (channelActivity[i] < 1.0f) { // Use a small threshold to prevent float underflow issues
            channelActivity[i] = 0;
        }

        // Decay/Recover brightness
        channelBrightness[i] = channelBrightness[i] * BRIGHTNESS_DECAY + BRIGHTNESS_RECOVERY;
        if (channelBrightness[i] > 1.0f) channelBrightness[i] = 1.0f;
        if (channelBrightness[i] < 0.05f) channelBrightness[i] = 0.05f; // Minimum brightness to avoid full off from this factor

        localChannelActivity[i] = channelActivity[i];
        localChannelBrightness[i] = channelBrightness[i];
      }
      xSemaphoreGive(xSemaphore);
    }

    // Section 2: Calculate ideal colors and set targets for LEDs
    for (int i = 0; i < CHANNELS; i++) {
      float intensity = fmin(localChannelActivity[i] / 70.0f, 1.0f); 
      uint32_t calculatedChannelColor;

      if (localChannelActivity[i] == 0) {
          calculatedChannelColor = OFF_COLOR;
      } else if (intensity <= 0.5f) {
        calculatedChannelColor = interpolateColor(LOW_COLOR, MID_COLOR, intensity * 2.0f);
      } else {
        calculatedChannelColor = interpolateColor(MID_COLOR, HIGH_COLOR, (intensity - 0.5f) * 2.0f);
      }

      uint8_t r_ideal = (calculatedChannelColor >> 16) & 0xFF;
      uint8_t g_ideal = (calculatedChannelColor >> 8) & 0xFF;
      uint8_t b_ideal = calculatedChannelColor & 0xFF;

      r_ideal *= localChannelBrightness[i];
      g_ideal *= localChannelBrightness[i];
      b_ideal *= localChannelBrightness[i];
      uint32_t idealColorForChannel = strip.Color(r_ideal, g_ideal, b_ideal);
      // --- End of idealColorForChannel calculation ---

      // --- Set targets based on your row logic ---
      // Primary LED (e.g., LED 13 for channel 0) targets new color immediately
      int primaryLedIndex = i + CHANNELS;
      targetLedStripColors[primaryLedIndex] = idealColorForChannel;

      // Delayed LED (e.g., LED 0 for channel i) - REVISED LOGIC HERE
      int delayedLedIndex = i;

      // If the current delay cycle for this delayed LED has finished (counter is 0)
      if (framesToDelayForLed[i] == 0) {
        // We are ready to capture the current idealColorForChannel for the *next* delay sequence.
        // This captured color will be applied once the new delay countdown finishes.
        
        pendingColorForDelayedLed[i] = idealColorForChannel; // Latch the current ideal color

        if (ROW_DELAY_FRAMES > 0) {
          // If there's an actual delay configured, start a new countdown.
          framesToDelayForLed[i] = ROW_DELAY_FRAMES;
        } else { // ROW_DELAY_FRAMES == 0
          // If no delay, apply the color to the target immediately.
          // framesToDelayForLed[i] will remain 0.
          targetLedStripColors[delayedLedIndex] = idealColorForChannel;
        }
      }
    }

    // Section 3: Process delay counters and update targets for delayed LEDs
    if (ROW_DELAY_FRAMES > 0) { // Only process if delay is configured
        for (int i = 0; i < CHANNELS; i++) {
            int delayedLedIndex = i;
            if (framesToDelayForLed[i] > 0) {
                framesToDelayForLed[i]--;
                if (framesToDelayForLed[i] == 0) {
                    targetLedStripColors[delayedLedIndex] = pendingColorForDelayedLed[i];
                }
            }
        }
    }


    // Section 4: Apply smoothing to all LEDs and show
    for (int k = 0; k < NUM_LEDS; k++) {
      // Smooth current color towards target color
      currentLedStripColors[k] = interpolateColor(currentLedStripColors[k], targetLedStripColors[k], SMOOTHING_FACTOR);
      strip.setPixelColor(k, currentLedStripColors[k]);
    }
    strip.show();
  }
}

// Task to switch WiFi channels
void ChannelTask(void *pvParameters) {
  const TickType_t xFrequency = 250 / portTICK_PERIOD_MS; // Switch channels less frequently (e.g., 4Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      // If current channel has low activity, dim it further (this effect will be smoothed)
      if (channelActivity[currentScanningChannel - 1] < LOW_ACTIVITY_THRESHOLD && channelActivity[currentScanningChannel - 1] > 0) {
        channelBrightness[currentScanningChannel - 1] *= 0.5; // Dim it more rapidly
         if(channelBrightness[currentScanningChannel - 1] < 0.1f) channelBrightness[currentScanningChannel - 1] = 0.1f;
      } else if (channelActivity[currentScanningChannel - 1] == 0) {
         channelBrightness[currentScanningChannel - 1] = 0.3f; // Reset brightness for completely dead channels
      }


      // Switch to the next channel
      currentScanningChannel = (currentScanningChannel % CHANNELS) + 1;
      esp_wifi_set_channel(currentScanningChannel, WIFI_SECOND_CHAN_NONE);
      
      // Briefly boost brightness of the new channel to make scan visible
      // channelBrightness[currentScanningChannel - 1] = fmin(1.0f, channelBrightness[currentScanningChannel - 1] + 0.3f);


      xSemaphoreGive(xSemaphore);
    }
     //Serial.print("Scanning Channel: "); Serial.println(currentScanningChannel);
  }
}

