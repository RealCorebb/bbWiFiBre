#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "WiFi.h"

#define WIFI_CHANNEL_MAX        (13)
#define WIFI_CHANNEL_SWITCH_INTERVAL  (500)
#define GPIO_OUTPUT_IO_17    GPIO_NUM_17

static wifi_country_t wifi_country = {.cc="CN", .schan=1, .nchan=13, .policy=WIFI_COUNTRY_POLICY_AUTO};

typedef struct {
  unsigned frame_ctrl:16;
  unsigned duration_id:16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl:16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

const char * wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
    switch(type) {
    case WIFI_PKT_MGMT: return "MGMT";
    case WIFI_PKT_DATA: return "DATA";
    case WIFI_PKT_CTRL: return "CTRL";
    case WIFI_PKT_MISC: return "MISC";
    default:    
        return "UNKNOWN";
    }
}

void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA && type != WIFI_PKT_CTRL)
        return;

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    // Fixed-width format with labels
    Serial.printf("TYPE: %4s|CH: %02d|RSSI: %+04d|SOURCE_MAC: %02x:%02x:%02x:%02x:%02x:%02x|DEST_MAC: %02x:%02x:%02x:%02x:%02x:%02x|LEN: %03d|SEQ: %04d\n",
           wifi_sniffer_packet_type2str(type),
           ppkt->rx_ctrl.channel,
           ppkt->rx_ctrl.rssi,
           hdr->addr2[0],hdr->addr2[1],hdr->addr2[2],hdr->addr2[3],hdr->addr2[4],hdr->addr2[5], // Source
           hdr->addr1[0],hdr->addr1[1],hdr->addr1[2],hdr->addr1[3],hdr->addr1[4],hdr->addr1[5], // Destination
           ppkt->rx_ctrl.sig_len,
           (hdr->sequence_ctrl & 0xFFF0) >> 4
    );
}

void wifi_sniffer_init(void)
{
    // Initialize WiFi in NULL mode for promiscuous mode
    WiFi.mode(WIFI_OFF);
    delay(100);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_country(&wifi_country); /* set country for channel range [1, 13] */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel)
{
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

void setup()
{
    // Initialize Serial communication
    Serial.begin(115200);
    delay(1000);
    
    // Configure GPIO17 as output and set it high
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_OUTPUT_IO_17);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_IO_17, 1); // Set GPIO17 high

    Serial.println("ESP32 WiFi Sniffer Started");
    Serial.println("GPIO17 set to HIGH");
    Serial.println("Monitoring WiFi packets on all channels...");
    Serial.println();

    // Initialize WiFi sniffer
    wifi_sniffer_init();
}

void loop()
{
    static uint8_t channel = 1;
    static unsigned long lastChannelSwitch = 0;
    
    if (millis() - lastChannelSwitch >= WIFI_CHANNEL_SWITCH_INTERVAL) {
        Serial.printf("Switching to channel %d\n", channel);
        wifi_sniffer_set_channel(channel);
        lastChannelSwitch = millis();
        
        if (++channel > WIFI_CHANNEL_MAX) {
            channel = 1;
        }
    }
    
    delay(10); // Small delay to prevent watchdog issues
}