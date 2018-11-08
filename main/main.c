#ifdef ESP_AP_CONNECT
#define WIFI_SSID "ESP32_wifi"
#define WIFI_PASSWORD "esp32pass"
#define OPPONENT_UDP_PORT 5004
#define SET_IPADDR4(ipAddr) IP_ADDR4((ipAddr), 192, 168, 4, 255);
#else
#define WIFI_SSID "WX03_Todoroki"
#define WIFI_PASSWORD "TodorokiWX03"
#define OPPONENT_UDP_PORT 4444
#define SET_IPADDR4(ipAddr) IP_ADDR4((ipAddr), 192, 168, 179, 5);
#endif

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

#include "stdio.h"
#include <stdint.h> // Boolean
#include <stdlib.h>
#include <string.h>

#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/udp.h"

#define UDP_PAYLOAD_SIZE 50

static const char *TAG = "MyModule";
static const int SSRC_ID = 123456;
static const int SAMPLE_DATA[24] = {
    -5601769,
    -7388378,
    6516204,
    6315093,
    3295605,
    -195466,
    6121253,
    2347758,
    -749841,
    -5233408,
    8168938,
    -2977557,
    6219119,
    72656,
    -2332890,
    4057447,
    8209684,
    -1015844,
    -2515686,
    -2618522,
    -4799178,
    464494,
    -2445642,
    3821064,
};

esp_err_t
event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}

void esp32setup()
{
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
}

void esp32connectToWiFi()
{
  esp_err_t wifiConnected = ESP_ERR_WIFI_NOT_INIT;

  wifi_config_t sta_config = {.sta = {.ssid = WIFI_SSID,
                                      .password = WIFI_PASSWORD,
                                      .bssid_set = false}};
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_connect());

  ESP_LOGI(TAG, "Connecting to wifi: %s", WIFI_SSID);

  // wait until wifi connects
  while (wifiConnected != ESP_OK)
  {
    ESP_LOGI(TAG, "wifi connect is %d", wifiConnected);
    wifiConnected = esp_wifi_connect();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  };

  ESP_LOGI(TAG, "wifi connect is %d", wifiConnected);
  ESP_LOGI(TAG, "-> Connected to: %s", WIFI_SSID);

  vTaskDelay(3000 / portTICK_PERIOD_MS); // 終わるまで待つ
}

unsigned int microsFromStart() { return (unsigned int)esp_timer_get_time(); }

typedef struct _RTP_header
{
  union {
    struct
    {
      unsigned char l;
      unsigned char k;
      unsigned char j;
      unsigned char i;
      unsigned char h;
      unsigned char g;
      unsigned char f;
      unsigned char e;
      unsigned char d;
      unsigned char c;
      unsigned char b;
      unsigned char a;
    };
    struct
    {
      int ssrc_id : 32;
      unsigned int timestamp : 32;
      unsigned short sequence_number : 16;
      unsigned char payload_type : 7;
      unsigned char marker : 1;
      unsigned char csrc_count : 4;
      unsigned char extention : 1;
      unsigned char padding : 1;
      unsigned char version : 2;
    } __attribute__((packed));
  };
} __attribute__((packed)) RTP_header;

typedef enum
{
  L16,
  L24,
} AES67PF;

RTP_header
create_aes67_header(AES67PF pf)
{
  unsigned char payload_type =
      pf == L16
          ? 11
          : pf == L24
                ? 100
                : 99;

  RTP_header aes67h = {
      .version = 2,
      .padding = 0,
      .extention = 0,
      .csrc_count = 0,
      .marker = 0,
      .payload_type = payload_type,
      .sequence_number = rand(),
      .timestamp = microsFromStart(),
      .ssrc_id = SSRC_ID,
  };

  return aes67h;
}

RTP_header create_next_aes67_header(RTP_header *prevHeader)
{
  RTP_header aes67h = {
      .version = prevHeader->version,
      .padding = prevHeader->padding,
      .extention = prevHeader->extention,
      .csrc_count = prevHeader->csrc_count,
      .marker = prevHeader->marker,
      .payload_type = prevHeader->payload_type,
      .sequence_number = prevHeader->sequence_number + 1,
      .timestamp = microsFromStart(),
      .ssrc_id = prevHeader->ssrc_id,
  };

  return aes67h;
};

struct pbuf *alloc_AES_header()
{
  return pbuf_alloc(
      PBUF_TRANSPORT,
      12, // RTP Header struct
      PBUF_RAM);
};

struct pbuf *alloc_AES_payload(unsigned int bytes)
{
  return pbuf_alloc(
      PBUF_RAW,
      bytes, // how many unsigned chars you need, e.g. 3 for 24 bits.
      PBUF_RAM);
};

typedef struct _l24_data
{
  union {
    struct
    {
      unsigned char third;
      unsigned char second;
      unsigned char first;
    };
    unsigned int whole : 24;
  };
} __attribute__((packed)) L24Data;

typedef struct _l16_data
{
  union {
    struct
    {
      unsigned char second;
      unsigned char first;
    };
    unsigned short whole : 16;
  };
} __attribute__((packed)) L16Data;

void app_main(void)
{
  esp32setup();
  esp32connectToWiFi();

  // wait
  vTaskDelay(2000);

  unsigned int count = 0;

  // UDP inits
  struct udp_pcb *udp;
  err_t err, result;
  ip_addr_t ipAddr;
  SET_IPADDR4(&ipAddr);
  udp = udp_new();
  err = udp_connect(udp, &ipAddr, OPPONENT_UDP_PORT);

  L24Data data; // for encoding 24bit audio data to chars

  for (int z = 0; z < 50; z++)
  {
    // Alloc pbufs
    struct pbuf *aes67_header = alloc_AES_header();
    struct pbuf *aes67_payloads[12] = {
        // 12 samples
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
        alloc_AES_payload(3),
    };

    // place AES67 data in pbuf
    RTP_header rtp_header = create_aes67_header(L24);
    pbuf_put_at(aes67_header, 0, rtp_header.a);
    pbuf_put_at(aes67_header, 1, rtp_header.b);
    pbuf_put_at(aes67_header, 2, rtp_header.c);
    pbuf_put_at(aes67_header, 3, rtp_header.d);
    pbuf_put_at(aes67_header, 4, rtp_header.e);
    pbuf_put_at(aes67_header, 5, rtp_header.f);
    pbuf_put_at(aes67_header, 6, rtp_header.g);
    pbuf_put_at(aes67_header, 7, rtp_header.h);
    pbuf_put_at(aes67_header, 8, rtp_header.i);
    pbuf_put_at(aes67_header, 9, rtp_header.j);
    pbuf_put_at(aes67_header, 10, rtp_header.k);
    pbuf_put_at(aes67_header, 11, rtp_header.l);

    for (unsigned char i = 0; i < 12; i++)
    {
      data.whole = SAMPLE_DATA[i];

      pbuf_put_at(aes67_payloads[i], 0, data.first);
      pbuf_put_at(aes67_payloads[i], 1, data.second);
      pbuf_put_at(aes67_payloads[i], 2, data.third);
    }

    // Chain pbufs from the tail(!)
    for (short i = 10; i > -1; i--)
    {
      pbuf_cat(aes67_payloads[i], aes67_payloads[i + 1]);
    }
    pbuf_cat(aes67_header, aes67_payloads[0]);

    // Send data
    result = udp_send(udp, aes67_header);
    if (result != 0)
    {
      ESP_LOGI(TAG, "%d - error code is %d", count, result);

      switch (result)
      {
      case -1:
        ESP_LOGI(TAG, "OUT OF MEMORY");
        break;
      case -2:
        ESP_LOGI(TAG, "BUFFER ERROR");
        break;
      case -3:
        ESP_LOGI(TAG, "TIMEOUT");
        break;
      case -4:
        ESP_LOGI(TAG, "ROUTING PROBLEM");
        break;
      case -5:
        ESP_LOGI(TAG, "IN PROGRESS");
        break;
      case -6:
        ESP_LOGI(TAG, "ILLEGAL VALUE");
        break;
      case -7:
        ESP_LOGI(TAG, "ROUTING PROBLEM");
        break;

      default:
        break;
      }
    }
    else
    {
      ESP_LOGI(TAG, "SUCCESS");
    };
  }

  // // Clean-ups
  // pbuf_free(aes67_header);
  // for (unsigned char i = 0; i < 12; i++)
  // {
  //   pbuf_free(aes67_payloads[i]);
  // }

  while (true)
  {
    vTaskDelay(1000);
  };
}
