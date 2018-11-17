#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>
#include <sys/param.h>

#include "stdio.h"
#include <stdint.h> // Boolean
#include <stdlib.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/* defines */

#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

#if defined CONFIG_OPPONENT_IS_IPV4
#define HOST_IP_ADDR CONFIG_OPPONENT_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_OPPONENT_IPV6_ADDR
#endif

#define OPPONENT_UDP_PORT CONFIG_OPPONENT_UDP_PORT

#define RTP_HEADER_BYTES 12
#define BYTES_PER_SAMPLE 3
#define SAMPLES_PER_PACKET 48

static const unsigned int PACKET_DATA_LENGTH = RTP_HEADER_BYTES + BYTES_PER_SAMPLE * SAMPLES_PER_PACKET;

static const char *TAG = "MyModule";
static const int SSRC_ID = 123456;
static const int SAMPLE_DATA[24] = {
    /* 12しか使わんくない？？ */
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

/* inlined function for logging udp success/failure */
// なんか `sendto()` を使用するように変更してから帰ってくるエラー変わった気がする
// こっちも変えなくては
inline static void ESP_LOG_UDP_SEND_RESULT(esp_err_t result, unsigned int count)
{
  if (result < 0)
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
    ESP_LOGI(TAG, "SUCCESS with code %d", result);
  };
}

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
                                      .password = WIFI_PASS,
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

/* 破壊的に新しいヘッダを作る */
void renew_aes67_header(RTP_header *header)
{
  header->sequence_number++;
  header->timestamp = microsFromStart();
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

static esp_err_t send_udp(int sock, const void *dataptr, size_t datalength, struct sockaddr *destAddr, int destAddrSize)
{
  int err = sendto(sock, dataptr, datalength, 0, destAddr, destAddrSize);
  if (err < 0)
  {
    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    return err;
  }
  ESP_LOGI(TAG, "Message sent");
  return err;
}

void app_main(void)
{
  esp32setup();
  esp32connectToWiFi(); // Taskにすると良さそう

  /* Disable logging from now on */
  esp_log_level_set(TAG, ESP_LOG_NONE);

  // wait
  vTaskDelay(2000);

  unsigned char *PACKET_DATA = calloc(PACKET_DATA_LENGTH, sizeof(unsigned char));

  // UDP inits
  esp_err_t send_res;
  char addr_str[128];
  int addr_family;
  int ip_protocol;

  RTP_header rtp_header = create_aes67_header(L24);
  L24Data data; // for encoding 24bit audio data to chars

  /* Initialize first packet */
  PACKET_DATA[0] = rtp_header.a;
  PACKET_DATA[1] = rtp_header.b;
  PACKET_DATA[2] = rtp_header.c;
  PACKET_DATA[3] = rtp_header.d;
  PACKET_DATA[4] = rtp_header.e;
  PACKET_DATA[5] = rtp_header.f;
  PACKET_DATA[6] = rtp_header.g;
  PACKET_DATA[7] = rtp_header.h;
  PACKET_DATA[8] = rtp_header.i;
  PACKET_DATA[9] = rtp_header.j;
  PACKET_DATA[10] = rtp_header.k;
  PACKET_DATA[11] = rtp_header.l;
  //
  for (int i = 0; i < SAMPLES_PER_PACKET; i++)
  {
    data.whole = SAMPLE_DATA[i % 24];
    const unsigned int offset = 12; // RTP header

    PACKET_DATA[offset + i * 3 + 0] = data.first;
    PACKET_DATA[offset + i * 3 + 1] = data.second;
    PACKET_DATA[offset + i * 3 + 2] = data.third;
  }

#ifdef CONFIG_OPPONENT_IS_IPV4
  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(OPPONENT_UDP_PORT);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
  struct sockaddr_in6 destAddr;
  inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
  destAddr.sin6_family = AF_INET6;
  destAddr.sin6_port = htons(OPPONENT_UDP_PORT);
  addr_family = AF_INET6;
  ip_protocol = IPPROTO_IPV6;
  inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

  int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
  if (sock < 0)
  {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
  }
  ESP_LOGI(TAG, "Socket created");

  for (;;)
  {
    sendto(
        sock,
        PACKET_DATA,
        PACKET_DATA_LENGTH,
        0,
        (struct sockaddr *)&destAddr,
        sizeof(destAddr));
    // ESP_LOG_UDP_SEND_RESULT(send_res, count);

    renew_aes67_header(&rtp_header);
    // rtp_header = create_next_aes67_header(&rtp_header); // メモリあろケート的に不利？

    /* rewrite PACKET_DATA */
    // PACKET_DATA[0] = rtp_header.a;
    // PACKET_DATA[1] = rtp_header.b;
    PACKET_DATA[2] = rtp_header.c; // change only the sequence numbers
    PACKET_DATA[3] = rtp_header.d;
    PACKET_DATA[4] = rtp_header.e; // and the timestamps
    PACKET_DATA[5] = rtp_header.f;
    PACKET_DATA[6] = rtp_header.g;
    PACKET_DATA[7] = rtp_header.h;
    // PACKET_DATA[8] = rtp_header.i;
    // PACKET_DATA[9] = rtp_header.j;
    // PACKET_DATA[10] = rtp_header.k;
    // PACKET_DATA[11] = rtp_header.l;
    //
    for (unsigned char i = 0; i < SAMPLES_PER_PACKET; i++)
    {
      data.whole = SAMPLE_DATA[i % 24];
      const unsigned int offset = 12; // RTP header

      PACKET_DATA[offset + i * 3 + 0] = data.first;
      PACKET_DATA[offset + i * 3 + 1] = data.second;
      PACKET_DATA[offset + i * 3 + 2] = data.third;
    }
  }
}
