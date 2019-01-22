/** 
 * This file is for checking the maximum speed of packet sending.
 * This is forked from square-dualcore.c, but will not read from I2S DMA, 
 *   and will keep on sending packets.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2s.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
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
#define BITS_PER_SAMPLE (BYTES_PER_SAMPLE * 8)
#define SAMPLES_PER_PACKET 48

#define SAMPLE_RATE (48000)
#define I2S_NUM (0)

#define READ_CORE 1
#define SEND_CORE 0

static const char *TAG = "MyModule";

static const unsigned int PACKET_DATA_LENGTH = RTP_HEADER_BYTES + BYTES_PER_SAMPLE * SAMPLES_PER_PACKET;
static const int SSRC_ID = 123456;

// Ringbuffer
static const int RINGBUFFER_SIZE = 2 << 8;
static int RB_WROTE_AT = 0;
static int RB_READ_AT = 0;

/**
 * Event handler callback function for lwIP.
 */
esp_err_t
event_handler(void *ctx, system_event_t *event)
{
  return ESP_OK;
}

/**
 * Setup functions
 */

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
  esp_err_t wifiConnected = 1; // ESP_ERR_WIFI_NOT_INIT;

  wifi_config_t sta_config = {.sta = {.ssid = WIFI_SSID,
                                      .password = WIFI_PASS,
                                      .bssid_set = false}};
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_connect());

  // wait until wifi connects
  do
  {
    wifiConnected = esp_wifi_connect();
    ESP_LOGI(TAG, "wifi connect is %d", wifiConnected);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } while (wifiConnected != ESP_OK);

  ESP_LOGI(TAG, "-> Connected to: %s", WIFI_SSID);

  // vTaskDelay(3000 / portTICK_PERIOD_MS); // 終わるまで待つ
}

void esp32_i2s_setup()
{
  i2s_config_t i2s_config = {
      .mode = I2S_MODE_MASTER | I2S_MODE_RX, // Only RX
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = 24,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // 1 channel, mono
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .dma_buf_count = 30,
      .dma_buf_len = 48,
      .use_apll = true,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1 //Interrupt level 1
  };
  i2s_pin_config_t pin_config = {
      .bck_io_num = GPIO_NUM_18,
      .ws_io_num = GPIO_NUM_22,
      .data_in_num = GPIO_NUM_23,
      .data_out_num = I2S_PIN_NO_CHANGE // not used
  };

  esp_err_t driver_res = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  esp_err_t pin_res = i2s_set_pin(I2S_NUM, &pin_config);

  ESP_LOGI(TAG, "i2s DRIVER: %d, PIN: %d", driver_res, pin_res);
}

unsigned int microsFromStart() { return (unsigned int)esp_timer_get_time(); }

/**
 * AES67, RTP関連 
 */

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

/**
 * Tasks
 */

typedef struct _writeToRingbufferTaskParams
{
  unsigned char *dma_readout;
  RTP_header *rtp_header;
  unsigned char **packet_data_buffer;
} WriteToRingbufferTaskParams;

void writeToRingbufferTask(WriteToRingbufferTaskParams *pvParameters)
{
  size_t i2s_bytes_read = 0;
  const unsigned int offset = RTP_HEADER_BYTES;

  unsigned char packet_i = 0;
  const unsigned char max_packet_count = 1;
  bool is_positive = true;
  int square_pos = ((2 << 20) - 1);
  int square_neg = (1 - (2 << 20));

  while (true)
  {
    int next_write_pos = (RB_WROTE_AT + 1) < RINGBUFFER_SIZE ? RB_WROTE_AT + 1 : 0;

    renew_aes67_header(pvParameters->rtp_header);

    /* rewrite packet_data */
    pvParameters->packet_data_buffer[next_write_pos][2] = pvParameters->rtp_header->c; // change only the sequence numbers
    pvParameters->packet_data_buffer[next_write_pos][3] = pvParameters->rtp_header->d;
    pvParameters->packet_data_buffer[next_write_pos][4] = pvParameters->rtp_header->e; // and the timestamps
    pvParameters->packet_data_buffer[next_write_pos][5] = pvParameters->rtp_header->f;
    pvParameters->packet_data_buffer[next_write_pos][6] = pvParameters->rtp_header->g;
    pvParameters->packet_data_buffer[next_write_pos][7] = pvParameters->rtp_header->h;
    //
    for (int i = 0; i < SAMPLES_PER_PACKET; i++)
    {
      pvParameters->packet_data_buffer[next_write_pos][offset + i * 3 + 0] =
          is_positive ? (square_pos >> 16) & 0b11111111 : (square_neg >> 16) & 0b11111111;
      pvParameters->packet_data_buffer[next_write_pos][offset + i * 3 + 1] =
          is_positive ? (square_pos >> 8) & 0b11111111 : (square_neg >> 8) & 0b11111111;
      pvParameters->packet_data_buffer[next_write_pos][offset + i * 3 + 2] =
          is_positive ? square_pos & 0b11111111 : square_neg & 0b11111111;
    }

    packet_i++;
    if (packet_i >= max_packet_count)
    {
      packet_i = 0;
      is_positive = !is_positive;
    }

    RB_WROTE_AT = next_write_pos; // 書き終わってからインクリメント
  }
}

typedef struct _readRingbufferAndSendPacketTaskParams
{
  int sock;
  unsigned char **packet_data_buffer;
  struct sockaddr_in *destAddr;
  int destAddrSize;
} ReadRingbufferAndSendPacketTaskParams;

void readRingbufferAndSendPacketTask(ReadRingbufferAndSendPacketTaskParams *pvParameters)
{
  while (true)
  {
    ESP_LOGI(TAG, "RB_WROTE_AT %d RB_READ_AT %d", RB_WROTE_AT, RB_READ_AT);
    if (RB_WROTE_AT != RB_READ_AT)
    {
      ESP_LOGI(TAG, "RB_WROTE_AT != RB_READ_AT");
      ESP_LOGI(TAG, "size %d", pvParameters->destAddr->sin_family);

      int next_read_pos = (RB_READ_AT + 1) < RINGBUFFER_SIZE ? RB_READ_AT + 1 : 0;
      RB_READ_AT = next_read_pos;

      sendto(
          pvParameters->sock,
          pvParameters->packet_data_buffer[RB_READ_AT],
          PACKET_DATA_LENGTH,
          0,
          (struct sockaddr *)pvParameters->destAddr,
          pvParameters->destAddrSize);
    }
  }
}

/* 
 * MAIN関数
 */

void app_main(void)
{
  esp32setup();
  esp32connectToWiFi(); // Taskにすると良さそう
  esp32_i2s_setup();

  /* Disable logging from now on */
  esp_log_level_set(TAG, ESP_LOG_NONE);

  // wait
  // vTaskDelay(2000);

  // Ring Buffer for packet data
  RTP_header rtp_header = create_aes67_header(L24);
  unsigned char *packet_data_buffer[RINGBUFFER_SIZE];
  for (size_t i = 0; i < RINGBUFFER_SIZE; i++)
  {
    packet_data_buffer[i] = (unsigned char *)calloc(PACKET_DATA_LENGTH, sizeof(unsigned char));
  }

  // I2S and DMA
  unsigned char *dma_readout = (unsigned char *)calloc(SAMPLES_PER_PACKET * 4, sizeof(unsigned char));

  // UDP inits
  char addr_str[128];
  int addr_family;
  int ip_protocol;

  /* Initialize packet headers */

  for (size_t i = 0; i < RINGBUFFER_SIZE; i++)
  {
    packet_data_buffer[i][0] = rtp_header.a;
    packet_data_buffer[i][1] = rtp_header.b;
    packet_data_buffer[i][2] = rtp_header.c;
    packet_data_buffer[i][3] = rtp_header.d;
    packet_data_buffer[i][4] = rtp_header.e;
    packet_data_buffer[i][5] = rtp_header.f;
    packet_data_buffer[i][6] = rtp_header.g;
    packet_data_buffer[i][7] = rtp_header.h;
    packet_data_buffer[i][8] = rtp_header.i;
    packet_data_buffer[i][9] = rtp_header.j;
    packet_data_buffer[i][10] = rtp_header.k;
    packet_data_buffer[i][11] = rtp_header.l;
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

  WriteToRingbufferTaskParams w_params = {
      .dma_readout = dma_readout,
      .rtp_header = &rtp_header,
      .packet_data_buffer = packet_data_buffer,
  };

  ReadRingbufferAndSendPacketTaskParams r_params = {
      .packet_data_buffer = packet_data_buffer,
      .sock = sock,
      .destAddr = &destAddr,
      .destAddrSize = sizeof(destAddr),
  };

  xTaskCreatePinnedToCore(
      writeToRingbufferTask, /* Function to implement the task */
      "writeToRingbuffer",   /* Name of the task */
      10000,                 /* Stack size in words */
      &w_params,             /* Task input parameter */
      100,                   /* Priority of the task */
      NULL,                  /* Task handle. */
      READ_CORE              /* Core where the task should run */
  );

  xTaskCreatePinnedToCore(
      readRingbufferAndSendPacketTask, /* Function to implement the task */
      "sendPacket",                    /* Name of the task */
      10000,                           /* Stack size in words */
      &r_params,                       /* Task input parameter */
      100,                             /* Priority of the task */
      NULL,                            /* Task handle. */
      SEND_CORE                        /* Core where the task should run */
  );

  ESP_LOGI(TAG, "Task created...");

  while (true)
  {
  }

  // readRingbufferAndSendPacketTask(&r_params);
}
