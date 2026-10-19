/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : smart_cane — 双判断跌倒 + AO光控 + 按键开关灯（合并版）
  ******************************************************************************
  * 接线：
  *   光敏 DO → PA1（可选，仅打印）
  *   光敏 AO → PA5（ADC1_IN5，主光控，参考文档 lux 思路）
  *   手动开灯键 → PA4（点按切换：开↔关）
  *   照明 LED(+1k) → PB0
  *   取消报警 KEY → PA0    蜂鸣器 → PB1    报警灯 → PC13
  *   OLED/MPU → PB6/PB7
  *   调试串口：USART2  PA2=TX  PA3=RX  115200（USB-TTL）
  *   ESP-01S：USART1  PA9=TX→ESP RX，PA10=RX←ESP TX；供电 AMS1117 3.3V
  *   通知：ALARM 超时 → NOTIFY：OLED + 串口 SOS + Wi‑Fi 求助（ESP）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "mpu6500.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef enum {
  ST_MONITOR = 0,
  ST_ALARM,
  ST_NOTIFY
} AppState;

static AppState g_state = ST_MONITOR;
static uint32_t g_alarm_blink_ms = 0;
static uint32_t g_alarm_enter_ms = 0;

static uint8_t g_home_axis = 2;
static uint8_t  g_seen_accel = 0;
static uint8_t  g_seen_gyro  = 0;
static uint32_t g_event_ms   = 0;
static uint8_t g_ff_cnt   = 0;
static uint8_t g_gyro_cnt = 0;
static uint8_t g_tilt_cnt = 0;
static uint8_t g_candidate = 0;
static uint32_t g_cand_ms = 0;

static uint8_t  g_was_upright = 0;
static uint8_t  g_up_cnt = 0;
static uint32_t g_cooldown_until = 0;

typedef enum {
  LAMP_AUTO = 0,
  LAMP_FORCE_ON,
  LAMP_FORCE_OFF
} LampMode;

static uint16_t g_ao = 0;
static uint8_t  g_auto_dark = 0;
static uint8_t  g_light_on = 0;
static LampMode g_lamp_mode = LAMP_AUTO;
static uint32_t g_force_off_until = 0;
static uint8_t  g_key_prev = 1;
static uint32_t g_key_edge_ms = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN 0 */
#ifndef LDR_DO_Pin
#define LDR_DO_Pin          GPIO_PIN_1
#define LDR_DO_GPIO_Port    GPIOA
#endif
#ifndef LIGHT_KEY_Pin
#define LIGHT_KEY_Pin       GPIO_PIN_4
#define LIGHT_KEY_GPIO_Port GPIOA
#endif
#ifndef NIGHT_LED_Pin
#define NIGHT_LED_Pin       GPIO_PIN_0
#define NIGHT_LED_GPIO_Port GPIOB
#endif

#define LUX_DARK_TH           1800U
#define LUX_BRIGHT_TH         1400U
#define LUX_DARK_WHEN_HIGH       1
#define LIGHT_KEY_DEBOUNCE_MS  200U
#define FORCE_OFF_MS          1200U

#define FREEFALL_TH    5000
#define IMPACT_TH     24000
#define REQUIRE_IMPACT    1
#define GYRO_TH        7000
#define GYRO_NEED         4
#define FF_NEED           3
#define EVENT_WIN       800U
#define HOME_LOW       8000
#define HOME_OK       12500
#define STILL_GYRO     1800
#define TILT_NEED        14
#define CAND_TIMEOUT   1800U
#define UPRIGHT_NEED      8
#define COOLDOWN_MS    2500U
#define ALARM_TO_NOTIFY_MS 10000U

#define WIFI_SSID       "queshi"
#define WIFI_PASS       "12345678"
#define PUSHPLUS_TOKEN  "4561baeb0693479ea7a405aba3f48343"

static void show_monitor(void)
{
  char line[22];

  SSD1306_Clear();
  SSD1306_DrawString(0, 0, "smart_cane");
  SSD1306_DrawString(0, 16, "MONITORING");

  if (g_ao <= 5U) {
    SSD1306_DrawString(0, 32, "lux:0 checkAO");
  } else {
    sprintf(line, "lux:%u", (unsigned)g_ao);
    SSD1306_DrawString(0, 32, line);
  }

  if (g_light_on) {
    SSD1306_DrawString(0, 48, "lamp: ON ");
  } else if (g_lamp_mode == LAMP_FORCE_OFF) {
    SSD1306_DrawString(0, 48, "lamp:WAIT");
  } else {
    SSD1306_DrawString(0, 48, "lamp: OFF");
  }
  SSD1306_UpdateScreen();

  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

static void show_alarm(void)
{
  SSD1306_Clear();
  SSD1306_DrawString(0, 0, "smart_cane");
  SSD1306_DrawString(0, 16, "CHECK NOW");
  SSD1306_DrawString(0, 32, "press KEY");
  SSD1306_UpdateScreen();
}

static void show_notify(void)
{
  SSD1306_Clear();
  SSD1306_DrawString(0, 0, "smart_cane");
  SSD1306_DrawString(0, 16, "NOTIFIED");
  SSD1306_DrawString(0, 32, "family SOS");
  SSD1306_DrawString(0, 48, "press KEY");
  SSD1306_UpdateScreen();
}

static int32_t iabs32(int32_t v) { return (v < 0) ? -v : v; }

static int32_t accel_abs_max(const MPU6500_Motion *m)
{
  int32_t ax = iabs32(m->ax), ay = iabs32(m->ay), az = iabs32(m->az);
  int32_t p = ax;
  if (ay > p) p = ay;
  if (az > p) p = az;
  return p;
}

static int32_t gyro_abs_max(const MPU6500_Motion *m)
{
  int32_t gx = iabs32(m->gx), gy = iabs32(m->gy), gz = iabs32(m->gz);
  int32_t g = gx;
  if (gy > g) g = gy;
  if (gz > g) g = gz;
  return g;
}

static int32_t axis_abs(const MPU6500_Motion *m, uint8_t axis)
{
  if (axis == 0) return iabs32(m->ax);
  if (axis == 1) return iabs32(m->ay);
  return iabs32(m->az);
}

static void learn_home_axis(const MPU6500_Motion *m)
{
  int32_t ax = iabs32(m->ax), ay = iabs32(m->ay), az = iabs32(m->az);
  g_home_axis = 2;
  if (ax >= ay && ax >= az) g_home_axis = 0;
  else if (ay >= ax && ay >= az) g_home_axis = 1;
}

static void clear_events(void)
{
  g_seen_accel = 0;
  g_seen_gyro = 0;
  g_ff_cnt = 0;
  g_gyro_cnt = 0;
  g_tilt_cnt = 0;
  g_candidate = 0;
  g_cand_ms = 0;
}

static uint16_t read_ao_adc(void)
{
  uint32_t sum = 0;
  uint8_t ok = 0;
  uint8_t i;

  for (i = 0; i < 4U; i++) {
    if (HAL_ADC_Start(&hadc1) != HAL_OK) continue;
    if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK) {
      sum += HAL_ADC_GetValue(&hadc1);
      ok++;
    }
    HAL_ADC_Stop(&hadc1);
  }
  if (ok == 0U) return g_ao;
  return (uint16_t)(sum / ok);
}

static void update_night_light(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t key_now;
  uint8_t auto_dark;
  uint8_t do_level;
  static uint8_t dark_sticky = 0;

  g_ao = read_ao_adc();
  do_level = (HAL_GPIO_ReadPin(LDR_DO_GPIO_Port, LDR_DO_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if (g_ao <= 5U) {
    auto_dark = do_level;
    dark_sticky = auto_dark;
  } else {
#if LUX_DARK_WHEN_HIGH
    if (g_ao > LUX_DARK_TH) {
      dark_sticky = 1U;
    } else if (g_ao < LUX_BRIGHT_TH) {
      dark_sticky = 0U;
    }
#else
    if (g_ao < LUX_DARK_TH) {
      dark_sticky = 1U;
    } else if (g_ao > LUX_BRIGHT_TH) {
      dark_sticky = 0U;
    }
#endif
    auto_dark = dark_sticky;
  }
  g_auto_dark = auto_dark;

  if (g_lamp_mode == LAMP_FORCE_OFF) {
    if (auto_dark == 0U) {
      g_lamp_mode = LAMP_AUTO;
      g_force_off_until = 0U;
    } else if ((int32_t)(now - g_force_off_until) >= 0) {
      g_lamp_mode = LAMP_AUTO;
      g_force_off_until = 0U;
      {
        char msg[] = "light:auto-recover\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 50);
      }
    }
  }

  key_now = (HAL_GPIO_ReadPin(LIGHT_KEY_GPIO_Port, LIGHT_KEY_Pin) == GPIO_PIN_RESET) ? 0U : 1U;
  if (g_key_prev == 1U && key_now == 0U) {
    if ((now - g_key_edge_ms) >= LIGHT_KEY_DEBOUNCE_MS) {
      g_key_edge_ms = now;

      if (g_lamp_mode == LAMP_FORCE_OFF) {
        g_lamp_mode = LAMP_FORCE_ON;
        g_force_off_until = 0U;
      } else if (g_light_on) {
        g_lamp_mode = LAMP_FORCE_OFF;
        g_force_off_until = now + FORCE_OFF_MS;
      } else {
        g_lamp_mode = LAMP_FORCE_ON;
      }
    }
  }
  g_key_prev = key_now;

  if (g_lamp_mode == LAMP_FORCE_ON) {
    g_light_on = 1U;
  } else if (g_lamp_mode == LAMP_FORCE_OFF) {
    g_light_on = 0U;
  } else {
    g_light_on = auto_dark ? 1U : 0U;
  }

  HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port, NIGHT_LED_Pin,
                    g_light_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void notify_send_sos(void)
{
  char sos[] = "SOS|FALL|please_check_elder\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)sos, strlen(sos), 100);
}

static char esp_rx[1024];

static void esp_dbg(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 80);
}

static int esp_poll_byte(void)
{
  uint32_t sr = huart1.Instance->SR;
  if ((sr & UART_FLAG_RXNE) != 0U) {
    return (int)(huart1.Instance->DR & 0xFFU);
  }
  if ((sr & UART_FLAG_ORE) != 0U) {
    (void)huart1.Instance->DR;
  }
  return -1;
}

static void esp_clear_ore(void)
{
  if ((huart1.Instance->SR & UART_FLAG_ORE) != 0U) {
    (void)huart1.Instance->DR;
  }
}

static void esp_buf_putc(char *buf, uint16_t *n, uint16_t buflen, char c)
{
  if (c == 0 || buflen < 2U) return;
  if (*n + 1U >= buflen) {
    uint16_t keep = (uint16_t)((buflen - 1U) / 2U);
    memmove(buf, buf + (*n - keep), keep);
    *n = keep;
  }
  buf[(*n)++] = c;
  buf[*n] = 0;
}

static void esp_flush_rx(void)
{
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < 8U) {
    if (esp_poll_byte() >= 0) t0 = HAL_GetTick();
  }
}

static void esp_send_raw(const char *s)
{
  while (*s != 0) {
    esp_clear_ore();
    HAL_UART_Transmit(&huart1, (uint8_t*)s, 1, 20);
    s++;
  }
  esp_clear_ore();
}

static void esp_send_slow(const char *s)
{
  while (*s != 0) {
    uint32_t t0;
    esp_clear_ore();
    HAL_UART_Transmit(&huart1, (uint8_t*)s, 1, 50);
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 2U) {
      (void)esp_poll_byte();
    }
    s++;
  }
}

static const char *esp_match_keys(const char *buf, const char *const *keys)
{
  const char *const *k;
  if (keys == NULL) return NULL;
  for (k = keys; *k != NULL; k++) {
    if ((*k)[0] != 0 && strstr(buf, *k) != NULL) return *k;
  }
  return NULL;
}

static const char *esp_collect(char *buf, uint16_t buflen,
                               const char *const *keys, uint32_t timeout_ms)
{
  uint16_t n = 0;
  uint32_t t0;
  uint32_t last_rx;
  uint8_t have = 0;
  uint8_t scanned = 1;

  if (buflen < 2U) return NULL;
  buf[0] = 0;
  t0 = HAL_GetTick();
  last_rx = t0;
  while ((HAL_GetTick() - t0) < timeout_ms) {
    int c = esp_poll_byte();
    const char *hit;
    if (c >= 0) {
      have = 1;
      scanned = 0;
      last_rx = HAL_GetTick();
      esp_buf_putc(buf, &n, buflen, (char)c);
      continue;
    }
    if (!have || scanned || (HAL_GetTick() - last_rx) < 6U) continue;
    scanned = 1;
    hit = esp_match_keys(buf, keys);
    if (hit != NULL) return hit;
  }
  return esp_match_keys(buf, keys);
}

static uint8_t esp_wait_str(const char *kw, uint32_t timeout_ms)
{
  const char *keys[2];
  keys[0] = kw;
  keys[1] = NULL;
  return (esp_collect(esp_rx, sizeof(esp_rx), keys, timeout_ms) != NULL) ? 1U : 0U;
}

static uint8_t esp_cmd_ok(const char *cmd_crlf, uint32_t timeout_ms)
{
  esp_flush_rx();
  esp_send_raw(cmd_crlf);
  return esp_wait_str("OK", timeout_ms);
}

static void esp_dbg_snip(const char *tag, const char *buf)
{
  char line[120];
  char snip[52];
  size_t len = strlen(buf);
  const char *p = buf;
  uint16_t j = 0;

  if (len > 48U) p = buf + (len - 48U);
  while (*p != 0 && j < 48U) {
    unsigned char c = (unsigned char)(*p++);
    if (c == '\r' || c == '\n' || c == '\t') c = ' ';
    if (c < 32U || c > 126U) continue;
    snip[j++] = (char)c;
  }
  snip[j] = 0;
  if (j == 0U) {
    sprintf(line, "WIFI: %s no reply\r\n", tag);
  } else {
    sprintf(line, "WIFI: %s [%s]\r\n", tag, snip);
  }
  esp_dbg(line);
}

static void esp_log_cwjap_fail(void)
{
  if (strstr(esp_rx, "+CWJAP:3") != NULL) {
    esp_dbg("WIFI: CWJAP fail AP not found\r\n");
  } else if (strstr(esp_rx, "+CWJAP:2") != NULL) {
    esp_dbg("WIFI: CWJAP fail wrong password\r\n");
  } else if (strstr(esp_rx, "+CWJAP:1") != NULL) {
    esp_dbg("WIFI: CWJAP fail timeout\r\n");
  } else if (strstr(esp_rx, "+CWJAP:4") != NULL) {
    esp_dbg("WIFI: CWJAP fail connect\r\n");
  } else {
    esp_dbg_snip("CWJAP fail", esp_rx);
  }
}

static uint8_t esp_parse_staip(const char *buf, char *ip, uint16_t cap)
{
  const char *p;
  uint16_t i = 0;
  uint8_t dots = 0;
  uint8_t digits = 0;

  p = strstr(buf, "STAIP");
  if (p == NULL || cap < 2U) return 0U;
  while (*p != 0 && !(*p >= '0' && *p <= '9')) p++;
  while (*p != 0 && (uint16_t)(i + 1U) < cap) {
    if (*p >= '0' && *p <= '9') {
      ip[i++] = *p;
      digits++;
    } else if (*p == '.') {
      ip[i++] = *p;
      dots++;
    } else {
      break;
    }
    p++;
  }
  ip[i] = 0;
  if (digits < 4U || dots > 3U || ip[0] == '0') return 0U;
  if (dots == 3U || digits >= 7U) return 1U;
  return 0U;
}

static uint8_t esp_sta_has_ip(void)
{
  char ip[20];
  uint16_t n = 0;
  uint32_t t0;
  uint32_t last_rx;
  uint8_t have = 0;
  uint8_t scanned = 1;

  esp_flush_rx();
  esp_send_raw("AT+CIFSR\r\n");
  esp_rx[0] = 0;
  t0 = HAL_GetTick();
  last_rx = t0;
  while ((HAL_GetTick() - t0) < 2000U) {
    int c = esp_poll_byte();
    if (c >= 0) {
      have = 1;
      scanned = 0;
      last_rx = HAL_GetTick();
      esp_buf_putc(esp_rx, &n, sizeof(esp_rx), (char)c);
      continue;
    }
    if (!have || scanned || (HAL_GetTick() - last_rx) < 6U) continue;
    scanned = 1;
    if (esp_parse_staip(esp_rx, ip, sizeof(ip))) {
      char msg[48];
      sprintf(msg, "WIFI: ip %s\r\n", ip);
      esp_dbg(msg);
      return 1U;
    }
    if (strstr(esp_rx, "\nOK") != NULL || strstr(esp_rx, "ERROR") != NULL) break;
  }
  if (esp_parse_staip(esp_rx, ip, sizeof(ip))) {
    char msg[48];
    sprintf(msg, "WIFI: ip %s\r\n", ip);
    esp_dbg(msg);
    return 1U;
  }
  return 0U;
}

static uint8_t esp_join_ap(void)
{
  char cmd[96];
  char msg[80];
  uint8_t try_i;
  const char *qkeys[] = { "OK", "ERROR", "FAIL", NULL };
  const char *jkeys[] = {
    "FAIL", "ERROR", "+CWJAP:1", "+CWJAP:2", "+CWJAP:3", "+CWJAP:4",
    "GOT IP", "OK", NULL
  };
  const char *hit;

  (void)esp_cmd_ok("ATE0\r\n", 800U);
  (void)esp_cmd_ok("AT+CWMODE=1\r\n", 1500U);

  esp_flush_rx();
  esp_send_raw("AT+CWJAP?\r\n");
  hit = esp_collect(esp_rx, sizeof(esp_rx), qkeys, 3000U);
  if (hit != NULL && strstr(esp_rx, WIFI_SSID) != NULL && strstr(esp_rx, "No AP") == NULL) {
    esp_dbg("WIFI: already joined\r\n");
    if (esp_sta_has_ip()) return 1U;
  }

  sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASS);
  sprintf(msg, "WIFI: cmd %s", cmd);
  esp_dbg(msg);

  for (try_i = 0U; try_i < 2U; try_i++) {
    sprintf(msg, "WIFI: join %s try%u\r\n", WIFI_SSID, (unsigned)(try_i + 1U));
    esp_dbg(msg);

    esp_flush_rx();
    esp_send_slow(cmd);
    hit = esp_collect(esp_rx, sizeof(esp_rx), jkeys, 12000U);
    if ((hit != NULL && (strcmp(hit, "GOT IP") == 0 || strcmp(hit, "OK") == 0))
        || strstr(esp_rx, "CONNECTED") != NULL
        || strstr(esp_rx, "CONETD") != NULL
        || strstr(esp_rx, "WIIO") != NULL) {
      esp_dbg("WIFI: joined\r\n");
      (void)esp_sta_has_ip();
      return 1U;
    }
    HAL_Delay(800);
    if (esp_sta_has_ip()) {
      esp_dbg("WIFI: joined\r\n");
      return 1U;
    }
    esp_log_cwjap_fail();
    HAL_Delay(400);
  }
  return 0U;
}

static uint8_t esp_scan_reply_code(int *code_out, uint8_t allow_end)
{
  for (;;) {
    char *p;
    char *q;
    char *d;
    int code = 0;

    p = strstr(esp_rx, "\"code\"");
    if (p == NULL) return 0U;
    q = p + 6;
    while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') q++;
    if (*q == '\0') return 0U;
    if (*q != ':') {
      *p = ' ';
      continue;
    }
    q++;
    while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') q++;
    if (*q == '\0') return 0U;
    if (*q < '0' || *q > '9') {
      *p = ' ';
      continue;
    }
    d = q;
    while (*d >= '0' && *d <= '9') d++;
    if (*d == '\0' && !allow_end) return 0U;
    while (q < d) {
      code = code * 10 + (*q - '0');
      q++;
    }
    *code_out = code;
    return 1U;
  }
}

static uint8_t esp_read_reply_code(int *code_out, uint32_t timeout_ms)
{
  uint16_t n = (uint16_t)strlen(esp_rx);
  uint32_t t0 = HAL_GetTick();
  uint32_t last_rx = t0;
  uint8_t have = 0;
  uint8_t scanned = 1;

  if (esp_scan_reply_code(code_out, 0U)) return 1U;

  while ((HAL_GetTick() - t0) < timeout_ms) {
    int c = esp_poll_byte();
    if (c >= 0) {
      have = 1;
      scanned = 0;
      last_rx = HAL_GetTick();
      esp_buf_putc(esp_rx, &n, sizeof(esp_rx), (char)c);
      continue;
    }
    if (!have || scanned || (HAL_GetTick() - last_rx) < 6U) continue;
    scanned = 1;
    if (esp_scan_reply_code(code_out, 0U)) return 1U;
  }
  return esp_scan_reply_code(code_out, 1U);
}

static void esp_wifi_send_sos(void)
{
  static char http[320];
  char line[96];
  char cipsend[24];
  int http_len;
  const char *host;
  uint8_t use_push;
  const char *ckeys[] = { "ERROR", "FAIL", "Fail", "CONNECT", "OK", NULL };
  const char *skeys[] = { "SEND OK", "SEND FAIL", NULL };
  const char *hit;

  use_push = (PUSHPLUS_TOKEN[0] != '\0') ? 1U : 0U;
  host = use_push ? "www.pushplus.plus" : "www.baidu.com";

  esp_dbg("WIFI: try send...\r\n");

  if (!esp_cmd_ok("AT\r\n", 800U)) {
    esp_dbg("WIFI: AT fail\r\n");
    return;
  }

  if (!esp_join_ap()) {
    esp_dbg("WIFI: skip net (no AP)\r\n");
    return;
  }

  (void)esp_cmd_ok("AT+CIPMUX=0\r\n", 1000U);

  sprintf(line, "AT+CIPSTART=\"TCP\",\"%s\",80\r\n", host);
  esp_flush_rx();
  esp_send_raw(line);
  hit = esp_collect(esp_rx, sizeof(esp_rx), ckeys, 12000U);
  if (hit == NULL || (strcmp(hit, "CONNECT") != 0 && strcmp(hit, "OK") != 0)) {
    esp_dbg_snip("CIPSTART fail", esp_rx);
    return;
  }

  if (use_push) {
    http_len = snprintf(http, sizeof(http),
      "GET /send?token=%s&title=FALL&content=SOS_FALL_please_check_elder&template=txt&channel=wechat HTTP/1.0\r\n"
      "Host: www.pushplus.plus\r\n"
      "Connection: close\r\n\r\n",
      PUSHPLUS_TOKEN);
  } else {
    http_len = snprintf(http, sizeof(http),
      "GET / HTTP/1.0\r\n"
      "Host: www.baidu.com\r\n"
      "Connection: close\r\n\r\n");
  }
  if (http_len < 0 || http_len >= (int)sizeof(http)) {
    esp_dbg("WIFI: http too long\r\n");
    (void)esp_cmd_ok("AT+CIPCLOSE\r\n", 2000U);
    return;
  }

  sprintf(cipsend, "AT+CIPSEND=%d\r\n", http_len);
  esp_flush_rx();
  esp_send_raw(cipsend);
  if (!esp_wait_str(">", 3000U)) {
    esp_dbg("WIFI: CIPSEND fail\r\n");
    return;
  }

  esp_send_raw(http);
  hit = esp_collect(esp_rx, sizeof(esp_rx), skeys, 8000U);
  if (hit == NULL || strcmp(hit, "SEND OK") != 0) {
    esp_dbg("WIFI: SEND fail\r\n");
    (void)esp_cmd_ok("AT+CIPCLOSE\r\n", 1500U);
    return;
  }
  if (use_push) {
    int reply_code = 0;
    uint8_t got = esp_read_reply_code(&reply_code, 8000U);
    char msg[40];
    esp_dbg("WIFI: SEND OK\r\n");
    if (got) {
      sprintf(msg, "WIFI: reply code=%d\r\n", reply_code);
      esp_dbg(msg);
    } else {
      esp_dbg("WIFI: reply no code\r\n");
    }
  } else {
    esp_dbg("WIFI: SEND OK\r\n");
  }

  (void)esp_cmd_ok("AT+CIPCLOSE\r\n", 2000U);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  if (SSD1306_Init(&hi2c1) == 0) {
    show_monitor();
  } else {
    const char *err = "OLED not found on I2C\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), 100);
  }

  if (MPU6500_Init(&hi2c1) != 0) {
    const char *e = "MPU not found\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)e, strlen(e), 100);
  } else {
    const char *ok = "MPU ready\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);
    {
      MPU6500_Motion m0;
      HAL_Delay(80);
      if (MPU6500_ReadMotion(&m0) == 0) {
        learn_home_axis(&m0);
      }
    }
  }

  clear_events();
  g_was_upright = 0;
  g_up_cnt = 0;
  g_cooldown_until = 0;
  g_state = ST_MONITOR;
  g_lamp_mode = LAMP_AUTO;
  g_force_off_until = 0;
  g_light_on = 0;
  g_key_prev = 1;
  HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port, NIGHT_LED_Pin, GPIO_PIN_RESET);

  HAL_Delay(800);
  if (esp_cmd_ok("AT\r\n", 1000U)) {
    const char *ok = "ESP AT OK\r\n";
    const char *fw = "WIFI: fw rxfix\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ok, strlen(ok), 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)fw, strlen(fw), 100);
    if (!esp_join_ap()) {
      const char *e = "WIFI: boot join fail (GND/PA9/PA10)\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)e, strlen(e), 100);
    }
  } else {
    const char *e = "ESP AT fail (check PA9/10 + 3.3V + common GND)\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)e, strlen(e), 100);
  }
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    MPU6500_Motion m;
    static uint32_t print_div = 0;

    update_night_light();

    if (g_state == ST_MONITOR) {
      static uint32_t last_oled_ms = 0;
      uint32_t tnow = HAL_GetTick();
      if ((tnow - last_oled_ms) >= 200U) {
        last_oled_ms = tnow;
        show_monitor();
      }
    }

    if (MPU6500_ReadMotion(&m) == 0) {
      int32_t peak  = accel_abs_max(&m);
      int32_t gpeak = gyro_abs_max(&m);
      int32_t home_a = axis_abs(&m, g_home_axis);
      uint32_t now = HAL_GetTick();

      if ((++print_div % 5U) == 0U) {
        char line[160];
        int n = sprintf(line,
          "p=%ld g=%ld home=%ld a=%u gy=%u c=%u u=%u ao=%u dark=%u led=%u mode=%u\r\n",
          (long)peak, (long)gpeak, (long)home_a,
          (unsigned)g_seen_accel, (unsigned)g_seen_gyro,
          (unsigned)g_candidate, (unsigned)g_was_upright,
          (unsigned)g_ao, (unsigned)g_auto_dark,
          (unsigned)g_light_on, (unsigned)g_lamp_mode);
        HAL_UART_Transmit(&huart2, (uint8_t*)line, n, 100);
      }

      if (g_state == ST_MONITOR) {
        if (now < g_cooldown_until) {
          clear_events();
          if (home_a >= HOME_OK && home_a < 25000 && gpeak < STILL_GYRO) {
            if (g_up_cnt < 255) g_up_cnt++;
            if (g_up_cnt >= UPRIGHT_NEED) g_was_upright = 1;
          } else {
            g_up_cnt = 0;
          }
        } else {
          if (!g_candidate) {
            if (home_a >= HOME_OK && home_a < 25000 && gpeak < STILL_GYRO) {
              if (g_up_cnt < 255) g_up_cnt++;
              if (g_up_cnt >= UPRIGHT_NEED) g_was_upright = 1;
            } else {
              g_up_cnt = 0;
            }
          }

          if ((g_seen_accel || g_seen_gyro) && (now - g_event_ms) > EVENT_WIN) {
            if (!g_candidate) {
              g_seen_accel = 0;
              g_seen_gyro = 0;
              g_ff_cnt = 0;
              g_gyro_cnt = 0;
            }
          }

#if REQUIRE_IMPACT
          (void)g_ff_cnt;
          if (peak >= IMPACT_TH) {
            g_seen_accel = 1;
            g_event_ms = now;
          }
#else
          if (peak < FREEFALL_TH) {
            if (g_ff_cnt < 255) g_ff_cnt++;
            if (g_ff_cnt >= FF_NEED) {
              g_seen_accel = 1;
              g_event_ms = now;
            }
          } else {
            g_ff_cnt = 0;
          }
          if (peak >= IMPACT_TH) {
            g_seen_accel = 1;
            g_event_ms = now;
          }
#endif

          if (gpeak > GYRO_TH) {
            if (g_gyro_cnt < 255) g_gyro_cnt++;
            if (g_gyro_cnt >= GYRO_NEED) {
              g_seen_gyro = 1;
              g_event_ms = now;
            }
          } else {
            g_gyro_cnt = 0;
          }

          if (g_was_upright && g_seen_accel && g_seen_gyro) {
            if (!g_candidate) {
              g_candidate = 1;
              g_cand_ms = now;
              g_tilt_cnt = 0;
            }
          }

          if (g_candidate) {
            if ((now - g_cand_ms) > CAND_TIMEOUT) {
              clear_events();
            } else if (home_a < HOME_LOW) {
              if (g_tilt_cnt < 255) g_tilt_cnt++;
              if (g_tilt_cnt >= TILT_NEED) {
                clear_events();
                g_was_upright = 0;
                g_up_cnt = 0;
                g_state = ST_ALARM;
                g_alarm_blink_ms = now;
                g_alarm_enter_ms = now;
                show_alarm();
                {
                  char msg[] = ">>> ALARM\r\n";
                  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
                }
              }
            } else if (home_a >= HOME_OK && home_a < 25000 && gpeak < STILL_GYRO) {
              clear_events();
            } else {
              if (g_tilt_cnt > 0) g_tilt_cnt--;
            }
          }
        }
      }
    }

    if (g_state == ST_ALARM) {
      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
      if ((HAL_GetTick() - g_alarm_blink_ms) >= 200U) {
        g_alarm_blink_ms = HAL_GetTick();
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }

      if ((HAL_GetTick() - g_alarm_enter_ms) >= ALARM_TO_NOTIFY_MS) {
        g_state = ST_NOTIFY;
        show_notify();
        notify_send_sos();
        {
          char msg[] = ">>> NOTIFY (UART + WiFi SOS)\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
        esp_wifi_send_sos();
      } else if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
        g_state = ST_MONITOR;
        clear_events();
        g_was_upright = 0;
        g_up_cnt = 0;
        g_cooldown_until = HAL_GetTick() + COOLDOWN_MS;
        show_monitor();
        {
          char msg[] = "cancel -> MONITOR\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
        HAL_Delay(300);
      }
    }

    if (g_state == ST_NOTIFY) {
      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
      if ((HAL_GetTick() - g_alarm_blink_ms) >= 200U) {
        g_alarm_blink_ms = HAL_GetTick();
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
      if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) {
        g_state = ST_MONITOR;
        clear_events();
        g_was_upright = 0;
        g_up_cnt = 0;
        g_cooldown_until = HAL_GetTick() + COOLDOWN_MS;
        show_monitor();
        {
          char msg[] = "notify ack -> MONITOR\r\n";
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
        HAL_Delay(300);
      }
    }

    HAL_Delay(20);
    /* USER CODE END 3 */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NIGHT_LED_GPIO_Port, NIGHT_LED_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LDR_DO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LDR_DO_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LIGHT_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIGHT_KEY_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NIGHT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NIGHT_LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif