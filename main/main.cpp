#include <cstdio>
#include <cmath>
#include "esp_log.h"
#include "esp_spiffs.h"
extern "C" {
  #include "ft8/decode.h"
  #include "ft8/constants.h"
  #include "ft8/message.h"
  #include "ft8/encode.h"
  #include "common/monitor.h"
}

#include "ui.h"
#include <vector>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"
#include "autoseq.h"
#include <M5Cardputer.h>
#include <sstream>
#include <iterator>
#include <cstdio>
#include <string>
#include <cstdint>
#include <vector>
#include <cstring>
#include <unordered_map>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <memory>
#include "driver/usb_serial_jtag.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_random.h"
#include <cctype>
#include <cstdlib>
#include <ctime>
#include "esp_timer.h"
#include "esp_sleep.h"
#include "stream_uac.h"
#define ENABLE_BLE 0

#if ENABLE_BLE
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"
#include "esp_bt.h"
#endif
#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 12000
#endif

#define UART_SVC_UUID   0xFFE0
#define UART_RX_UUID    0xFFE1
#define UART_TX_UUID    0xFFE2

#if ENABLE_BLE
static const ble_uuid16_t uart_svc_uuid =
    BLE_UUID16_INIT(UART_SVC_UUID);
static const ble_uuid16_t uart_rx_uuid =
    BLE_UUID16_INIT(UART_RX_UUID);
static const ble_uuid16_t uart_tx_uuid =
    BLE_UUID16_INIT(UART_TX_UUID);
#endif


#if ENABLE_BLE

static QueueHandle_t ble_rx_queue = nullptr;
static uint16_t gatt_tx_handle = 0;
static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static const char* BT_TAG = "BLE_INIT";

static int gap_cb(struct ble_gap_event *event, void *arg);
static void nimble_host_task(void *param);
static void ble_on_sync(void);

// RX write callback (ok as you have)
static int uart_rx_cb(uint16_t conn_handle,
                      uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt,
                      void *arg)
{
    if (!ble_rx_queue) return 0;
    ESP_LOGI(BT_TAG, "RX write cb: conn=%u len=%u", conn_handle, (unsigned)ctxt->om->om_len);
    const uint8_t *p = ctxt->om->om_data;
    uint16_t len = ctxt->om->om_len;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = p[i];
        xQueueSend(ble_rx_queue, &b, 0);
    }
    return 0;
}

// TX characteristic doesn't need reads/writes; return success.
static int uart_tx_cb(uint16_t conn_handle,
                      uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt,
                      void *arg)
{
    return 0;
}

#include "esp_nimble_hci.h"   // <-- add

// C++-safe static characteristics table
static const struct ble_gatt_chr_def gatt_uart_chrs[] = {
    {
        .uuid = &uart_rx_uuid.u,
        .access_cb = uart_rx_cb,
        .arg = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &uart_tx_uuid.u,              // <-- IMPORTANT: no BLE_UUID16_DECLARE here
        .access_cb = uart_tx_cb,
        .arg = nullptr,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &gatt_tx_handle,  // CCCD will follow
    },
    { 0 }  // terminator
};

// C++-safe service table
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uart_svc_uuid.u,
        .characteristics = gatt_uart_chrs,
    },
    { 0 }  // terminator
};

static void init_bluetooth(void)
{
    static bool inited = false;
    if (inited) return;
    inited = true;
    ESP_LOGI(BT_TAG, "init_bluetooth start");

    esp_err_t nvrc = nvs_flash_init();
    if (nvrc == ESP_ERR_NVS_NO_FREE_PAGES || nvrc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvrc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvrc);

    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "nimble_port_init failed: %d", rc);
        return;
    }
    ESP_LOGI(BT_TAG, "nimble_port_init OK");

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_LOGI(BT_TAG, "GAP/GATT init done");

    ble_rx_queue = xQueueCreate(256, 1);
    assert(ble_rx_queue);

    ble_svc_gap_device_name_set("Mini-FT8");

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }
    ESP_LOGI(BT_TAG, "Services added");

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(BT_TAG, "Host task started");
}


static void ble_app_advertise(void);

static int gap_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            ESP_LOGI(BT_TAG, "GAP connect, handle=%u", g_conn_handle);
        } else {
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGW(BT_TAG, "GAP connect failed; restarting adv");
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGW(BT_TAG, "GAP disconnect; restarting adv");
        ble_app_advertise();
        break;

    default:
        ESP_LOGI(BT_TAG, "GAP event type=%d", event->type);
        break;
    }
    return 0;
}


// BLE UART-style service (Nordic-like) UUIDs
[[maybe_unused]] static uint8_t ble_rx_placeholder = 0;
[[maybe_unused]] static uint8_t ble_tx_placeholder = 0;
#endif // ENABLE_BLE

static const char* TAG = "FT8";
enum class UIMode { RX, TX, BAND, MENU, HOST, CONTROL, DEBUG, LIST, STATUS, QSO };
static UIMode ui_mode = UIMode::RX;
static int tx_page = 0;
static std::vector<UiRxLine> g_rx_lines;
int64_t g_decode_slot_idx = -1; // set at decode trigger to tag RX lines with slot parity
static const char* STATION_FILE = "/spiffs/StationData.ini";

//enum class BeaconMode { OFF = 0, EVEN, EVEN2, ODD, ODD2 };
enum class BeaconMode { OFF = 0, EVEN, ODD };
struct BandItem {
  const char* name;
  int freq;
};
static std::vector<BandItem> g_bands = {
    {"160m", 1840},   {"80m", 3573},   {"60m", 5357},   {"40m", 7074},
    {"30m", 10136},   {"20m", 14074},  {"17m", 18100},  {"15m", 21074},
    {"12m", 24915},   {"10m", 28074},  {"6m", 50313},   {"2m", 144174},
};
static std::string g_active_band_text = "80 40 20 17 15 12 10";
static std::vector<int> g_active_band_indices;
static int band_page = 0;
static int band_edit_idx = -1;       // absolute index into g_bands
static std::string band_edit_buffer; // text while editing
static BeaconMode g_beacon = BeaconMode::OFF;
static int g_offset_hz = 1500;
static int g_band_sel = 1; // default 80m
static bool g_tune = false;
static BeaconMode g_status_beacon_temp = BeaconMode::OFF;
static bool g_cat_toggle_high = false;
static std::string g_date = "2025-12-11";
static std::string g_time = "10:10:00";
static int status_edit_idx = -1;     // 0-5
static std::string status_edit_buffer;
static int status_cursor_pos = -1;
static void host_send_bt(const std::string& s);
static std::vector<std::string> g_debug_lines;
static int debug_page = 0;
static const size_t DEBUG_MAX_LINES = 18; // 3 pages
static void debug_log_line(const std::string& msg);
static void host_handle_line(const std::string& line);
// TX entry for display and scheduling (populated by autoseq)
static AutoseqTxEntry g_pending_tx;
static bool g_pending_tx_valid = false;
static void host_process_bytes(const uint8_t* buf, size_t len);
static void poll_host_uart();
static void poll_ble_uart();
static void enter_mode(UIMode new_mode);
static bool g_rx_dirty = false;

static constexpr uart_port_t SOFT_UART_NUM = UART_NUM_1;
static constexpr int SOFT_UART_TX_PIN = 1;  // G1
static constexpr int SOFT_UART_RX_PIN = 2;  // G2


static std::vector<std::string> g_list_lines = {
    "10:34 20m WA4HR",
    "10:36 40m K4ABC",
    "10:40 17m DL2XYZ",
    "10:45 30m JA1ZZZ",
    "10:50 15m VK2AAA",
    "10:52 10m KH6BBB"
};
static int list_page = 0;
static std::vector<std::string> g_uac_lines = {
    "HOST MODE: USB serial",
    "Commands:",
    "WRITEBIN <file> <size> <crc32_hex>",
    "WRITE/APPEND",
    "READ/DELETE",
    "LIST/INFO/HELP",
    "EXIT to leave"
};
static std::vector<std::string> g_ctrl_lines = {
    "C MODE: USB serial",
    "Commands:",
    "WRITEBIN <file> <size> <crc32_hex>",
    "WRITE/APPEND",
    "READ/DELETE",
    "LIST/INFO/HELP",
    "EXIT to leave"
};
static std::vector<std::string> g_q_lines;
static std::vector<std::string> g_q_files;
static bool g_q_show_entries = false;
static int q_page = 0;
static std::string g_q_current_file;
static std::string host_input;
static const char* HOST_PROMPT = "MINIFT8> ";
static bool usb_ready = false;
static bool host_bin_active = false;
static size_t host_bin_remaining = 0;
static FILE* host_bin_fp = nullptr;
static uint32_t host_bin_crc = 0;
static uint32_t host_bin_expected_crc = 0;
static size_t host_bin_received = 0;
static std::vector<uint8_t> host_bin_buf;
static const size_t HOST_BIN_CHUNK = 512;
static size_t host_bin_chunk_expect = 0; // payload bytes this chunk (excludes CRC trailer)
static uint8_t host_bin_first8[8] = {0};
static uint8_t host_bin_last8[8] = {0};
static size_t host_bin_first_filled = 0;
static std::string host_bin_path;

// Software RTC
static time_t rtc_epoch_base = 0;
static int64_t rtc_ms_start = 0;
static int64_t rtc_last_update = 0;
static bool rtc_valid = false;

enum class CqType { CQ, CQSOTA, CQPOTA, CQQRP, CQFD, CQFREETEXT };
enum class OffsetSrc { RANDOM, CURSOR, RX };
enum class RadioType { NONE, TRUSDX, QMX, KH1 };
static CqType g_cq_type = CqType::CQ;
static std::string g_cq_freetext = "FreeText";
static bool g_skip_tx1 = false;
static std::string g_free_text = "DE AG6AQ CM97";
static std::string g_call = "AG6AQ";
static std::string g_grid = "CM97";
bool g_decode_enabled = true;
static OffsetSrc g_offset_src = OffsetSrc::RANDOM;
static RadioType g_radio = RadioType::NONE;
static std::string g_ant = "EFHW";
static std::string g_comment1 = "MiniFT8 /Radio /Ant";
static bool g_rxtx_log = true;
static TaskHandle_t s_tx_task_handle = NULL;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static int menu_page = 0;
static int menu_edit_idx = -1;
static std::string menu_edit_buf;
static bool menu_long_edit = false;
static enum { LONG_NONE, LONG_FT, LONG_COMMENT, LONG_ACTIVE } menu_long_kind = LONG_NONE;
static std::string menu_long_buf;
static std::string menu_long_backup;
static int menu_flash_idx = -1;          // absolute index to flash highlight
static int64_t menu_flash_deadline = 0;  // ms timestamp when flash ends
static int rx_flash_idx = -1;
static int64_t rx_flash_deadline = 0;
bool g_streaming = false;
static void draw_menu_view();
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging);
static void draw_status_view();
static void draw_status_line(int idx, const std::string& text, bool highlight);
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();
static void update_countdown();
static void menu_flash_tick();
static void rx_flash_tick();
#if ENABLE_BLE
static uint8_t g_own_addr_type;
#endif
static bool looks_like_grid(const std::string& s);
static bool looks_like_report(const std::string& s, int& out);
static std::string g_last_reply_text;
static void rebuild_active_bands();
static void schedule_tx_if_idle();
static int64_t last_beacon_slot = -1000;
static bool g_sync_pending = false;
static int g_sync_delta_ms = 0;
static void maybe_enqueue_beacon();
static void qso_load_file_list();
static void qso_load_entries(const std::string& path);

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter = -1);
static void log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd);
#if !MIC_PROBE_APP
void log_heap(const char* tag) {
  size_t free_sz = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  ESP_LOGI(tag, "HEAP: free=%u min=%u largest=%u", (unsigned)free_sz, (unsigned)min_free, (unsigned)largest);
}
#else
static inline void log_heap(const char*) {}
#endif

static void log_rxtx_line(char dir, int snr, int offset_hz, const std::string& text, int repeat_counter) {
  if (!g_rxtx_log) return;
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char ts[32];
  snprintf(ts, sizeof(ts), "%04d%02d%02d %02d%02d%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  double freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;
  FILE* f = fopen("/spiffs/RxTxLog.txt", "a");
  if (!f) {
    ESP_LOGW(TAG, "RxTxLog open failed");
    return;
  }
  // For TX, omit SNR and repeat; for RX keep SNR.
  if (dir == 'T') {
    fprintf(f, "%c [%s][%.3f] %s %d\n",
            dir, ts, freq_mhz, text.c_str(), offset_hz);
  } else {
    fprintf(f, "%c [%s][%.3f] %s %d %d\n",
            dir, ts, freq_mhz, text.c_str(), snr, offset_hz);
  }
  fclose(f);
}

static void qso_load_file_list() {
  g_q_files.clear();
  g_q_lines.clear();
  DIR* dir = opendir("/spiffs");
  if (!dir) {
    g_q_lines.push_back("No ADIF logs");
    return;
  }
  struct dirent* ent;
  while ((ent = readdir(dir)) != nullptr) {
    const char* name = ent->d_name;
    size_t len = strlen(name);
    if (len >= 4 && strcasecmp(name + len - 4, ".adi") == 0) {
      g_q_files.emplace_back(name);
    }
  }
  closedir(dir);
  std::sort(g_q_files.begin(), g_q_files.end(), std::greater<std::string>());
  if (g_q_files.empty()) {
    g_q_lines.push_back("No ADIF logs");
    return;
  }
  for (size_t i = 0; i < g_q_files.size(); ++i) {
    g_q_lines.push_back(g_q_files[i]);
  }
}

static void qso_load_entries(const std::string& path) {
  g_q_lines.clear();
  std::string full = std::string("/spiffs/") + path;
  FILE* f = fopen(full.c_str(), "r");
  if (!f) {
    g_q_lines.push_back("Open fail");
    return;
  }
  char line[256];
  while (fgets(line, sizeof(line), f)) {
    std::string s(line);
    if (s.find("<call:") == std::string::npos) continue;
    auto get_field = [&](const std::string& tag)->std::string {
      size_t p = s.find("<" + tag);
      if (p == std::string::npos) return "";
      size_t gt = s.find('>', p);
      if (gt == std::string::npos) return "";
      size_t end = s.find(' ', gt);
      if (end == std::string::npos) end = s.size();
      return s.substr(gt + 1, end - gt - 1);
    };
    std::string call = get_field("call:");
    std::string time_on = get_field("time_on:");
    std::string freq = get_field("freq:");
    std::string band = freq;
    if (!freq.empty()) {
      // crude map: take MHz and map to band name from our band list
      double mhz = atof(freq.c_str());
      for (const auto& b : g_bands) {
        double bm = b.freq * 0.001;
        if (fabs(bm - mhz) < 0.1) { band = b.name; break; }
      }
    }
    if (time_on.size() >= 4) {
      time_on = time_on.substr(0,4);
      time_on.insert(2, ":");
    }
    if (call.empty()) call = "?";
    if (band.empty()) band = freq.empty() ? "?" : freq;
    g_q_lines.push_back(time_on + " " + band + " " + call);
  }
  fclose(f);
  if (g_q_lines.empty()) g_q_lines.push_back("No QSOs");
}

static void log_adif_entry(const std::string& dxcall, const std::string& dxgrid, int rst_sent, int rst_rcvd) {
  // Build file name based on current date
  time_t now = (time_t)(rtc_now_ms() / 1000);
  struct tm t;
  localtime_r(&now, &t);
  char date[16];
  int year = t.tm_year + 1900;
  int month = t.tm_mon + 1;
  int day = t.tm_mday;
  snprintf(date, sizeof(date), "%04d%02d%02d", year % 10000, month % 100, day % 100);
  char path[64];
  snprintf(path, sizeof(path), "/spiffs/%s.adi", date);

  bool need_header = false;
  struct stat st;
  if (stat(path, &st) != 0) need_header = true;

  FILE* f = fopen(path, "a");
  if (!f) {
    ESP_LOGW(TAG, "ADIF open failed");
    return;
  }
  if (need_header) {
    fprintf(f, "ADIF EXPORT\n<eoh>\n");
  }

  char time_on[16];
  int hour = t.tm_hour;
  int min = t.tm_min;
  int sec = t.tm_sec;
  snprintf(time_on, sizeof(time_on), "%02d%02d%02d", hour % 100, min % 100, sec % 100);
  double freq_mhz = 0.001 * (double)g_bands[g_band_sel].freq;
  char freq_str[16];
  snprintf(freq_str, sizeof(freq_str), "%.3f", freq_mhz);

  std::string comment_expanded = g_comment1;
  auto repl = [](std::string& s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
  };
  // Expand placeholders using current radio/ant strings
  auto radio_name_local = [](RadioType r) {
    switch (r) {
      case RadioType::TRUSDX: return "TRUSDX";
      case RadioType::QMX: return "QMX";
      case RadioType::KH1: return "KH1";
      default: return "None";
    }
  };
  repl(comment_expanded, "/Radio", radio_name_local(g_radio));
  repl(comment_expanded, "/Ant", g_ant);
  fprintf(f, "<call:%zu>%s <gridsquare:%zu>%s <mode:3>FT8<qso_date:8>%s <time_on:6>%s <freq:%zu>%s <station_callsign:%zu>%s <my_gridsquare:%zu>%s <rst_sent:%d>%d <rst_rcvd:%d>%d <comment:%zu>%s <eor>\n",
          dxcall.size(), dxcall.c_str(),
          dxgrid.size(), dxgrid.c_str(),
          date, time_on,
          strlen(freq_str), freq_str,
          g_call.size(), g_call.c_str(),
          g_grid.size(), g_grid.c_str(),
          rst_sent == -99 ? 1 : (int)snprintf(nullptr,0,"%d",rst_sent), rst_sent,
          rst_rcvd == -99 ? 1 : (int)snprintf(nullptr,0,"%d",rst_rcvd), rst_rcvd,
          comment_expanded.size(), comment_expanded.c_str());
  fclose(f);
}

// Log redirection to soft UART (G1/G2)
static vprintf_like_t s_prev_vprintf = nullptr;
static bool s_log_soft_uart = false;
static int soft_uart_vprintf(const char* fmt, va_list ap) {
  char buf[256];
  va_list copy;
  va_copy(copy, ap);
  int n = vsnprintf(buf, sizeof(buf), fmt, copy);
  va_end(copy);
  if (n > 0) {
    size_t to_write = (n < (int)sizeof(buf)) ? n : sizeof(buf);
    uart_write_bytes(SOFT_UART_NUM, buf, to_write);
  }
  return n;
}
static void set_log_to_soft_uart(bool enable) {
  if (enable && !s_log_soft_uart) {
    s_prev_vprintf = esp_log_set_vprintf(soft_uart_vprintf);
    s_log_soft_uart = true;
  } else if (!enable && s_log_soft_uart) {
    if (s_prev_vprintf) {
      esp_log_set_vprintf(s_prev_vprintf);
    }
    s_log_soft_uart = false;
  }
}

static void ensure_usb() {
  if (usb_ready) return;
  usb_serial_jtag_driver_config_t cfg = {
    .tx_buffer_size = 1024,
    .rx_buffer_size = 4096,
  };
  if (usb_serial_jtag_driver_install(&cfg) == ESP_OK) {
    usb_ready = true;
  }
}

static void host_write_str(const std::string& s) {
  ensure_usb();
  if (usb_ready) {
    usb_serial_jtag_write_bytes((const uint8_t*)s.data(), s.size(), 0);
  }
  host_send_bt(s);
}

struct WAVHeader {
  char riff[4];
  uint32_t file_size;
  char wave[4];
  char fmt[4];
  uint32_t fmt_size;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align;
  uint16_t bits_per_sample;
  char data[4];
  uint32_t data_size;
};

[[maybe_unused]] static esp_err_t decode_wav(const char* path) {
  ESP_LOGI(TAG, "Decoding %s", path);
  FILE* f = fopen(path, "rb");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s", path);
    return ESP_FAIL;
  }

  WAVHeader hdr;
  if (fread(&hdr, 1, sizeof(hdr), f) != sizeof(hdr)) {
    ESP_LOGE(TAG, "Failed to read WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (memcmp(hdr.riff, "RIFF", 4) != 0 || memcmp(hdr.wave, "WAVE", 4) != 0) {
    ESP_LOGE(TAG, "Invalid WAV header");
    fclose(f);
    return ESP_FAIL;
  }
  if (hdr.sample_rate != FT8_SAMPLE_RATE || hdr.num_channels != 1) {
    ESP_LOGE(TAG, "WAV must be mono %d Hz (got %u Hz, %u ch)", FT8_SAMPLE_RATE, hdr.sample_rate, hdr.num_channels);
    fclose(f);
    return ESP_FAIL;
  }

  const int bytes_per_sample = hdr.bits_per_sample / 8;

  monitor_config_t mon_cfg;
  mon_cfg.f_min = 200.0f;
  mon_cfg.f_max = 3000.0f;
  mon_cfg.sample_rate = FT8_SAMPLE_RATE;
  mon_cfg.time_osr = 1;
  mon_cfg.freq_osr = 2;
  mon_cfg.protocol = FTX_PROTOCOL_FT8;

  monitor_t mon;
  monitor_init(&mon, &mon_cfg);
  monitor_reset(&mon);

  float* chunk = (float*)malloc(sizeof(float) * mon.block_size);
  if (!chunk) {
    ESP_LOGE(TAG, "Chunk alloc failed");
    fclose(f);
    monitor_free(&mon);
    return ESP_ERR_NO_MEM;
  }

  while (!feof(f)) {
    int read_samples = 0;
    while (read_samples < mon.block_size && !feof(f)) {
      float sample_value = 0.0f;
      if (bytes_per_sample == 1) {
        int s = fgetc(f);
        if (s == EOF) break;
        sample_value = ((float)s - 128.0f) / 128.0f;
      } else if (bytes_per_sample == 2) {
        int low = fgetc(f);
        int high = fgetc(f);
        if (low == EOF || high == EOF) break;
        int16_t s = (int16_t)((high << 8) | low);
        sample_value = (float)s / 32768.0f;
      }
      chunk[read_samples++] = sample_value;
    }
    if (read_samples == 0) break;
    for (int i = read_samples; i < mon.block_size; ++i) {
      chunk[i] = 0.0f;
    }

    // Simple per-block AGC to ~0.1 target level
    double acc = 0.0;
    for (int i = 0; i < mon.block_size; ++i) acc += fabsf(chunk[i]);
    float level = (float)(acc / mon.block_size);
    float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
    if (gain < 0.1f) gain = 0.1f;
    if (gain > 10.0f) gain = 10.0f;
    for (int i = 0; i < mon.block_size; ++i) {
      chunk[i] *= gain;
    }

    monitor_process(&mon, chunk);
  }

  free(chunk);
  fclose(f);

  if (mon.wf.num_blocks == 0) {
    ESP_LOGW(TAG, "No audio blocks processed");
    monitor_free(&mon);
    return ESP_FAIL;
  }
  decode_monitor_results(&mon, &mon_cfg, false); // defer UI to main loop on core1
  monitor_free(&mon);

  return ESP_OK;
}

static void redraw_tx_view() {
  // Get QSO states from autoseq for display
  std::vector<std::string> qtext;
  autoseq_get_qso_states(qtext);

  std::vector<bool> marks(qtext.size(), false);  // No delete marks with autoseq
  std::vector<int> slots;

  // Slot color for pending TX
  slots.push_back(g_pending_tx_valid ? (g_pending_tx.slot_id & 1) : 0);
  // All QSO entries use their context's slot
  for (size_t i = 0; i < qtext.size(); ++i) {
    slots.push_back(0);  // Default to even; autoseq manages internally
  }

  std::string next_line;
  if (g_pending_tx_valid && !g_pending_tx.text.empty()) {
    next_line = g_pending_tx.text;
  }

  ui_draw_tx(next_line, qtext, tx_page, -1, marks, slots);
}

static void draw_band_view() {
  std::vector<std::string> lines;
  lines.reserve(g_bands.size());
  for (size_t i = 0; i < g_bands.size(); ++i) {
    std::string freq_str;
    if ((int)i == band_edit_idx && !band_edit_buffer.empty()) {
      freq_str = band_edit_buffer;
    } else {
      freq_str = std::to_string(g_bands[i].freq);
    }
    lines.push_back(std::string(g_bands[i].name) + ": " + freq_str);
  }
  ui_draw_list(lines, band_page, band_edit_idx);
}

static const char* beacon_name(BeaconMode m) {
  switch (m) {
    case BeaconMode::OFF: return "OFF";
    case BeaconMode::EVEN: return "EVEN";
    //case BeaconMode::EVEN2: return "EVEN2";
    case BeaconMode::ODD: return "ODD";
    //case BeaconMode::ODD2: return "ODD2";
  }
  return "OFF";
}

static const char* cq_type_name(CqType t) {
  switch (t) {
    case CqType::CQ: return "CQ";
    case CqType::CQSOTA: return "CQ SOTA";
    case CqType::CQPOTA: return "CQ POTA";
    case CqType::CQQRP: return "CQ QRP";
    case CqType::CQFD: return "CQ FD";
    case CqType::CQFREETEXT: return "FreeText";
  }
  return "CQ";
}

static const char* offset_name(OffsetSrc o) {
  switch (o) {
    case OffsetSrc::RANDOM: return "Random";
    case OffsetSrc::CURSOR: return "Cursor";
    case OffsetSrc::RX: return "RX";
  }
  return "Random";
}

static const char* radio_name(RadioType r) {
  switch (r) {
    case RadioType::NONE: return "None";
    case RadioType::TRUSDX: return "truSDX";
    case RadioType::QMX: return "QMX";
    case RadioType::KH1: return "KH1";
  }
  return "None";
}

static std::string expand_comment1() {
  std::string out = g_comment1;
  auto repl = [](std::string& s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
  };
  repl(out, "/Radio", radio_name(g_radio));
  repl(out, "/Ant", g_ant);
  return out;
}

static std::string battery_status_line() {
  int level = (int)M5.Power.getBatteryLevel();
  bool charging = M5.Power.isCharging();
  if (level < 0 || level > 100) level = 0;
  char buf[32];
  snprintf(buf, sizeof(buf), "Batt:%3d%%%s", level, charging ? " CHG" : "");
  return buf;
}

static std::string elide_right(const std::string& s, size_t max_len = 22) {
  if (s.size() <= max_len) return s;
  if (max_len <= 3) return s.substr(s.size() - max_len);
  return std::string("...") + s.substr(s.size() - (max_len - 3));
}

static std::string head_trim(const std::string& s, size_t max_len = 16) {
  if (s.size() <= max_len) return s;
  if (max_len == 0) return "";
  if (max_len == 1) return ">";
  return s.substr(0, max_len - 1) + ">";
}

static std::string highlight_pos(const std::string& s, int pos) {
  if (pos < 0 || pos >= (int)s.size()) return s;
  std::string out;
  out.reserve(s.size() + 2);
  out.append(s, 0, pos);
  out.push_back('[');
  out.push_back(s[pos]);
  out.push_back(']');
  out.append(s, pos + 1, std::string::npos);
  return out;
}

static void draw_status_view();

static bool rtc_set_from_strings() {
  int y, M, d, h, m, s;
  if (sscanf(g_date.c_str(), "%d-%d-%d", &y, &M, &d) != 3) return false;
  if (sscanf(g_time.c_str(), "%d:%d:%d", &h, &m, &s) != 3) return false;
  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = M - 1;
  t.tm_mday = d;
  t.tm_hour = h;
  t.tm_min = m;
  t.tm_sec = s;
  time_t epoch = mktime(&t);
  if (epoch == (time_t)-1) return false;
  rtc_epoch_base = epoch;
  rtc_ms_start = esp_timer_get_time() / 1000;
  rtc_last_update = rtc_ms_start;
  rtc_valid = true;
  return true;
}

static void rtc_update_strings() {
  if (!rtc_valid) return;
  struct tm t;
  time_t now = rtc_epoch_base + (esp_timer_get_time() / 1000 - rtc_ms_start) / 1000;
  localtime_r(&now, &t);
  char buf_date[32];
  snprintf(buf_date, sizeof(buf_date), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
  g_date = buf_date;
  char buf_time[16];
  snprintf(buf_time, sizeof(buf_time), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
  g_time = buf_time;
}

int64_t rtc_now_ms() {
  if (!rtc_valid) {
    return esp_timer_get_time() / 1000;
  }
  return (int64_t)rtc_epoch_base * 1000 + (esp_timer_get_time() / 1000 - rtc_ms_start);
}

static void rtc_tick() {
  if (!rtc_valid) {
    rtc_set_from_strings();
    if (!rtc_valid) return;
  }
  int64_t now_ms = esp_timer_get_time() / 1000;
  if (now_ms - rtc_last_update >= 1000) {
    rtc_last_update += 1000; // Increment by interval to prevent drift accumulation
    if (status_edit_idx != 5) { // keep time ticking unless editing time
      std::string old_date = g_date;
      std::string old_time = g_time;
      rtc_update_strings();
      if (ui_mode == UIMode::STATUS && status_edit_idx == -1) {
        if (old_date != g_date) {
          draw_status_line(4, std::string("Date: ") + g_date, false);
        }
        if (old_time != g_time) {
          draw_status_line(5, std::string("Time: ") + g_time, false);
        }
      }
    }
  }
}

static void update_countdown() {
  int64_t now_ms = rtc_now_ms();
  int64_t slot_idx = now_ms / 15000;
  int64_t slot_ms = now_ms % 15000;
  static int64_t last_slot_idx = -1;
  static int last_sec = -1;
  int sec = (int)(slot_ms / 1000);
  if (slot_idx != last_slot_idx || sec != last_sec) {
    float frac = (float)slot_ms / 15000.0f;
    bool even = (slot_idx % 2) == 0;
    ui_draw_countdown(frac, even);
    last_slot_idx = slot_idx;
    last_sec = sec;
  }
}

  static void menu_flash_tick() {
    if (menu_flash_idx < 0) return;
    int64_t now = rtc_now_ms();
    if (now >= menu_flash_deadline) {
      menu_flash_idx = -1;
    if (ui_mode == UIMode::MENU && !menu_long_edit && menu_edit_idx < 0) {
      draw_menu_view();
    }
  }
}

static void rx_flash_tick() {
  if (rx_flash_idx < 0) return;
  int64_t now = rtc_now_ms();
  if (now >= rx_flash_deadline) {
    rx_flash_idx = -1;
    rx_flash_deadline = 0;
    if (ui_mode == UIMode::RX) {
      ui_draw_rx();
    }
  }
}

static void apply_pending_sync() {}

static int band_number_from_name(const std::string& name) {
  int num = 0;
  for (char c : name) {
    if (c >= '0' && c <= '9') {
      num = num * 10 + (c - '0');
    } else {
      break;
    }
  }
  return num;
}

static void rebuild_active_bands() {
  std::string cleaned = g_active_band_text;
  for (char& c : cleaned) {
    if (c == ',' || c == '/' || c == '\\' || c == ';') c = ' ';
    if (c == 'm' || c == 'M') c = ' ';
  }
  std::istringstream iss(cleaned);
  std::vector<int> bands;
  int v;
  while (iss >> v) {
    if (v <= 0) continue;
    // match to g_bands by number prefix
    for (size_t i = 0; i < g_bands.size(); ++i) {
      if (band_number_from_name(g_bands[i].name) == v) {
        if (std::find(bands.begin(), bands.end(), (int)i) == bands.end()) {
          bands.push_back((int)i);
        }
        break;
      }
    }
  }
  if (bands.empty()) {
    bands.resize(g_bands.size());
    for (size_t i = 0; i < g_bands.size(); ++i) bands[i] = (int)i;
  }
  g_active_band_indices = bands;
  if (std::find(g_active_band_indices.begin(), g_active_band_indices.end(), g_band_sel) == g_active_band_indices.end()) {
    g_band_sel = g_active_band_indices[0];
  }
  // normalize text
  std::ostringstream oss;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (i) oss << ' ';
    oss << band_number_from_name(g_bands[g_active_band_indices[i]].name);
  }
  g_active_band_text = oss.str();
}

static void advance_active_band(int delta) {
  if (g_active_band_indices.empty()) rebuild_active_bands();
  if (g_active_band_indices.empty()) return;
  int pos = 0;
  for (size_t i = 0; i < g_active_band_indices.size(); ++i) {
    if (g_active_band_indices[i] == g_band_sel) { pos = (int)i; break; }
  }
  int n = (int)g_active_band_indices.size();
  pos = (pos + delta + n) % n;
  g_band_sel = g_active_band_indices[pos];
}

static void fft_waterfall_tx_tone(uint8_t tone) {
  // Map tone 0-7 to screen width and push a bright bin
  std::array<uint8_t, 240> row{};
  int pos = (int)((tone * row.size()) / 8);
  if (pos < 0) pos = 0;
  if (pos >= (int)row.size()) pos = (int)row.size() - 1;
  row[pos] = 200;
  ui_push_waterfall_row(row.data(), (int)row.size());
}

static bool is_grid4(const std::string& s) {
  if (s.size() != 4) return false;
  auto is_letter = [](char c){ return c >= 'A' && c <= 'R'; };
  auto is_digitc = [](char c){ return c >= '0' && c <= '9'; };
  return is_letter(toupper((unsigned char)s[0])) &&
         is_letter(toupper((unsigned char)s[1])) &&
         is_digitc(s[2]) &&
         is_digitc(s[3]);
}

static int parse_report_snr(const std::string& f3) {
  if (f3.empty()) return -99;
  std::string s = f3;
  if (!s.empty() && (s[0] == 'R' || s[0] == 'r')) {
    s = s.substr(1);
  }
  if (s.empty()) return -99;
  bool neg = false;
  size_t idx = 0;
  if (s[0] == '+' || s[0] == '-') {
    neg = (s[0] == '-');
    idx = 1;
  }
  int val = 0;
  bool found = false;
  for (; idx < s.size(); ++idx) {
    char c = s[idx];
    if (c < '0' || c > '9') break;
    val = val * 10 + (c - '0');
    found = true;
    if (val > 99) break;
  }
  if (!found) return -99;
  if (neg) val = -val;
  return val;
}

void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui) {
  const int max_cand = 50;
  ftx_candidate_t candidates[max_cand];
  int num_candidates = ftx_find_candidates(&mon->wf, max_cand, candidates, 5);
  ESP_LOGI(TAG, "Candidates found: %d", num_candidates);

  // Estimate noise floor in dB from the waterfall (mean of all bins in slot)
  float noise_db = -120.0f;
  if (mon->wf.mag && mon->wf.num_blocks > 0) {
    const size_t total = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
    uint32_t hist[256] = {0};
    for (size_t i = 0; i < total; ++i) {
      hist[mon->wf.mag[i]]++;
    }
    uint64_t target = total / 3;  // 33th percentile to avoid signal bias
    uint64_t accum = 0;
    int noise_scaled = 0;
    for (int v = 0; v < 256; ++v) {
      accum += hist[v];
      if (accum >= target) {
        noise_scaled = v;
        break;
      }
    }
    noise_db = 0.5f * ((float)noise_scaled - 240.0f);  // scaled back to dB
  }

  int slot_id = 0;
  if (g_decode_slot_idx >= 0) {
    slot_id = (int)(g_decode_slot_idx & 1);
  } else {
    int64_t now_ms = rtc_now_ms();
    slot_id = (int)((now_ms / 15000LL) & 1);
  }

  auto to_upper = [](const std::string& s) {
    std::string out = s;
    for (auto& ch : out) ch = toupper((unsigned char)ch);
    return out;
  };
  std::string mycall_up = to_upper(g_call);

  auto fill_fields = [](UiRxLine& line, const std::string& text,
                        const char* call_to, const char* call_de, const char* extra) {
    // Prefer library-provided fields when present
    if (call_to && call_de && extra &&
        call_to[0] != '\0' && call_de[0] != '\0') {
      line.field1 = call_to;
      line.field2 = call_de;
      line.field3 = extra ? extra : "";
      return;
    }
    // Fallback heuristic split
    std::vector<std::string> toks;
    {
      std::istringstream iss(text);
      std::string tok;
      while (iss >> tok) toks.push_back(tok);
    }
    auto is_digits = [](const std::string& s) {
      return !s.empty() && std::all_of(s.begin(), s.end(),
        [](char c){ return c >= '0' && c <= '9'; });
    };
    auto is_alpha = [](const std::string& s) {
      return !s.empty() && std::all_of(s.begin(), s.end(),
        [](char c){ return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); });
    };
    if (!toks.empty() && toks[0] == "CQ" && toks.size() >= 2) {
      bool short_token = (toks[1].size() <= 3 && is_digits(toks[1])) ||
                         (toks[1].size() <= 4 && is_alpha(toks[1]));
      if (short_token) {
        // CQ <num/word> CALL GRID
        line.field1 = toks[1];
        if (toks.size() > 2) line.field2 = toks[2];
        if (toks.size() > 3) line.field3 = toks[3];
        return;
      }
    }
    if (!toks.empty()) line.field1 = toks[0];
    if (toks.size() > 1) line.field2 = toks[1];
    if (toks.size() > 2) line.field3 = toks[2];
  };

    std::vector<UiRxLine> ui_lines;
    std::vector<float> time_offsets;
    if (num_candidates > 0) {
      int decodedCount = 0;
      std::unordered_map<std::string, int> seen_idx; // text -> index in ui_lines
      for (int i = 0; i < num_candidates; ++i) {
        ftx_message_t message;
      ftx_decode_status_t status;
      memset(&message, 0, sizeof(message));
      memset(&status, 0, sizeof(status));

      if (!ftx_decode_candidate(&mon->wf, &candidates[i], 25, &message, &status)) {
        continue;
      }

      char text[40] = {0};
      char call_to[14] = {0};
      char call_de[14] = {0};
      char extra[10] = {0};
      ftx_field_t fields[FTX_MAX_MESSAGE_FIELDS];
      ftx_message_rc_t rc = ftx_message_decode_std(&message, NULL, call_to, call_de, extra, fields);
      if (rc == FTX_MESSAGE_RC_OK) {
        snprintf(text, sizeof(text), "%s %s %s", call_to, call_de, extra);
      } else {
        ftx_message_decode_free(&message, text);
      }

      if (text[0] != '\0') {
        float freq_hz = (mon->min_bin + candidates[i].freq_offset +
                         candidates[i].freq_sub / (float)cfg->freq_osr) / mon->symbol_period;
        float time_s = (candidates[i].time_offset +
                        candidates[i].time_sub / (float)cfg->time_osr) * mon->symbol_period;
        // Approximate candidate power from waterfall bin
        float cand_db = noise_db;
        {
          int t_index = candidates[i].time_offset * mon->wf.time_osr + candidates[i].time_sub;
          int f_index = candidates[i].freq_offset * mon->wf.freq_osr + candidates[i].freq_sub;
          size_t offset = (size_t)t_index * (size_t)mon->wf.block_stride + (size_t)f_index;
          size_t total = (size_t)mon->wf.num_blocks * (size_t)mon->wf.block_stride;
          if (mon->wf.mag && offset < total) {
            int scaled = mon->wf.mag[offset];
            cand_db = 0.5f * ((float)scaled - 240.0f);
          }
        }
        float snr_db = cand_db - noise_db;
        // Quantize to even integers and clamp to WSJT-like range [-30, 32]
        int snr_q = (int)lrintf(snr_db / 2.0f) * 2;
        if (snr_q < -30) snr_q = -30;
        if (snr_q > 32) snr_q = 32;

        ESP_LOGI(TAG, "Decoded[%d] t=%.2fs f=%.1fHz snr=%d : %s",
                 decodedCount, time_s, freq_hz, snr_q, text);
        // Deduplicate exact text but keep the highest SNR
        std::string text_str(text);
        auto it = seen_idx.find(text_str);
        if (it != seen_idx.end()) {
          int idx = it->second;
          if (snr_q > ui_lines[idx].snr) {
            ui_lines[idx].snr = snr_q;
            ui_lines[idx].offset_hz = (int)lrintf(freq_hz);
            ui_lines[idx].slot_id = slot_id;
            // Refresh parsed fields from the stronger decode
            fill_fields(ui_lines[idx], ui_lines[idx].text,
                        rc == FTX_MESSAGE_RC_OK ? call_to : nullptr,
                        rc == FTX_MESSAGE_RC_OK ? call_de : nullptr,
                        rc == FTX_MESSAGE_RC_OK ? extra : nullptr);
            if (!mycall_up.empty() && to_upper(ui_lines[idx].field1) == mycall_up) {
            // active call tracking disabled
          }
          }
          continue;
        }

          UiRxLine line;
          line.text = text;
          line.snr = snr_q;
          line.offset_hz = (int)lrintf(freq_hz);
          line.slot_id = slot_id;
          time_offsets.push_back(time_s);
          fill_fields(line, line.text, rc == FTX_MESSAGE_RC_OK ? call_to : nullptr,
                      rc == FTX_MESSAGE_RC_OK ? call_de : nullptr,
                    rc == FTX_MESSAGE_RC_OK ? extra : nullptr);
        if (line.text.rfind("CQ ", 0) == 0 || line.text == "CQ") {
          line.is_cq = true;
        }
        std::string f1_up = to_upper(line.field1);
        if (!mycall_up.empty() && f1_up == mycall_up) {
          line.is_to_me = true;
          // active call tracking disabled
        }
        ui_lines.push_back(line);
        seen_idx[text_str] = (int)ui_lines.size() - 1;
        log_rxtx_line('R', snr_q, (int)lrintf(freq_hz), text_str, -1);
        decodedCount++;
        if (decodedCount >= 32) break; // safety cap
      }
    }
    if (decodedCount == 0) {
      ESP_LOGW(TAG, "Candidates present but no messages decoded");
    }
  } else {
    ESP_LOGW(TAG, "No candidates found");
  }

  // Auto sync: if >3 decodes, use median time offset and apply immediately (bounded)
    if (time_offsets.size() > 3) {
      std::vector<float> tmp = time_offsets;
      std::sort(tmp.begin(), tmp.end());
      float median = tmp[tmp.size() / 2];
      if (std::fabs(median) > 0.3f) {
        int delta_ms = (int)lrintf(-median * 1000.0f);
        if (delta_ms > 320) delta_ms = 320;
        if (delta_ms < -320) delta_ms = -320;
        rtc_ms_start -= delta_ms;
        rtc_last_update -= delta_ms;
        rtc_update_strings();
        ESP_LOGI("SYNC", "Applied RTC sync: median=%.2fs delta=%dms", median, delta_ms);
      } else {
        ESP_LOGD("SYNC", "Median=%.2fs within threshold; no sync", median);
      }
    }

  // Sort into reply-to-me, CQ, other; preserve order within each group
  std::vector<UiRxLine> to_me;
  std::vector<UiRxLine> cqs;
  std::vector<UiRxLine> others;
  std::string mycall = g_call;
  for (auto& ch : mycall) ch = toupper((unsigned char)ch);
  for (auto& l : ui_lines) {
    std::string f1 = l.field1;
    for (auto& ch : f1) ch = toupper((unsigned char)ch);
    if (!mycall.empty() && !f1.empty() && f1 == mycall) {
      l.is_to_me = true;
      to_me.push_back(l);
    } else if (l.is_cq) {
      cqs.push_back(l);
    } else {
      others.push_back(l);
    }
  }

  // Auto-sequencing for reply-to-me messages
  if (!to_me.empty()) {
    // Feed all to-me messages to autoseq
    autoseq_on_decodes(to_me);

    // Update last reply text to avoid re-processing
    g_last_reply_text = to_me.front().text;
  }

  // Always try to schedule TX after decode - handles both responses and retries
  // This is the main trigger for TX scheduling after the decode window closes
  if (autoseq_has_active_qso()) {
    schedule_tx_if_idle();
  }

  std::vector<UiRxLine> merged;
  merged.reserve(to_me.size() + cqs.size() + others.size());
  merged.insert(merged.end(), to_me.begin(), to_me.end());
  merged.insert(merged.end(), cqs.begin(), cqs.end());
  merged.insert(merged.end(), others.begin(), others.end());
  if (merged.size() > 12) merged.resize(12);

  g_rx_lines = merged;
  if (update_ui) {
    ui_set_rx_list(g_rx_lines);
    ui_draw_rx();
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  } else {
    g_rx_dirty = true;
  }
}

static void draw_menu_long_edit() {
  std::vector<std::string> lines(6, "");
  std::string text = menu_long_buf;
  size_t idx = 0;
  int line = 0;
  while (idx < text.size() && line < 6) {
    size_t chunk = std::min<size_t>(18, text.size() - idx);
    lines[line] = text.substr(idx, chunk);
    idx += chunk;
    line++;
  }
  // cursor indicator on the last line
  if (line == 0) {
    lines[0] = "_";
  } else {
    if (lines[line - 1].size() < 20) lines[line - 1].push_back('_');
    else if (line < 6) lines[line] = "_";
  }
  ui_draw_list(lines, 0, -1);
}

static void log_tones(const uint8_t* tones, size_t n) {
  std::string line;
  for (size_t i = 0; i < n; ++i) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", (unsigned)tones[i]);
    line += buf;
    if ((i + 1) % 20 == 0 || i + 1 == n) {
      debug_log_line(line);
      line.clear();
    }
  }
}

static void encode_and_log_pending_tx() {
  if (!g_pending_tx_valid || g_pending_tx.text.empty()) {
    debug_log_line("No pending TX to encode");
    return;
  }
  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, NULL, g_pending_tx.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    debug_log_line("Encode failed");
    return;
  }
  uint8_t tones[79] = {0};
  ft8_encode(msg.payload, tones);
  debug_log_line(std::string("Tones for '") + g_pending_tx.text + "'");
  log_tones(tones, 79);
}

static bool looks_like_grid(const std::string& s) {
  if (s.size() != 4) return false;
  return std::isalpha((unsigned char)s[0]) && std::isalpha((unsigned char)s[1]) &&
         std::isdigit((unsigned char)s[2]) && std::isdigit((unsigned char)s[3]);
}

static bool looks_like_report(const std::string& s, int& out) {
  if (s.empty()) return false;
  int sign = 1;
  size_t idx = 0;
  if (s[0] == '-') { sign = -1; idx = 1; }
  else if (s[0] == '+') { idx = 1; }
  if (idx >= s.size()) return false;
  int val = 0;
  for (; idx < s.size(); ++idx) {
    if (!std::isdigit((unsigned char)s[idx])) return false;
    val = val * 10 + (s[idx] - '0');
  }
  out = sign * val;
  return true;
}

static void maybe_enqueue_beacon() {
  if (g_beacon == BeaconMode::OFF) return;
  if (autoseq_has_active_qso()) return;  // Don't beacon during active QSO

  int64_t now_ms = rtc_now_ms();
  int64_t now_slot = now_ms / 15000;
  int parity = (int)(now_slot & 1);

  bool allow = false;
  switch (g_beacon) {
    case BeaconMode::EVEN: allow = (parity == 0); break;
    case BeaconMode::ODD:  allow = (parity == 1); break;
    default: break;
  }
  if (!allow) return;
  if (now_slot == last_beacon_slot) return;

  // Use autoseq to start CQ
  autoseq_start_cq();
  last_beacon_slot = now_slot;
  debug_log_line("Beacon CQ queued");
}

struct TxTaskContext {
  AutoseqTxEntry e;
  int delay_ms;
  int64_t slot_idx;
  int skip_tones;
};

static void tx_send_task(void* param);
static int64_t s_last_tx_slot_idx = -1000;

static void schedule_tx_if_idle() {
  // Use critical section to atomically check and set task handle
  taskENTER_CRITICAL(&mux);
  if (s_tx_task_handle != NULL) {
    taskEXIT_CRITICAL(&mux);
    return;
  }
  // Mark as "about to create" by setting a sentinel value
  s_tx_task_handle = (TaskHandle_t)1;
  taskEXIT_CRITICAL(&mux);

  // Try beacon first
  maybe_enqueue_beacon();

  // Get timing info
  int64_t now_ms = rtc_now_ms();
  int64_t now_slot = now_ms / 15000;
  int64_t slot_ms = now_ms % 15000;
  int now_parity = (int)(now_slot & 1);

  // NOTE: Don't call autoseq_tick here! Tick is for retry management and
  // should only be called AFTER TX completes. Here we just fetch current next_tx.

  // Try to fetch pending TX from autoseq (reads current state, doesn't modify)
  AutoseqTxEntry pending;
  if (!autoseq_fetch_pending_tx(pending)) {
    s_tx_task_handle = NULL;
    g_pending_tx_valid = false;
    return;
  }

  g_pending_tx = pending;
  g_pending_tx_valid = true;

  int target_parity = pending.slot_id & 1;
  bool can_inline = (target_parity == now_parity) && (slot_ms < 4000) && (now_slot >= s_last_tx_slot_idx + 2);

  int64_t slot_idx = now_slot;
  int64_t delay_ms = 0;
  int skip_tones = 0;
  if (can_inline) {
    // start in current slot, skip elapsed symbols
    skip_tones = (int)(slot_ms / 160);
    if (skip_tones >= 79) {
      s_tx_task_handle = NULL;
      return;
    }
    delay_ms = 0;
  } else {
    if (slot_ms != 0) slot_idx += 1;
    if (slot_idx <= s_last_tx_slot_idx + 1) {
      slot_idx = s_last_tx_slot_idx + 2;
    }
    while ((slot_idx & 1) != target_parity) {
      slot_idx += 1;
    }
    delay_ms = slot_idx * 15000 - now_ms;
  }

  auto* ctx = new TxTaskContext{pending, (int)delay_ms, slot_idx, skip_tones};
  xTaskCreatePinnedToCore(tx_send_task, "tx_sched", 4096, ctx, 5, &s_tx_task_handle, 0);
}

static void tx_send_task(void* param) {
  std::unique_ptr<TxTaskContext> ctx(reinterpret_cast<TxTaskContext*>(param));
  if (ctx->delay_ms > 0) vTaskDelay(pdMS_TO_TICKS(ctx->delay_ms));

  const AutoseqTxEntry& e = ctx->e;
  if (e.text.empty()) {
    s_tx_task_handle = NULL;
    schedule_tx_if_idle();
    vTaskDelete(NULL);
    return;
  }

  ftx_message_t msg;
  ftx_message_rc_t rc = ftx_message_encode(&msg, NULL, e.text.c_str());
  if (rc != FTX_MESSAGE_RC_OK) {
    ESP_LOGE(TAG, "Encode failed for TX");
    s_tx_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }
  uint8_t tones[79] = {0};
  ft8_encode(msg.payload, tones);
  log_rxtx_line('T', 0, e.offset_hz, e.text, e.repeat_counter);

  // CAT: compute base TX audio offset
  int base_hz = 0;
  auto rand_off = []() {
    uint32_t r = esp_random();
    return 500 + (int)(r % 2001); // 500-2500
  };
  if (g_offset_src == OffsetSrc::CURSOR) {
    base_hz = g_offset_hz;
  } else if (g_offset_src == OffsetSrc::RX && e.offset_hz > 0 && e.text.rfind("CQ ", 0) != 0) {
    base_hz = e.offset_hz;
  } else {
    base_hz = rand_off();
  }

  bool cat_ok = cat_cdc_ready();
  if (cat_ok) {
    const char* md = "MD6;";
    const char* tx = "TX;";
    cat_cdc_send(reinterpret_cast<const uint8_t*>(md), strlen(md), 200);
    cat_cdc_send(reinterpret_cast<const uint8_t*>(tx), strlen(tx), 200);
  }

  int start_tone = ctx->skip_tones;
  if (start_tone >= 79) start_tone = 79;
  if (ctx->skip_tones > 0) {
    ESP_LOGI("TXTONE", "Skipping first %d tones due to late start", ctx->skip_tones);
  }
  int last_ta_int = -1;
  int last_ta_frac = -1;
  auto send_ta = [&](float tone_hz) {
    int ta_int = (int)lrintf(tone_hz);
    float frac = tone_hz - (float)ta_int;
    int ta_frac = (int)lrintf(frac * 100.0f); // two decimals
    if (ta_int == last_ta_int && ta_frac == last_ta_frac) return;
    char ta[16];
    snprintf(ta, sizeof(ta), "TA%04d.%02d;", ta_int, ta_frac);
    cat_cdc_send(reinterpret_cast<const uint8_t*>(ta), strlen(ta), 10);
    last_ta_int = ta_int;
    last_ta_frac = ta_frac;
  };

  // Send first tone TA if CAT is ready
  if (cat_ok && start_tone < 79) {
    float tone_hz = base_hz + 6.25f * tones[start_tone];
    send_ta(tone_hz);
  }

  for (int i = start_tone; i < 79; ++i) {
    ESP_LOGI("TXTONE", "%02d %u", i, (unsigned)tones[i]);
    fft_waterfall_tx_tone(tones[i]);
    if (cat_ok) {
      float tone_hz = base_hz + 6.25f * tones[i];
      send_ta(tone_hz);
    }
    vTaskDelay(pdMS_TO_TICKS(160));
  }
  if (cat_ok) {
    const char* rx = "RX;";
    cat_cdc_send(reinterpret_cast<const uint8_t*>(rx), strlen(rx), 200);
  }

  // Record slot index for spacing and notify autoseq
  s_last_tx_slot_idx = ctx->slot_idx;
  autoseq_mark_sent(ctx->slot_idx);

  // Call tick to set up retry for next attempt (in case no response comes)
  // This is like the reference architecture where tick is called at slot boundary after TX
  // If a response comes, autoseq_on_decodes will update state and overwrite next_tx
  int64_t now_ms = rtc_now_ms();
  int64_t now_slot = now_ms / 15000;
  int now_parity = (int)(now_slot & 1);
  autoseq_tick(now_slot, now_parity, 0);

  g_pending_tx_valid = false;
  redraw_tx_view();
  s_tx_task_handle = NULL;
  // NOTE: Don't call schedule_tx_if_idle here - wait for decode window to close first!
  // The decode handler will schedule next TX after processing responses.
  vTaskDelete(NULL);
}

static void draw_menu_view() {
    if (menu_long_edit) {
      draw_menu_long_edit();
      return;
    }
  std::vector<std::string> lines;
  lines.reserve(12);

  std::string cq_line = std::string("CQ Type:");
  if (g_cq_type == CqType::CQFREETEXT) cq_line += g_cq_freetext;
  else cq_line += cq_type_name(g_cq_type);
  lines.push_back(cq_line);
  lines.push_back("Send FreeText");
  lines.push_back(std::string("F:") + head_trim(menu_edit_idx == 3 ? menu_edit_buf : g_free_text, 16));
  lines.push_back(std::string("Call:") + elide_right(menu_edit_idx == 4 ? menu_edit_buf : g_call));
  lines.push_back(std::string("Grid:") + elide_right(menu_edit_idx == 5 ? menu_edit_buf : g_grid));
  lines.push_back(std::string("Sleep:") + (M5.Power.isCharging() ? "press" : "USB?"));

  lines.push_back(std::string("Offset:") + offset_name(g_offset_src));
  if (menu_edit_idx == 7) {
    lines.push_back(std::string("Cursor:") + menu_edit_buf);
  } else {
    lines.push_back(std::string("Cursor:") + std::to_string(g_offset_hz));
  }
  lines.push_back(std::string("Radio:") + radio_name(g_radio));
  lines.push_back(std::string("Antenna:") + elide_right(menu_edit_idx == 8 ? menu_edit_buf : g_ant));
  lines.push_back(std::string("C:") + head_trim(menu_edit_idx == 9 ? menu_edit_buf : expand_comment1(), 16));
  lines.push_back(battery_status_line());

  // Page 2 content (index 12+)
  lines.push_back(std::string("RxTxLog:") + (g_rxtx_log ? "ON" : "OFF"));
  lines.push_back(std::string("SkipTX1:") + (g_skip_tx1 ? "ON" : "OFF"));
  lines.push_back(std::string("ActiveBand:") + head_trim(g_active_band_text, 16));
  lines.push_back(""); // padding
  lines.push_back(""); // padding
  lines.push_back(""); // padding

  int highlight_abs = -1;
  int64_t now = rtc_now_ms();
  if (menu_edit_idx >= 0) {
    highlight_abs = menu_edit_idx;
  } else if (menu_flash_idx >= 0 && now < menu_flash_deadline) {
    highlight_abs = menu_flash_idx;
  } else {
    menu_flash_idx = -1;
  }
  // Auto-clear flash after timeout
  if (menu_flash_idx >= 0 && now >= menu_flash_deadline) {
    menu_flash_idx = -1;
  }
  ui_draw_list(lines, menu_page, highlight_abs);
  // Draw battery icon on visible battery line
  int battery_abs_idx = (int)lines.size() - 1;
  if (menu_page == (battery_abs_idx / 6)) {
    int line_on_page = battery_abs_idx % 6;
    const int line_h = 19;
    const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
    (void)line_on_page;
    (void)line_h;
    (void)start_y;
    //int y = start_y + line_on_page * line_h + 3;
    //int level = (int)M5.Power.getBatteryLevel();
    //bool charging = M5.Power.isCharging();
    //draw_battery_icon(190, y, 24, 12, level, charging);
  }
}

static void draw_status_view() {
  std::string lines[6];
  BeaconMode disp_beacon = (ui_mode == UIMode::STATUS) ? g_status_beacon_temp : g_beacon;
  lines[0] = std::string("Beacon: ") + beacon_name(disp_beacon);
  if (uac_is_streaming()) {
    lines[1] = std::string("Sync to ") + radio_name(g_radio);
  } else {
    lines[1] = std::string("Connect to ") + radio_name(g_radio);
  }
  lines[2] = std::string("Band: ") +
             std::string(g_bands[g_band_sel].name) + " " +
             std::to_string(g_bands[g_band_sel].freq);
  lines[3] = std::string("Tune: ") + (g_tune ? "ON" : "OFF");
  if (status_edit_idx == 4 && !status_edit_buffer.empty()) {
    lines[4] = std::string("Date: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[4] = std::string("Date: ") + g_date;
  }
  if (status_edit_idx == 5 && !status_edit_buffer.empty()) {
    lines[5] = std::string("Time: ") + highlight_pos(status_edit_buffer, status_cursor_pos);
  } else {
    lines[5] = std::string("Time: ") + g_time;
  }
  for (int i = 0; i < 6; ++i) {
    bool hl = (status_edit_idx == i);
    draw_status_line(i, lines[i], hl);
  }
}

static void debug_log_line(const std::string& msg) {
  if (g_debug_lines.size() >= DEBUG_MAX_LINES) {
    g_debug_lines.erase(g_debug_lines.begin());
  }
  g_debug_lines.push_back(msg);
  debug_page = (int)((g_debug_lines.size() - 1) / 6);
  if (ui_mode == UIMode::DEBUG) {
    ui_draw_debug(g_debug_lines, debug_page);
  }
}

#if ENABLE_BLE

static void host_send_bt(const std::string& s)
{
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    if (!gatt_tx_handle) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(s.data(), s.size());
    if (!om) return;

    ble_gatts_notify_custom(g_conn_handle, gatt_tx_handle, om);
}

static void ble_on_sync(void);

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "ensure addr failed: %d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "infer auto addr failed: %d", rc);
        return;
    }
    uint8_t addr_val[6];
    ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
    ESP_LOGI(BT_TAG, "Sync, address type %d, addr %02x:%02x:%02x:%02x:%02x:%02x",
             g_own_addr_type,
             addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    ble_app_advertise();
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv{};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t*)"Mini-FT8";
    fields.name_len = strlen("Mini-FT8");
    fields.name_is_complete = 1;

    ble_gap_adv_stop();  // safe if not advertising
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "adv_set_fields failed: %d", rc);
        return;
    }
    rc = ble_gap_adv_start(g_own_addr_type, nullptr,
                           BLE_HS_FOREVER,
                           &adv, gap_cb, nullptr);
    if (rc != 0) {
        ESP_LOGE(BT_TAG, "adv_start failed: %d", rc);
    } else {
        ESP_LOGI(BT_TAG, "Advertising as Mini-FT8");
    }
}

#else  // ENABLE_BLE
static void host_send_bt(const std::string& s) { (void)s; }
static void init_bluetooth(void) {}
static void poll_ble_uart() {}
#endif // ENABLE_BLE

static std::string trim_copy(const std::string& s) {
  size_t b = 0, e = s.size();
  while (b < e && isspace((unsigned char)s[b])) ++b;
  while (e > b && isspace((unsigned char)s[e - 1])) --e;
  return s.substr(b, e - b);
}

static uint32_t parse_crc_hex(const std::string& hex) {
  if (hex.empty()) return 0;
  char* end = nullptr;
  unsigned long v = strtoul(hex.c_str(), &end, 16);
  if (end == hex.c_str() || *end != '\0') return 0;
  return (uint32_t)v;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  crc = crc ^ 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static void host_debug_hex8(const char* prefix, const uint8_t* b) {
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%s ", prefix);
  for (int i = 0; i < 8 && n + 3 < (int)sizeof(buf); ++i) {
    n += snprintf(buf + n, sizeof(buf) - n, "%02X ", b[i]);
  }
  if (n > 0 && buf[n - 1] == ' ') buf[n - 1] = 0;
  host_write_str(std::string(buf) + "\r\n");
}

static void host_handle_line(const std::string& line_in) {
  bool send_prompt = true;
  std::string line = trim_copy(line_in);
  if (line.empty()) { host_write_str(HOST_PROMPT); return; }
  debug_log_line(std::string("[HOST RX] ") + line);
  std::string echo = std::string("ECHO: ") + line + "\r\n";
  host_write_str(echo);

  auto to_upper = [](std::string s) {
    for (auto& c : s) c = toupper((unsigned char)c);
    return s;
  };
  std::istringstream iss(line);
  std::string cmd;
  iss >> cmd;
  std::string cmd_up = to_upper(cmd);
  std::string rest;
  std::getline(iss, rest);
  rest = trim_copy(rest);

  auto send = [](const std::string& msg) { host_write_str(msg + "\r\n"); };

  if (cmd_up == "WRITE" || cmd_up == "APPEND") {
    std::istringstream rs(rest);
    std::string fname;
    rs >> fname;
    std::string content;
    std::getline(rs, content);
    content = trim_copy(content);
    if (fname.empty()) {
      send("ERROR: filename required");
    } else {
      std::string path = std::string("/spiffs/") + fname;
      const char* mode = (cmd_up == "WRITE") ? "w" : "a";
      FILE* f = fopen(path.c_str(), mode);
      if (!f) send("ERROR: open failed");
      else { fwrite(content.data(), 1, content.size(), f); fclose(f); send("OK"); }
    }
  } else if (cmd_up == "READ") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      std::string path = std::string("/spiffs/") + rest;
      FILE* f = fopen(path.c_str(), "r");
      if (!f) send("ERROR: open failed");
      else {
        char buf[128];
        while (fgets(buf, sizeof(buf), f)) host_write_str(std::string(buf));
        fclose(f);
        send("OK");
      }
    }
  } else if (cmd_up == "DELETE") {
    if (rest.empty()) send("ERROR: filename required");
    else {
      std::string path = std::string("/spiffs/") + rest;
      if (unlink(path.c_str()) == 0) send("OK"); else send("ERROR: delete failed");
    }
  } else if (cmd_up == "LIST") {
    DIR* d = opendir("/spiffs");
    if (!d) send("ERROR: opendir failed");
    else {
      struct dirent* ent;
      while ((ent = readdir(d)) != nullptr) {
        send(ent->d_name);
      }
      closedir(d);
      send("OK");
    }
  } else if (cmd_up == "WRITEBIN") {
    std::istringstream rs(rest);
    std::string fname;
    size_t size = 0;
    std::string crc_hex;
    rs >> fname >> size >> crc_hex;
    uint32_t crc_exp = parse_crc_hex(crc_hex);
    if (fname.empty() || size == 0 || crc_hex.empty()) {
      send("ERROR: filename, size, crc32_hex required");
    } else if (host_bin_active) {
      send("ERROR: binary upload in progress");
    } else {
      std::string path = std::string("/spiffs/") + fname;
      FILE* f = fopen(path.c_str(), "wb");
      if (!f) {
        send("ERROR: open failed");
      } else {
        host_bin_path = path;
        host_bin_active = true;
        host_bin_remaining = size;
        host_bin_fp = f;
        host_bin_crc = 0;
        host_bin_expected_crc = crc_exp;
        host_bin_received = 0;
        host_bin_buf.clear();
        host_bin_buf.reserve(HOST_BIN_CHUNK);
        host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        host_bin_first_filled = 0;
        memset(host_bin_first8, 0, sizeof(host_bin_first8));
        memset(host_bin_last8, 0, sizeof(host_bin_last8));
        host_write_str("OK: send " + std::to_string(size) + " bytes, chunk " + std::to_string(HOST_BIN_CHUNK) + " +4crc\r\n");
        send_prompt = false; // prompt after binary upload completes
      }
    }
  } else if (cmd_up == "INFO") {
    send("Heap: " + std::to_string(heap_caps_get_free_size(MALLOC_CAP_DEFAULT)));
    send("OK");
  } else if (cmd_up == "HELP") {
    for (auto& l : g_ctrl_lines) send(l);
    send("RX decode: press 'x' in RX to stream /kfs40m.wav");
  } else if (cmd_up == "EXIT") {
    send("OK: exit host");
    enter_mode(UIMode::RX);
    return;
  } else {
    send("ERROR: Unknown command. Type HELP.");
  }

  if (send_prompt) host_write_str(std::string(HOST_PROMPT));
}

static void host_process_bytes(const uint8_t* buf, size_t len) {
  ESP_LOGI(TAG, "host_process_bytes len=%u", (unsigned)len);
  for (size_t i = 0; i < len; ) {
    if (host_bin_active) {
      // Skip any stray CR/LF before first payload byte
      if (host_bin_received == 0 && host_bin_buf.empty() && (buf[i] == '\r' || buf[i] == '\n')) {
        ++i;
        continue;
      }
      size_t payload_need = host_bin_chunk_expect;
      size_t total_need = payload_need + 4; // payload + crc32 trailer
      size_t avail = len - i;
      size_t copy = total_need - host_bin_buf.size();
      if (copy > avail) copy = avail;
      host_bin_buf.insert(host_bin_buf.end(), buf + i, buf + i + copy);
      i += copy;

      if (host_bin_buf.size() >= total_need) {
        size_t payload_len = payload_need;
        uint32_t recv_crc = (uint32_t(host_bin_buf[payload_len])) |
                            (uint32_t(host_bin_buf[payload_len + 1]) << 8) |
                            (uint32_t(host_bin_buf[payload_len + 2]) << 16) |
                            (uint32_t(host_bin_buf[payload_len + 3]) << 24);
        uint32_t calc_crc = crc32_update(0, host_bin_buf.data(), payload_len);
        if (calc_crc != recv_crc) {
          char dbg[128];
          snprintf(dbg, sizeof(dbg), "ERROR: chunk crc off=%u len=%u calc=%08X recv=%08X\r\n",
                   (unsigned)(host_bin_received + payload_len), (unsigned)payload_len,
                   (unsigned)calc_crc, (unsigned)recv_crc);
          host_write_str(std::string(dbg));
          // Send first/last bytes of the chunk to compare
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK FIRST8", host_bin_buf.data());
          if (payload_len >= 8) host_debug_hex8("DBG CHUNK LAST8", host_bin_buf.data() + payload_len - 8);
          if (payload_len < 8) host_debug_hex8("DBG CHUNK PART", host_bin_buf.data());
          // Also report the CRC trailer bytes as seen
          uint8_t crc_bytes[4] = {
            host_bin_buf[payload_len],
            host_bin_buf[payload_len + 1],
            host_bin_buf[payload_len + 2],
            host_bin_buf[payload_len + 3]
          };
          host_debug_hex8("DBG CRC BYTES", crc_bytes);
          fclose(host_bin_fp);
          host_bin_fp = nullptr;
          host_bin_active = false;
          host_bin_remaining = 0;
          host_bin_buf.clear();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }

        // Capture first/last bytes for debugging
        if (host_bin_first_filled < 8) {
          size_t need = 8 - host_bin_first_filled;
          if (need > payload_len) need = payload_len;
          memcpy(host_bin_first8 + host_bin_first_filled, host_bin_buf.data(), need);
          host_bin_first_filled += need;
        }
        // update last8 buffer
        if (payload_len >= 8) {
          memcpy(host_bin_last8, host_bin_buf.data() + payload_len - 8, 8);
        } else {
          // shift existing and append
          size_t shift = (payload_len + 8 > 8) ? (payload_len) : payload_len;
          if (shift > 0) {
            memmove(host_bin_last8, host_bin_last8 + shift, 8 - shift);
            memcpy(host_bin_last8 + (8 - payload_len), host_bin_buf.data(), payload_len);
          }
        }

        size_t written = fwrite(host_bin_buf.data(), 1, payload_len, host_bin_fp);
        if (written != payload_len) {
          host_write_str("ERROR: write failed\r\n");
          fclose(host_bin_fp);
          host_bin_fp = nullptr;
          host_bin_active = false;
          host_bin_remaining = 0;
          host_bin_buf.clear();
          host_write_str(std::string(HOST_PROMPT));
          continue;
        }
        host_bin_crc = crc32_update(host_bin_crc, host_bin_buf.data(), payload_len);
        host_bin_remaining -= payload_len;
        host_bin_received += payload_len;
        host_bin_buf.clear();
        host_write_str("ACK " + std::to_string(host_bin_received) + "\r\n");

        if (host_bin_remaining == 0) {
          fclose(host_bin_fp);
          host_bin_fp = nullptr;
          host_bin_active = false;
          uint32_t crc_final = host_bin_crc;
          // Reopen file to send first/last 8 bytes for debugging
          host_debug_hex8("DBG FIRST8", host_bin_first8);
          host_debug_hex8("DBG LAST8", host_bin_last8);
          char crc_line[64];
          snprintf(crc_line, sizeof(crc_line), "DBG CRC %08X EXPECT %08X\r\n",
                   (unsigned)crc_final, (unsigned)host_bin_expected_crc);
          host_write_str(std::string(crc_line));
          if (crc_final != host_bin_expected_crc) {
            host_write_str("ERROR: crc mismatch\r\n");
          } else {
            host_write_str("OK crc " + std::to_string(crc_final) + "\r\n");
          }
          host_write_str(std::string(HOST_PROMPT));
        } else {
          host_bin_chunk_expect = (host_bin_remaining < HOST_BIN_CHUNK) ? host_bin_remaining : HOST_BIN_CHUNK;
        }
      }
      continue;
    }
    char ch = (char)buf[i++];
    if (ch == '\r' || ch == '\n') {
      if (!host_input.empty()) {
    ESP_LOGI(TAG, "HOST line: %s", host_input.c_str());
        host_handle_line(host_input);
        host_input.clear();
      } else {
        host_write_str(std::string(HOST_PROMPT));
      }
    } else if (ch == 0x08 || ch == 0x7f) {
      if (!host_input.empty()) host_input.pop_back();
    } else if (ch >= 32 && ch < 127) {
      host_input.push_back(ch);
    }
  }
}

static void poll_host_uart() {
  ensure_usb();
  if (!usb_ready) return;
  uint8_t buf[512];
  while (true) {
    int r = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    if (r <= 0) break;
    host_process_bytes(buf, (size_t)r);
  }
}

#if ENABLE_BLE
static void poll_ble_uart() {
  if (!ble_rx_queue) return;
  uint8_t buf[256];
  size_t n = 0;
  uint8_t b = 0;
  while (xQueueReceive(ble_rx_queue, &b, 0) == pdTRUE) {
    buf[n++] = b;
    if (n == sizeof(buf)) {
      ESP_LOGI(BT_TAG, "BLE RX chunk %u bytes", (unsigned)n);
      host_process_bytes(buf, n);
      // If no newline was seen, synthesize one to flush short commands over BLE.
      if (!host_bin_active && n > 0 && buf[n - 1] != '\n' && buf[n - 1] != '\r') {
        const uint8_t nl = '\n';
        host_process_bytes(&nl, 1);
      }
      n = 0;
    }
  }
  if (n > 0) {
    ESP_LOGI(BT_TAG, "BLE RX chunk %u bytes", (unsigned)n);
    host_process_bytes(buf, n);
    if (!host_bin_active && buf[n - 1] != '\n' && buf[n - 1] != '\r') {
      const uint8_t nl = '\n';
      host_process_bytes(&nl, 1);
    }
  }
}

#endif // ENABLE_BLE

static void load_station_data() {
  FILE* f = fopen(STATION_FILE, "r");
  if (!f) return;
  char line[64];
  while (fgets(line, sizeof(line), f)) {
    int idx = -1;
    int val = 0;
    if (sscanf(line, "band%d=%d", &idx, &val) == 2) {
      if (idx >= 0 && idx < (int)g_bands.size()) {
        g_bands[idx].freq = val;
      }
    } else if (sscanf(line, "beacon=%d", &val) == 1) {
      // beacon persists OFF only; ignore saved value
    } else if (sscanf(line, "offset=%d", &val) == 1) {
      g_offset_hz = val;
    } else if (sscanf(line, "band_sel=%d", &val) == 1) {
      if (val >= 0 && val < (int)g_bands.size()) g_band_sel = val;
    } else if (sscanf(line, "date=%63s", line) == 1) {
      g_date = line;
    } else if (sscanf(line, "time=%63s", line) == 1) {
      g_time = line;
    } else if (sscanf(line, "cq_type=%d", &val) == 1) {
      if (val >= 0 && val <= 5) g_cq_type = (CqType)val;
    } else if (sscanf(line, "offset_src=%d", &val) == 1) {
      if (val >= 0 && val <= 2) g_offset_src = (OffsetSrc)val;
    } else if (sscanf(line, "radio=%d", &val) == 1) {
      if (val >= 0 && val <= 3) g_radio = (RadioType)val;
    } else if (strncmp(line, "cq_ft=", 6) == 0) {
      g_cq_freetext = trim_copy(line + 6);
    } else if (strncmp(line, "free_text=", 10) == 0) {
      g_free_text = trim_copy(line + 10);
    } else if (strncmp(line, "call=", 5) == 0) {
      g_call = trim_copy(line + 5);
    } else if (strncmp(line, "grid=", 5) == 0) {
      g_grid = trim_copy(line + 5);
    } else if (strncmp(line, "ant=", 4) == 0) {
      g_ant = trim_copy(line + 4);
    } else if (strncmp(line, "comment1=", 9) == 0) {
      g_comment1 = trim_copy(line + 9);
    } else if (sscanf(line, "rxtx_log=%d", &val) == 1) {
      g_rxtx_log = (val != 0);
    } else if (sscanf(line, "skiptx1=%d", &val) == 1) {
      g_skip_tx1 = (val != 0);
    } else if (sscanf(line, "active_band=%d", &val) == 1) { // legacy single value
      g_active_band_text = std::to_string(val);
    } else if (strncmp(line, "active_bands=", 13) == 0) {
      g_active_band_text = trim_copy(line + 13);
    }
  }
  fclose(f);
  rtc_set_from_strings();
  rebuild_active_bands();
  g_beacon = BeaconMode::OFF; // force off on load
}

static void save_station_data() {
  FILE* f = fopen(STATION_FILE, "w");
  if (!f) {
    ESP_LOGE(TAG, "Failed to open %s for write", STATION_FILE);
    return;
  }
  for (size_t i = 0; i < g_bands.size(); ++i) {
    fprintf(f, "band%u=%d\n", (unsigned)i, g_bands[i].freq);
  }
  // Beacon is not persisted (stays OFF on reload)
  fprintf(f, "offset=%d\n", g_offset_hz);
  fprintf(f, "band_sel=%d\n", g_band_sel);
  fprintf(f, "date=%s\n", g_date.c_str());
  fprintf(f, "time=%s\n", g_time.c_str());
  fprintf(f, "cq_type=%d\n", (int)g_cq_type);
  fprintf(f, "cq_ft=%s\n", g_cq_freetext.c_str());
  fprintf(f, "skiptx1=%d\n", g_skip_tx1 ? 1 : 0);
  fprintf(f, "free_text=%s\n", g_free_text.c_str());
  fprintf(f, "call=%s\n", g_call.c_str());
  fprintf(f, "grid=%s\n", g_grid.c_str());
  fprintf(f, "offset_src=%d\n", (int)g_offset_src);
  fprintf(f, "radio=%d\n", (int)g_radio);
  fprintf(f, "ant=%s\n", g_ant.c_str());
  fprintf(f, "comment1=%s\n", g_comment1.c_str());
  fprintf(f, "rxtx_log=%d\n", g_rxtx_log ? 1 : 0);
  fprintf(f, "active_bands=%s\n", g_active_band_text.c_str());
  fclose(f);
}

static void init_soft_uart() {
  static bool initialized = false;
  if (initialized) return;
  initialized = true;

  uart_config_t cfg = {};
  cfg.baud_rate = 115200;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#ifdef UART_SCLK_REF_TICK
  cfg.source_clk = UART_SCLK_REF_TICK; // 1 MHz ref tick gives precise low baud divisors
#else
  cfg.source_clk = UART_SCLK_DEFAULT;
#endif

  ESP_ERROR_CHECK(uart_driver_install(SOFT_UART_NUM, 512, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(SOFT_UART_NUM, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(SOFT_UART_NUM, SOFT_UART_TX_PIN, SOFT_UART_RX_PIN,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  // Match known-good STM32 setup: invert TX/RX levels (idle still high at line), TX-only use
  // uart_set_line_inverse(SOFT_UART_NUM, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
  ESP_ERROR_CHECK(gpio_set_drive_capability((gpio_num_t)SOFT_UART_TX_PIN, GPIO_DRIVE_CAP_3));
  ESP_LOGI(TAG, "Soft UART ready on GPIO%d/GPIO%d @ %d baud",
           SOFT_UART_TX_PIN, SOFT_UART_RX_PIN, cfg.baud_rate);
  uart_write_bytes(SOFT_UART_NUM, "Hello World\n", 12);  //debug
}


static void enter_mode(UIMode new_mode) {
  // No special handling needed when leaving TX mode - autoseq manages queue internally
  if (ui_mode == UIMode::STATUS && new_mode != UIMode::STATUS) {
  if (g_beacon != g_status_beacon_temp) {
    g_beacon = g_status_beacon_temp;
    save_station_data();
    if (g_beacon == BeaconMode::OFF) {
      // Abort any pending TX and clear autoseq
      autoseq_clear();
      g_pending_tx = AutoseqTxEntry{};
      g_pending_tx_valid = false;
    }
  }
  status_edit_idx = -1;
  status_edit_buffer.clear();
}
  ui_mode = new_mode;
  rx_flash_idx = -1;
  switch (ui_mode) {
    case UIMode::RX:
      // Force RX list redraw
      ui_force_redraw_rx();
      ui_draw_rx();
      break;
    case UIMode::TX:
      tx_page = 0;
      redraw_tx_view();
      break;
    case UIMode::BAND:
      band_page = 0;
      band_edit_idx = -1;
      draw_band_view();
      break;
    case UIMode::MENU:
      menu_page = 0;
      menu_edit_idx = -1;
      menu_edit_buf.clear();
      draw_menu_view();
      break;
    case UIMode::DEBUG:
      debug_page = (int)((g_debug_lines.size() - 1) / 6);
      ui_draw_debug(g_debug_lines, debug_page);
      break;
    case UIMode::CONTROL:
      ui_draw_list(g_ctrl_lines, 0, -1);
      host_input.clear();
      ensure_usb();
      if (usb_ready) {
        host_write_str("READY\r\n");
        host_write_str(std::string(HOST_PROMPT));
      } else {
        debug_log_line("USB serial not ready");
      }
      break;
  case UIMode::HOST:
    // Start UAC audio streaming
    g_uac_lines.clear();
    g_uac_lines.push_back("USB Audio Host Mode");
    if (uac_start()) {
      g_decode_enabled = true;
      ui_set_paused(false);
      ui_clear_waterfall();
      g_uac_lines.push_back("Starting USB host...");
      g_uac_lines.push_back("Connect 24-bit/48kHz");
      g_uac_lines.push_back("stereo USB mic");
    } else {
      g_uac_lines.push_back("Failed to start UAC");
        debug_log_line("UAC start failed");
      }
      ui_draw_list(g_uac_lines, 0, -1);
      break;
    case UIMode::LIST:
      list_page = 0;
      ui_draw_list(g_list_lines, list_page, -1);
      break;
    case UIMode::QSO:
      g_q_show_entries = false;
      q_page = 0;
      qso_load_file_list();
      ui_draw_list(g_q_lines, q_page, -1);
      break;
    case UIMode::STATUS:
      g_status_beacon_temp = g_beacon;
      status_edit_idx = -1;
      status_cursor_pos = -1;
      draw_status_view();
      break;
  }
}

static void app_task_core0(void* /*param*/) {
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = true
  };
  ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
  init_soft_uart();
  ui_init();
  
  std::vector<UiRxLine> empty;
  ui_set_rx_list(empty);
  ui_draw_rx();

  // Initialize autoseq engine
  autoseq_init();
  autoseq_set_adif_callback(log_adif_entry);

  ui_mode = UIMode::RX;
  load_station_data();

  // Update autoseq with station info after loading
  autoseq_set_station(g_call, g_grid);

  ESP_LOGI(TAG, "Free heap: %u, internal: %u, 8bit: %u",
           heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_free_size(MALLOC_CAP_8BIT));
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "Heap %u", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    debug_log_line(buf);
  }
  log_heap("BOOT");

  // UI loop
  char last_key = 0;
  while (true) {
    M5Cardputer.update();
    M5Cardputer.Keyboard.updateKeysState();
    auto &state = M5Cardputer.Keyboard.keysState();
    char c = 0;
    if (!state.word.empty()) {
      c = state.word.back();
      state.word.clear();  // consume key
    } else if (state.del) {
      c = 0x7f;  // treat delete/backspace
    } else if (state.enter) {
      c = '\n';  // enter/return
    }

    rtc_tick();
    update_countdown();

  // HOST mode: UAC audio streaming - update status display
  if (ui_mode == UIMode::HOST) {
    // Update status display periodically
    static TickType_t last_update = 0;
    TickType_t now = xTaskGetTickCount();
    if (now - last_update > pdMS_TO_TICKS(500)) {
      last_update = now;
      // Update status line (keep only header)
      if (g_uac_lines.size() > 1) {
        g_uac_lines.resize(1);
      }
      g_uac_lines.push_back(uac_get_status_string());
      if (uac_is_streaming()) {
        g_uac_lines.push_back("Decoding FT8...");
        // Show debug lines with raw USB sample data
        const char* dbg1 = uac_get_debug_line1();
        const char* dbg2 = uac_get_debug_line2();
        if (dbg1[0]) g_uac_lines.push_back(dbg1);
        if (dbg2[0]) g_uac_lines.push_back(dbg2);
      }
      ui_draw_list(g_uac_lines, 0, -1);
    }
    // Handle keyboard - only 'h'/'H' to exit
    if (c == 0) {
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c == last_key) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    last_key = c;
    if (c == 'h' || c == 'H') {
      enter_mode(UIMode::RX);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }

  // CONTROL mode: legacy host serial protocol over USB (and BLE)
  if (ui_mode == UIMode::CONTROL) {
    poll_host_uart();
    if (host_bin_active) { // block keyboard exits during binary upload
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c == 0) {
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (c == last_key) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    last_key = c;
    if (c == 'c' || c == 'C' || c == 'h' || c == 'H') {
      enter_mode(UIMode::RX);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }

    if (c == 0) {
      if (g_rx_dirty && ui_mode == UIMode::RX) {
        ui_set_rx_list(g_rx_lines);
        ui_draw_rx(rx_flash_idx);
        g_rx_dirty = false;
      }
      ui_draw_waterfall_if_dirty();
      menu_flash_tick();
      rx_flash_tick();
      last_key = 0;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
  if (c == last_key) {
    ui_draw_waterfall_if_dirty();
    vTaskDelay(pdMS_TO_TICKS(10));
    continue;
  }
  last_key = c;

  rtc_tick();
  update_countdown();
  menu_flash_tick();
  rx_flash_tick();
  apply_pending_sync();
  // NOTE: Don't call schedule_tx_if_idle here! TX scheduling happens:
  // 1. After decode processing (decode_monitor_results)
  // 2. After user touch on decoded message
  // 3. After sending free text
  // Calling it in the main loop causes a race condition where TX is
  // scheduled before decodes update the state.

  static int last_status_uac = -1; // -1 forces a redraw on first entry
  int cur_uac = uac_is_streaming() ? 1 : 0;
  if (cur_uac && last_status_uac == 0) {
    set_log_to_soft_uart(true);
  }
  if (ui_mode == UIMode::STATUS && cur_uac != last_status_uac) {
    draw_status_view();
  }
  if (ui_mode != UIMode::STATUS) {
    last_status_uac = -1;
  } else {
    last_status_uac = cur_uac;
  }

  // Ensure decode is enabled whenever streaming becomes active.
  if (uac_is_streaming() && !g_decode_enabled) {
    g_decode_enabled = true;
    ui_set_paused(false);
  }

  if (g_rx_dirty && ui_mode == UIMode::RX) {
      ui_set_rx_list(g_rx_lines);
      ui_draw_rx(rx_flash_idx);
      g_rx_dirty = false;
  }
  ui_draw_waterfall_if_dirty();

  bool switched = false;
  auto cancel_status_edit = []() {
    if (status_edit_idx != -1) {
      status_edit_idx = -1;
      status_edit_buffer.clear();
      status_cursor_pos = -1;
    }
  };
  if (!(ui_mode == UIMode::MENU && (menu_edit_idx >= 0 || menu_long_edit))) {
      // Mode switch keys (disabled while editing in MENU)
      if (c == 'r' || c == 'R') { cancel_status_edit(); enter_mode(UIMode::RX); ui_force_redraw_rx(); ui_draw_rx(); switched = true; }
      else if (c == 't' || c == 'T') { cancel_status_edit(); enter_mode(ui_mode == UIMode::TX ? UIMode::RX : UIMode::TX); switched = true; }
      else if (c == 'b' || c == 'B') { cancel_status_edit(); enter_mode(ui_mode == UIMode::BAND ? UIMode::RX : UIMode::BAND); switched = true; }
      else if (c == 'm' || c == 'M') { cancel_status_edit(); enter_mode(ui_mode == UIMode::MENU ? UIMode::RX : UIMode::MENU); switched = true; }
      else if (c == 'n' || c == 'N') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          enter_mode(UIMode::RX);
        } else {
          menu_page = 0;
          enter_mode(UIMode::MENU);
          if (menu_page < 2) menu_page++;  // one "." press
          draw_menu_view();
        }
        switched = true;
      }
      else if (c == 'o' || c == 'O') {
        cancel_status_edit();
        if (ui_mode == UIMode::MENU) {
          enter_mode(UIMode::RX);
        } else {
          menu_page = 0;
          enter_mode(UIMode::MENU);
          if (menu_page < 2) menu_page++;  // first "."
          if (menu_page < 2) menu_page++;  // second "."
          draw_menu_view();
        }
        switched = true;
      }
      else if (c == 'q' || c == 'Q') { cancel_status_edit(); enter_mode(ui_mode == UIMode::QSO ? UIMode::RX : UIMode::QSO); switched = true; }
      else if (c == 'h' || c == 'H') { cancel_status_edit(); set_log_to_soft_uart(true); enter_mode(ui_mode == UIMode::HOST ? UIMode::RX : UIMode::HOST); switched = true; }
      else if (c == 'c' || c == 'C') { cancel_status_edit(); set_log_to_soft_uart(false); enter_mode(ui_mode == UIMode::CONTROL ? UIMode::RX : UIMode::CONTROL); switched = true; }
      else if (c == 'd' || c == 'D') { cancel_status_edit(); enter_mode(ui_mode == UIMode::DEBUG ? UIMode::RX : UIMode::DEBUG); switched = true; }
      else if (c == 'l' || c == 'L') { cancel_status_edit(); enter_mode(ui_mode == UIMode::LIST ? UIMode::RX : UIMode::LIST); switched = true; }
      else if (c == 's' || c == 'S') { cancel_status_edit(); enter_mode(ui_mode == UIMode::STATUS ? UIMode::RX : UIMode::STATUS); switched = true; }
    }

  if (!switched && c) {
    // Mode-specific handling
    switch (ui_mode) {
      case UIMode::RX: {
        int sel = ui_handle_rx_key(c);
        if (sel >= 0 && sel < (int)g_rx_lines.size()) {
          // User tapped on a decoded message - let autoseq handle it
          autoseq_on_touch(g_rx_lines[sel]);
          schedule_tx_if_idle();
          rx_flash_idx = sel;
          rx_flash_deadline = rtc_now_ms() + 500;
          ui_draw_rx(rx_flash_idx);
        }
        break;
      }
      case UIMode::TX: {
        // TX view shows QSO states from autoseq
        // Pagination through QSO list (max 9 QSOs)
        int qso_count = autoseq_queue_size();
        int start_idx = tx_page * 5;
        if (c == ';') {
          if (tx_page > 0) { tx_page--; redraw_tx_view(); }
        } else if (c == '.') {
          if (start_idx + 5 < qso_count) { tx_page++; redraw_tx_view(); }
        } else if (c == 'e' || c == 'E') {
          encode_and_log_pending_tx();
        }
        break;
      }
        case UIMode::BAND: {
          if (band_edit_idx >= 0) {
            if (c >= '0' && c <= '9') { band_edit_buffer.push_back(c); draw_band_view(); }
            else if (c == 0x08 || c == 0x7f) {
              if (!band_edit_buffer.empty()) { band_edit_buffer.pop_back(); draw_band_view(); }
            } else if (c == '\r' || c == '\n') {
              if (!band_edit_buffer.empty()) {
                int val = std::stoi(band_edit_buffer);
                g_bands[band_edit_idx].freq = val;
                save_station_data();
              }
              band_edit_idx = -1;
              band_edit_buffer.clear();
              draw_band_view();
            }
          } else {
            if (c == ';') {
              if (band_page > 0) { band_page--; draw_band_view(); }
            } else if (c == '.') {
              if ((band_page + 1) * 6 < (int)g_bands.size()) { band_page++; draw_band_view(); }
            } else if (c >= '1' && c <= '6') {
              int idx = band_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_bands.size()) {
                band_edit_idx = idx;
                band_edit_buffer = std::to_string(g_bands[idx].freq);
                draw_band_view();
              }
            }
          }
          break;
        }
        case UIMode::STATUS: {
        if (status_edit_idx == -1) {
          if (c == '1') { g_status_beacon_temp = (BeaconMode)(((int)g_status_beacon_temp + 1) % 5); draw_status_view(); }
          else if (c == '2') {
            status_edit_idx = 1;
            draw_status_view();
            if (!uac_is_streaming()) {
              debug_log_line("Starting UAC host...");
              if (!uac_start()) {
                debug_log_line("UAC start failed");
              } else {
                set_log_to_soft_uart(true);
                g_decode_enabled = true;
                ui_set_paused(false);
                ui_clear_waterfall();
              }
            } else {
              int freq_hz = g_bands[g_band_sel].freq * 1000;
              char cmd[32];
              snprintf(cmd, sizeof(cmd), "FA%011d;", freq_hz);
              bool ok = false;
              if (cat_cdc_ready()) {
                if (cat_cdc_send(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), 200) == ESP_OK) {
                  ok = true;
                }
                const char* md = "MD6;";
                cat_cdc_send(reinterpret_cast<const uint8_t*>(md), strlen(md), 200);
              }
              debug_log_line(ok ? "CAT sync sent" : "CAT not ready");
            }
            status_edit_idx = -1;
            draw_status_view();
          }
          else if (c == '3') {
            advance_active_band(1);
            save_station_data();
            draw_status_view();
              debug_log_line("Band changed");
            }
              else if (c == '4') {
                g_tune = !g_tune;
                if (cat_cdc_ready()) {
                  int freq_hz = g_bands[g_band_sel].freq * 1000;
                  char cmd[32];
                  snprintf(cmd, sizeof(cmd), "FA%011d;", freq_hz);
                  cat_cdc_send(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), 200); // set VFO
                  cat_cdc_send(reinterpret_cast<const uint8_t*>(cmd), strlen(cmd), 200); // confirm VFO
                  const char* md = "MD6;";
                  cat_cdc_send(reinterpret_cast<const uint8_t*>(md), strlen(md), 200);   // USB mode
                  if (g_tune) {
                const char* tx = "TX;";
                cat_cdc_send(reinterpret_cast<const uint8_t*>(tx), strlen(tx), 200); // key down
                // Use current cursor/random offset ~1500 Hz as tune tone
                int tune_hz = (g_offset_src == OffsetSrc::CURSOR) ? g_offset_hz : 1500;
                char ta[16];
                snprintf(ta, sizeof(ta), "TA%04d.%02d;", tune_hz, 0);
                cat_cdc_send(reinterpret_cast<const uint8_t*>(ta), strlen(ta), 200); // start tune carrier
                debug_log_line("CAT tune: TX TA");
                  } else {
                    const char* rx = "RX;";
                    cat_cdc_send(reinterpret_cast<const uint8_t*>(rx), strlen(rx), 200); // release
                    debug_log_line("CAT tune: RX");
                  }
                } else {
                  ESP_LOGW(TAG, "CAT not ready; tune skipped");
                }
                draw_status_view();
              }
              else if (c == '5') { status_edit_idx = 4; status_edit_buffer = g_date; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == '-')) status_cursor_pos++; draw_status_view(); }
              else if (c == '6') { status_edit_idx = 5; status_edit_buffer = g_time; status_cursor_pos = 0; while (status_cursor_pos < (int)status_edit_buffer.size() && (status_edit_buffer[status_cursor_pos] == ':')) status_cursor_pos++; draw_status_view(); }
            } else {
              if (status_edit_idx == 1) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); draw_status_view(); }
                if (c == ';') { g_offset_hz += 100; draw_status_view(); }
                else if (c == '.') { g_offset_hz -= 100; draw_status_view(); }
                else if (c == ',') { g_offset_hz -= 10; draw_status_view(); }
                else if (c == '/') { g_offset_hz += 10; draw_status_view(); }
                else if (c == '\n') { save_station_data(); status_edit_idx = -1; draw_status_view(); }
              } else if (status_edit_idx == 4 || status_edit_idx == 5) {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == ',') { // left
                  int pos = status_cursor_pos - 1;
                  while (pos >= 0 && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos--;
                  if (pos >= 0) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c == '/') { // right
                  int pos = status_cursor_pos + 1;
                  while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                  if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  draw_status_view();
                } else if (c >= '0' && c <= '9') {
                  if (status_cursor_pos >= 0 && status_cursor_pos < (int)status_edit_buffer.size()) {
                    status_edit_buffer[status_cursor_pos] = c;
                    int pos = status_cursor_pos + 1;
                    while (pos < (int)status_edit_buffer.size() && (status_edit_buffer[pos] == '-' || status_edit_buffer[pos] == ':')) pos++;
                    if (pos < (int)status_edit_buffer.size()) status_cursor_pos = pos;
                  }
                  draw_status_view();
                } else if (c == '\n') {
                  if (status_edit_idx == 4) g_date = status_edit_buffer;
                  else g_time = status_edit_buffer;
                  save_station_data();
                  rtc_set_from_strings();
                  status_edit_idx = -1;
                  status_cursor_pos = -1;
                  status_edit_buffer.clear();
                  draw_status_view();
                }
              } else {
                if (c == '`') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
                else if (c == '\n') { status_edit_idx = -1; status_edit_buffer.clear(); status_cursor_pos = -1; draw_status_view(); }
              }
            }
            break;
          }
        case UIMode::DEBUG: {
          if (c == ';') {
            if (debug_page > 0) { debug_page--; ui_draw_debug(g_debug_lines, debug_page); }
          } else if (c == '.') {
            if ((debug_page + 1) * 6 < (int)g_debug_lines.size()) { debug_page++; ui_draw_debug(g_debug_lines, debug_page); }
          }
          break;
        }
        case UIMode::LIST: {
          if (c == ';') {
            if (list_page > 0) { list_page--; ui_draw_list(g_list_lines, list_page, -1); }
          } else if (c == '.') {
            if ((list_page + 1) * 6 < (int)g_list_lines.size()) { list_page++; ui_draw_list(g_list_lines, list_page, -1); }
          }
          break;
        }
        case UIMode::QSO: {
          if (!g_q_show_entries) {
            if (c == ';') {
              if (q_page > 0) { q_page--; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c >= '1' && c <= '6') {
              int idx = q_page * 6 + (c - '1');
              if (idx >= 0 && idx < (int)g_q_files.size()) {
                g_q_current_file = g_q_files[idx];
                qso_load_entries(g_q_current_file);
                g_q_show_entries = true;
                q_page = 0;
                ui_draw_list(g_q_lines, q_page, -1);
              }
            }
          } else {
            if (c == ';') {
              if (q_page > 0) { q_page--; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c == '.') {
              if ((q_page + 1) * 6 < (int)g_q_lines.size()) { q_page++; ui_draw_list(g_q_lines, q_page, -1); }
            } else if (c == '`') {
              // back to file list
              g_q_show_entries = false;
              q_page = 0;
              qso_load_file_list();
              ui_draw_list(g_q_lines, q_page, -1);
            }
          }
          break;
        }
        case UIMode::CONTROL:
          break;
        case UIMode::HOST:
        case UIMode::MENU: {
          if (ui_mode == UIMode::MENU) {
            if (menu_long_edit) {
              if (c == '\n' || c == '\r') {
                if (menu_long_kind == LONG_FT) {
                  g_free_text = menu_long_buf;
                  if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text;
                } else if (menu_long_kind == LONG_COMMENT) {
                  g_comment1 = menu_long_buf;
                } else if (menu_long_kind == LONG_ACTIVE) {
                  g_active_band_text = menu_long_buf;
                  rebuild_active_bands();
                }
                save_station_data();
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == '`') {
                menu_long_edit = false;
                menu_long_kind = LONG_NONE;
                menu_long_buf.clear();
                menu_long_backup.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_long_buf.empty()) menu_long_buf.pop_back();
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_long_kind == LONG_FT) ch = toupper((unsigned char)ch);
                menu_long_buf.push_back(ch);
                draw_menu_view();
              }
              break;
            } else if (menu_edit_idx >= 0) {
              if (c == '\n' || c == '\r') {
                int idx = menu_edit_idx % 6;
                int page = menu_edit_idx / 6;
                if (page == 0) {
                  if (idx == 3) { g_free_text = menu_edit_buf; if (g_cq_type == CqType::CQFREETEXT) g_cq_freetext = g_free_text; }
                  else if (idx == 4) g_call = menu_edit_buf;
                  else if (idx == 5) g_grid = menu_edit_buf;
                } else {
                  if (idx == 1) {
                    g_offset_hz = atoi(menu_edit_buf.c_str());
                  } else if (idx == 2) g_ant = menu_edit_buf;
                  else if (idx == 3) g_comment1 = menu_edit_buf;
                }
                save_station_data();
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c == 0x08 || c == 0x7f) {
                if (!menu_edit_buf.empty()) menu_edit_buf.pop_back();
                draw_menu_view();
              } else if (c == '`') {
                menu_edit_idx = -1;
                menu_edit_buf.clear();
                draw_menu_view();
              } else if (c >= 32 && c < 127) {
                char ch = c;
                if (menu_edit_idx % 6 == 3 || menu_edit_idx % 6 == 4 || menu_edit_idx % 6 == 5) {
                  ch = toupper((unsigned char)ch);
                }
                menu_edit_buf.push_back(ch);
                draw_menu_view();
              }
              break;
            }
        if (c == ';') {
          if (menu_page > 0) { menu_page--; draw_menu_view(); }
        } else if (c == '.') {
          if (menu_page < 2) { menu_page++; draw_menu_view(); }
        } else if (menu_page == 0) {
              if (c == '1') {
                g_cq_type = (CqType)(((int)g_cq_type + 1) % 6);
                save_station_data();
                draw_menu_view();
              } else if (c == '2') {
                // Send freetext - one-off transmission, bypass autoseq
                int64_t now_slot = rtc_now_ms() / 15000;
                g_pending_tx.text = g_free_text;
                g_pending_tx.dxcall = "FreeText";
                g_pending_tx.offset_hz = g_offset_hz;
                g_pending_tx.slot_id = (int)((now_slot + 1) & 1); // next slot
                g_pending_tx.repeat_counter = 1;
                g_pending_tx.is_signoff = false;
                g_pending_tx_valid = true;
                schedule_tx_if_idle();
                menu_flash_idx = 1; // absolute index of "Send FreeText"
                menu_flash_deadline = rtc_now_ms() + 500;
                draw_menu_view();
                debug_log_line(std::string("Queued: ") + g_free_text);
              } else if (c == '3') {
                menu_long_edit = true;
                menu_long_kind = LONG_FT;
                menu_long_buf = g_free_text;
                menu_long_backup = g_free_text;
                draw_menu_view();
              } else if (c == '4') {
                menu_edit_idx = 4;
                menu_edit_buf = g_call;
                draw_menu_view();
              } else if (c == '5') {
                menu_edit_idx = 5;
                menu_edit_buf = g_grid;
                draw_menu_view();
              } else if (c == '6') {
                  if (M5.Power.isCharging()) {
                    ESP_LOGI(TAG, "Sleep button pressed while charging; entering deep sleep");
                    M5.Display.sleep();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_deep_sleep_start();
                  } else {
                    debug_log_line("Sleep skipped: not charging");
                    draw_menu_view();
                  }
              }
            } else if (menu_page == 1) {
                if (c == '1') {
                  g_offset_src = (OffsetSrc)(((int)g_offset_src + 1) % 3);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '2') {
                  menu_edit_idx = 6 + 1; // Cursor
                  menu_edit_buf = std::to_string(g_offset_hz);
                  draw_menu_view();
                } else if (c == '3') {
                  g_radio = (RadioType)(((int)g_radio + 1) % 4);
                  save_station_data();
                  draw_menu_view();
                } else if (c == '4') {
                  menu_edit_idx = 6 + 2;
                  menu_edit_buf = g_ant;
                  draw_menu_view();
                } else if (c == '5') {
                  menu_long_edit = true;
                  menu_long_kind = LONG_COMMENT;
                  menu_long_buf = g_comment1;
                  menu_long_backup = g_comment1;
                  draw_menu_view();
                }
            } else if (menu_page == 2) {
              if (c == '1') {
                g_rxtx_log = !g_rxtx_log;
                save_station_data();
                draw_menu_view();
              } else if (c == '2') {
                g_skip_tx1 = !g_skip_tx1;
                save_station_data();
                draw_menu_view();
              } else if (c == '3') {
                menu_long_edit = true;
                menu_long_kind = LONG_ACTIVE;
                menu_long_buf = g_active_band_text;
                menu_long_backup = g_active_band_text;
                draw_menu_view();
              }
            }
          }
          break;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  // Run the main application loop on core0; BLE host stays on core0.
  xTaskCreatePinnedToCore(app_task_core0, "app_core0", 12288, nullptr, 5, nullptr, 0);
  init_bluetooth();   // runs on core0
}
static void draw_status_line(int idx, const std::string& text, bool highlight) {
  const int line_h = 19;
  const int start_y = 18 + 3 + 3; // WATERFALL_H + COUNTDOWN_H + gap
  int y = start_y + idx * line_h;
  uint16_t bg = highlight ? M5.Display.color565(30, 30, 60) : TFT_BLACK;
  M5.Display.setTextSize(2);
  M5.Display.fillRect(0, y, 240, line_h, bg);
  M5.Display.setTextColor(TFT_WHITE, bg);
  M5.Display.setCursor(0, y);
  M5.Display.printf("%d %s", idx + 1, text.c_str());
}
static void draw_battery_icon(int x, int y, int w, int h, int level, bool charging) {
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  // Outline
  M5.Display.startWrite();
  M5.Display.fillRect(x, y, w, h, TFT_BLACK);
  M5.Display.drawRect(x, y, w - 3, h, TFT_WHITE);
  M5.Display.fillRect(x + w - 3, y + h / 4, 3, h / 2, TFT_WHITE); // tab
  // Fill
  int inner_w = w - 5;
  int inner_h = h - 4;
  int fill_w = (inner_w * level) / 100;
  uint16_t fill_color = (level > 30) ? M5.Display.color565(0, 200, 0)
                        : (level > 15) ? M5.Display.color565(200, 180, 0)
                                        : M5.Display.color565(200, 0, 0);
  M5.Display.fillRect(x + 2, y + 2, fill_w, inner_h, fill_color);
  // Charging bolt
  if (charging) {
    int bx = x + w / 2 - 2;
    int by = y + 2;
    M5.Display.fillTriangle(bx, by, bx + 4, by + h / 2, bx + 2, by, M5.Display.color565(255, 255, 0));
    M5.Display.fillTriangle(bx + 2, by + h / 2, bx + 6, by + h - 2, bx + 4, by + h - 2, M5.Display.color565(255, 255, 0));
  }
  M5.Display.endWrite();
}


