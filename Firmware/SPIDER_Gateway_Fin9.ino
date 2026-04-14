// =============================================================================
// SPIDER Gateway — nRF52840 (Nice!nano) BLE MIDI bridge + OLED UI
// =============================================================================

#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <InternalFileSystem.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_TinyUSB.h>

// -----------------------------------------------------------------------------
// USB MIDI: host(3 ports) / client(1); active pointer set in setup()
// -----------------------------------------------------------------------------
Adafruit_USBD_MIDI usb_midi_host(3);
Adafruit_USBD_MIDI usb_midi_client(1);
Adafruit_USBD_MIDI *usb_midi_ptr = nullptr;

#define MIDI_SVC_UUID_STR "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHR_UUID_STR "7772e5db-3868-4112-a1a9-f2669d106bf3"

#define MY_USB_MANUFACTURER "ASH SOUND WORKS"
#define MY_USB_PRODUCT "SPIDER"
#define MY_USB_PID 0x033A
#define MY_USB_SERIAL "0001"
#define usb_midi (*usb_midi_ptr)

#define MAGIC_UF2_BOOTLOADER 0x57

// -----------------------------------------------------------------------------
// BLE central (host): battery client + 3x MIDI + 3x battery services
// -----------------------------------------------------------------------------
BLEClientService host_bat_svc_client(UUID16_SVC_BATTERY);
BLEClientCharacteristic host_bat_chr_client(UUID16_CHR_BATTERY_LEVEL);
BLEUuid midiSvcUuid(MIDI_SVC_UUID_STR);
BLEUuid midiChrcUuid(MIDI_CHR_UUID_STR);
BLEClientService ble_midi_svc[3] = {
  BLEClientService(midiSvcUuid), BLEClientService(midiSvcUuid), BLEClientService(midiSvcUuid)
};
BLEClientCharacteristic ble_midi_chr[3] = {
  BLEClientCharacteristic(midiChrcUuid), BLEClientCharacteristic(midiChrcUuid), BLEClientCharacteristic(midiChrcUuid)
};
BLEClientService ble_bat_svc[3] = {
  BLEClientService(UUID16_SVC_BATTERY), BLEClientService(UUID16_SVC_BATTERY), BLEClientService(UUID16_SVC_BATTERY)
};
BLEClientCharacteristic ble_bat_chr[3] = {
  BLEClientCharacteristic(UUID16_CHR_BATTERY_LEVEL), BLEClientCharacteristic(UUID16_CHR_BATTERY_LEVEL), BLEClientCharacteristic(UUID16_CHR_BATTERY_LEVEL)
};

using namespace Adafruit_LittleFS_Namespace;

// -----------------------------------------------------------------------------
// Filesystem / scan list / pins
// -----------------------------------------------------------------------------
#define CONFIG_FILE "/slots_config.dat"
#define MAX_DEVICES 50
#define NAME_LEN 15
#define DEVICE_TIMEOUT 3000

#define SDA_PIN 6
#define SCL_PIN 7
#define VCC_EXT_PIN 21
#define ENCODER_A 4
#define ENCODER_B 5
#define BUTTON_PIN 9

// -----------------------------------------------------------------------------
// Display, encoder, UI timing
// -----------------------------------------------------------------------------
uint8_t ignoredMac[6] = { 0 };
uint32_t ignoreUntil = 0;
/** 블랙리스트 중인 피어 표시용 (CLEAR&BLOCK 시 clientSlot 이름 복사) */
char ignoredPeerName[NAME_LEN] = "";
unsigned long searchStartTime = 0;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
long encoderPos = 0;
/** A=bit1, B=bit0 — AB 인터럽트 전이표용 이전 상태 */
volatile uint8_t encPrevState = 0;
bool buttonState = false;
int currentSelection = -1;
unsigned long lastInteractionTime = 0;
int displayDeviceCount = 0;
uint8_t lockedMac[6] = { 0 };
bool isTargetLocked = false;
bool isBackSelected = false;
ble_gap_evt_adv_report_t device_reports[MAX_DEVICES];
unsigned long lastMidiTime[3] = { 0, 0, 0 };
unsigned long lastUsbMidiTime[3] = { 0, 0, 0 };
volatile int encoderAccumulator = 0;
/** 디텐트당 LUT 누적(보통 4; 메뉴가 2칸씩 움직이면 2로 낮춤) */
const int ENC_STEPS_PER_DETENT = 4;
unsigned long lastEncoderMoveTime = 0;
const unsigned long ENCODER_GAP = 80;
uint8_t remoteHostBat = 0;
bool blinkState = false;


// -----------------------------------------------------------------------------
// BLE peripheral (client): 128-bit MIDI UUID (little-endian byte order)
// -----------------------------------------------------------------------------
const uint8_t UUID128_SVC_MIDI[] = {
  0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
  0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
};
const uint8_t UUID128_CHR_MIDI[] = {
  0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
  0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
};
BLEService client_midi_svc = BLEService(UUID128_SVC_MIDI);
BLECharacteristic client_midi_chr = BLECharacteristic(UUID128_CHR_MIDI);
BLEBas client_bat;
int lastRemainingSeconds = -1;

unsigned long scanStartTime = 0;
const unsigned long UI_TIMEOUT = 7000;
const unsigned long DIM_TIMEOUT = 300000;
/** 클라이언트: 플래시에 마스터 없을 때만 첫 연결 후 자동 저장까지 카운트다운(초). 15초로 줄이려면 15000UL */
const unsigned long CLIENT_TRUST_SAVE_MS = 30000UL;
bool isDimmed = false;
String pendingName = "";
char errorMsg[20] = "";
unsigned long errorMsgUntil = 0;
/** BLE 콜백에서 OLED(I2C)를 직접 건드리지 않고, loop()에서 updateDisplay()만 호출하도록 요청 */
volatile bool g_displayRefreshRequest = false;
static bool g_displayReady = false;

enum SystemMode { MODE_HOST,
                  MODE_CLIENT };

SystemMode loadSystemMode();
void showSetupProgress(const char *msg);
void setupHostMode();
void setupClientMode();

enum TxPowerMode { POWER_NORMAL,
                   POWER_STRONG,
                   POWER_BOOST };
TxPowerMode currentTxMode = POWER_NORMAL;
int8_t activeTxPower = 0;

struct Device {
  uint8_t mac[6];
  int8_t rssi;
  char name[NAME_LEN];
  unsigned long lastSeen;
};

enum State { MAIN_MENU,
             SLOT_MENU,
             SCAN_MENU,
             MODE_SELECT,
             CLIENT_MENU };
State currentState = MAIN_MENU;
SystemMode currentSystemMode = MODE_HOST;
int modeSelection = 0;
int clientMenuSelection = 0;
unsigned long buttonPressStartTime = 0;

enum SlotStatus { EMPTY,
                  OFFLINE,
                  CONNECTED };

struct Slot {
  SlotStatus status;
  char name[NAME_LEN];
  uint8_t mac[6];
  int8_t battery;
};

struct SystemSettings {
  uint8_t savedMasterMac[6];
  char savedMasterName[NAME_LEN];
  bool hasSavedMaster;
};

SystemSettings config = { { 0 }, "", false };
unsigned long connectionStartTime = 0;
bool isMasterSavedThisSession = false;
typedef Slot SlotConfig;

int slotMenuSelection = 0;
int selectedSlotIdx = -1;
int scanMenuSelection = 0;

bool macEqual(const uint8_t *a, const uint8_t *b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// Slots & scan device list
// -----------------------------------------------------------------------------
Slot slots[3] = {
  { EMPTY, "", { 0 }, -1 },
  { EMPTY, "", { 0 }, -1 },
  { EMPTY, "", { 0 }, -1 }
};
Slot clientSlot = { EMPTY, "", { 0 }, -1 };

bool isScanRequired() {
  if (currentState == SCAN_MENU) return true;
  for (int i = 0; i < 3; i++) {
    if (slots[i].status == OFFLINE) return true;
  }
  return false;
}

Device devices[MAX_DEVICES];
int deviceCount = 0;

// -----------------------------------------------------------------------------
// Config file (host slots + client master info)
// -----------------------------------------------------------------------------
void saveConfig() {
  Watchdog.reset();
  SlotConfig toSave[3];

  if (currentSystemMode == MODE_CLIENT) {
    Adafruit_LittleFS_Namespace::File readFile(CONFIG_FILE, FILE_O_READ, InternalFS);
    if (readFile) {
      readFile.read((uint8_t *)toSave, sizeof(toSave));
      readFile.close();
    } else {
      memset(toSave, 0, sizeof(toSave));
    }
  } else {
    for (int i = 0; i < 3; i++) {
      toSave[i].status = slots[i].status;
      memcpy(toSave[i].mac, slots[i].mac, 6);
      strncpy(toSave[i].name, slots[i].name, NAME_LEN - 1);
    }
  }

  InternalFS.remove(CONFIG_FILE);
  Adafruit_LittleFS_Namespace::File file(CONFIG_FILE, FILE_O_WRITE, InternalFS);
  if (file) {
    file.write((uint8_t *)toSave, sizeof(toSave));
    file.write((uint8_t *)&config, sizeof(config));
    file.close();
    Serial.println("Flash Config Saved (Mode Isolated)");
  }
}
void loadConfig() {
  InternalFS.begin();
  Adafruit_LittleFS_Namespace::File file(CONFIG_FILE, FILE_O_READ, InternalFS);

  if (file) {
    SlotConfig loaded[3];
    memset(&config, 0, sizeof(config));
    file.read((uint8_t *)loaded, sizeof(loaded));
    size_t cfgRead = file.read((uint8_t *)&config, sizeof(config));
    file.close();
    if (cfgRead != sizeof(config)) {
      memset(&config, 0, sizeof(config));
      config.hasSavedMaster = false;
      Serial.println(F("Config tail missing or old file — master pairing reset"));
    }

    for (int i = 0; i < 3; i++) {
      slots[i].status = loaded[i].status;
      // CONNECTED 상태로 저장되었어도 부팅 시에는 일단 찾기 전까지 OFFLINE으로 설정
      if (slots[i].status == CONNECTED) {
        slots[i].status = OFFLINE;
      }
      strncpy(slots[i].name, loaded[i].name, NAME_LEN - 1);
      slots[i].name[NAME_LEN - 1] = '\0';
      memcpy(slots[i].mac, loaded[i].mac, 6);
    }
    Serial.println("Config loaded from Flash!");
  } else {
    Serial.println("No config file found (First boot)");
  }
}
void removeOldDevices() {
  unsigned long now = millis();
  for (int i = 0; i < deviceCount;) {
    // 마지막으로 발견된 지 5초(DEVICE_TIMEOUT)가 지났다면
    if (now - devices[i].lastSeen > DEVICE_TIMEOUT) {
      // 리스트에서 삭제 (뒤의 항목들을 앞으로 당김)
      for (int j = i; j < deviceCount - 1; j++) {
        devices[j] = devices[j + 1];
      }
      deviceCount--;
      // 인덱스 i는 증가시키지 않음 (당겨진 다음 항목을 검사해야 하므로)
    } else {
      i++;
    }
  }
}

void parseName(ble_gap_evt_adv_report_t *report, char *name) {
  uint8_t *data = report->data.p_data;
  uint8_t len = report->data.len;

  name[0] = 0;
  char shortLocal[NAME_LEN] = { 0 };

  for (int i = 0; i < len;) {
    uint8_t field_len = data[i];
    if (field_len == 0 || (i + 1 + field_len) > len) break;
    uint8_t field_type = data[i + 1];

    if (field_type == 0x09) {
      int n = (int)field_len - 1;
      if (n < 0) n = 0;
      if (n >= NAME_LEN) n = NAME_LEN - 1;
      memcpy(name, &data[i + 2], n);
      name[n] = 0;
      return;
    }
    if (field_type == 0x08 && shortLocal[0] == 0) {
      int n = (int)field_len - 1;
      if (n < 0) n = 0;
      if (n >= NAME_LEN) n = NAME_LEN - 1;
      memcpy(shortLocal, &data[i + 2], n);
      shortLocal[n] = 0;
    }

    i += field_len + 1;
  }
  if (name[0] == 0 && shortLocal[0] != 0) {
    strncpy(name, shortLocal, NAME_LEN - 1);
    name[NAME_LEN - 1] = 0;
  }
}

// OLED 슬롯 행 안테나 막대: 6단(0=없음 … 6=만칸). 가로 폭 ≈4칸 때와 비슷(간격 2px).
enum { ANT_RSSI_BAR_MAX = 6 };

static int antennaBarsFromRssi(float rssi, int lastBars) {
  int c = lastBars;
  if (c > ANT_RSSI_BAR_MAX) c = ANT_RSSI_BAR_MAX;
  if (c < 0) c = 0;
  if (rssi > (c == 5 ? -55 : -58)) return 6;
  if (rssi > (c == 4 ? -61 : -64)) return 5;
  if (rssi > (c == 3 ? -67 : -70)) return 4;
  if (rssi > (c == 2 ? -73 : -76)) return 3;
  if (rssi > (c == 1 ? -79 : -82)) return 2;
  if (rssi > (c == 0 ? -88 : -91)) return 1;
  if (rssi <= -94) return 0;
  return c;
}

static void drawAntennaBars(int antX, int y, int bars, uint16_t color) {
  const int spacing = 2;
  for (int b = 0; b < ANT_RSSI_BAR_MAX; b++) {
    if (b < bars) {
      // 맨 오른쪽(최장)만 예전과 동일 8px, 나머지는 각 +1px (3,4,5,6,7,8)
      int h_val = 3 + b;
      display.drawFastVLine(antX + b * spacing, y + 11 - h_val, h_val, color);
    }
  }
}

void updateDevice(ble_gap_evt_adv_report_t *report, int8_t rssi, char *name) {
  uint8_t *mac = report->peer_addr.addr;

  // 1. 이미 슬롯(1~4번)에 등록된 기기인지 확인
  bool isStoredInSlot = false;
  for (int i = 0; i < 3; i++) {
    // [중요] 상태가 EMPTY가 아닐 때만 슬롯 기기로 인정
    if (slots[i].status != EMPTY && macEqual(slots[i].mac, mac)) {
      isStoredInSlot = true;
      break;
    }
  }

  // 등록되지 않은 기기인데 신호가 너무 약하면 무시
  if (!isStoredInSlot && rssi < -85) return;

  // 2. 리스트 업데이트 (스캔 메뉴용)
  for (int i = 0; i < deviceCount; i++) {
    if (macEqual(devices[i].mac, mac)) {
      devices[i].rssi = rssi;
      devices[i].lastSeen = millis();
      memcpy(&device_reports[i], report, sizeof(ble_gap_evt_adv_report_t));
      return;
    }
  }

  // 새 장치 추가
  if (deviceCount < MAX_DEVICES) {
    memcpy(devices[deviceCount].mac, mac, 6);
    devices[deviceCount].rssi = rssi;
    strncpy(devices[deviceCount].name, name, NAME_LEN - 1);
    devices[deviceCount].name[NAME_LEN - 1] = '\0';
    devices[deviceCount].lastSeen = millis();
    memcpy(&device_reports[deviceCount], report, sizeof(ble_gap_evt_adv_report_t));
    deviceCount++;
  }
}

void scan_callback(ble_gap_evt_adv_report_t *report) {
  char name[NAME_LEN];
  parseName(report, name);

  // 이름이 AD에 없는 패킷(스캔 응답 전용 등)에서도 MAC만 맞으면 재연결 시도
  for (int i = 0; i < 3; i++) {
    if (slots[i].status == OFFLINE && macEqual(slots[i].mac, report->peer_addr.addr)) {
      Serial.printf("Slot %d device found! Attempting Auto-Connect...\n", i + 1);
      Bluefruit.Scanner.stop();
      Bluefruit.Central.connect(report);
      return;
    }
  }

  if (strlen(name) == 0) {
    Bluefruit.Scanner.resume();
    return;
  }

  updateDevice(report, report->rssi, name);
  Bluefruit.Scanner.resume();
}

void drawSlotMenu() {
  display.fillRect(0, 0, 128, 14, WHITE);
  display.setTextColor(BLACK);
  display.setCursor(2, 4);
  display.print("SLOT");
  drawInvertedNumber(display.getCursorX() + 2, 3, selectedSlotIdx);
  display.setTextColor(BLACK);
  display.println(" SETTINGS");

  display.drawRect(0, 16, 128, 48, WHITE);

  // 순서를 BACK을 맨 위로 변경
  const char *options[] = { "<< BACK", "RESCAN DEVICE", "CLEAR SLOT" };

  for (int i = 0; i < 3; i++) {
    int yPos = 24 + (i * 12);  // 24(BACK), 36(RESCAN), 48(CLEAR)
    if (slotMenuSelection == i) {
      display.fillRect(2, yPos - 1, 124, 11, WHITE);
      display.setTextColor(BLACK);
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(5, yPos);
    display.println(options[i]);
  }
}

/** 블랙리스트 안내 — 예전과 동일: 기본 폰트 size1, 줄 위치 y+16 / y+28 / y+40, 제목 BLOCKED */
static void drawBlockedDevicePanelForSlotY(int y, int secLeft) {
  display.setTextSize(1);
  display.setTextColor(WHITE);

  const char *hdr = "BLOCKED";
  int hw = (int)strlen(hdr) * 6;
  display.setCursor((128 - hw) / 2, y + 16);
  display.print(hdr);

  char lineName[NAME_LEN];
  strncpy(lineName, ignoredPeerName, NAME_LEN - 1);
  lineName[NAME_LEN - 1] = '\0';
  if (lineName[0] == '\0') {
    snprintf(lineName, sizeof(lineName), "ID:%02X%02X%02X", ignoredMac[2], ignoredMac[1], ignoredMac[0]);
  }
  size_t nl = strlen(lineName);
  if (nl > 18) {
    lineName[18] = '\0';
    nl = 18;
  }
  int nw = (int)nl * 6;
  display.setCursor((128 - nw) / 2, y + 28);
  display.print(lineName);

  char tbuf[16];
  snprintf(tbuf, sizeof(tbuf), "%ds LEFT", secLeft);
  int tw = (int)strlen(tbuf) * 6;
  display.setCursor((128 - tw) / 2, y + 39);
  display.print(tbuf);
}

static int blockSecondsRemaining() {
  if (ignoreUntil == 0 || millis() >= ignoreUntil) return 0;
  unsigned long leftMs = (unsigned long)(ignoreUntil - millis());
  int secLeft = (int)((leftMs + 999UL) / 1000UL);
  if (secLeft < 1 && leftMs > 0) secLeft = 1;
  if (secLeft > 99) secLeft = 99;
  return secLeft;
}

void drawClientSlotRow(int y) {
  // 클라이언트는 선택 상태가 없으므로 기본 color/bgColor 설정
  uint16_t color = WHITE;
  uint16_t bgColor = BLACK;

  // 1. 슬롯 테두리
  display.drawRect(0, y, 128, 14, WHITE);
  display.drawRect(0, y + 13, 128, 64 - (y + 13), WHITE);

  // 2. [슬롯 번호 반전 박스] - 클라이언트는 항상 0번
  display.fillRect(2, y + 2, 10, 10, color);
  display.setTextColor(bgColor);
  display.setCursor(4, y + 3);
  display.print("0");

  // 3. 기기 이름 또는 상태 표시 (X=14 시작)
  display.setCursor(14, y + 3);

  if (Bluefruit.connected() && clientSlot.status == CONNECTED) {
    // --- [이름 출력] ---
    String devName = String(clientSlot.name);  // 호스트가 쓰는 변수 그대로 사용
    if (devName.length() > 10) devName = devName.substring(0, 10);
    display.setTextColor(color);
    display.print(devName);
    // --- 첫 마스터만: 플래시에 저장된 MAC 없을 때 자동 저장 카운트다운 (connect_callback에서만 타이머 시작) ---
    if (!isMasterSavedThisSession && connectionStartTime != 0) {
      unsigned long elapsed = millis() - connectionStartTime;
      if (elapsed <= CLIENT_TRUST_SAVE_MS) {
        int remaining = (int)(CLIENT_TRUST_SAVE_MS / 1000UL) - (int)(elapsed / 1000UL);
        if (remaining < 0) remaining = 0;
        int pct = (int)((elapsed * 100UL) / CLIENT_TRUST_SAVE_MS);
        if (pct > 99) pct = 99;
        int barWidth = (int)((elapsed * 100L) / (long)CLIENT_TRUST_SAVE_MS);
        if (barWidth > 100) barWidth = 100;
        const int saveBarOuterW = 102;
        int barX = (128 - saveBarOuterW) / 2;
        display.drawRect(barX, y + 18, saveBarOuterW, 5, WHITE);
        display.fillRect(barX + 1, y + 19, barWidth, 3, WHITE);
        char saveLine[20];
        snprintf(saveLine, sizeof(saveLine), "%d%% %ds SAVE", pct, remaining);
        int textW = (int)strlen(saveLine) * 6;
        int textX = (128 - textW) / 2;
        if (textX < 0) textX = 0;
        display.setCursor(textX, y + 26);
        display.print(saveLine);
        display.setCursor(8, y + 36);
        display.print(F("PRESS BTN TO CANCEL"));
      }
    } else if (isMasterSavedThisSession && config.hasSavedMaster && macEqual(config.savedMasterMac, clientSlot.mac)) {
      // 저장 완료된 페어링 표시
      display.setCursor(46, y + 26);
      display.setTextColor(WHITE);
      display.print(F("PAIRED"));  // "신뢰하는 호스트"
    }
    // 1. 컨트롤러 입력 (△) - 75번 위치 (마스터로 송신 시)
    if (millis() - lastMidiTime[0] < 100) {
      display.setCursor(81, y + 4);
      display.write(30);
    }

    // 2. 에이블톤 출력 (▽) - 81번 위치 (마스터에서 수신 시)
    if (millis() - lastUsbMidiTime[0] < 100) {
      display.setCursor(75, y + 3);
      display.write(31);
    }

    // --- [5. 안테나] RSSI 측정 (클라이언트는 0번 연결 고정) ---
    int8_t rawRssi = -128;
    BLEConnection *conn = Bluefruit.Connection(0);
    if (conn && conn->connected()) rawRssi = conn->getRssi();

    // --- [안테나 - 호스트와 동일 로직] ---

    // LPF용 static 변수 (클라이언트는 1개니까 index 0만 사용)
    static float smoothRssi = -70.0;

    // 유효한 RSSI일 때만 필터 적용
    if (rawRssi > -120 && rawRssi < 0) {
      smoothRssi = (rawRssi * 0.4f) + (smoothRssi * 0.6f);
    }

    float rssi = smoothRssi;

    static int lastBars = 0;
    int currentBars = antennaBarsFromRssi(rssi, lastBars);
    lastBars = currentBars;
    int bars = currentBars;

    drawAntennaBars(89, y, bars, color);

    // 6. [배터리 반전 박스] - 오른쪽 끝(102) 위치
    int batBoxX = 102;

    if (remoteHostBat >= 1) {
      int batVal = remoteHostBat;

      if (batVal > 40) {
        display.fillRect(batBoxX, y + 2, 24, 10, color);
        display.setTextColor(bgColor);

      } else {
        if (batVal <= 20) {
          // 👉 프레임 기반 깜빡임
          static uint8_t blinkCounter = 0;
          blinkCounter++;

          if (blinkCounter >= 30) {  // 속도 조절 (작을수록 빠름)
            blinkCounter = 0;
            blinkState = !blinkState;
          }

          if (blinkState) {
            display.drawRect(batBoxX, y + 2, 24, 10, color);
          }

        } else {
          display.drawRect(batBoxX, y + 2, 24, 10, color);
        }

        display.setTextColor(color);
      }

      // --- 텍스트는 항상 표시 ---
      int xOff = (batVal >= 100) ? 1 : (batVal >= 10 ? 3 : 6);
      display.setCursor(batBoxX + xOff, y + 3);
      display.print(batVal);
      display.print("%");

    } else if (remoteHostBat >= 100) {
      display.fillRect(batBoxX, y + 2, 24, 10, color);
      display.setTextColor(bgColor);
      display.setCursor(batBoxX + 4, y + 3);
      display.print("EXT");

    } else {
      display.drawRect(batBoxX, y + 2, 24, 10, color);
      display.setTextColor(color);
      display.setCursor(batBoxX + 3, y + 3);
      display.print("N/A");
    }

  } else {
    // --- [연결되지 않은 경우 (OFFLINE)] ---
    display.setTextColor(WHITE);

    if (ignoreUntil != 0 && millis() < ignoreUntil) {
      drawBlockedDevicePanelForSlotY(y, blockSecondsRemaining());

    } else if (config.hasSavedMaster) {
      // 1. 저장된 마스터가 있는 경우
      display.setCursor(36, y + 20);
      display.print("WAITING...");

      String masterName = String(config.savedMasterName);
      if (masterName.length() == 0) masterName = "MASTER";

      int textWidth = masterName.length() * 6;
      int x = (128 - textWidth) / 2;

      display.setCursor(x, y + 32);
      display.print(masterName);

    } else {
      // 2. 완전 초기 상태

      display.setCursor(26, y + 26);
      display.print("ADVERTISING...");
    }
  }
}

void drawClientMenu() {
  display.clearDisplay();

  // 1. 상단 타이틀 바
  display.fillRect(0, 0, 128, 14, WHITE);
  display.setTextColor(BLACK);
  display.setCursor(2, 3);
  display.print("CLIENT SETTING");  // 타이틀 변경

  // 2. 메인 테두리 박스
  display.drawRect(0, 16, 128, 48, WHITE);

  // --- 메뉴 리스트 (0: BACK, 1: CLEAR & BLOCK) ---

  // 0: << BACK (기존 위치)
  int yPos0 = 24;
  if (clientMenuSelection == 0) {
    display.fillRect(2, yPos0 - 1, 124, 11, WHITE);
    display.setTextColor(BLACK);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(5, yPos0);
  display.print("<< BACK");

  // 1: CLEAR & BLOCK 30s (합쳐진 메뉴)
  int yPos1 = 38;  // 간격을 조금 넓혔습니다.
  if (clientMenuSelection == 1) {
    display.fillRect(2, yPos1 - 1, 124, 11, WHITE);
    display.setTextColor(BLACK);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(5, yPos1);
  display.print("CLEAR & BLOCK 30s");

  display.display();
}

static void drawErrorPopupIfActive() {
  if (millis() >= errorMsgUntil || errorMsg[0] == '\0') return;

  bool blockStylePopup =
    (ignoreUntil != 0 && millis() < ignoreUntil) &&
    (strncmp(errorMsg, "BLOCKED DEVICE", 14) == 0 || strncmp(errorMsg, "BLOCKED 30 SEC", 14) == 0);

  if (blockStylePopup) {
    // 클라이언트 메인 슬롯은 항상 y=16 — 팝업도 동일 좌표로 그려 이전과 같은 글자 배치
    drawBlockedDevicePanelForSlotY(16, blockSecondsRemaining());
    return;
  }

  int boxW = 120;
  int boxH = 22;
  int boxX = (128 - boxW) / 2;
  int boxY = 29;
  display.fillRect(boxX, boxY, boxW, boxH, BLACK);
  display.drawRect(boxX, boxY, boxW, boxH, WHITE);
  display.setTextColor(WHITE);
  int msgWidth = (int)strlen(errorMsg) * 6;
  int cx = 64 - (msgWidth / 2);
  if (cx < 0) cx = 0;
  display.setCursor(cx, boxY + 7);
  display.print(errorMsg);
}

void drawStatusBarLeft() {
  display.setTextColor(BLACK);
  display.setCursor(2, 3);

  bool hasWeakSig = false;

  int maxCheck = (currentSystemMode == MODE_HOST) ? 3 : 1;

  // 👉 슬롯별 상태 유지용 (히스테리시스 + 시간 디바운스)
  static bool weakState[3] = { false, false, false };
  static bool weakPending[3] = { false, false, false };
  static unsigned long weakChangedAt[3] = { 0, 0, 0 };
  const unsigned long WEAK_HOLD_MS = 700;
  // updateTxPower() BOOST는 필터 RSSI ≤-78에서 이미 켜짐. WEAK를 -90만 쓰면 “안테나 1~2칸 + BOOST”인데도
  // (원시 RSSI가 -90보다 나은) WEAK가 안 뜨는 구간이 생김 → ON을 안테나 저칸 구간에 가깝게.
  // 히스테리시스는 약 6 dB 유지.
  const int WEAK_ON_RSSI = -75;
  const int WEAK_OFF_RSSI = -72;

  for (int i = 0; i < maxCheck; i++) {
    bool isSlotActive = (currentSystemMode == MODE_HOST) ? (slots[i].status == CONNECTED) : Bluefruit.connected();

    if (isSlotActive) {
      int8_t rssi = -128;

      if (currentSystemMode == MODE_HOST) {
        for (uint8_t h = 0; h < BLE_MAX_CONNECTION; h++) {
          BLEConnection *conn = Bluefruit.Connection(h);
          if (conn && conn->connected() && macEqual(slots[i].mac, conn->getPeerAddr().addr)) {
            rssi = conn->getRssi();
            break;
          }
        }
      } else {
        BLEConnection *conn = Bluefruit.Connection(0);
        if (conn && conn->connected()) rssi = conn->getRssi();
      }

      // 👉 히스테리시스 + 시간 디바운스 적용
      if (rssi != -128) {
        bool targetWeak = weakState[i];
        if (!weakState[i] && rssi < WEAK_ON_RSSI) targetWeak = true;
        else if (weakState[i] && rssi > WEAK_OFF_RSSI) targetWeak = false;

        if (targetWeak != weakPending[i]) {
          weakPending[i] = targetWeak;
          weakChangedAt[i] = millis();
        }
        if (weakState[i] != weakPending[i] && (millis() - weakChangedAt[i] >= WEAK_HOLD_MS)) {
          weakState[i] = weakPending[i];
        }

        if (weakState[i]) hasWeakSig = true;
      }
    } else {
      weakState[i] = false;
      weakPending[i] = false;
      weakChangedAt[i] = millis();
    }
  }

  if (currentSystemMode == MODE_HOST && selectedSlotIdx != -1) {
    display.print("WAIT");
    drawInvertedNumber(display.getCursorX() + 1, 2, selectedSlotIdx);

  } else if (hasWeakSig) {
    display.print("WEAK SIG:");
    if (currentSystemMode == MODE_HOST) {
      for (int i = 0; i < 3; i++) {
        if (weakState[i]) {
          int xPos = display.getCursorX() + 1;
          drawInvertedNumber(xPos, 2, i);
          display.setCursor(xPos + 7, 3);
        }
      }
    } else {
      drawInvertedNumber(display.getCursorX() + 1, 2, 0);
    }

  } else {
    display.print(currentSystemMode == MODE_HOST ? "HOST" : "CLIENT");
    uint8_t myBat = readHostBattery();

    if (myBat < 100) {
      const int boxW = 22;
      const int boxH = 12;
      const int xPos = 75 - boxW - 1;
      const int yPos = 1;

      if (myBat <= 40) {
        display.drawRect(xPos, yPos, boxW, boxH, BLACK);
        display.setCursor(xPos + 2, 3);
        display.print(myBat);
        display.print("%");
      } else {
        display.fillRect(xPos, yPos, boxW, boxH, BLACK);
        display.setTextColor(WHITE);
        display.setCursor(xPos + 2, 3);
        display.print(myBat);
        display.print("%");
        display.setTextColor(BLACK);
      }
    }
  }
}

void updateDisplay() {
  unsigned long now = millis();
  display.clearDisplay();
  display.setTextSize(1);

  if (currentState == MODE_SELECT) {
    drawModeSelect();
    return;
  }

  if (currentSystemMode == MODE_CLIENT) {
    if (currentState == CLIENT_MENU) {
      drawClientMenu();
      return;
    }
    int8_t currentRssi = -128;
    if (Bluefruit.connected()) {
      BLEConnection *conn = Bluefruit.Connection(0);
      if (conn && conn->connected()) currentRssi = conn->getRssi();
    }

    updateTxPower(currentRssi);

    display.fillRect(0, 0, 128, 14, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(2, 3);

    drawStatusBarLeft();
    // DIN MIDI: [M] 아이콘
    if (!TinyUSBDevice.mounted()) {
      display.fillRect(75, 1, 9, 12, BLACK);
      display.setTextColor(WHITE);
      display.setCursor(77, 3);
      display.print("M");
    }
    // 상단바 오른쪽 전력 (BOOST 등)
    if (activeTxPower == 8) {
      display.fillRect(85, 1, 42, 12, BLACK);
      display.setTextColor(WHITE);
      display.setCursor(92, 3);
      display.print("BOOST");
    } else {
      display.drawRect(85, 1, 42, 12, BLACK);
      display.setTextColor(BLACK);
      display.setCursor(89, 3);
      if (activeTxPower == 0) display.print("NORMAL");
      else if (activeTxPower == 4) display.print("STRONG");
    }

    drawClientSlotRow(16);
    drawErrorPopupIfActive();
    display.display();
    return;
  }

  if (currentState == SCAN_MENU) {
    removeOldDevices();
    sortDevices();
    displayDeviceCount = deviceCount;
    if (isTargetLocked) {
      for (int i = 0; i < deviceCount; i++) {
        if (macEqual(devices[i].mac, lockedMac)) {
          scanMenuSelection = i + 1;
          encoderPos = scanMenuSelection;
          break;
        }
      }
    }
    display.fillRect(0, 0, 128, 14, WHITE);
    display.setTextColor(BLACK);
    display.setCursor(2, 3);
    display.print("SCANNING SLOT");
    drawInvertedNumber(display.getCursorX() + 2, 2, selectedSlotIdx);
    drawScanMenu();
  } else if (currentState == MAIN_MENU) {
    display.fillRect(0, 0, 128, 14, WHITE);

    drawStatusBarLeft();

    // DIN MIDI: [M] 아이콘
    if (!TinyUSBDevice.mounted()) {
      display.fillRect(75, 1, 9, 12, BLACK);
      display.setTextColor(WHITE);
      display.setCursor(77, 3);
      display.print("M");
    }

    // 실제 송신 전력이 8dBm일 때만 BOOST 표시 (검색 필요 상태와 표시를 분리)
    if (activeTxPower == 8) {
      display.fillRect(85, 1, 42, 12, BLACK);
      display.setTextColor(WHITE);
      display.setCursor(92, 3);
      display.print("BOOST");
    } else {
      display.drawRect(85, 1, 42, 12, BLACK);
      display.setTextColor(BLACK);
      display.setCursor(89, 3);
      if (activeTxPower == 0) display.print("NORMAL");
      else if (activeTxPower == 4) display.print("STRONG");
    }

    if (currentSystemMode == MODE_HOST) {
      drawSlotRow(16, 0);
      drawSlotRow(33, 1);
      drawSlotRow(50, 2);
    } else {
      display.drawRect(0, 16, 128, 48, WHITE);
      if (Bluefruit.connected()) {
        display.setCursor(8, 28);
        display.print("MASTER:");
        display.setCursor(8, 42);
        if (pendingName.length() > 0) display.print(pendingName.c_str());
        else display.print("UNKNOWN");
      } else {
        display.setCursor(20, 38);
        display.print(F("ADVERTISING..."));
      }
    }
  } else if (currentState == SLOT_MENU) {
    drawSlotMenu();
  }

  drawErrorPopupIfActive();

  display.display();
}

void drawInvertedNumber(int x, int y, int num) {
  display.fillRect(x, y, 9, 10, BLACK);
  display.setTextColor(WHITE);
  display.setCursor(x + 2, y + 1);
  display.print(num);
  display.setTextColor(BLACK);
}

// OLED 밝기 직접 제어 (SSD1306 명령)
void setOLEDContrast(uint8_t contrast) {
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(contrast);
}

// 조작 시간에 따른 자동 밝기 관리
void handleOLEDDimming(unsigned long now) {
  // 1. 배터리 상태 확인 (100 이상이면 USB 연결 중)
  uint8_t bat = readHostBattery();
  bool isUSB = (bat >= 100);

  // 2. 모드별 타겟 밝기 설정
  uint8_t normalBrightness = isUSB ? 150 : 50;

  if (now - lastInteractionTime > DIM_TIMEOUT) {
    if (!isDimmed) {
      setOLEDContrast(1);
      isDimmed = true;
      Serial.println("OLED Dimmed (Power Save)");
    }
  } else {
    static uint8_t lastTarget = 0;
    if (isDimmed || lastTarget != normalBrightness) {
      setOLEDContrast(normalBrightness);
      isDimmed = false;
      lastTarget = normalBrightness;
      Serial.printf("OLED Brightness Set to: %d (%s)\n", normalBrightness, isUSB ? "USB" : "BAT");
    }
  }
}

void drawScanMenu() {
  display.setTextColor(WHITE);
  display.drawRect(0, 16, 128, 48, WHITE);  // 테두리

  // 상단 스캔 카운트
  display.setCursor(5, 20);
  display.print("Found: ");
  display.println(displayDeviceCount);

  // 현재 선택된 번호(scanMenuSelection)를 무조건 첫 줄(y=32)에 표시
  int startIdx = scanMenuSelection;

  for (int i = 0; i < 3; i++) {
    int itemIdx = startIdx + i;
    int yPos = 32 + (i * 10);  // 32, 42, 52

    // 전체 항목 수(장치수 + BACK 1개)를 넘어가면 그리지 않음
    if (itemIdx > displayDeviceCount) break;

    // 첫 줄만 하이라이트
    if (i == 0) {
      display.fillRect(2, yPos - 1, 124, 10, WHITE);
      display.setTextColor(BLACK);
    } else {
      display.setTextColor(WHITE);
    }

    display.setCursor(5, yPos);

    if (itemIdx == 0) {
      display.println("<< BACK");
    } else {
      // 실제 장치 데이터 인덱스는 (항목번호 - 1)
      int devIdx = itemIdx - 1;
      if (devIdx < displayDeviceCount) {
        String n = String(devices[devIdx].name);
        if (n.length() > 14) n = n.substring(0, 14);
        display.print(n);

        display.setCursor(100, yPos);
        display.print(devices[devIdx].rssi);
      }
    }
  }
}

void drawSlotRow(int y, int slotIdx) {
  bool isSelected = (currentSelection == slotIdx);
  uint16_t color = isSelected ? BLACK : WHITE;
  uint16_t bgColor = isSelected ? WHITE : BLACK;

  // 1. 슬롯 전체 배경 및 테두리 (14px 높이)
  if (isSelected) {
    display.fillRect(0, y, 128, 14, WHITE);
  } else {
    display.drawRect(0, y, 128, 14, WHITE);
  }

  // 2. [슬롯 번호 반전 박스]
  display.fillRect(2, y + 2, 10, 10, color);
  display.setTextColor(bgColor);
  display.setCursor(4, y + 3);
  display.print(slotIdx);

  // 3. 기기 이름 또는 상태 표시 (X=14 고정 시작)
  display.setCursor(14, y + 3);

  if (slots[slotIdx].status == CONNECTED) {
    // --- [이름 출력] ---
    String devName = String(slots[slotIdx].name);
    if (devName.length() > 10) devName = devName.substring(0, 10);  // 최대 10자 제한
    display.setTextColor(color);
    display.print(devName);

    // 1. 컨트롤러 입력 (정삼각형 △) - 74번 위치
    if (millis() - lastMidiTime[slotIdx] < 100) {
      display.setCursor(81, y + 4);
      display.write(30);  // ASCII 30: 위를 향한 삼각형
    }

    // 2. 에이블톤 출력 (역삼각형 ▽) - 82번 위치
    if (millis() - lastUsbMidiTime[slotIdx] < 100) {
      display.setCursor(75, y + 3);
      display.write(31);  // ASCII 31: 아래를 향한 삼각형
    }

    // --- [5. 안테나] RSSI 측정 및 필터링 ---
    int8_t rawRssi = -128;
    for (uint8_t i = 0; i < BLE_MAX_CONNECTION; i++) {
      BLEConnection *conn = Bluefruit.Connection(i);
      if (conn && conn->connected() && macEqual(slots[slotIdx].mac, conn->getPeerAddr().addr)) {
        rawRssi = conn->getRssi();
        break;
      }
    }

    // 👉 필터 + 주기 제한 + 더 차분하게
    static float smoothRssi[3] = { -70.0, -70.0, -70.0 };
    static unsigned long lastRssiUpdate[3] = { 0, 0, 0 };

    unsigned long now = millis();

    if (rawRssi > -120 && rawRssi < 0) {
      if (now - lastRssiUpdate[slotIdx] > 200) {  // 200ms 주기
        lastRssiUpdate[slotIdx] = now;
        smoothRssi[slotIdx] = (rawRssi * 0.1f) + (smoothRssi[slotIdx] * 0.9f);  // 👉 더 차분
      }
    }

    float rssi = smoothRssi[slotIdx];
    static int lastBars[3] = { 0, 0, 0 };
    int currentBars = antennaBarsFromRssi(rssi, lastBars[slotIdx]);
    lastBars[slotIdx] = currentBars;
    int bars = currentBars;

    drawAntennaBars(89, y, bars, color);

    int batBoxX = 102;

    if (slots[slotIdx].battery >= 1) {
      int batVal = slots[slotIdx].battery;

      if (batVal > 40) {
        display.fillRect(batBoxX, y + 2, 24, 10, color);
        display.setTextColor(bgColor);

      } else {
        if (batVal <= 20) {
          // 👉 프레임 기반 깜빡임
          static uint8_t blinkCounter = 0;
          blinkCounter++;

          if (blinkCounter >= 30) {  // 속도 조절
            blinkCounter = 0;
            blinkState = !blinkState;
          }

          if (blinkState) {
            display.drawRect(batBoxX, y + 2, 24, 10, color);
          }

        } else {
          display.drawRect(batBoxX, y + 2, 24, 10, color);
        }

        display.setTextColor(color);
      }

      int xOff = (batVal >= 100) ? 1 : (batVal >= 10 ? 3 : 6);
      display.setCursor(batBoxX + xOff, y + 3);
      display.print(batVal);
      display.print("%");

    } else if (remoteHostBat >= 100) {
      display.fillRect(batBoxX, y + 2, 24, 10, color);
      display.setTextColor(bgColor);
      display.setCursor(batBoxX + 4, y + 3);
      display.print("EXT");

    } else {
      display.drawRect(batBoxX, y + 2, 24, 10, color);
      display.setTextColor(color);
      display.setCursor(batBoxX + 3, y + 3);
      display.print("N/A");
    }

  } else {
    // --- [연결되지 않은 경우] ---
    display.setTextColor(color);

    if (slots[slotIdx].status == EMPTY) {
      // 1. 아예 비어있는 슬롯
      display.print(" ");
    } else if (slots[slotIdx].status == OFFLINE) {
      // 2. 등록은 되어있으나 연결이 끊긴 상태 (OFFLINE - 이름)
      display.print("OFFLINE:");

      // 저장된 이름 가져오기 (최대 글자수 조절)
      String storedName = String(slots[slotIdx].name);
      if (storedName.length() > 10) storedName = storedName.substring(0, 10);  // 공간 확보를 위해 8자 제한
      display.print(storedName);
    }
  }
}

// 1. 이름순 정렬 함수 추가 (기기 위치 고정용)
void sortDevices() {
  for (int i = 0; i < deviceCount - 1; i++) {
    for (int j = i + 1; j < deviceCount; j++) {
      // strcmp를 사용하여 이름순(ABC/가나다) 정렬
      if (strcmp(devices[i].name, devices[j].name) > 0) {
        Device temp = devices[i];
        devices[i] = devices[j];
        devices[j] = temp;

        ble_gap_evt_adv_report_t tempReport = device_reports[i];
        device_reports[i] = device_reports[j];
        device_reports[j] = tempReport;
      }
    }
  }
}

// --- [모드 저장/불러오기 실제 구현부] ---

void saveSystemMode(SystemMode mode) {
  // 1. 기존 파일 삭제 (가장 확실한 방법)
  if (InternalFS.exists("/sys_mode.bin")) {
    InternalFS.remove("/sys_mode.bin");
  }

  // 2. 라이브러리 정의에 따른 플래그 조합으로 열기
  // FILE_O_WRITE: 쓰기 모드
  // FILE_O_CREAT: 파일이 없으면 생성
  // (내부적으로 Adafruit_LittleFS_Namespace에 정의되어 있습니다)
  File file = InternalFS.open("/sys_mode.bin", Adafruit_LittleFS_Namespace::FILE_O_WRITE);

  if (file) {
    uint8_t data = (uint8_t)mode;
    file.write(&data, 1);
    file.flush();
    file.close();

    Serial.print(">>> SYSTEM MODE SAVED: ");
    Serial.println(mode == MODE_HOST ? "HOST" : "CLIENT");
  } else {
    Serial.println(">>> ERROR: Failed to open /sys_mode.bin for writing!");
  }
}

SystemMode loadSystemMode() {
  SystemMode mode = MODE_HOST;  // 기본값

  // 파일이 없으면 기본값 반환, 있으면 읽기
  if (InternalFS.exists("/sys_mode.bin")) {
    File file = InternalFS.open("/sys_mode.bin", FILE_O_READ);
    if (file) {
      uint8_t data;
      file.read(&data, 1);
      mode = (SystemMode)data;
      file.close();
    }
  }
  return mode;
}

void manageScanner(unsigned long now) {
  static unsigned long lastScanCheck = 0;
  if (now - lastScanCheck < 2000) return;
  lastScanCheck = now;

  if (currentSystemMode != MODE_HOST) return;

  // 1. "실제로 기기가 등록되어 있는데, 현재 연결 안 된 슬롯"이 있는지 체크
  bool needToSearch = false;

  for (int i = 0; i < 3; i++) {
    // [판단 기준]
    // 1. MAC 주소의 첫 바이트가 0이 아니다 = 기기 정보가 저장되어 있다
    // 2. 현재 상태가 CONNECTED가 아니다 = 연결이 필요하다
    if (slots[i].mac[0] != 0 && (slots[i].status != CONNECTED)) {
      needToSearch = true;
      break;
    }
  }

  // 2. 사용자가 스캔 메뉴에 진입한 경우 (수동 등록 모드)
  if (currentState == SCAN_MENU) {
    Bluefruit.Scanner.setInterval(200, 80);
    if (!Bluefruit.Scanner.isRunning()) Bluefruit.Scanner.start(0);
    return;
  }

  // 3. 메인 화면인데 "찾아야 할 등록 기기"가 있는 경우만 백그라운드 스캔
  if (needToSearch) {
    Bluefruit.Scanner.setInterval(300, 80);  // 연주에 방해 안 되게 최적화된 주기
    if (!Bluefruit.Scanner.isRunning()) {
      Bluefruit.Scanner.start(0);
      Serial.println(F(">>> BACKGROUND SCAN (Searching for Stored Devices) <<<"));
    }
  }
  // 4. 저장된 게 없거나 다 연결되었으면 스캔 완전 중지
  else {
    // 실행 중이 아니더라도 "상태가 바뀌었을 때" 한 번만 찍도록 변경
    static bool lastStateWasOn = false;
    if (Bluefruit.Scanner.isRunning()) {
      Bluefruit.Scanner.stop();
      lastStateWasOn = true;
    } else if (lastStateWasOn) {
      Serial.println(F(">>> SCANNER OFF (Confirmed) <<<"));
      lastStateWasOn = false;
    }
  }
}

// (이전<<2)|현재 → -1,0,+1 (비정상 전이·채터는 0)
static const int8_t ENC_QDEC_LUT[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void encoderISR() {
  uint8_t curr = ((uint8_t)digitalRead(ENCODER_A) << 1) | (uint8_t)digitalRead(ENCODER_B);
  uint8_t idx = (uint8_t)((encPrevState << 2) | curr);
  int8_t delta = ENC_QDEC_LUT[idx];
  encPrevState = curr;
  if (delta) {
    // 반대 방향으로 첫 펄스가 오면 잔여 누적(+/- 몇)과 상쇄되어 첫 디텐트가 묻힘 → 방향 바뀌면 새 방향만 남김
    if ((encoderAccumulator > 0 && delta < 0) || (encoderAccumulator < 0 && delta > 0)) {
      encoderAccumulator = delta;
    } else {
      encoderAccumulator += delta;
    }
  }
}

void handleEncoder(unsigned long now) {
  if (abs(encoderAccumulator) < ENC_STEPS_PER_DETENT) return;
  if (now - lastEncoderMoveTime < ENCODER_GAP) return;

  int moveStep = (encoderAccumulator > 0) ? 1 : -1;
  encoderAccumulator -= moveStep * ENC_STEPS_PER_DETENT;

  lastEncoderMoveTime = now;
  lastInteractionTime = now;

  // --- 상태별 엔코더 동작 ---
  if (currentState == MODE_SELECT) {
    encoderPos = constrain(encoderPos + moveStep, 0, 1);
    modeSelection = (int)encoderPos;
  } else if (currentState == MAIN_MENU) {
    if (currentSystemMode == MODE_HOST) {
      // [호스트 모드] 기존 슬롯 0, 1, 2 이동 허용
      if (currentSelection == -1) encoderPos = 0;
      else encoderPos = constrain(encoderPos + moveStep, 0, 2);
      currentSelection = (int)encoderPos;
    } else {
      // [클라이언트 모드] 슬롯 개념이 없으므로 선택을 -1(없음)로 고정
      // 이렇게 하면 호스트용 슬롯 UI가 나타나지 않습니다.
      currentSelection = -1;
      encoderPos = 0;
    }
  } else if (currentState == SLOT_MENU) {
    slotMenuSelection = constrain(slotMenuSelection + moveStep, 0, 2);
    encoderPos = slotMenuSelection;
  } else if (currentState == SCAN_MENU) {
    int totalOptions = displayDeviceCount + 1;
    scanMenuSelection = constrain(scanMenuSelection + moveStep, 0, totalOptions - 1);
    encoderPos = scanMenuSelection;

    // [중요] 타겟 락 로직은 SCAN_MENU 블록 '내부'에 있어야 합니다.
    if (scanMenuSelection == 0) {
      isTargetLocked = false;
      isBackSelected = true;
      memset(lockedMac, 0, 6);
    } else {
      int devIdx = scanMenuSelection - 1;
      if (devIdx >= 0 && devIdx < displayDeviceCount) {
        memcpy(lockedMac, devices[devIdx].mac, 6);
        isTargetLocked = true;
        isBackSelected = false;
      }
    }
  } else if (currentState == CLIENT_MENU) {
    clientMenuSelection = constrain(clientMenuSelection + moveStep, 0, 1);
    encoderPos = clientMenuSelection;
  }
}

void handleButton(unsigned long now) {
  static bool isLongPressed = false;
  static bool isUltraLongPressed = false;
  bool currButton = (digitalRead(BUTTON_PIN) == LOW);

  if (currButton && !buttonState) {
    buttonPressStartTime = now;
    buttonState = true;
    isLongPressed = false;
  }

  // 버튼을 누르고 있는 동안 실시간 감시
  if (currButton && buttonState) {
    unsigned long duration = now - buttonPressStartTime;

    // --- [5초 이상 누를 시: 부트로더 진입 카운트다운] ---
    if (duration > 5000) {
      int countdown = 8 - (duration / 1000);
      if (countdown > 0) {
        display.clearDisplay();

        // 1. 상단 타이틀 (Font Size 1, 약 13글자)
        // (128 - (13 * 6)) / 2 = 25
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(4, 18);  // 약간 위로 올림
        display.print(F("FIRMWARE UPDATE MODE"));

        display.setCursor(61, 32);
        display.print(countdown);

        // 3. 하단 안내 문구 (Font Size 1, 약 17글자)
        // (128 - (17 * 6)) / 2 = 13
        display.setTextSize(1);
        display.setCursor(13, 46);  // 하단 여백 확보
        display.print(F("RELEASE TO CANCEL"));

        display.display();
      }

      // 8초 도달 시 실제 진입
      if (duration > 8000 && !isUltraLongPressed) {
        isUltraLongPressed = true;
        Serial.println(F(">>> JUMPING TO BOOTLOADER... <<<"));

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);

        // 가로 중앙: (128px - (15글자 * 6px)) / 2 = 19
        // 세로 중앙: (64px - 8px) / 2 = 28
        display.setCursor(17, 28);
        display.print(F("BOOTLOADER READY"));
        display.display();

        delay(500);
        NRF_POWER->GPREGRET = MAGIC_UF2_BOOTLOADER;
        NVIC_SystemReset();
      }
    }

    // --- 기존 1.2초 롱프레스: 시스템 모드 선택 ---
    if (!isLongPressed && !isUltraLongPressed && duration > 1200) {
      isLongPressed = true;
      lastInteractionTime = now;
      currentState = MODE_SELECT;
      modeSelection = 0;
      encoderPos = 0;
    }
  }

  // 짧은 클릭
  if (!currButton && buttonState) {
    unsigned long duration = now - buttonPressStartTime;

    // --- [추가] 5초 이상 눌렀으나 8초 전에 뗀 경우: 취소 처리 ---
    if (duration > 5000 && duration <= 8000) {
      Serial.println(F(">>> BOOTLOADER CANCELED <<<"));
      currentState = MAIN_MENU;
      currentSelection = -1;
    }

    else if (!isLongPressed && duration > 50) {
      lastInteractionTime = now;

      // A. 시스템 모드 선택
      if (currentState == MODE_SELECT) {
        if (modeSelection == 0) {
          // [0: BACK] 선택 시
          currentState = MAIN_MENU;
          currentSelection = -1;
          encoderPos = 0;
        } else if (modeSelection == 1) {
          // [1: MODE CHANGE] 선택 시
          SystemMode selected = (currentSystemMode == MODE_HOST) ? MODE_CLIENT : MODE_HOST;
          saveSystemMode(selected);

          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(37, 30);
          display.print(F("REBOOTING"));
          display.display();

          Serial.println(F(">>> MODE CHANGED: REBOOTING SYSTEM <<<"));
          NVIC_SystemReset();
        }
      }

      // B. 메인 메뉴 상태일 때
      else if (currentState == MAIN_MENU) {
        if (currentSystemMode == MODE_HOST) {
          // [호스트 로직]
          if (currentSelection == -1) {
            currentSelection = 0;
          } else {
            selectedSlotIdx = currentSelection;
            if (slots[selectedSlotIdx].status == EMPTY) {
              currentState = SCAN_MENU;
              scanMenuSelection = 0;
              encoderPos = 0;
            } else {
              currentState = SLOT_MENU;
              slotMenuSelection = 0;
              encoderPos = 0;
            }
          }
        } else {

          // 1. 저장 카운트다운 중일 때 버튼을 누르면? -> 저장만 취소!
          if (!isMasterSavedThisSession && connectionStartTime != 0) {
            Serial.println(F(">>> AUTO-SAVE CANCELED (KEEP CONNECTION) <<<"));

            connectionStartTime = 0;  // 카운트다운 루프 중단 (화면에서 바 사라짐)
            isMasterSavedThisSession = true;
            strncpy(errorMsg, "SAVE CANCELED", 19);
            errorMsg[19] = '\0';
            errorMsgUntil = millis() + 1500;
            // 💡 여기서 return하지 않고 그대로 두면, 다음 클릭 시 메뉴 진입이 가능해집니다.
          }

          // 2. 메뉴 진입 조건 변경:
          // (저장된 기기가 있거나) OR (현재 기기가 연결되어 있는 상태)라면 메뉴 진입 허용
          else if (config.hasSavedMaster || Bluefruit.connected()) {
            currentState = CLIENT_MENU;
            clientMenuSelection = 0;
            encoderPos = 0;
            Serial.println(F(">>> ENTERING CLIENT MENU <<<"));
          }

          // 3. 아무것도 없을 때 (연결도 없고 저장된 기기도 없을 때)
          else {
            strncpy(errorMsg, "NO DEVICE", 19);
            errorMsgUntil = millis() + 1500;
          }
        }
      }

      // 클라이언트 메뉴
      else if (currentState == CLIENT_MENU) {
        if (clientMenuSelection == 0) {
          // 0: BACK (메인 화면으로 복귀)
          currentState = MAIN_MENU;
          currentSelection = -1;
          encoderPos = 0;
        } else if (clientMenuSelection == 1) {
          // 1: CLEAR & BLOCK (또는 DISCONNECT & BLOCK)
          // 저장 마스터가 플래시에만 남아 RAM은 false인 경우가 있어, 이 메뉴에서는 항상 마스터 슬롯을 플래시까지 지움.
          bool hadSavedMaster = config.hasSavedMaster;

          memset(config.savedMasterMac, 0, 6);
          config.savedMasterName[0] = '\0';
          config.hasSavedMaster = false;
          memset(slots[0].mac, 0, 6);
          memset(slots[0].name, 0, NAME_LEN);
          slots[0].status = EMPTY;
          isMasterSavedThisSession = false;
          connectionStartTime = 0;
          saveConfig();
          Serial.println(F(">>> FLASH: CLIENT MASTER CLEARED <<<"));
          activeTxPower = 4;
          currentTxMode = POWER_STRONG;
          Bluefruit.setTxPower(4);

          // [Step 2] 연결 끊기. 저장된 마스터를 방금 지운 경우에는 30초 블랙리스트를 쓰지 않음
          // (블랙리스트면 connect_callback 입구에서 끊겨 connectionStartTime/카운트다운이 아예 안 잡힘)
          if (Bluefruit.connected()) {
            BLEConnection *conn = Bluefruit.Connection(0);
            if (conn) {
              ble_gap_addr_t addr = conn->getPeerAddr();

              if (!hadSavedMaster) {
                memcpy(ignoredMac, addr.addr, 6);
                ignoreUntil = millis() + 30000;
                strncpy(ignoredPeerName, clientSlot.name, NAME_LEN - 1);
                ignoredPeerName[NAME_LEN - 1] = '\0';
                if (ignoredPeerName[0] == '\0') {
                  snprintf(ignoredPeerName, NAME_LEN, "ID:%02X%02X%02X", addr.addr[2], addr.addr[1], addr.addr[0]);
                }
                Serial.print(F(">>> BLOCKING MAC: "));
                for (int i = 5; i >= 0; i--) {
                  Serial.print(addr.addr[i], HEX);
                  if (i > 0) Serial.print(":");
                }
                Serial.println(F(" (For 30s)"));
              } else {
                memset(ignoredMac, 0, 6);
                ignoreUntil = 0;
                ignoredPeerName[0] = '\0';
                Serial.println(F(">>> Saved master cleared — no BLE block (re-pair OK) <<<"));
              }

              conn->disconnect();
            }
          }

          if (hadSavedMaster) {
            strncpy(errorMsg, "MASTER CLEARED", 19);
          } else {
            strncpy(errorMsg, "BLOCKED 30 SEC", 19);
          }
          errorMsg[19] = '\0';
          errorMsgUntil = millis() + 2000;

          // 처리 후 메인 화면으로 이동
          currentState = MAIN_MENU;
          currentSelection = -1;
          encoderPos = 0;
        }
      }

      // C. 스캔 메뉴
      else if (currentState == SCAN_MENU) {
        if (scanMenuSelection == 0) {
          currentState = MAIN_MENU;
          currentSelection = -1;
          selectedSlotIdx = -1;
        } else {
          int devIdx = scanMenuSelection - 1;
          if (isTargetLocked && macEqual(lockedMac, devices[devIdx].mac)) {
            Bluefruit.Scanner.stop();
            delay(100);
            Bluefruit.Central.connect(&device_reports[devIdx]);
            currentState = MAIN_MENU;
            currentSelection = -1;
          }
        }
      }

      // D. 슬롯 메뉴
      else if (currentState == SLOT_MENU) {

        // 0: BACK
        if (slotMenuSelection == 0) {
          currentState = MAIN_MENU;
          currentSelection = -1;
          selectedSlotIdx = -1;
        }

        // 1: RESCAN (← 추가해야 정상 동작)
        else if (slotMenuSelection == 1) {
          currentState = SCAN_MENU;
          scanMenuSelection = 0;
          encoderPos = 0;

          // 기존 타겟 락 해제
          isTargetLocked = false;

          // 스캐너 시작 보장
          if (!Bluefruit.Scanner.isRunning()) {
            Bluefruit.Scanner.start(0);
          }
        }

        // 2: DELETE (← 핵심 수정)
        else if (slotMenuSelection == 2) {

          // 연결 끊기
          for (uint8_t h = 0; h < BLE_MAX_CONNECTION; h++) {
            BLEConnection *conn = Bluefruit.Connection(h);
            if (conn && conn->connected()) {
              ble_gap_addr_t addr = conn->getPeerAddr();

              if (macEqual(slots[selectedSlotIdx].mac, addr.addr)) {
                conn->disconnect();
                break;
              }
            }
          }

          // 슬롯 비우기
          slots[selectedSlotIdx].status = EMPTY;
          memset(slots[selectedSlotIdx].mac, 0, 6);
          memset(slots[selectedSlotIdx].name, 0, NAME_LEN);

          saveConfig();

          // 상태 초기화
          currentState = MAIN_MENU;
          currentSelection = -1;
          selectedSlotIdx = -1;
          encoderPos = 0;
        }
      }
    }

    buttonState = false;
    delay(50);
  }
}

void drawModeSelect() {
  // 부트로더 카운트다운 중에는 시스템 메뉴 미표시
  // handleButton의 카운트다운 화면이 우선권을 갖도록 양보하는 로직입니다.
  if (buttonState && (millis() - buttonPressStartTime > 5000)) {
    // updateDisplay()가 이미 clearDisplay() 했으므로 검은 화면을 OLED에 반영
    display.display();
    return;
  }

  display.clearDisplay();

  // 상단 타이틀
  display.fillRect(0, 0, 128, 14, WHITE);
  display.setTextColor(BLACK);
  display.setCursor(2, 3);
  display.print(F("SYSTEM MODE"));
  display.setCursor(84, 3);
  display.print(F("Ver 1.0"));

  display.drawRect(0, 16, 128, 48, WHITE);

  // 현재 모드에 따라 메뉴 리스트 구성
  const char *options[2];
  options[0] = "<< BACK";
  options[1] = (currentSystemMode == MODE_HOST) ? "TO CLIENT MODE" : "TO HOST MODE";

  for (int i = 0; i < 2; i++) {
    int yPos = 24 + (i * 12);

    if (modeSelection == i) {
      display.fillRect(2, yPos - 2, 124, 13, WHITE);
      display.setTextColor(BLACK);
    } else {
      display.setTextColor(WHITE);
    }

    display.setCursor(5, yPos);
    display.println(options[i]);
  }

  display.display();
}

void checkOfflineSlots(unsigned long now) {
  static unsigned long lastSlotCheck = 0;
  if (now - lastSlotCheck > 2000) {
    lastSlotCheck = now;
    bool slotChanged = false;

    for (int i = 0; i < 3; i++) {
      // [핵심] EMPTY인 슬롯은 검사하지 않고 건너뜁니다 (continue)
      if (slots[i].status == EMPTY) continue;

      if (slots[i].status == CONNECTED) {
        bool stillConnected = false;
        for (uint8_t conn_h = 0; conn_h < BLE_MAX_CONNECTION; conn_h++) {
          BLEConnection *conn = Bluefruit.Connection(conn_h);
          if (conn && conn->connected()) {
            ble_gap_addr_t addr = conn->getPeerAddr();
            if (macEqual(slots[i].mac, addr.addr)) {
              stillConnected = true;
              break;
            }
          }
        }

        if (!stillConnected) {
          Serial.printf("Slot %d: Connection lost.\n", i + 1);
          slots[i].status = OFFLINE;
          slotChanged = true;
        }
      }
    }
  }
}

void handleUITimeout(unsigned long now) {
  // [수정] 이미 메인 메뉴이고 커서도 초기화된 상태(-1)라면,
  // 더 이상 타임아웃 로직을 실행하지 않고 리턴합니다.
  // 이렇게 해야 lastInteractionTime이 5000ms 이상으로 계속 누적되어 Dimming이 작동합니다.
  if (currentState == MAIN_MENU && currentSelection == -1) {
    return;
  }

  // 조작이 없는지 확인
  if (now - lastInteractionTime > UI_TIMEOUT) {

    // 1. 메인 메뉴인데 커서만 활성화된 경우 -> 커서 초기화
    if (currentState == MAIN_MENU && currentSelection != -1) {
      currentSelection = -1;
      selectedSlotIdx = -1;
      encoderPos = 0;
      Serial.println(F("UI Timeout: Cursor Reset"));
    }
    // 2. 각종 메뉴 상태일 때 -> 메인 화면으로 강제 복귀
    else if (currentState == MODE_SELECT || currentState == SLOT_MENU || currentState == SCAN_MENU || currentState == CLIENT_MENU) {
      currentState = MAIN_MENU;
      currentSelection = -1;
      selectedSlotIdx = -1;
      encoderPos = 0;
      isTargetLocked = false;
      Serial.println(F("UI Timeout: Returned to Main Menu"));
    }

    // [중요] 타임아웃 처리가 완료되었음을 표시하기 위해 시간을 밀어줌
    // 위에서 이미 메인 메뉴인 경우 return 처리를 했으므로,
    // 이 코드는 '메뉴 복귀'라는 액션이 일어난 직후에만 한 번 실행됩니다.
    lastInteractionTime = now;
  }
}

void handleExternalMidiIn() {
  // 1. 연결 상태 확인
  bool isConnected = (currentSystemMode == MODE_CLIENT) ? Bluefruit.connected() : (slots[0].status == CONNECTED);

  // USB 마운트 또는 비연결 시 시리얼 MIDI 스킵
  if (TinyUSBDevice.mounted() || !isConnected) {
    while (Serial1.available()) Serial1.read();
    return;
  }

  static uint8_t noteVel[128] = { 0 };
  unsigned long now = millis();
  uint8_t header = 0x80;
  uint8_t ts = (now & 0x7F) | 0x80;

  while (Serial1.available()) {
    uint8_t status = Serial1.peek();

    // 실시간 메시지: 송신 삼각형 타이밍
    if (status >= 0x80) {
      lastMidiTime[0] = now;
    }

    // --- 실시간 메시지 처리 (Clock 등) ---
    if (status >= 0xF8) {
      status = Serial1.read();
      uint8_t bleRT[3] = { header, ts, status };

      if (currentSystemMode == MODE_CLIENT) {
        client_midi_chr.notify(bleRT, 3);
      } else {
        ble_midi_chr[0].write(bleRT, 3);
      }
    }

    // --- 일반 미디 메시지 처리 (Note, CC 등) ---
    else if (status >= 0x80) {
      int expectedLen = ((status & 0xF0) == 0xC0 || (status & 0xF0) == 0xD0) ? 2 : 3;

      if (Serial1.available() >= expectedLen) {
        uint8_t d[3] = { 0, 0, 0 };
        for (int i = 0; i < expectedLen; i++) d[i] = Serial1.read();

        uint8_t type = d[0] & 0xF0;
        if (type == 0x90 && d[2] == 0) type = 0x80;

        bool send = false;
        if (type == 0x90 || type == 0x80) {
          if (type == 0x90 && noteVel[d[1]] != d[2]) {
            noteVel[d[1]] = d[2];
            send = true;
          } else if (type == 0x80 && noteVel[d[1]] > 0) {
            noteVel[d[1]] = 0;
            send = true;
          }
        } else send = true;

        if (send) {
          uint8_t blePKT[5] = { header, ts, d[0], d[1], d[2] };

          if (currentSystemMode == MODE_CLIENT) {
            client_midi_chr.notify(blePKT, 2 + expectedLen);
          } else {
            ble_midi_chr[0].write(blePKT, 2 + expectedLen);
          }
        }
      } else {
        break;
      }
    } else {
      Serial1.read();
    }
  }
}

void ble_midi_rx_callback(BLEClientCharacteristic *chr, uint8_t *data, uint16_t len) {
  if (len < 3) return;

  int slotIdx = -1;
  for (int i = 0; i < 3; i++) {
    if (&ble_midi_chr[i] == chr) {
      slotIdx = i;
      break;
    }
  }

  if (slotIdx != -1) {
    uint8_t status = data[2];
    uint8_t d1 = (len > 3) ? data[3] : 0;
    uint8_t d2 = (len > 4) ? data[4] : 0;

    // 1. USB 패킷 생성
    uint8_t packet[4] = { (uint8_t)((slotIdx << 4) | (status >> 4)), status, d1, d2 };
    if (status >= 0xF8) packet[0] = (slotIdx << 4) | 0x0F;

    // 2. PC 연결 상태에 따른 경로 분기
    if (TinyUSBDevice.mounted() && usb_midi_ptr != NULL) {
      // ✅ PC 연결 시: 오직 USB로만 전송 (DIN 출력 차단으로 병목 및 간섭 방지)
      usb_midi_ptr->writePacket(packet);
    } else {
      // ✅ 보조배터리(Standalone) 모드일 때만 DIN MIDI(Serial1)로 출력
      // PC가 없을 때만 기기가 직접 '브릿지' 역할을 수행합니다.
      if (slotIdx == 0) {
        Serial1.write(status);

        if (status < 0xF8) {
          Serial1.write(d1);
          if ((status & 0xF0) != 0xC0 && (status & 0xF0) != 0xD0) {
            Serial1.write(d2);
          }
        }
      }
    }

    lastUsbMidiTime[slotIdx] = millis();  // lastMidiTime -> lastUsbMidiTime으로 변경
  }
}

void ble_midi_peripheral_rx_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
  if (len < 3) return;

  // 1. 데이터 추출 (BLE MIDI 프로토콜: index 2부터 실제 MIDI 데이터 시작)
  uint8_t status = data[2];
  uint8_t d1 = (len > 3) ? data[3] : 0;
  uint8_t d2 = (len > 4) ? data[4] : 0;

  // 2. PC 연결 상태에 따른 경로 분기
  if (TinyUSBDevice.mounted() && usb_midi_ptr != NULL) {
    // ✅ [CASE 1] PC 연결 시: 오직 USB MIDI로만 전송 (DIN 차단)
    uint8_t packet[4] = { 0 };
    uint8_t cin = (status >= 0xF8) ? 0x0F : (status >> 4);
    packet[0] = (0 << 4) | cin;  // Cable 0
    packet[1] = status;
    packet[2] = d1;
    packet[3] = d2;

    usb_midi_ptr->writePacket(packet);
  } else {
    // ✅ [CASE 2] 보조배터리(Standalone) 모드: DIN MIDI(Serial1)로 출력
    // PC가 없을 때만 이 기기가 스마트폰-악기 사이의 브릿지 역할을 합니다.
    Serial1.write(status);

    if (status < 0xF8) {
      Serial1.write(d1);
      // 3바이트 메시지 체크 (Program Change 0xC0, Channel Pressure 0xD0 제외)
      if ((status & 0xF0) != 0xC0 && (status & 0xF0) != 0xD0) {
        Serial1.write(d2);
      }
    }
  }

  // 3. UI 알림 및 상태 업데이트
  lastUsbMidiTime[0] = millis();  // lastMidiTime -> lastUsbMidiTime으로 변경
}

static bool findScanListNameForMac(const uint8_t mac[6], char *out, size_t outSz) {
  if (!out || outSz == 0) return false;
  out[0] = '\0';
  for (int i = 0; i < deviceCount; i++) {
    if (macEqual(devices[i].mac, mac) && devices[i].name[0] != '\0') {
      strncpy(out, devices[i].name, outSz - 1);
      out[outSz - 1] = '\0';
      return true;
    }
  }
  return false;
}

void connect_callback(uint16_t conn_handle) {
  // 1. 연결 객체 및 주소 확보
  BLEConnection *conn = Bluefruit.Connection(conn_handle);
  if (!conn) return;

  ble_gap_addr_t peer_addr = conn->getPeerAddr();

  // ================= [입구 컷: 블랙리스트 차단 로직 보강] =================
  if (millis() < ignoreUntil) {
    // 🔍 [디버깅 로그] 차단 리스트와 들어온 주소를 대조 출력
    Serial.println(F("--- [BLOCK CHECK] ---"));
    Serial.print(F("  Incoming MAC: "));
    for (int i = 5; i >= 0; i--) {
      Serial.print(peer_addr.addr[i], HEX);
      if (i > 0) Serial.print(":");
    }
    Serial.println();

    Serial.print(F("  Ignored  MAC: "));
    for (int i = 5; i >= 0; i--) {
      Serial.print(ignoredMac[i], HEX);
      if (i > 0) Serial.print(":");
    }
    Serial.println();

    // 주소 대조 실행
    if (memcmp(ignoredMac, peer_addr.addr, 6) == 0) {
      Serial.println(F(">>> [MATCH] Blacklisted! REJECTING... <<<"));

      char pn[32] = { 0 };
      if (conn->getPeerName(pn, sizeof(pn)) && pn[0] != '\0') {
        strncpy(ignoredPeerName, pn, NAME_LEN - 1);
        ignoredPeerName[NAME_LEN - 1] = '\0';
      } else if (ignoredPeerName[0] == '\0') {
        snprintf(ignoredPeerName, NAME_LEN, "ID:%02X%02X%02X", peer_addr.addr[2], peer_addr.addr[1], peer_addr.addr[0]);
      }

      strncpy(errorMsg, "BLOCKED DEVICE", 19);
      errorMsg[19] = '\0';
      errorMsgUntil = millis() + 1500;
      g_displayRefreshRequest = true;

      conn->disconnect();
      return;
    } else {
      Serial.println(F(">>> [MISMATCH] Different device. Proceeding..."));
    }
    Serial.println(F("----------------------"));
  }
  // =====================================================================

  // 2. 공통: 딤 방지 — OLED 명령은 콜백에서 보내지 않음 (I2C와 BLE 충돌 방지). loop의 handleOLEDDimming이 처리.
  lastInteractionTime = millis();

  conn->requestConnectionParameter(6, 0, 200);

  // ==========================================
  // [CASE 1] 클라이언트 모드일 때 (내가 Slave)
  // ==========================================
  if (currentSystemMode == MODE_CLIENT) {
    Watchdog.reset();
    ble_gap_addr_t addr = conn->getPeerAddr();
    char peerName[32] = { 0 };
    conn->getPeerName(peerName, sizeof(peerName));

    // 1. 이름이 없을 경우 ID 생성 (기존 유지)
    if (strlen(peerName) == 0 || strcmp(peerName, "UNKNOWN") == 0) {
      sprintf(peerName, "ID:%02X%02X%02X", addr.addr[2], addr.addr[1], addr.addr[0]);
    }

    // 🔍 [디버깅 로그]
    Serial.println(F("--- DEBUG: WHITELIST CHECK ---"));
    Serial.print(F("Saved Master Name: ["));
    Serial.print(config.savedMasterName);
    Serial.println(F("]"));
    Serial.print(F("Current Peer Name: ["));
    Serial.print(peerName);
    Serial.println(F("]"));
    Serial.println(F("------------------------------"));

    // 화이트리스트: MAC만 비교
    if (config.hasSavedMaster) {
      bool macMatched = macEqual(config.savedMasterMac, addr.addr);

      if (macMatched) {
        isMasterSavedThisSession = true;
        connectionStartTime = 0;  // 신규 등록 타이머 잔여 제거
        strncpy(errorMsg, "WELCOME BACK, BOSS!", 19);
        errorMsgUntil = millis() + 2500;
        g_displayRefreshRequest = true;
      } else {
        // 모르는 기기 차단 (이름이 같아도 MAC이 다르면 여기로 와서 끊어짐 -> 30초 로직 유도)
        Serial.println(F(">>> UNKNOWN HOST! REJECTED <<<"));
        conn->disconnect();
        strncpy(errorMsg, "REJECTED: UNKNOWN", 19);
        errorMsgUntil = millis() + 1500;
        g_displayRefreshRequest = true;
        return;
      }
    } else {
      // 신규 등록 모드 (저장된 마스터 없음) — 카운트다운 시작은 GATT 디스커버 이후(블로킹 시간이 유예를 잡아먹지 않도록)
      isMasterSavedThisSession = false;
    }

    // 3. 화면 표시용 데이터 업데이트
    strncpy(clientSlot.name, peerName, NAME_LEN - 1);
    clientSlot.name[NAME_LEN - 1] = '\0';
    memcpy(clientSlot.mac, addr.addr, 6);
    clientSlot.status = CONNECTED;

    // 호스트 배터리 Notify
    Serial.println(F(">>> Discovering Host Battery Service..."));
    if (host_bat_svc_client.discover(conn_handle)) {
      if (host_bat_chr_client.discover()) {
        host_bat_chr_client.enableNotify();
        Serial.println(F(">>> Host Battery Notify ENABLED! <<<"));

        // 연결 직후 현재 값을 한 번 읽어옵니다.
        remoteHostBat = host_bat_chr_client.read8();
      } else {
        Serial.println(F(">>> Host Battery Char NOT FOUND! <<<"));
      }
    } else {
      Serial.println(F(">>> Host Battery Service NOT FOUND! <<<"));
    }

    // 유예시간은 연결·디스커버가 끝난 시점부터 (콜백 앞부분 millis가 아님)
    if (!config.hasSavedMaster && Bluefruit.connected()) {
      connectionStartTime = millis();
      Serial.println(F(">>> NEW MASTER: TIMER START (post-setup) <<<"));
    }

    return;  // 클라이언트 모드 로직 완료
  }

  // ==========================================
  // [CASE 2] 호스트 모드일 때 (내가 Master)
  // ==========================================
  int foundSlot = -1;
  char currentPeerName[NAME_LEN] = { 0 };
  conn->getPeerName(currentPeerName, sizeof(currentPeerName));

  if (selectedSlotIdx != -1) {
    foundSlot = selectedSlotIdx;
  } else {
    for (int i = 0; i < 3; i++) {
      if (slots[i].status != EMPTY) {
        // 1. MAC 주소 직접 비교
        bool macMatched = macEqual(slots[i].mac, peer_addr.addr);

        // 2. 이름 비교 (저장된 이름 길이만큼 비교)
        int savedLen = strlen(slots[i].name);
        bool nameMatched = (savedLen > 0 && strncasecmp(slots[i].name, currentPeerName, savedLen) == 0);

        if (macMatched || nameMatched) {
          foundSlot = i;

          // 이름 일치 시 MAC 갱신(가변 MAC)
          if (!macMatched && nameMatched) {
            Serial.print(F(">>> HOST SLOT["));
            Serial.print(i);
            Serial.println(F("] MAC UPDATED BY NAME MATCH <<<"));
            memcpy(slots[i].mac, peer_addr.addr, 6);
            saveConfig();  // 변경된 MAC 저장
          }
          break;
        }
      }
    }
  }

  if (foundSlot == -1) {
    Serial.print(F(">>> UNKNOWN HOST REJECTED: "));
    Serial.println(currentPeerName);
    strncpy(errorMsg, "UNKNOWN DEVICE", 19);
    errorMsgUntil = millis() + 1000;
    g_displayRefreshRequest = true;
    conn->disconnect();
    return;
  }

  // 수동 연결 메시지 표시 (화면 갱신은 loop에서만)
  if (selectedSlotIdx != -1) {
    strncpy(errorMsg, "CONNECTING...", 19);
    errorMsgUntil = millis() + 10000;
    g_displayRefreshRequest = true;
  }

  // 2. 서비스 검색: 연결 직후 ATT가 준비되기 전에 discover 하면 실패할 수 있음 → 짧은 안정화 + 재시도
  Watchdog.reset();
  delay(200);
  Watchdog.reset();

  bool svcFound = false;
  for (int retry = 0; retry < 5; retry++) {
    Watchdog.reset();
    if (ble_midi_svc[foundSlot].discover(conn_handle)) {
      svcFound = true;
      Watchdog.reset();
      break;
    }
    lastInteractionTime = millis();
    if (retry < 4) {
      uint32_t waitMs = (retry == 0) ? 400UL : (800UL + (uint32_t)retry * 400UL);
      while (waitMs > 0) {
        uint32_t chunk = waitMs > 400 ? 400 : waitMs;
        delay(chunk);
        Watchdog.reset();
        waitMs -= chunk;
      }
    }
  }

  if (!svcFound) {
    Watchdog.reset();
    if (selectedSlotIdx != -1) {
      strncpy(errorMsg, "NOT A MIDI DEVICE", 19);
      errorMsgUntil = millis() + 2000;
      g_displayRefreshRequest = true;
    }
    selectedSlotIdx = -1;
    conn->disconnect();
    return;
  }

  // 3. 캐릭터리스틱 검색 — 첫 시도 직후 실패할 때만 재시도 전 대기
  bool chrFound = false;
  for (int c_retry = 0; c_retry < 3; c_retry++) {
    Watchdog.reset();
    if (ble_midi_chr[foundSlot].discover()) {
      chrFound = true;
      break;
    }
    lastInteractionTime = millis();
    if (c_retry < 2) delay(300);
  }

  if (!chrFound) {
    if (selectedSlotIdx != -1) {
      strncpy(errorMsg, "MIDI PATH ERROR", 19);
      errorMsgUntil = millis() + 2000;
      g_displayRefreshRequest = true;
    }
    selectedSlotIdx = -1;
    conn->disconnect();
    return;
  }

  // 4. 성공 시 노티파이 활성화 및 정보 저장
  ble_midi_chr[foundSlot].setNotifyCallback(ble_midi_rx_callback);
  if (ble_midi_chr[foundSlot].enableNotify()) {
    Watchdog.reset();
    // 배터리 서비스 시도 (옵션)
    if (ble_bat_svc[foundSlot].discover(conn_handle)) {
      if (ble_bat_chr[foundSlot].discover()) {
        ble_bat_chr[foundSlot].enableNotify();
        slots[foundSlot].battery = ble_bat_chr[foundSlot].read8();
      }
    }

    char connectedName[NAME_LEN] = { 0 };
    conn->getPeerName(connectedName, sizeof(connectedName));
    char scanName[NAME_LEN] = { 0 };
    bool haveScanName = findScanListNameForMac(peer_addr.addr, scanName, sizeof(scanName));
    const char *pick = connectedName;
    if (haveScanName) {
      if (strlen(connectedName) == 0 || strlen(scanName) > strlen(connectedName)) pick = scanName;
    }
    if (strlen(pick) == 0) pick = "MIDI Device";
    strncpy(slots[foundSlot].name, pick, 14);
    slots[foundSlot].name[14] = '\0';
    memcpy(slots[foundSlot].mac, peer_addr.addr, 6);
    slots[foundSlot].status = CONNECTED;

    if (selectedSlotIdx != -1) {
      errorMsg[0] = '\0';
      errorMsgUntil = 0;
      g_displayRefreshRequest = true;
    }

    saveConfig();
    selectedSlotIdx = -1;
    Serial.println(">>> SUCCESS: HOST CONNECTED <<<");
  } else {
    strncpy(errorMsg, "NOTIFY FAILED", 19);
    errorMsgUntil = millis() + 2500;
    g_displayRefreshRequest = true;
    conn->disconnect();
  }
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  // --- 공통 초기화 ---
  if (selectedSlotIdx != -1) {
    selectedSlotIdx = -1;
  }

  // --- [CASE 1] 클라이언트 모드일 때 (내가 Slave) ---
  if (currentSystemMode == MODE_CLIENT) {
    clientSlot.status = OFFLINE;
    connectionStartTime = 0;
    isMasterSavedThisSession = false;

    // 재검색 타이머
    searchStartTime = millis();

    if (config.hasSavedMaster) {
      // "주인님을 놓쳤다! 다시 찾기 위해 20분간 최대 전력(BOOST) 가동"
      activeTxPower = 8;
      currentTxMode = POWER_BOOST;
      Serial.println(F(">>> Master Lost! Boost Power (8dBm) for 20min... <<<"));
    } else {
      // "주인이 원래 없었다! 적당히(STRONG) 광고하자"
      activeTxPower = 4;
      currentTxMode = POWER_STRONG;
      // 저장된 마스터가 없으므로 타이머는 사실상 무의미하게 처리 (0으로 리셋)
      searchStartTime = 0;
      Serial.println(F(">>> No Master Saved. Strong Power (4dBm) Active. <<<"));
    }

    // 설정된 전력 즉시 적용
    Bluefruit.setTxPower(activeTxPower);
  }

  // --- [CASE 2] 호스트 모드일 때 (내가 Master) ---
  else {
    BLEConnection *conn = Bluefruit.Connection(conn_handle);
    if (conn) {
      ble_gap_addr_t peer_addr = conn->getPeerAddr();
      for (int i = 0; i < 3; i++) {
        if (slots[i].status == CONNECTED && macEqual(slots[i].mac, peer_addr.addr)) {
          slots[i].status = OFFLINE;
          break;
        }
      }
    }
    Serial.println(F(">>> HOST DISCONNECTED: SLOT UPDATED <<<"));
  }
}

void ble_bat_rx_callback(BLEClientCharacteristic *chr, uint8_t *data, uint16_t len) {
  int slotIdx = -1;
  for (int i = 0; i < 3; i++) {
    if (&ble_bat_chr[i] == chr) {
      slotIdx = i;
      break;
    }
  }

  if (slotIdx != -1 && len > 0) {
    slots[slotIdx].battery = data[0];  // 배터리 값 업데이트
    Serial.printf("[Real-time] Slot %d Battery Updated: %d%%\n", slotIdx + 1, slots[slotIdx].battery);
  }
}

void updateTxPower(int8_t rssi) {
  if (rssi == -128 || rssi == 0) return;

  // 1. RSSI 필터 (거리 변화에 더 빨리 반응하도록 완화)
  static int filteredRssi = -70;
  filteredRssi = (filteredRssi + rssi) / 2;
  rssi = filteredRssi;

  static unsigned long lastChange = 0;

  TxPowerMode targetMode = currentTxMode;

  // 2. 히스테리시스 + 임계값 (NORMAL 복귀를 쉽게, STRONG 체류를 줄임)
  //    BOOST 진입: <= -74, BOOST 해제: > -70
  if (rssi <= -74) {
    targetMode = POWER_BOOST;
  } else if (rssi <= -64) {
    if (currentTxMode == POWER_BOOST && rssi > -70) targetMode = POWER_STRONG;
    else if (currentTxMode == POWER_NORMAL) targetMode = POWER_STRONG;
  } else if (rssi > -64) {
    targetMode = POWER_NORMAL;
  }

  // 3. 쿨다운 (반응 속도 향상)
  if (targetMode != currentTxMode) {
    if (millis() - lastChange < 1200) return;

    lastChange = millis();
    currentTxMode = targetMode;

    if (currentTxMode == POWER_BOOST) activeTxPower = 8;
    else if (currentTxMode == POWER_STRONG) activeTxPower = 4;
    else activeTxPower = 0;

    Bluefruit.setTxPower(activeTxPower);

    Serial.printf(">>> [POWER CHANGE] RSSI: %d | Power: %d dBm <<<\n", rssi, activeTxPower);
  }
}

void tud_midi_rx_cb(uint8_t itf) {
  uint8_t packet[4];

  // USB MIDI 패킷 읽기
  while (usb_midi.readPacket(packet)) {
    uint8_t status = packet[1];

    // 1. 대상 슬롯 결정
    uint8_t targetIdx = (currentSystemMode == MODE_CLIENT) ? 0 : (packet[0] >> 4);
    if (targetIdx >= 3) targetIdx = 0;

    // 2. BLE 연결 확인
    bool isConnected = (currentSystemMode == MODE_CLIENT) ? Bluefruit.connected() : (slots[targetIdx].status == CONNECTED);
    if (!isConnected) continue;

    // USB MIDI 수신 -> BLE 송신
    // 따라서 송신용 삼각형인 lastMidiTime (△)을 업데이트합니다.
    if (status >= 0x80) {
      lastMidiTime[targetIdx] = millis();
    }

    // 3. 타임스탬프 계산
    static uint32_t ts_counter[3] = { 0, 0, 0 };
    uint32_t ts_full = (micros() >> 10);
    if ((int32_t)(ts_full - ts_counter[targetIdx]) <= 0) ts_full = ts_counter[targetIdx] + 1;
    ts_counter[targetIdx] = ts_full;

    uint8_t header = 0x80;
    uint8_t ts = (ts_full & 0x7F) | 0x80;

    // 4. 메시지 종류별 필터링
    if (status >= 0xF8) {
      uint8_t bleRT[3] = { header, ts, status };
      if (currentSystemMode == MODE_CLIENT) client_midi_chr.notify(bleRT, 3);
      else ble_midi_chr[targetIdx].write(bleRT, 3);
    } else {
      uint8_t type = status & 0xF0;
      bool send = false;

      static uint8_t noteVel[3][128] = { { 0 } };
      uint8_t note = packet[2];
      uint8_t vel = packet[3];
      if (type == 0x90 && vel == 0) type = 0x80;

      if (type == 0x90 || type == 0x80) {
        if (type == 0x90 && noteVel[targetIdx][note] != vel) {
          send = true;
          noteVel[targetIdx][note] = vel;
        } else if (type == 0x80 && noteVel[targetIdx][note] > 0) {
          send = true;
          noteVel[targetIdx][note] = 0;
        }
      } else if (type == 0xB0) {
        static uint8_t lastCCVal[3][128] = { { 0 } };
        if (lastCCVal[targetIdx][packet[2]] != packet[3]) {
          send = true;
          lastCCVal[targetIdx][packet[2]] = packet[3];
        }
      } else if (type == 0xE0) {
        static uint16_t lastPB[3] = { 0 };
        uint16_t pb = (packet[3] << 7) | packet[2];
        if (pb != lastPB[targetIdx]) {
          send = true;
          lastPB[targetIdx] = pb;
        }
      } else {
        send = true;
      }

      // 5. 최종 BLE 전송
      if (send) {
        uint8_t blePKT[5] = { header, ts, status, packet[2], packet[3] };
        if (currentSystemMode == MODE_CLIENT) client_midi_chr.notify(blePKT, 5);
        else ble_midi_chr[targetIdx].write(blePKT, 5);

        // 💡 역삼각형(lastUsbMidiTime) 업데이트는 여기서 하지 않습니다.
        // 역삼각형은 외부 기기로부터 신호를 '받았을 때' 콜백 함수에서 켜집니다.
      }
    }
  }
}

// ==========================================
// [HOST-ONLY] 호스트용 RSSI 모니터링 함수
// ==========================================
void monitorHostRssi(unsigned long now) {
  static unsigned long lastRssiCheck = 0;
  if (now - lastRssiCheck > 2000) {
    lastRssiCheck = now;

    int8_t lowestRssi = 0;
    bool anyConnected = false;

    for (int i = 0; i < 3; i++) {
      if (slots[i].status == CONNECTED) {
        for (uint8_t h_idx = 0; h_idx < BLE_MAX_CONNECTION; h_idx++) {
          BLEConnection *conn = Bluefruit.Connection(h_idx);
          if (conn && conn->connected() && macEqual(slots[i].mac, conn->getPeerAddr().addr)) {
            anyConnected = true;
            conn->monitorRssi();
            int8_t r = conn->getRssi();
            if (r != -128 && r != 0) {
              if (lowestRssi == 0 || r < lowestRssi) lowestRssi = r;
            }
            break;
          }
        }
      }
    }

    if (anyConnected && lowestRssi != 0) {
      updateTxPower(lowestRssi);
    } else if (!anyConnected && activeTxPower == 8) {
      // 아무도 없는데 파워가 올라가 있다면 복구
      activeTxPower = 0;
      currentTxMode = POWER_NORMAL;
      Bluefruit.setTxPower(0);
    }
  }
}

// [함수 정의] loopClientMode 바깥에 위치
void checkSearchPowerTimeout(unsigned long now) {
  // 💡 [핵심] 연결된 상태(사용 중)라면 타이머 로직을 완전히 건너뜁니다.
  // 이렇게 해야 RSSI가 낮아서 전력을 올렸을 때, 이 타이머가 강제로 낮추지 못합니다.
  if (Bluefruit.connected()) {
    searchStartTime = 0;  // 타이머 초기화
    return;
  }

  // 이 아래는 "연결이 안 된 검색 상태"에서만 작동합니다.
  if (searchStartTime > 0 && (now - searchStartTime > 1200000)) {
    if (activeTxPower == 8) {
      activeTxPower = 0;
      currentTxMode = POWER_NORMAL;
      Bluefruit.setTxPower(0);
    }
    searchStartTime = 0;
  }
}
uint8_t readHostBattery() {
  static unsigned long lastRead = 0;
  static uint8_t cachedBat = 0;

  unsigned long now = millis();
  if (now - lastRead < 400) {  // 👉 0.5초마다만 갱신
    return cachedBat;
  }
  lastRead = now;

  analogReadResolution(12);

  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogReadVDDHDIV5();
  }
  uint32_t vbat_raw = sum / 20;

  if (vbat_raw == 0) return cachedBat;

  float voltage = (vbat_raw * 18.0f) / 4096.0f;

  static float filteredVolt = 0;
  if (filteredVolt == 0) filteredVolt = voltage;

  filteredVolt = (filteredVolt * 0.7f) + (voltage * 0.3f);

  int percentage = map(filteredVolt * 100, 340, 420, 0, 100);

  cachedBat = (uint8_t)constrain(percentage, 1, 100);
  return cachedBat;
}

// =============================================================================
// Arduino setup / loop
// =============================================================================
void setup() {
  delay(200);
  // setup 단계(특히 Bluefruit.begin)에서 드물게 멈출 때 자동 복구되도록 조기 워치독 활성화
  Watchdog.enable(8000);
  Watchdog.reset();
  InternalFS.begin();
  currentSystemMode = loadSystemMode();

  if (currentSystemMode == MODE_HOST) {
    usb_midi_ptr = &usb_midi_host;
    TinyUSBDevice.setID(0x2E8A, MY_USB_PID);
    TinyUSBDevice.setProductDescriptor(MY_USB_PRODUCT);
  } else {
    usb_midi_ptr = &usb_midi_client;
    TinyUSBDevice.setID(0x2E8A, MY_USB_PID + 1);
    TinyUSBDevice.setProductDescriptor("SPIDER CLIENT");
  }
  TinyUSBDevice.setManufacturerDescriptor(MY_USB_MANUFACTURER);
  TinyUSBDevice.setSerialDescriptor(MY_USB_SERIAL);

  Serial.begin(115200);
  Serial1.begin(31250);
  usb_midi.begin();

  uint32_t usbWait = millis();
  while (!TinyUSBDevice.mounted()) {
    Watchdog.reset();
    delay(10);
    if (millis() - usbWait > 2000) break;
  }
  delay(100);

  pinMode(VCC_EXT_PIN, OUTPUT);
  digitalWrite(VCC_EXT_PIN, HIGH);
  delay(120);
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  delay(20);

  g_displayReady = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!g_displayReady) {
    Serial.println("Display Fail!");
    // OLED 실패 시에도 BLE/MIDI는 동작하도록 setup 진행
  } else {
    display.clearDisplay();
    display.display();
    showSetupProgress("Display OK");
    delay(150);
  }

  showSetupProgress("Loading Config...");
  if (currentSystemMode == MODE_HOST) {
    showSetupProgress("Loading Slots...");
  } else {
    showSetupProgress("Loading Master...");
  }
  loadConfig();
  delay(120);

  if (currentSystemMode == MODE_CLIENT) {
    setupClientMode();
  } else {
    setupHostMode();
  }
  delay(40);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  encPrevState = ((uint8_t)digitalRead(ENCODER_A) << 1) | (uint8_t)digitalRead(ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

  delay(80);
  updateDisplay();
  Watchdog.enable(4000);
}

// GATT battery service exposed by host (to connected BLE centrals)
BLEService host_bat_svc = BLEService(UUID16_SVC_BATTERY);
BLECharacteristic host_bat_chr = BLECharacteristic(UUID16_CHR_BATTERY_LEVEL);

void setupHostMode() {
  showSetupProgress("BLE Host Init...");
  Watchdog.reset();
  delay(250);

  showSetupProgress("BLE Bandwidth MAX...");
  Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
  delay(100);
  showSetupProgress("BLE Config 1...");
  Bluefruit.configCentralConn(247, 3, 3, 3);
  showSetupProgress("BLE Begin...");
  Watchdog.reset();
  delay(600);
  Watchdog.reset();
  Bluefruit.begin(0, 3);
  Watchdog.reset();
  delay(50);
  showSetupProgress("BLE Config 2...");
  // 3. [begin 이후] 레이턴시 최적화 (7.5ms)
  // Bluefruit.setConnInterval 대신 아래 함수를 사용하세요
  Bluefruit.Central.setConnInterval(6, 6);
  Bluefruit.setName(MY_USB_PRODUCT);
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  showSetupProgress("BLE Ready");
  delay(80);
  // 슬롯 3개 초기화
  for (int i = 0; i < 3; i++) {
    char buf[24];
    sprintf(buf, "Slot %d Init...", i);
    showSetupProgress(buf);

    ble_midi_svc[i].begin();
    ble_midi_chr[i].begin(&ble_midi_svc[i]);

    ble_bat_svc[i].begin();
    ble_bat_chr[i].setNotifyCallback(ble_bat_rx_callback);
    ble_bat_chr[i].begin(&ble_bat_svc[i]);
    delay(20);
  }

  // [추가] 호스트 자신의 배터리 서비스 시작
  host_bat_svc.begin();
  host_bat_chr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  host_bat_chr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  host_bat_chr.setFixedLen(1);
  host_bat_chr.begin();
  host_bat_chr.write8(readHostBattery());  // 초기값 100%

  showSetupProgress("Scanner...");
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setInterval(200, 80);
  Bluefruit.Scanner.useActiveScan(true);
  delay(40);
}
void updateHostBattery() {
  uint8_t level = readHostBattery();  // 호스트 배터리 읽기 로직
  host_bat_chr.notify8(level);        // 연결된 모든 클라이언트에게 전송
}

void setupClientMode() {
  showSetupProgress("BLE Client Init...");
  Watchdog.reset();
  delay(250);
  showSetupProgress("BLE Bandwidth MAX...");
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  delay(100);
  showSetupProgress("BLE Begin...");
  Watchdog.reset();
  delay(600);
  Watchdog.reset();
  Bluefruit.begin(1, 0);
  Watchdog.reset();
  delay(50);
  showSetupProgress("BLE Config...");
  Bluefruit.Periph.setConnInterval(6, 6);
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Scanner.stop();  // 클라이언트 모드에선 스캐너 중지
  Bluefruit.setName("SPIDER CLIENT");
  Bluefruit.setTxPower(8);

  showSetupProgress("BAT Service");
  client_bat.begin();
  client_bat.write(100);  // 초기값 설정 (예: 99%)
  showSetupProgress("BLE Ready");
  delay(80);
  showSetupProgress("MIDI Service...");
  delay(100);
  // 2. MIDI 서비스 시작
  client_midi_svc.begin();

  // 3. MIDI 캐릭터리스틱 설정
  // MIDI는 Notify가 필수이며, Write Without Response(WO_RESP) 속성이 핵심입니다.
  client_midi_chr.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_NOTIFY);
  client_midi_chr.setPermission(SECMODE_OPEN, SECMODE_OPEN);  // 보안 없이 개방
  client_midi_chr.setMaxLen(20);                              // MIDI 표준 패킷 크기
  client_midi_chr.setWriteCallback(ble_midi_peripheral_rx_callback);

  // 서비스(client_midi_svc) 안에서 캐릭터리스틱 시작
  client_midi_chr.begin();

  delay(100);
  // [추가] 호스트(서버)의 배터리 서비스를 읽기 위한 클라이언트 객체 초기화
  host_bat_svc_client.begin();

  host_bat_chr_client.setNotifyCallback(host_bat_rx_callback);  // 콜백 등록
  host_bat_chr_client.begin(&host_bat_svc_client);

  showSetupProgress("Advertising...");

  // 4. 광고 설정
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addAppearance(0x03C1);  // MIDI 기기 아이콘(지원 시)

  // 호스트가 검색할 때 MIDI 서비스가 있음을 알림
  Bluefruit.Advertising.addService(client_midi_svc);

  // 이름이 검색 목록에 잘 뜨도록 설정
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);

  // [신호 세기 결정 로직]
  if (config.hasSavedMaster) {
    activeTxPower = 8;
    currentTxMode = POWER_BOOST;
    // 변수명을 searchStartTime으로 통일하여 loop의 감시 함수와 연결
    searchStartTime = millis();
  } else {
    activeTxPower = 4;
    currentTxMode = POWER_STRONG;
    searchStartTime = 0;
  }

  Bluefruit.setTxPower(activeTxPower);
  Bluefruit.Advertising.start(0);
  Serial.println(">>> Client Mode Ready: MIDI Service Active <<<");
}

void showSetupProgress(const char *msg) {
  if (!g_displayReady) return;

  display.clearDisplay();

  // 1. 상단 타이틀 (검은색 바에 흰색 글씨)
  display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(10, 3);
  display.print(F("SPIDER BLE GATEWAY"));

  // 2. 중앙 상태 메시지
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  int msgLen = strlen(msg);
  int xPos = (128 - (msgLen * 6)) / 2;  // 중앙 정렬 계산
  if (xPos < 0) xPos = 0;

  display.setCursor(xPos, 28);  // 요청하신 위치(y=28)
  display.print(msg);

  // 3. 하단 장식선 및 브랜드명
  display.drawRect(0, 48, 128, 16, SSD1306_WHITE);
  display.setCursor(18, 52);
  display.print(F("Ash Sound Works"));

  display.display();
}

static unsigned long lastDisplayMillis = 0;
const unsigned long DISPLAY_INTERVAL = 33;  // 30fps (1000ms / 30)

void loop() {
  Watchdog.reset();
  unsigned long now = millis();
  // --- [추가] USB 상태 변화 감지 로직 ---
  static bool lastUsbMounted = false;
  bool currentUsbMounted = TinyUSBDevice.mounted();
  if (currentUsbMounted != lastUsbMounted) {
    lastUsbMounted = currentUsbMounted;
  }
  // 1. 모든 데이터 입력 및 시간 갱신 (최우선)
  handleExternalMidiIn();
  handleEncoder(now);
  handleButton(now);

  // 2. [핵심] 프레임 제한 + BLE 콜백에서 요청 시 즉시 갱신 (OLED는 loop에서만 I2C)
  if (g_displayRefreshRequest || (now - lastDisplayMillis >= DISPLAY_INTERVAL)) {
    updateDisplay();
    lastDisplayMillis = now;
    if (g_displayRefreshRequest) {
      g_displayRefreshRequest = false;
    }
  }

  // 3. 나머지 무거운 로직들 (RSSI, 배터리 등)
  handleUITimeout(now);
  handleOLEDDimming(now);
  if (currentSystemMode == MODE_HOST) loopHostMode(now);
  else loopClientMode(now);
}

void host_bat_rx_callback(BLEClientCharacteristic *chr, uint8_t *data, uint16_t len) {
  if (len > 0) {
    remoteHostBat = data[0];  // 수신된 값을 전역 변수에 저장
    Serial.printf("Received Host Battery: %d%%\n", remoteHostBat);
  }
}
// ==========================================
// [HOST] 호스트 모드 전용 루프 로직
// ==========================================
void loopHostMode(unsigned long now) {
  static unsigned long lastHostBatSend = 0;
  if (now - lastHostBatSend > 60000) {
    lastHostBatSend = now;
    updateHostBattery();
  }

  // 1. 오프라인 슬롯 체크 (2초 주기)
  checkOfflineSlots(now);
  manageScanner(now);
  // 2. 스캐너 및 연결 전력 관리
  bool scanNeeded = isScanRequired();
  bool inScanMenu = (currentState == SCAN_MENU);

  if (scanNeeded || inScanMenu) {
    if (scanStartTime == 0) scanStartTime = now;

    // 20분 경과 시 저전력 모드 (배터리 보호)
    if (now - scanStartTime > 1200000) {
      if (activeTxPower != 0) {
        activeTxPower = 0;
        currentTxMode = POWER_NORMAL;
        Bluefruit.setTxPower(0);
      }
    } else {
      // 스캔 중에는 강력한 신호 유지
      if (activeTxPower != 8) {
        activeTxPower = 8;
        currentTxMode = POWER_BOOST;
        Bluefruit.setTxPower(8);
      }
    }

    // [핵심 수정] 스캐너 실행 조건: 스캔이 필요하거나 사용자가 스캔 메뉴에 있을 때
    if (!Bluefruit.Scanner.isRunning()) {
      // 스캔 메뉴일 때는 즉시 시작, 자동 연결 중일 때는 헬퍼 함수 로직에 따름
      Bluefruit.Scanner.start(0);
      Serial.println(">>> Scanner Started (Host Mode) <<<");
    }
  } else {
    // 모든 기기 연결됨 & 메뉴 밖임 -> 타이머 리셋 및 가변 전력 모니터링
    scanStartTime = 0;
    monitorHostRssi(now);

    // 스캔이 필요 없는 상황이면 스캐너 중지 (자원 절약)
    if (Bluefruit.Scanner.isRunning()) {
      Bluefruit.Scanner.stop();
    }
  }
}

// ==========================================
// [CLIENT] 클라이언트 모드 전용 루프 로직
// ==========================================
void loopClientMode(unsigned long now) {
  checkSearchPowerTimeout(now);

  if (ignoreUntil != 0 && millis() >= ignoreUntil) {
    memset(ignoredMac, 0, 6);
    ignoreUntil = 0;
    ignoredPeerName[0] = '\0';
  }

  // 1. 마스터 정보 자동 저장 로직 (30초 카운트다운)
  // loop() 맨 앞의 now는 BLE connect_callback 이후면 "과거"라서
  // elapsed = now - connectionStartTime 이 unsigned 언더플로 → 곧바로 저장되는 버그가 남.
  // 반드시 이 블록에서는 millis()로 경과 시간을 잰다.
  if (Bluefruit.connected() && !isMasterSavedThisSession && connectionStartTime != 0) {
    unsigned long tmono = millis();
    unsigned long elapsed = (tmono >= connectionStartTime) ? (tmono - connectionStartTime) : 0;

    if (elapsed < CLIENT_TRUST_SAVE_MS) {
      int remaining = (int)(CLIENT_TRUST_SAVE_MS / 1000UL) - (int)(elapsed / 1000UL);
      if (remaining < 0) remaining = 0;
      if (remaining != lastRemainingSeconds) {
        lastRemainingSeconds = remaining;
      }
    } else {
      memcpy(config.savedMasterMac, clientSlot.mac, 6);
      strncpy(config.savedMasterName, clientSlot.name, NAME_LEN - 1);
      config.savedMasterName[NAME_LEN - 1] = '\0';
      config.hasSavedMaster = true;
      saveConfig();

      isMasterSavedThisSession = true;
      connectionStartTime = 0;
      lastRemainingSeconds = 0;
      strncpy(errorMsg, "MASTER SAVED!", 19);
      errorMsgUntil = tmono + 2000;
      Serial.println(F(">>> AUTO SAVE COMPLETE <<<"));
    }
  }

  // 2. 연결 상태 유지 및 배터리/RSSI 업데이트 (통합 틱)
  static unsigned long lastClientTick = 0;
  static unsigned long lastBatCheck = 0;

  // 2초마다 실행되는 관리 블록
  if (now - lastClientTick > 2000) {
    lastClientTick = now;

    if (Bluefruit.connected()) {
      // (1) RSSI 모니터링 (무선 신호 세기 측정)
      BLEConnection *conn = Bluefruit.Connection(0);
      if (conn) conn->monitorRssi();

      // (2) 배터리 잔량 업데이트 (1분마다 실행)BLE brg
      if (now - lastBatCheck > 60000 || lastBatCheck == 0) {
        lastBatCheck = now;

        uint8_t level = readHostBattery();
        client_bat.write(level);

        Serial.print(F(">>> Client Battery Updated: "));
        Serial.print(level);
        Serial.println(F("% <<<"));
      }
    }
  }
}
