/*
  pcf79xxprog
  ---------------------------------
  Target: classic ESP32-WROOM / ESP32 Dev Module (Arduino-ESP32 core)
  Interface: NXP PCF79xx Monitor and Download Interface (MDI)

  This is a clean ESP32 port based on the PCF79xx_Tool command set, while the
  edge handling follows the older/stable target-clocked MDI transaction model.

  IMPORTANT HARDWARE RULES
  - The PCF target and MDI GPIO use 3.3 V only.
  - Do not power a key PCB from an ESP32 GPIO.
  - GPIO25 controls the EN/gate of an external 3.3 V load switch by default.
  - Remove/isolate the coin cell before connecting target power.
  - MSDA uses the ESP32 internal pull-up while released.
  - An external 4.7k-10k pull-up to the switched 3.3 V rail is optional for longer/noisier wiring.
  - Add 330R-1k series resistors in MSDA and MSCL.

  Default pins (change below if needed):
    GPIO25 -> target 3.3 V load-switch enable
    GPIO26 -> MSDA
    GPIO27 <- MSCL
    GND    -> target GND

  No third-party libraries are required.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/gpio_reg.h"

// ---------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------

// Optional connection to an existing Wi-Fi network. Leave SSID empty to use
// only the access point.
static const char WIFI_SSID[] = "";
static const char WIFI_PASSWORD[] = "";

// The programmer always starts this local access point.
static const char AP_SSID[] = "PCF79xx-Tool";
static const char AP_PASSWORD[] = "pcf79xx-tool"; // minimum 8 characters

// GPIO numbers, not Arduino Uno header labels.
static constexpr uint8_t PIN_TARGET_POWER = 25;
static constexpr uint8_t PIN_MSDA = 26;
static constexpr uint8_t PIN_MSCL = 27;

// true for an active-high load-switch EN pin.
// Set false for a P-channel MOSFET high-side switch whose gate is active low.
static constexpr bool TARGET_POWER_ACTIVE_HIGH = true;

// MDI timing values recovered from PCF79xx_Tool.
static constexpr uint32_t TIMEOUT_MSCL_LOW_US = (73U + 62U + 1U) * 8U; // 1088 us
static constexpr uint32_t TIMEOUT_MSCL_HIGH_US = 200U + 480U;           // 680 us
static constexpr uint32_t TIMEOUT_BYTE_US = 1000000U;                  // 1 s
static constexpr uint32_t TIMEOUT_ACK_US = 200000U;                    // 200 ms
static constexpr uint32_t TIMEOUT_TRACE_RISE_US = 400U;
static constexpr uint32_t TIMEOUT_TRACE_FALL_US = 50U;
static constexpr uint32_t PCF7945_POWER_TO_MSDA_HIGH_US = 500U;
static constexpr uint32_t PCF7945_AFTER_MSDA_HIGH_US = 200U;
static constexpr uint32_t DIRECTION_TURNAROUND_US = 500U;
static constexpr uint32_t TX_BYTE_RECOVERY_US = 25U;
static constexpr uint32_t EEPROM_WRITE_DELAY_MS = 5U;
static constexpr uint32_t ERASE_DELAY_MS = 100U;

static constexpr size_t MAX_EROM_SIZE = 16384;
static constexpr size_t MAX_EEPROM_SIZE = 1024;
static constexpr size_t EROM_PAGE_32 = 32;
static constexpr size_t EROM_PAGE_64 = 64;
static constexpr size_t EEPROM_PAGE_SIZE = 4;

static_assert(PIN_MSDA < 32 && PIN_MSCL < 32,
              "This fast GPIO implementation requires MSDA/MSCL below GPIO32.");

// ---------------------------------------------------------------------------
// MDI protocol constants
// ---------------------------------------------------------------------------

enum MdiCommand : uint8_t {
  C_GO = 0x01,
  C_TRACE = 0x02,
  C_GETDAT = 0x03,
  C_SETDAT = 0x04,
  C_SETPC = 0x05,
  C_RESET = 0x06,
  C_SETBRK = 0x07,
  C_ER_EROM = 0x08,
  C_WR_EROM = 0x09,
  C_WR_EEPROM = 0x0A,
  C_WR_EROM_B = 0x0B,
  C_SIG_ROM = 0x0C,
  C_SIG_EROM = 0x0D,
  C_EE_DUMP = 0x0E,
  C_ER_DUMP = 0x0F,
  C_SIG_EEROM = 0x11,
  C_PROTECT = 0x12,
  C_PROG_CONFIG = 0x14,
  C_WR_EROM64 = 0x18,
  C_RD_ER_BYTE = 0x1B,
  C_SIG_EROM_NORM = 0x1D,
  C_SIG_XROM_OR_EEROM_NORM = 0x1E
};

static const uint8_t ERASE_MAGIC[16] = {
  0x55, 0x45, 0xE8, 0x92, 0xD6, 0xB1, 0x62, 0x59,
  0xFC, 0x8A, 0xC8, 0xF2, 0xD6, 0xE1, 0x4A, 0x35
};

static constexpr uint8_t DEVICE_ACK = 0x55;

// ---------------------------------------------------------------------------
// Device profiles
// ---------------------------------------------------------------------------

enum InitSequence : uint8_t {
  INIT_26A0700,
  INIT_PCF7945_FAMILY
};

struct DeviceProfile {
  const char *name;
  InitSequence init;
  uint16_t eromSize;
  uint16_t eepromSize;
  uint8_t preferredEromPageSize;
  bool hasPcf7961SpecialPages;
  bool experimental;
};

static const DeviceProfile PROFILES[] = {
  {
    "PCF7945 / PCF7953 / PCF7961 (8K EROM, 1K EEPROM)",
    INIT_PCF7945_FAMILY, 8192, 1024, 64, true, false
  },
  {
    "26A0700 legacy (8K EROM, 512B EEPROM)",
    INIT_26A0700, 8192, 512, 32, false, false
  },
  {
    "26A0700 experimental (16K EROM, 512B EEPROM)",
    INIT_26A0700, 16384, 512, 64, false, true
  }
};

static constexpr size_t PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);
static uint8_t selectedProfile = 0;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

WebServer server(80);

static uint8_t eromBuffer[MAX_EROM_SIZE];
static uint8_t eepromBuffer[MAX_EEPROM_SIZE];
static size_t eromBufferLength = 0;
static size_t eepromBufferLength = 0;
static bool eromLoaded = false;
static bool eepromLoaded = false;
static bool targetConnected = false;
static bool operationBusy = false;
static String lastMessage = "Booting";
static uint8_t lastEecon = 0xFF;
static uint8_t lastTraceBytes[2] = {0, 0};
static uint8_t lastSignature[3] = {0, 0, 0};

// Upload state. WebServer processes one request at a time.
enum UploadTarget : uint8_t { UPLOAD_NONE, UPLOAD_EROM, UPLOAD_EEPROM };
static UploadTarget uploadTarget = UPLOAD_NONE;
static size_t uploadPosition = 0;
static bool uploadFailed = false;
static String uploadError;

// ---------------------------------------------------------------------------
// Fast GPIO and MDI ISR
// ---------------------------------------------------------------------------

static constexpr uint32_t MSDA_MASK = (1UL << PIN_MSDA);
static constexpr uint32_t MSCL_MASK = (1UL << PIN_MSCL);

static inline void IRAM_ATTR msdaDriveLowFast() {
  REG_WRITE(GPIO_OUT_W1TC_REG, MSDA_MASK);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, MSDA_MASK);
}

static inline void IRAM_ATTR msdaDriveHighFast() {
  REG_WRITE(GPIO_OUT_W1TS_REG, MSDA_MASK);
  REG_WRITE(GPIO_ENABLE_W1TS_REG, MSDA_MASK);
}

static inline void IRAM_ATTR msdaReleaseFast() {
  REG_WRITE(GPIO_ENABLE_W1TC_REG, MSDA_MASK);
}

static inline bool IRAM_ATTR msdaReadFast() {
  return (REG_READ(GPIO_IN_REG) & MSDA_MASK) != 0;
}

static inline bool IRAM_ATTR msclReadFast() {
  return (REG_READ(GPIO_IN_REG) & MSCL_MASK) != 0;
}

enum BusState : uint8_t {
  BUS_IDLE,
  TX_WAIT_RISE,
  TX_FALL_EDGES,
  RX_WAIT_RISE,
  RX_DATA_FALLS,
  RX_FINAL_FALL,
  BUS_DONE
};

enum BusDirection : uint8_t {
  DIR_NONE,
  DIR_TX,
  DIR_RX
};

static volatile BusState busState = BUS_IDLE;
static volatile uint8_t txByteIsr = 0;
static volatile uint8_t rxByteIsr = 0;
static volatile uint8_t bitIndexIsr = 0;
static BusDirection lastDirection = DIR_NONE;

void IRAM_ATTR onMsclChange() {
  const bool high = msclReadFast();

  switch (busState) {
    case TX_WAIT_RISE:
      if (high) {
        bitIndexIsr = 0;
        if (txByteIsr & 0x01) {
          msdaDriveHighFast();
        } else {
          msdaDriveLowFast();
        }
        busState = TX_FALL_EDGES;
      }
      break;

    case TX_FALL_EDGES:
      if (!high) {
        if (bitIndexIsr < 7) {
          ++bitIndexIsr;
          if ((txByteIsr >> bitIndexIsr) & 0x01) {
            msdaDriveHighFast();
          } else {
            msdaDriveLowFast();
          }
        } else {
          // The extra falling edge completes the byte. The original tool
          // actively drives MSDA high between transactions.
          msdaDriveHighFast();
          busState = BUS_DONE;
        }
      }
      break;

    case RX_WAIT_RISE:
      if (high) {
        // Target owns MSDA for the receive phase. External pull-up is required.
        msdaReleaseFast();
        rxByteIsr = 0;
        bitIndexIsr = 0;
        busState = RX_DATA_FALLS;
      }
      break;

    case RX_DATA_FALLS:
      if (!high) {
        if (msdaReadFast()) {
          rxByteIsr |= (uint8_t)(1U << bitIndexIsr);
        }
        ++bitIndexIsr;
        if (bitIndexIsr >= 8) {
          busState = RX_FINAL_FALL;
        }
      }
      break;

    case RX_FINAL_FALL:
      if (!high) {
        msdaDriveHighFast();
        busState = BUS_DONE;
      }
      break;

    default:
      break;
  }
}

static bool waitBusDone(uint32_t timeoutUs) {
  const uint32_t started = micros();
  while (busState != BUS_DONE) {
    if ((uint32_t)(micros() - started) >= timeoutUs) {
      busState = BUS_IDLE;
      msdaDriveHighFast();
      return false;
    }
    delayMicroseconds(1);
  }
  busState = BUS_IDLE;
  return true;
}

static bool waitMsclLevel(bool wantedHigh, uint32_t timeoutUs) {
  const uint32_t started = micros();
  while (msclReadFast() != wantedHigh) {
    if ((uint32_t)(micros() - started) >= timeoutUs) {
      return false;
    }
    delayMicroseconds(1);
  }
  return true;
}

static void directionTurnaround(BusDirection next) {
  if (lastDirection != DIR_NONE && lastDirection != next) {
    delayMicroseconds(DIRECTION_TURNAROUND_US);
  }
}

static bool mdiSendByte(uint8_t value) {
  directionTurnaround(DIR_TX);

  txByteIsr = value;
  bitIndexIsr = 0;
  busState = TX_WAIT_RISE;
  msdaDriveLowFast(); // request a target-clocked transfer

  if (!waitBusDone(TIMEOUT_BYTE_US)) {
    targetConnected = false;
    lastMessage = "MDI send timeout: no complete MSCL byte sequence; reconnect required";
    return false;
  }

  lastDirection = DIR_TX;
  delayMicroseconds(TX_BYTE_RECOVERY_US);
  return true;
}

static bool mdiReceiveByte(uint8_t &value, uint32_t timeoutUs = TIMEOUT_BYTE_US) {
  directionTurnaround(DIR_RX);

  rxByteIsr = 0;
  bitIndexIsr = 0;
  busState = RX_WAIT_RISE;
  msdaDriveLowFast(); // request a target-clocked transfer

  if (!waitBusDone(timeoutUs)) {
    targetConnected = false;
    lastMessage = "MDI receive timeout: no complete MSCL byte sequence; reconnect required";
    return false;
  }

  value = rxByteIsr;
  lastDirection = DIR_RX;
  return true;
}

static bool mdiSend(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    if (!mdiSendByte(data[i])) {
      lastMessage += " at byte " + String(i);
      return false;
    }
    if ((i & 0xFF) == 0) yield();
  }
  return true;
}

static bool mdiReceive(uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    if (!mdiReceiveByte(data[i])) {
      lastMessage += " at byte " + String(i);
      return false;
    }
    if ((i & 0xFF) == 0) yield();
  }
  return true;
}

// ---------------------------------------------------------------------------
// Target power and helpers
// ---------------------------------------------------------------------------

static const DeviceProfile &profile() {
  return PROFILES[selectedProfile];
}

static void setTargetPower(bool on) {
  const bool pinLevel = TARGET_POWER_ACTIVE_HIGH ? on : !on;
  digitalWrite(PIN_TARGET_POWER, pinLevel ? HIGH : LOW);
  if (!on) {
    targetConnected = false;
    lastDirection = DIR_NONE;
    busState = BUS_IDLE;
    msdaDriveLowFast();
  }
}

static void powerOffTarget(const String &reason = "Target power off") {
  setTargetPower(false);
  lastMessage = reason;
}

static uint8_t reverseBits(uint8_t value) {
  value = (uint8_t)(((value & 0xAA) >> 1) | ((value & 0x55) << 1));
  value = (uint8_t)(((value & 0xCC) >> 2) | ((value & 0x33) << 2));
  return (uint8_t)((value >> 4) | (value << 4));
}

static bool eeconIsOkay(uint8_t eecon) {
  return (eecon & 0xC0) == 0;
}

static String hexByte(uint8_t value) {
  char text[5];
  snprintf(text, sizeof(text), "%02X", value);
  return String(text);
}

static String jsonEscape(const String &input) {
  String out;
  out.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)c >= 0x20) out += c;
        break;
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// PCF monitor-mode operations
// ---------------------------------------------------------------------------

static bool enterMonitorMode() {
  targetConnected = false;
  lastDirection = DIR_NONE;
  busState = BUS_IDLE;

  // Deterministic power-down and line state are part of monitor-mode entry.
  setTargetPower(false);
  msdaDriveLowFast();
  delay(100);

  if (profile().init == INIT_PCF7945_FAMILY) {
    pinMode(PIN_MSCL, INPUT); // floating, target-driven clock
  } else {
    pinMode(PIN_MSCL, INPUT_PULLDOWN);
  }

  setTargetPower(true);

  if (profile().init == INIT_PCF7945_FAMILY) {
    delayMicroseconds(PCF7945_POWER_TO_MSDA_HIGH_US);
    msdaDriveHighFast();
    delayMicroseconds(PCF7945_AFTER_MSDA_HIGH_US);

    if (!msclReadFast()) {
      lastMessage = "Connect failed: MSCL was not high after PCF7945 power-up window";
      return false;
    }
    if (!waitMsclLevel(false, TIMEOUT_MSCL_LOW_US)) {
      lastMessage = "Connect failed: PCF7945 MSCL did not fall during mode selection";
      return false;
    }

    msdaDriveLowFast();
    delayMicroseconds(8);
    msdaDriveHighFast();
  } else {
    if (!waitMsclLevel(true, TIMEOUT_MSCL_HIGH_US)) {
      lastMessage = "Connect failed: 26A0700 MSCL did not rise after power-up";
      return false;
    }
    if (!waitMsclLevel(false, TIMEOUT_MSCL_LOW_US)) {
      lastMessage = "Connect failed: 26A0700 MSCL did not fall after power-up";
      return false;
    }
    delayMicroseconds(2);
    msdaDriveHighFast();
  }

  delayMicroseconds(50);

  uint8_t ack = 0;
  if (!mdiReceiveByte(ack, TIMEOUT_ACK_US)) {
    lastMessage = "Connect failed while waiting for monitor-mode ACK";
    return false;
  }
  if (ack != DEVICE_ACK) {
    lastMessage = "Connect failed: expected ACK 55, received " + hexByte(ack);
    return false;
  }

  return true;
}

static bool pcfTrace() {
  if (!mdiSendByte(C_TRACE)) return false;

  // C_TRACE acknowledgement is a separate MSCL watchdog-disable pulse.
  if (!waitMsclLevel(true, TIMEOUT_TRACE_RISE_US)) {
    lastMessage = "TRACE confirmation absent; target may be protected/locked";
    return false;
  }
  if (!waitMsclLevel(false, TIMEOUT_TRACE_FALL_US)) {
    lastMessage = "TRACE confirmation pulse stayed high too long";
    return false;
  }

  if (!mdiReceive(lastTraceBytes, sizeof(lastTraceBytes))) return false;
  return true;
}

static bool pcfErase() {
  uint8_t response = 0;

  if (!mdiSendByte(C_ER_EROM)) return false;
  if (!mdiReceiveByte(response)) return false;
  if (response != 0x88) {
    lastMessage = "Erase rejected: expected challenge 88, received " + hexByte(response);
    return false;
  }

  if (!mdiSend(ERASE_MAGIC, sizeof(ERASE_MAGIC))) return false;
  delay(ERASE_DELAY_MS);

  if (!mdiReceiveByte(lastEecon)) return false;
  if (!eeconIsOkay(lastEecon)) {
    lastMessage = "Erase EECON error: " + hexByte(lastEecon);
    return false;
  }

  lastMessage = "Erase/unlock completed; EECON=" + hexByte(lastEecon);
  return true;
}

static bool pcfConnect(bool eraseImmediately) {
  if (!enterMonitorMode()) {
    const String failure = lastMessage;
    setTargetPower(false);
    targetConnected = false;
    lastMessage = failure;
    return false;
  }

  if (eraseImmediately) {
    if (!pcfErase()) {
      const String failure = lastMessage;
      setTargetPower(false);
      targetConnected = false;
      lastMessage = failure;
      return false;
    }
  } else {
    if (!pcfTrace()) {
      const String failure = lastMessage;
      setTargetPower(false);
      targetConnected = false;
      lastMessage = failure;
      return false;
    }
    lastMessage = "Connected; TRACE bytes=" + hexByte(lastTraceBytes[0]) + " " +
                  hexByte(lastTraceBytes[1]);
  }

  targetConnected = true;
  return true;
}

static bool requireConnected() {
  if (!targetConnected) {
    lastMessage = "No active MDI connection. Use Connect first.";
    return false;
  }
  return true;
}

static bool pcfProtect() {
  if (!requireConnected()) return false;
  if (!mdiSendByte(C_PROTECT)) return false;
  if (!mdiReceiveByte(lastEecon)) return false;
  if (!eeconIsOkay(lastEecon)) {
    lastMessage = "Protect returned EECON error " + hexByte(lastEecon);
    return false;
  }
  targetConnected = false;
  lastMessage = "Protect/lock command accepted; EECON=" + hexByte(lastEecon) +
                ". Reconnect or power off before further operations.";
  return true;
}

static bool pcfReset() {
  if (!requireConnected()) return false;
  if (!mdiSendByte(C_RESET)) return false;
  targetConnected = false;
  lastMessage = "C_RESET sent; reconnect before further MDI operations";
  return true;
}

static bool pcfRun() {
  if (!requireConnected()) return false;
  if (!mdiSendByte(C_GO)) return false;
  targetConnected = false;
  lastMessage = "C_GO sent; PCF program execution started";
  return true;
}

static bool pcfWriteRegister(uint8_t address, uint8_t value) {
  if (!requireConnected()) return false;
  if (!mdiSendByte(C_SETDAT) || !mdiSendByte(address) || !mdiSendByte(value)) {
    return false;
  }
  lastMessage = "C_SETDAT sent: register " + hexByte(address) + " = " + hexByte(value);
  return true;
}

// ---------------------------------------------------------------------------
// Memory read, program, verify
// ---------------------------------------------------------------------------

static bool readEromToBuffer() {
  if (!requireConnected()) return false;
  if (!mdiSendByte(C_ER_DUMP)) return false;
  if (!mdiReceive(eromBuffer, profile().eromSize)) return false;

  eromBufferLength = profile().eromSize;
  eromLoaded = true;
  lastMessage = "Read " + String(eromBufferLength) + " EROM bytes";
  return true;
}

static bool readEepromToBuffer() {
  if (!requireConnected()) return false;
  if (!mdiSendByte(C_EE_DUMP)) return false;

  for (size_t i = 0; i < profile().eepromSize; ++i) {
    uint8_t onWire = 0;
    if (!mdiReceiveByte(onWire)) {
      lastMessage += " while reading EEPROM byte " + String(i);
      return false;
    }
    eepromBuffer[i] = reverseBits(onWire);
    if ((i & 0xFF) == 0) yield();
  }

  eepromBufferLength = profile().eepromSize;
  eepromLoaded = true;
  lastMessage = "Read " + String(eepromBufferLength) + " EEPROM bytes (bit order normalized)";
  return true;
}

static bool programErom(uint8_t pageSize) {
  if (!requireConnected()) return false;
  if (!eromLoaded || eromBufferLength != profile().eromSize) {
    lastMessage = "Load an EROM binary exactly matching the selected profile first";
    return false;
  }
  if (pageSize != 32 && pageSize != 64) {
    lastMessage = "EROM page size must be 32 or 64";
    return false;
  }
  if ((profile().eromSize % pageSize) != 0) {
    lastMessage = "Profile EROM size is not divisible by selected page size";
    return false;
  }

  const uint8_t command = (pageSize == 64) ? C_WR_EROM64 : C_WR_EROM;
  const size_t pages = profile().eromSize / pageSize;
  if (pages > 256) {
    lastMessage = "Page count exceeds the one-byte MDI page index";
    return false;
  }

  for (size_t page = 0; page < pages; ++page) {
    if (!mdiSendByte(command) ||
        !mdiSendByte((uint8_t)page) ||
        !mdiSend(&eromBuffer[page * pageSize], pageSize)) {
      lastMessage += " while programming EROM page " + String(page);
      return false;
    }

    delay(EEPROM_WRITE_DELAY_MS);
    if (!mdiReceiveByte(lastEecon)) return false;
    if (!eeconIsOkay(lastEecon)) {
      lastMessage = "EROM page " + String(page) + " EECON error " + hexByte(lastEecon);
      return false;
    }
    if ((page & 0x0F) == 0) yield();
  }

  lastMessage = "Programmed " + String(profile().eromSize) + " EROM bytes in " +
                String(pageSize) + "-byte pages";
  return true;
}

static bool programEepromPage(uint8_t page, const uint8_t *normalOrderData) {
  uint8_t wireData[EEPROM_PAGE_SIZE];
  for (size_t i = 0; i < EEPROM_PAGE_SIZE; ++i) {
    wireData[i] = reverseBits(normalOrderData[i]);
  }

  if (!mdiSendByte(C_WR_EEPROM) || !mdiSendByte(page) ||
      !mdiSend(wireData, sizeof(wireData))) {
    return false;
  }
  delay(EEPROM_WRITE_DELAY_MS);

  if (!mdiReceiveByte(lastEecon)) return false;
  if (!eeconIsOkay(lastEecon)) {
    lastMessage = "EEPROM page " + String(page) + " EECON error " + hexByte(lastEecon);
    return false;
  }
  return true;
}

static bool programSpecialPage127(uint8_t byte2, uint8_t byte3) {
  const uint8_t data[2] = {byte2, byte3};
  if (!mdiSendByte(C_PROG_CONFIG) || !mdiSend(data, sizeof(data))) return false;
  if (!mdiReceiveByte(lastEecon)) return false;
  if (!eeconIsOkay(lastEecon)) {
    lastMessage = "Special page 127 EECON error " + hexByte(lastEecon);
    return false;
  }
  return true;
}

static bool programEeprom(const String &mode) {
  if (!requireConnected()) return false;
  if (!eepromLoaded || eepromBufferLength != profile().eepromSize) {
    lastMessage = "Load an EEPROM binary exactly matching the selected profile first";
    return false;
  }

  const bool manualMode = (mode == "manual");
  const bool skipSpecial = (mode == "skip_special");
  if (!manualMode && !skipSpecial && mode != "safe") {
    lastMessage = "Unknown EEPROM programming mode";
    return false;
  }

  const size_t pages = profile().eepromSize / EEPROM_PAGE_SIZE;

  // Documentation notes that C_PROG_CONFIG may work only immediately after
  // erase and before other memory programming. Therefore safe mode sends the
  // page-127 special bytes before any normal EEPROM page writes.
  if (!manualMode && !skipSpecial && profile().hasPcf7961SpecialPages && pages > 127) {
    const size_t offset = 127 * EEPROM_PAGE_SIZE;
    if (!programSpecialPage127(eepromBuffer[offset + 2], eepromBuffer[offset + 3])) {
      lastMessage += " while programming page 127 bytes 2/3. Try immediately after erase, or use preserve/skip mode.";
      return false;
    }
  }

  for (size_t page = 0; page < pages; ++page) {
    // Page 0 contains the PCF identity and is never written by this tool.
    if (page == 0) continue;

    if (!manualMode && profile().hasPcf7961SpecialPages) {
      // Conservative behavior matching the Tusker implementation. Its source
      // also skips 125, although the documentation explicitly describes 126.
      if (page == 125 || page == 126 || page == 127) continue;
    }

    if (!programEepromPage((uint8_t)page,
                           &eepromBuffer[page * EEPROM_PAGE_SIZE])) {
      lastMessage += " while programming EEPROM page " + String(page);
      return false;
    }
    if ((page & 0x0F) == 0) yield();
  }

  if (manualMode) {
    lastMessage = "Programmed EEPROM in manual mode (page 0 skipped only)";
  } else if (skipSpecial) {
    lastMessage = "Programmed EEPROM while preserving/skipping pages 125-127";
  } else {
    lastMessage = "Programmed EEPROM with page-127 special bytes first";
  }
  return true;
}

static bool verifyEromAgainstBuffer(size_t &mismatchCount, size_t &firstMismatch) {
  mismatchCount = 0;
  firstMismatch = SIZE_MAX;

  if (!requireConnected()) return false;
  if (!eromLoaded || eromBufferLength != profile().eromSize) {
    lastMessage = "No complete EROM reference buffer is loaded";
    return false;
  }
  if (!mdiSendByte(C_ER_DUMP)) return false;

  for (size_t i = 0; i < profile().eromSize; ++i) {
    uint8_t value = 0;
    if (!mdiReceiveByte(value)) return false;
    if (value != eromBuffer[i]) {
      if (firstMismatch == SIZE_MAX) firstMismatch = i;
      ++mismatchCount;
    }
    if ((i & 0xFF) == 0) yield();
  }

  if (mismatchCount == 0) {
    lastMessage = "EROM verify passed";
    return true;
  }
  lastMessage = "EROM verify failed: " + String(mismatchCount) +
                " differences; first at 0x" + String((unsigned)firstMismatch, HEX);
  return false;
}

static bool shouldSkipEepromVerifyByte(size_t index, const String &mode) {
  const size_t page = index / EEPROM_PAGE_SIZE;
  const size_t byteInPage = index % EEPROM_PAGE_SIZE;

  if (page == 0) return true;
  if (mode == "manual" || !profile().hasPcf7961SpecialPages) return false;
  if (page == 125 || page == 126) return true;
  if (page == 127) {
    if (mode == "skip_special") return true;
    return byteInPage < 2; // safe mode verifies writable bytes 2 and 3
  }
  return false;
}

static bool verifyEepromAgainstBuffer(const String &mode,
                                      size_t &mismatchCount,
                                      size_t &firstMismatch) {
  mismatchCount = 0;
  firstMismatch = SIZE_MAX;

  if (!requireConnected()) return false;
  if (!eepromLoaded || eepromBufferLength != profile().eepromSize) {
    lastMessage = "No complete EEPROM reference buffer is loaded";
    return false;
  }
  if (!mdiSendByte(C_EE_DUMP)) return false;

  for (size_t i = 0; i < profile().eepromSize; ++i) {
    uint8_t wireValue = 0;
    if (!mdiReceiveByte(wireValue)) return false;
    const uint8_t value = reverseBits(wireValue);
    if (!shouldSkipEepromVerifyByte(i, mode) && value != eepromBuffer[i]) {
      if (firstMismatch == SIZE_MAX) firstMismatch = i;
      ++mismatchCount;
    }
    if ((i & 0xFF) == 0) yield();
  }

  if (mismatchCount == 0) {
    lastMessage = "EEPROM verify passed for writable bytes";
    return true;
  }
  lastMessage = "EEPROM verify failed: " + String(mismatchCount) +
                " differences; first at 0x" + String((unsigned)firstMismatch, HEX);
  return false;
}

// Signature types: erom_norm, erom, eeprom, rom, eeprom_norm, xrom
static bool readSignature(const String &type) {
  if (!requireConnected()) return false;

  if (type == "erom_norm") {
    uint8_t challenge = 0;
    if (!mdiSendByte(C_SIG_EROM_NORM) || !mdiReceiveByte(challenge)) return false;
    if (challenge != 0x88) {
      lastMessage = "Normalized EROM signature challenge was " + hexByte(challenge) + ", not 88";
      return false;
    }
    if (!mdiSend(ERASE_MAGIC, sizeof(ERASE_MAGIC))) return false;
  } else if (type == "erom") {
    if (!mdiSendByte(C_SIG_EROM)) return false;
  } else if (type == "eeprom") {
    if (!mdiSendByte(C_SIG_EEROM)) return false;
  } else if (type == "rom") {
    if (!mdiSendByte(C_SIG_ROM)) return false;
  } else if (type == "eeprom_norm") {
    if (!mdiSendByte(C_SIG_XROM_OR_EEROM_NORM) || !mdiSendByte(0x20)) return false;
  } else if (type == "xrom") {
    if (!mdiSendByte(C_SIG_XROM_OR_EEROM_NORM) || !mdiSendByte(0x00)) return false;
  } else {
    lastMessage = "Unknown signature type";
    return false;
  }

  if (!mdiReceive(lastSignature, sizeof(lastSignature))) return false;
  lastMessage = "Signature " + type + " = " + hexByte(lastSignature[0]) + " " +
                hexByte(lastSignature[1]) + " " + hexByte(lastSignature[2]);
  return true;
}

// ---------------------------------------------------------------------------
// Web UI
// ---------------------------------------------------------------------------

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PCF79xx ESP32 Tool</title>
<style>
:root{color-scheme:dark;--bg:#11151a;--card:#1b222a;--line:#36414d;--accent:#58a6ff;--ok:#55d187;--bad:#ff6b6b;--text:#edf2f7;--muted:#a9b4c0}
*{box-sizing:border-box}body{margin:0;font:15px system-ui,sans-serif;background:var(--bg);color:var(--text)}main{max-width:1050px;margin:auto;padding:18px}.head{display:flex;justify-content:space-between;gap:12px;align-items:center;flex-wrap:wrap}h1{font-size:1.55rem;margin:.2rem 0}h2{font-size:1.05rem;margin:0 0 12px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px;margin-top:14px}.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:15px}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}button,select,input{border:1px solid var(--line);border-radius:7px;background:#10161c;color:var(--text);padding:9px 11px}button{cursor:pointer}button:hover{border-color:var(--accent)}button.danger{border-color:#8d3f46}button.lock{border-color:#8b742d}.wide{width:100%}.muted{color:var(--muted);font-size:.9rem}.status{padding:9px 11px;border-radius:7px;background:#10161c;border:1px solid var(--line)}.ok{color:var(--ok)}.bad{color:var(--bad)}#log{height:180px;overflow:auto;white-space:pre-wrap;font:13px ui-monospace,monospace}.pill{display:inline-block;border:1px solid var(--line);border-radius:999px;padding:3px 8px;margin:2px}.progress{width:100%;height:8px;background:#0d1116;border-radius:8px;overflow:hidden}.progress div{height:100%;width:0;background:var(--accent)}a{color:var(--accent)}label{display:block}.small{width:90px}
</style>
</head>
<body><main>
<div class="head"><div><h1>pcf79xxprog</h1><div class="muted">Target-clocked MDI · 3.3 V only</div></div><div id="connection" class="status">Loading…</div></div>

<div class="grid">
<section class="card"><h2>1. Target profile and connection</h2>
<label>Device profile<select id="profile" class="wide" onchange="setProfile()"></select></label>
<div class="row"><button onclick="action('/api/connect')">Connect</button><button class="danger" onclick="destructive('/api/connect?erase=1','ERASE ALL target memory?')">Connect + erase/unlock</button><button onclick="action('/api/poweroff')">Power off</button></div>
<div class="muted">A normal connect may report protected/locked. Connect + erase is destructive.</div>
</section>

<section class="card"><h2>2. Load binary buffers</h2>
<label>EROM binary<input id="eromFile" class="wide" type="file" accept=".bin,application/octet-stream"></label>
<div class="row"><button onclick="upload('erom')">Upload EROM</button><a href="/download/erom.bin">Download buffer</a></div>
<label>EEPROM binary<input id="eepromFile" class="wide" type="file" accept=".bin,application/octet-stream"></label>
<div class="row"><button onclick="upload('eeprom')">Upload EEPROM</button><a href="/download/eeprom.bin">Download buffer</a></div>
<div id="buffers" class="muted"></div>
</section>

<section class="card"><h2>3. Read target memory</h2>
<div class="row"><button onclick="action('/api/read?mem=erom')">Read EROM</button><button onclick="action('/api/read?mem=eeprom')">Read EEPROM</button></div>
<div class="muted">Read replaces the corresponding RAM buffer. Download it afterward.</div>
</section>

<section class="card"><h2>4. Program and verify</h2>
<div class="row"><label>EROM page<select id="eromPage"><option value="64">64 byte (recommended PCF7945 family)</option><option value="32">32 byte</option></select></label></div>
<div class="row"><button class="danger" onclick="programErom()">Program EROM</button><button onclick="action('/api/verify?mem=erom')">Verify EROM</button></div>
<div class="row"><label>EEPROM mode<select id="eeMode"><option value="safe">Safe: program special bytes first</option><option value="skip_special">Safe: preserve/skip page 127</option><option value="manual">Manual: skip page 0 only</option></select></label></div>
<div class="row"><button class="danger" onclick="programEeprom()">Program EEPROM</button><button onclick="verifyEeprom()">Verify EEPROM</button></div>
</section>

<section class="card"><h2>5. Signatures</h2>
<div class="row"><select id="sig"><option value="erom_norm">EROM normalized</option><option value="erom">EROM</option><option value="eeprom">EEPROM</option><option value="rom">ROM</option><option value="eeprom_norm">EEPROM normalized</option><option value="xrom">XROM</option></select><button onclick="readSig()">Read signature</button></div>
<div id="signature" class="muted"></div>
</section>

<section class="card"><h2>6. Advanced and finalization</h2>
<div class="row"><button onclick="action('/api/reset')">Reset PCF</button><button onclick="action('/api/run')">Run program</button><button class="lock" onclick="destructive('/api/protect','Protect/lock the PCF? Unlocking later requires erase.','PROTECT')">Protect / lock</button></div>
<div class="row"><input id="regAddr" class="small" placeholder="Reg hex"><input id="regValue" class="small" placeholder="Data hex"><button onclick="writeReg()">Write register</button></div>
<div class="muted">Register writes are raw C_SETDAT operations. Use only with a known register map.</div>
</section>
</div>

<section class="card" style="margin-top:14px"><h2>Operation log</h2><div id="log"></div></section>
</main>
<script>
const logEl=document.getElementById('log');
function log(s,bad=false){const t=new Date().toLocaleTimeString();logEl.textContent+=`[${t}] ${s}\n`;logEl.scrollTop=logEl.scrollHeight;}
async function jfetch(url,opt){try{const r=await fetch(url,opt);const j=await r.json();log(j.message||JSON.stringify(j),!j.ok);await refresh();return j}catch(e){log('Request failed: '+e,true);return null}}
async function action(url){return jfetch(url)}
async function destructive(url,msg,token='ERASE'){if(confirm(msg))return jfetch(url+(url.includes('?')?'&':'?')+'confirm='+encodeURIComponent(token))}
async function setProfile(){const i=document.getElementById('profile').value;await jfetch('/api/profile?index='+i)}
async function upload(which){const input=document.getElementById(which+'File');if(!input.files.length){log('Choose a '+which.toUpperCase()+' file first',true);return}const f=new FormData();f.append('file',input.files[0]);await jfetch('/upload/'+which,{method:'POST',body:f})}
async function programErom(){if(!confirm('Program the loaded EROM buffer into the target?'))return;const p=document.getElementById('eromPage').value;await jfetch('/api/program?mem=erom&page='+p+'&confirm=PROGRAM')}
async function programEeprom(){if(!confirm('Program the loaded EEPROM buffer into the target?'))return;const m=document.getElementById('eeMode').value;await jfetch('/api/program?mem=eeprom&mode='+m+'&confirm=PROGRAM')}
async function verifyEeprom(){const m=document.getElementById('eeMode').value;await jfetch('/api/verify?mem=eeprom&mode='+m)}
async function readSig(){await jfetch('/api/signature?type='+document.getElementById('sig').value)}
async function writeReg(){const a=document.getElementById('regAddr').value;const v=document.getElementById('regValue').value;if(!confirm('Send raw register write '+a+' = '+v+'?'))return;await jfetch('/api/register?addr='+encodeURIComponent(a)+'&value='+encodeURIComponent(v)+'&confirm=WRITE')}
async function refresh(){try{const r=await fetch('/api/status');const s=await r.json();const c=document.getElementById('connection');c.textContent=(s.connected?'CONNECTED':'DISCONNECTED')+' · '+s.profile;c.className='status '+(s.connected?'ok':'');document.getElementById('buffers').textContent=`EROM: ${s.eromLoaded?s.eromLength+' bytes':'not loaded'} · EEPROM: ${s.eepromLoaded?s.eepromLength+' bytes':'not loaded'}`;document.getElementById('signature').textContent='Last: '+s.signature;const p=document.getElementById('profile');if(!p.options.length){s.profiles.forEach((x,i)=>{const o=document.createElement('option');o.value=i;o.textContent=x;p.appendChild(o)})}p.value=s.profileIndex}catch(e){}}
refresh();setInterval(refresh,3000);
</script></body></html>
)HTML";

// ---------------------------------------------------------------------------
// Web response helpers
// ---------------------------------------------------------------------------

static void sendJson(bool ok, const String &message, const String &extra = "") {
  String json = "{\"ok\":" + String(ok ? "true" : "false") +
                ",\"message\":\"" + jsonEscape(message) + "\"";
  if (extra.length()) json += "," + extra;
  json += "}";
  server.send(ok ? 200 : 400, "application/json", json);
}

static bool acquireOperation() {
  if (operationBusy) {
    sendJson(false, "Another operation is active");
    return false;
  }
  operationBusy = true;
  return true;
}

static void releaseOperation() {
  operationBusy = false;
}

static bool parseHexByteArg(const String &name, uint8_t &value) {
  if (!server.hasArg(name)) return false;
  String text = server.arg(name);
  text.trim();
  if (text.startsWith("0x") || text.startsWith("0X")) text.remove(0, 2);
  if (text.length() < 1 || text.length() > 2) return false;
  char *end = nullptr;
  long parsed = strtol(text.c_str(), &end, 16);
  if (!end || *end != '\0' || parsed < 0 || parsed > 255) return false;
  value = (uint8_t)parsed;
  return true;
}

static void handleStatus() {
  String profilesJson = "[";
  for (size_t i = 0; i < PROFILE_COUNT; ++i) {
    if (i) profilesJson += ',';
    profilesJson += "\"" + jsonEscape(PROFILES[i].name) + "\"";
  }
  profilesJson += "]";

  String sig = hexByte(lastSignature[0]) + " " + hexByte(lastSignature[1]) + " " +
               hexByte(lastSignature[2]);
  String json = "{\"ok\":true,\"connected\":" + String(targetConnected ? "true" : "false") +
                ",\"busy\":" + String(operationBusy ? "true" : "false") +
                ",\"profileIndex\":" + String(selectedProfile) +
                ",\"profile\":\"" + jsonEscape(profile().name) + "\"" +
                ",\"profiles\":" + profilesJson +
                ",\"eromLoaded\":" + String(eromLoaded ? "true" : "false") +
                ",\"eromLength\":" + String(eromBufferLength) +
                ",\"eepromLoaded\":" + String(eepromLoaded ? "true" : "false") +
                ",\"eepromLength\":" + String(eepromBufferLength) +
                ",\"signature\":\"" + sig + "\"" +
                ",\"message\":\"" + jsonEscape(lastMessage) + "\"}";
  server.send(200, "application/json", json);
}

static void handleProfile() {
  if (!server.hasArg("index")) {
    sendJson(false, "Missing profile index");
    return;
  }
  const int index = server.arg("index").toInt();
  if (index < 0 || index >= (int)PROFILE_COUNT) {
    sendJson(false, "Invalid profile index");
    return;
  }

  powerOffTarget("Profile changed; target powered off");
  selectedProfile = (uint8_t)index;
  eromLoaded = false;
  eepromLoaded = false;
  eromBufferLength = 0;
  eepromBufferLength = 0;
  lastMessage = "Selected " + String(profile().name);
  sendJson(true, lastMessage);
}

static void handleConnect() {
  if (!acquireOperation()) return;
  const bool erase = server.hasArg("erase") && server.arg("erase") == "1";
  if (erase && (!server.hasArg("confirm") || server.arg("confirm") != "ERASE")) {
    releaseOperation();
    sendJson(false, "Connect + erase requires confirm=ERASE");
    return;
  }
  const bool ok = pcfConnect(erase);
  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void handlePowerOff() {
  powerOffTarget();
  sendJson(true, lastMessage);
}

static void handleErase() {
  if (!server.hasArg("confirm") || server.arg("confirm") != "ERASE") {
    sendJson(false, "Erase requires confirm=ERASE");
    return;
  }
  if (!acquireOperation()) return;
  const bool ok = requireConnected() && pcfErase();
  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void handleProtect() {
  if (!server.hasArg("confirm") || server.arg("confirm") != "PROTECT") {
    sendJson(false, "Protect requires confirm=PROTECT");
    return;
  }
  if (!acquireOperation()) return;
  const bool ok = pcfProtect();
  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void handleRead() {
  if (!server.hasArg("mem")) {
    sendJson(false, "Missing mem argument");
    return;
  }
  if (!acquireOperation()) return;
  bool ok = false;
  if (server.arg("mem") == "erom") ok = readEromToBuffer();
  else if (server.arg("mem") == "eeprom") ok = readEepromToBuffer();
  else lastMessage = "Unknown memory type";
  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void handleProgram() {
  if (!server.hasArg("confirm") || server.arg("confirm") != "PROGRAM") {
    sendJson(false, "Programming requires confirm=PROGRAM");
    return;
  }
  if (!server.hasArg("mem")) {
    sendJson(false, "Missing mem argument");
    return;
  }
  if (!acquireOperation()) return;

  bool ok = false;
  if (server.arg("mem") == "erom") {
    const uint8_t pageSize = server.hasArg("page") ? (uint8_t)server.arg("page").toInt()
                                                       : profile().preferredEromPageSize;
    ok = programErom(pageSize);
  } else if (server.arg("mem") == "eeprom") {
    const String mode = server.hasArg("mode") ? server.arg("mode") : String("safe");
    ok = programEeprom(mode);
  } else {
    lastMessage = "Unknown memory type";
  }

  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void handleVerify() {
  if (!server.hasArg("mem")) {
    sendJson(false, "Missing mem argument");
    return;
  }
  if (!acquireOperation()) return;

  size_t mismatches = 0;
  size_t first = SIZE_MAX;
  bool ok = false;
  if (server.arg("mem") == "erom") {
    ok = verifyEromAgainstBuffer(mismatches, first);
  } else if (server.arg("mem") == "eeprom") {
    const String mode = server.hasArg("mode") ? server.arg("mode") : String("safe");
    ok = verifyEepromAgainstBuffer(mode, mismatches, first);
  } else {
    lastMessage = "Unknown memory type";
  }

  const String message = lastMessage;
  const String extra = "\"mismatches\":" + String(mismatches) +
                       ",\"firstMismatch\":" +
                       String(first == SIZE_MAX ? -1 : (long)first);
  releaseOperation();
  sendJson(ok, message, extra);
}

static void handleSignature() {
  if (!server.hasArg("type")) {
    sendJson(false, "Missing signature type");
    return;
  }
  if (!acquireOperation()) return;
  const bool ok = readSignature(server.arg("type"));
  const String message = lastMessage;
  const String extra = "\"signature\":\"" + hexByte(lastSignature[0]) + " " +
                       hexByte(lastSignature[1]) + " " + hexByte(lastSignature[2]) + "\"";
  releaseOperation();
  sendJson(ok, message, extra);
}

static void handleReset() {
  if (!acquireOperation()) return;
  const bool ok = pcfReset();
  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void handleRun() {
  if (!acquireOperation()) return;
  const bool ok = pcfRun();
  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void handleRegister() {
  if (!server.hasArg("confirm") || server.arg("confirm") != "WRITE") {
    sendJson(false, "Register write requires confirm=WRITE");
    return;
  }
  uint8_t address = 0, value = 0;
  if (!parseHexByteArg("addr", address) || !parseHexByteArg("value", value)) {
    sendJson(false, "Register address/value must be one-byte hexadecimal values");
    return;
  }
  if (!acquireOperation()) return;
  const bool ok = pcfWriteRegister(address, value);
  const String message = lastMessage;
  releaseOperation();
  sendJson(ok, message);
}

static void sendBinaryDownload(const uint8_t *data, size_t length, const char *filename) {
  if (!data || length == 0) {
    server.send(404, "text/plain", "Buffer is empty");
    return;
  }

  WiFiClient client = server.client();
  client.print("HTTP/1.1 200 OK\r\n");
  client.print("Content-Type: application/octet-stream\r\n");
  client.print("Content-Disposition: attachment; filename=\"");
  client.print(filename);
  client.print("\"\r\nContent-Length: ");
  client.print(length);
  client.print("\r\nConnection: close\r\n\r\n");

  size_t sent = 0;
  while (sent < length && client.connected()) {
    const size_t chunk = min((size_t)1024, length - sent);
    const size_t written = client.write(data + sent, chunk);
    if (written == 0) break;
    sent += written;
    yield();
  }
}

static void handleDownloadErom() {
  if (!eromLoaded) {
    server.send(404, "text/plain", "EROM buffer is empty");
    return;
  }
  sendBinaryDownload(eromBuffer, eromBufferLength, "pcf_erom.bin");
}

static void handleDownloadEeprom() {
  if (!eepromLoaded) {
    server.send(404, "text/plain", "EEPROM buffer is empty");
    return;
  }
  sendBinaryDownload(eepromBuffer, eepromBufferLength, "pcf_eeprom.bin");
}

static void handleUploadChunk(UploadTarget target) {
  HTTPUpload &upload = server.upload();
  uint8_t *destination = (target == UPLOAD_EROM) ? eromBuffer : eepromBuffer;
  const size_t capacity = (target == UPLOAD_EROM) ? MAX_EROM_SIZE : MAX_EEPROM_SIZE;

  switch (upload.status) {
    case UPLOAD_FILE_START:
      uploadTarget = target;
      uploadPosition = 0;
      uploadFailed = false;
      uploadError = "";
      break;

    case UPLOAD_FILE_WRITE:
      if (uploadTarget != target || uploadFailed) break;
      if (uploadPosition + upload.currentSize > capacity) {
        uploadFailed = true;
        uploadError = "Upload exceeds ESP32 buffer capacity";
        break;
      }
      memcpy(destination + uploadPosition, upload.buf, upload.currentSize);
      uploadPosition += upload.currentSize;
      break;

    case UPLOAD_FILE_ABORTED:
      uploadFailed = true;
      uploadError = "Upload aborted";
      break;

    case UPLOAD_FILE_END:
      break;

    default:
      break;
  }
}

static void finishUpload(UploadTarget target) {
  const size_t required = (target == UPLOAD_EROM) ? profile().eromSize : profile().eepromSize;
  if (uploadTarget != target) {
    sendJson(false, "Upload state mismatch");
    return;
  }
  if (uploadFailed) {
    sendJson(false, uploadError);
    uploadTarget = UPLOAD_NONE;
    return;
  }
  if (uploadPosition != required) {
    const String message = "File size " + String(uploadPosition) +
                           " does not match selected profile size " + String(required);
    uploadTarget = UPLOAD_NONE;
    sendJson(false, message);
    return;
  }

  if (target == UPLOAD_EROM) {
    eromBufferLength = uploadPosition;
    eromLoaded = true;
    lastMessage = "Loaded " + String(uploadPosition) + " EROM bytes into ESP32 RAM";
  } else {
    eepromBufferLength = uploadPosition;
    eepromLoaded = true;
    lastMessage = "Loaded " + String(uploadPosition) + " EEPROM bytes into ESP32 RAM";
  }
  uploadTarget = UPLOAD_NONE;
  sendJson(true, lastMessage);
}

// ---------------------------------------------------------------------------
// Setup and loop
// ---------------------------------------------------------------------------

static void configureRoutes() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/profile", HTTP_GET, handleProfile);
  server.on("/api/connect", HTTP_GET, handleConnect);
  server.on("/api/poweroff", HTTP_GET, handlePowerOff);
  server.on("/api/erase", HTTP_GET, handleErase);
  server.on("/api/protect", HTTP_GET, handleProtect);
  server.on("/api/read", HTTP_GET, handleRead);
  server.on("/api/program", HTTP_GET, handleProgram);
  server.on("/api/verify", HTTP_GET, handleVerify);
  server.on("/api/signature", HTTP_GET, handleSignature);
  server.on("/api/reset", HTTP_GET, handleReset);
  server.on("/api/run", HTTP_GET, handleRun);
  server.on("/api/register", HTTP_GET, handleRegister);
  server.on("/download/erom.bin", HTTP_GET, handleDownloadErom);
  server.on("/download/eeprom.bin", HTTP_GET, handleDownloadEeprom);

  server.on("/upload/erom", HTTP_POST,
            []() { finishUpload(UPLOAD_EROM); },
            []() { handleUploadChunk(UPLOAD_EROM); });
  server.on("/upload/eeprom", HTTP_POST,
            []() { finishUpload(UPLOAD_EEPROM); },
            []() { handleUploadChunk(UPLOAD_EEPROM); });

  server.onNotFound([]() { server.send(404, "application/json", "{\"ok\":false,\"message\":\"Not found\"}"); });
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(PIN_TARGET_POWER, OUTPUT);
  pinMode(PIN_MSDA, INPUT_PULLUP);
  pinMode(PIN_MSCL, INPUT);
  setTargetPower(false);

  // Attach on the Arduino task's core. On classic ESP32 this normally keeps
  // the very small MDI ISR separate from most Wi-Fi work on core 0.
  attachInterrupt(digitalPinToInterrupt(PIN_MSCL), onMsclChange, CHANGE);

  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  if (strlen(WIFI_SSID) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 10000) {
      delay(100);
    }
  }

  configureRoutes();
  server.begin();

  lastMessage = "Ready. AP IP " + WiFi.softAPIP().toString();
  Serial.println();
  Serial.println("pcf79xxprog");
  Serial.println(lastMessage);
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("STA IP: "); Serial.println(WiFi.localIP());
  }
}

void loop() {
  server.handleClient();
  delay(1);
}
