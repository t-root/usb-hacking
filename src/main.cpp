#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBMSC.h"
#include "FS.h"
#include "SD.h"
#include <math.h>

USBHIDKeyboard Keyboard;

// ================= PIN CONFIG =================
constexpr uint8_t LCD_MOSI = 45;
constexpr uint8_t LCD_SCLK = 40;
constexpr uint8_t LCD_CS = 42;
constexpr uint8_t LCD_DC = 41;
constexpr uint8_t LCD_RST = 39;
constexpr uint8_t LCD_BL = 48;
constexpr uint8_t LED_PIN = 38;
constexpr uint8_t LED_COUNT = 1;

// SD Card Pins
#define SD_SCLK  14
#define SD_MOSI  15
#define SD_MISO  16
#define SD_CS    21

// LCD size
constexpr uint16_t LCD_WIDTH = 172;
constexpr uint16_t LCD_HEIGHT = 320;

// ================= COLOR PALETTE =================
#define COLOR_BLACK 0x0000
#define COLOR_GREEN 0x07E0

// ================= COUNTER CONFIG =================
uint16_t colStart = 34; // Fixed offset for 180 rotation
uint16_t rowStart = 0;  // Fixed offset for 180 rotation

// DIGIT bitmap uses 3x5 blocks
constexpr uint8_t DIGIT_COLS = 3;
constexpr uint8_t DIGIT_ROWS = 5;
constexpr uint8_t DIGIT_SCALE = 3;
constexpr uint16_t DIGIT_WIDTH = DIGIT_COLS * DIGIT_SCALE;
constexpr uint16_t DIGIT_HEIGHT = DIGIT_ROWS * DIGIT_SCALE;
constexpr uint16_t DIGIT_SPACING = 6;
constexpr uint32_t DEFAULT_COUNTER_INTERVAL_MS = 1000;

constexpr uint8_t MAX_VISIBLE_DIGITS = 10;
uint16_t COUNTER_START_X;

// Vị trí Y của 3 dòng - có thể điều chỉnh
uint16_t COUNTER_START_Y = 160;
uint16_t TIME_START_Y = 210;
uint16_t SPEED_START_Y = 210;

// Button
constexpr uint8_t BOOT_BUTTON_PIN = 0;
constexpr uint32_t DEBOUNCE_DELAY = 25;
constexpr uint32_t DOUBLE_CLICK_TIME = 400;
constexpr uint32_t LED_HOLD_DURATION = 5000; // 5 seconds for LED control

// MADCTL fixed for 180 rotation
uint8_t MADCTL = 0xC0; // MX|MY => 180deg

// runtime
uint32_t counterIntervalMs = DEFAULT_COUNTER_INTERVAL_MS;
uint8_t lastDigits[MAX_VISIBLE_DIGITS];
uint8_t currentDigitCount = 1;

// State variables
bool counterRunning = false;
bool counterPaused = false;
bool selectingSpeed = false;
// At power-up show only the logo/text. Only after pressing the boot button
// will the number-selection UI be shown.
bool initialScreenOnly = true;
uint32_t currentValue = 0;
uint8_t selectedDigitCount = 1;
uint32_t maxValueForSelectedDigits = 9;

// speed presets
const uint16_t speedPresets[] = {1000, 900, 800, 700, 600, 500, 400, 300, 200, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10};
const uint8_t speedPresetsCount = sizeof(speedPresets) / sizeof(speedPresets[0]);
uint8_t currentSpeedIndex = 0;

// button timing - TÁCH BIỆT CLICK VÀ HOLD
uint32_t lastButtonCheck = 0;
bool lastButtonState = HIGH;
uint32_t buttonPressStartTime = 0;
bool buttonBeingHeld = false;
bool waitingSecondClick = false;
uint32_t lastClickTime = 0;
uint8_t clickCount = 0;
bool pendingSingleClick = false;
uint32_t pendingSingleClickTime = 0;

// LED control - RIÊNG BIỆT
bool ledState = false; // false = off, true = on
uint32_t ledHoldStartTime = 0;
bool ledHoldDetected = false;
bool processingLEDHold = false; // Cờ để phân biệt đang xử lý hold LED

// auto-run timing
uint32_t lastUpdate = 0;
uint32_t startTime = 0;

// time render cache - TỐI ƯU HÓA: chỉ render phần thay đổi
uint32_t lastDisplayedTime = 0xFFFFFFFF;
char lastTimeText[16] = "";
uint8_t lastTimeParts[3] = {0xFF, 0xFF, 0xFF}; // hours, minutes, seconds cache

// HID variables
bool isSendingHID = false;
uint32_t hidSendStartTime = 0;
const uint16_t HID_DELAY_BETWEEN_DIGITS = 10;    // Delay giữa các chữ số (ms)
const uint16_t HID_DELAY_AFTER_NUMBER = 10;      // Delay sau mỗi số hoàn chỉnh (ms)
char hidBuffer[11]; // Buffer để lưu số dạng string (tối đa 10 chữ số + null)
uint8_t currentHIDDigit = 0;

// ================= SD CARD & MSC VARIABLES =================
USBMSC msc;
SPIClass sdSPI = SPIClass(HSPI);
bool sdCardInitialized = false;
bool usbMSCStarted = false;

// ================= LED RGB CONFIG =================
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Danh sách màu sắc cho LED RGB (khi bật)
struct RGBColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

RGBColor ledColors[] = {
  {255, 0, 0},     // Đỏ
  {255, 128, 0},   // Cam
  {255, 255, 0},   // Vàng
  {0, 255, 0},     // Xanh lá
  {0, 255, 255},   // Cyan
  {0, 0, 255},     // Xanh dương
  {128, 0, 255},   // Tím
  {255, 0, 255},   // Hồng
  {255, 255, 255}  // Trắng
};

const int ledColorCount = sizeof(ledColors) / sizeof(ledColors[0]);
int currentLedColorIndex = 0;
unsigned long lastLedChange = 0;
const long ledInterval = 300;  // Thời gian chuyển màu LED

// ================= FUNCTION PROTOTYPES =================
void processSingleClick();
void processDoubleClick();
void handleBootButton();
void toggleLED();
void updateLED();
void initLED();

// ================= LOGO BITMAP =================
// Bitmap 9x16 - định dạng từ trái sang phải, từ trên xuống dưới
const uint8_t logo_bitmap[] = {
    0b11111111, 0b10000000, // dòng 1
    0b00001000, 0b00000000, // dòng 2
    0b00001000, 0b00000000, // dòng 3
    0b10001111, 0b10000000, // dòng 4
    0b11001000, 0b10000000, // dòng 5
    0b10101000, 0b10000000, // dòng 6
    0b10011111, 0b10000000, // dòng 7
    0b10001000, 0b10000000, // dòng 8
    0b10001100, 0b10000000, // dòng 9
    0b10001010, 0b10000000, // dòng 10
    0b10001001, 0b10000000, // dòng 11
    0b10001000, 0b10000000, // dòng 12
    0b01000001, 0b00000000, // dòng 13
    0b00100010, 0b00000000, // dòng 14
    0b00010100, 0b00000000, // dòng 15
    0b00001000, 0b00000000  // dòng 16
};

// ================= MSC CALLBACK FUNCTIONS =================
static int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  if (!sdCardInitialized) return -1;
  
  size_t sectors = bufsize / 512;
  
  for(size_t i = 0; i < sectors; i++) {
    if(!SD.readRAW((uint8_t*)buffer + (i * 512), lba + i)) {
      return -1;
    }
  }
  return bufsize;
}

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  if (!sdCardInitialized) return -1;
  
  size_t sectors = bufsize / 512;
  
  for(size_t i = 0; i < sectors; i++) {
    if(!SD.writeRAW((uint8_t*)buffer + (i * 512), lba + i)) {
      return -1;
    }
  }
  return bufsize;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
  return true;
}

// ================= LED FUNCTIONS =================
void initLED() {
  led.begin();
  led.setBrightness(80); // Độ sáng mặc định
  led.show(); // Tắt LED lúc khởi động
  ledState = false;
  Serial.println("LED RGB initialized - OFF");
}

void toggleLED() {
  ledState = !ledState;
  if (ledState) {
    Serial.println("LED RGB turned ON");
  } else {
    led.clear();
    led.show();
    Serial.println("LED RGB turned OFF");
  }
}

void updateLED() {
  if (!ledState) return;
  
  unsigned long currentMillis = millis();
  if (currentMillis - lastLedChange >= ledInterval) {
    lastLedChange = currentMillis;
    
    // Áp dụng màu hiện tại cho LED
    RGBColor currentColor = ledColors[currentLedColorIndex];
    led.setPixelColor(0, currentColor.red, currentColor.green, currentColor.blue);
    led.show();
    
    // Chuyển sang màu tiếp theo
    currentLedColorIndex = (currentLedColorIndex + 1) % ledColorCount;
  }
}

// ================= LCD LOW LEVEL =================
void sendCmd(uint8_t cmd)
{
  digitalWrite(LCD_DC, LOW);
  digitalWrite(LCD_CS, LOW);
  SPI.transfer(cmd);
  digitalWrite(LCD_CS, HIGH);
}

void sendData(uint8_t data)
{
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);
  SPI.transfer(data);
  digitalWrite(LCD_CS, HIGH);
}

void sendData16(uint16_t data)
{
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);
  SPI.transfer16(data);
  digitalWrite(LCD_CS, HIGH);
}

bool swapXY() { return (MADCTL & 0x20) != 0; }

void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  if (!swapXY())
  {
    sendCmd(0x2A);
    sendData16(x0 + colStart);
    sendData16(x1 + colStart);

    sendCmd(0x2B);
    sendData16(y0 + rowStart);
    sendData16(y1 + rowStart);
  }
  else
  {
    // swapped X/Y case (MV bit set)
    sendCmd(0x2A);
    sendData16(y0 + colStart);
    sendData16(y1 + colStart);

    sendCmd(0x2B);
    sendData16(x0 + rowStart);
    sendData16(x1 + rowStart);
  }
  sendCmd(0x2C);
}

// Hàm fillArea được tối ưu - sử dụng transfer16 và fill nhanh
void fillArea(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  if (x1 < x0 || y1 < y0)
    return;

  // Set window to the entire area at once
  setWindow(x0, y0, x1, y1);

  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);

  uint32_t count = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);

  // Sử dụng transfer16 để gửi 2 byte cùng lúc
  for (uint32_t i = 0; i < count; i++)
  {
    SPI.transfer16(color);
  }

  digitalWrite(LCD_CS, HIGH);
}

// Hàm fillScreen tối ưu - fill toàn bộ màn hình ngay lập tức
void fillScreen(uint16_t color)
{
  // Set window to entire screen
  setWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);

  uint32_t totalPixels = (uint32_t)LCD_WIDTH * (uint32_t)LCD_HEIGHT;

  // Fill toàn bộ màn hình với màu
  for (uint32_t i = 0; i < totalPixels; i++)
  {
    SPI.transfer16(color);
  }

  digitalWrite(LCD_CS, HIGH);
}

void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if (w == 0 || h == 0)
    return;
  fillArea(x, y, x + w - 1, y + h - 1, color);
}

// ================= LOGO FUNCTIONS =================
void drawLogo(uint16_t x, uint16_t y, uint16_t color, uint8_t scale = 3)
{
  for (uint8_t row = 0; row < 16; row++)
  {
    // Mỗi dòng được biểu diễn bằng 2 byte, nhưng thực tế chỉ 9 bit đầu có ý nghĩa
    uint16_t line = (logo_bitmap[row * 2] << 8) | logo_bitmap[row * 2 + 1];
    for (uint8_t col = 0; col < 9; col++)
    {
      // Bit thứ col (từ trái sang) là bit (15 - col) trong line
      if (line & (1 << (15 - col)))
      {
        drawRect(x + col * scale, y + row * scale, scale, scale, color);
      }
    }
  }
}

void drawCenteredLogo(uint16_t y, uint16_t color, uint8_t scale = 3)
{
  uint16_t logoWidth = 9 * scale;
  uint16_t logoHeight = 16 * scale;
  uint16_t x = (LCD_WIDTH - logoWidth) / 2;
  drawLogo(x, y, color, scale);
}

// ================= FONT (3x5) =================
void drawChar(char c, uint16_t x, uint16_t y, uint16_t color, uint8_t size = 2)
{
  uint8_t charData[5] = {0};
  switch (c)
  {
  case 'T':
    charData[0] = 0b111;
    charData[1] = 0b010;
    charData[2] = 0b010;
    charData[3] = 0b010;
    charData[4] = 0b010;
    break;
  case 'R':
    charData[0] = 0b111;
    charData[1] = 0b101;
    charData[2] = 0b111;
    charData[3] = 0b110;
    charData[4] = 0b101;
    break;
  case 'O':
    charData[0] = 0b111;
    charData[1] = 0b101;
    charData[2] = 0b101;
    charData[3] = 0b101;
    charData[4] = 0b111;
    break;
  case 'M':
    charData[0] = 0b101;
    charData[1] = 0b111;
    charData[2] = 0b101;
    charData[3] = 0b101;
    charData[4] = 0b101;
    break;
  case 'S':
    charData[0] = 0b111;
    charData[1] = 0b100;
    charData[2] = 0b111;
    charData[3] = 0b001;
    charData[4] = 0b111;
    break;
  case '0':
    charData[0] = 0b111;
    charData[1] = 0b101;
    charData[2] = 0b101;
    charData[3] = 0b101;
    charData[4] = 0b111;
    break;
  case '1':
    charData[0] = 0b010;
    charData[1] = 0b010;
    charData[2] = 0b010;
    charData[3] = 0b010;
    charData[4] = 0b010;
    break;
  case '2':
    charData[0] = 0b111;
    charData[1] = 0b001;
    charData[2] = 0b111;
    charData[3] = 0b100;
    charData[4] = 0b111;
    break;
  case '3':
    charData[0] = 0b111;
    charData[1] = 0b001;
    charData[2] = 0b111;
    charData[3] = 0b001;
    charData[4] = 0b111;
    break;
  case '4':
    charData[0] = 0b101;
    charData[1] = 0b101;
    charData[2] = 0b111;
    charData[3] = 0b001;
    charData[4] = 0b001;
    break;
  case '5':
    charData[0] = 0b111;
    charData[1] = 0b100;
    charData[2] = 0b111;
    charData[3] = 0b001;
    charData[4] = 0b111;
    break;
  case '6':
    charData[0] = 0b111;
    charData[1] = 0b100;
    charData[2] = 0b111;
    charData[3] = 0b101;
    charData[4] = 0b111;
    break;
  case '7':
    charData[0] = 0b111;
    charData[1] = 0b001;
    charData[2] = 0b001;
    charData[3] = 0b001;
    charData[4] = 0b001;
    break;
  case '8':
    charData[0] = 0b111;
    charData[1] = 0b101;
    charData[2] = 0b111;
    charData[3] = 0b101;
    charData[4] = 0b111;
    break;
  case '9':
    charData[0] = 0b111;
    charData[1] = 0b101;
    charData[2] = 0b111;
    charData[3] = 0b001;
    charData[4] = 0b111;
    break;
  case ':':
    charData[0] = 0b000;
    charData[1] = 0b010;
    charData[2] = 0b000;
    charData[3] = 0b010;
    charData[4] = 0b000;
    break;
  case '-':
    charData[0] = 0b000;
    charData[1] = 0b000;
    charData[2] = 0b111;
    charData[3] = 0b000;
    charData[4] = 0b000;
    break;
  case ' ':
    charData[0] = 0b000;
    charData[1] = 0b000;
    charData[2] = 0b000;
    charData[3] = 0b000;
    charData[4] = 0b000;
    break;
  default:
    return;
  }

  for (uint8_t r = 0; r < 5; r++)
  {
    for (uint8_t col = 0; col < 3; col++)
    {
      if (charData[r] & (1 << (2 - col)))
      {
        // draw scaled square block
        drawRect(x + col * size, y + r * size, size, size, color);
      }
    }
  }
}

void drawText(const char *text, uint16_t x, uint16_t y, uint16_t color, uint8_t size = 2)
{
  while (*text)
  {
    drawChar(*text++, x, y, color, size);
    x += (3 + 1) * size;
  }
}

void drawCenteredText(const char *text, uint16_t y, uint16_t color, uint8_t size = 2)
{
  uint16_t textWidth = strlen(text) * (3 + 1) * size;
  uint16_t x = (LCD_WIDTH - textWidth) / 2;
  drawText(text, x, y, color, size);
}

// ================= DIGITS =================
const uint8_t DIGIT_BITMAPS[10][5] = {
    {0b111, 0b101, 0b101, 0b101, 0b111}, // 0
    {0b010, 0b010, 0b010, 0b010, 0b010}, // 1
    {0b111, 0b001, 0b111, 0b100, 0b111}, // 2
    {0b111, 0b001, 0b111, 0b001, 0b111}, // 3
    {0b101, 0b101, 0b111, 0b001, 0b001}, // 4
    {0b111, 0b100, 0b111, 0b001, 0b111}, // 5
    {0b111, 0b100, 0b111, 0b101, 0b111}, // 6
    {0b111, 0b001, 0b001, 0b001, 0b001}, // 7
    {0b111, 0b101, 0b111, 0b101, 0b111}, // 8
    {0b111, 0b101, 0b111, 0b001, 0b111}  // 9
};

void drawDigit(uint8_t digit, uint16_t x, uint16_t y, uint16_t color, uint8_t scale = DIGIT_SCALE)
{
  digit %= 10;
  for (uint8_t r = 0; r < 5; r++)
  {
    uint8_t p = DIGIT_BITMAPS[digit][r];
    for (uint8_t c = 0; c < 3; c++)
    {
      if (p & (1 << (2 - c)))
      {
        drawRect(x + c * scale, y + r * scale, scale, scale, color);
      }
    }
  }
}

// ================= UTILITY =================
void calculateCounterPosition(uint8_t digitCount)
{
  uint16_t totalWidth = digitCount * DIGIT_WIDTH + (digitCount - 1) * DIGIT_SPACING;
  COUNTER_START_X = (LCD_WIDTH - totalWidth) / 2;
}

uint32_t calculateMaxValue(uint8_t digitCount)
{
  uint32_t maxVal = 1;
  for (uint8_t i = 0; i < digitCount; i++)
    maxVal *= 10;
  return maxVal - 1;
}

uint16_t digitStartX(uint8_t i)
{
  return COUNTER_START_X + i * (DIGIT_WIDTH + DIGIT_SPACING);
}

void clearDigit(uint8_t i)
{
  if (i >= MAX_VISIBLE_DIGITS)
    return;
  uint16_t x0 = digitStartX(i);
  fillArea(x0, COUNTER_START_Y, x0 + DIGIT_WIDTH - 1, COUNTER_START_Y + DIGIT_HEIGHT - 1, COLOR_BLACK);
}

void resetDigitsCache()
{
  for (uint8_t i = 0; i < MAX_VISIBLE_DIGITS; i++)
    lastDigits[i] = 0xFF;
}

void clearNumberArea()
{
  fillArea(0, COUNTER_START_Y - 10, LCD_WIDTH - 1, COUNTER_START_Y + DIGIT_HEIGHT + 10, COLOR_BLACK);
}

// ================= DRAW NUMBER =================
void drawNumber(uint32_t value, uint16_t color, bool force, bool padZeros)
{
  uint8_t displayDigits = selectedDigitCount;
  if (displayDigits > MAX_VISIBLE_DIGITS)
    displayDigits = MAX_VISIBLE_DIGITS;

  if (displayDigits != currentDigitCount)
  {
    currentDigitCount = displayDigits;
    calculateCounterPosition(currentDigitCount);
    resetDigitsCache();
    clearNumberArea();
    force = true; // Force redraw when digit count changes
  }

  uint8_t d[MAX_VISIBLE_DIGITS];
  uint32_t tmp = value;
  for (int8_t i = displayDigits - 1; i >= 0; i--)
  {
    d[i] = tmp % 10;
    tmp /= 10;
  }

  for (uint8_t i = 0; i < displayDigits; i++)
  {
    if (force || lastDigits[i] != d[i])
    {
      clearDigit(i);
      drawDigit(d[i], digitStartX(i), COUNTER_START_Y, color, DIGIT_SCALE);
      lastDigits[i] = d[i];
    }
  }

  for (uint8_t i = displayDigits; i < MAX_VISIBLE_DIGITS; i++)
    if (lastDigits[i] != 0xFF)
    {
      clearDigit(i);
      lastDigits[i] = 0xFF;
    }
}

void drawNumberAuto(uint32_t value, bool force) { drawNumber(value, COLOR_GREEN, force, true); }
void drawNumberSelect(uint32_t value, bool force) { drawNumber(value, COLOR_GREEN, force, true); }

// ================= SPEED UI =================
void drawSpeedSelection()
{
  uint16_t y_top = SPEED_START_Y - 10;
  uint16_t y_bottom = SPEED_START_Y + DIGIT_HEIGHT + 10;
  fillArea(0, y_top, LCD_WIDTH - 1, y_bottom, COLOR_BLACK);

  char speedText[20];
  sprintf(speedText, "%luMS", counterIntervalMs);
  drawCenteredText(speedText, SPEED_START_Y, COLOR_GREEN, 3);
}

// ================= TIME UI - TỐI ƯU HÓA =================
void drawTimeChar(char c, uint16_t x, uint16_t y, uint16_t color, uint8_t size = 3)
{
  // Chỉ vẽ ký tự tại vị trí xác định
  drawChar(c, x, y, color, size);
}

void clearTimeChar(uint16_t x, uint16_t y, uint8_t size = 3)
{
  // Xóa một ký tự (3x5 scaled) + khoảng cách
  uint16_t charWidth = (3 + 1) * size;
  uint16_t charHeight = 5 * size;
  fillArea(x, y, x + charWidth - 1, y + charHeight - 1, COLOR_BLACK);
}

void drawTimeOptimized(uint32_t timeMs)
{
  if (timeMs == lastDisplayedTime) return;
  lastDisplayedTime = timeMs;

  uint32_t seconds = timeMs / 1000;
  uint32_t minutes = seconds / 60;
  uint32_t hours = minutes / 60;

  uint8_t currentHours = hours;
  uint8_t currentMinutes = minutes % 60;
  uint8_t currentSeconds = seconds % 60;

  // Tính toán vị trí trung tâm cho thời gian
  uint16_t totalWidth = 8 * (3 + 1) * 3; // 8 ký tự (HH:MM:SS), mỗi ký tự rộng 12 pixel
  uint16_t startX = (LCD_WIDTH - totalWidth) / 2;

  // Chỉ cập nhật phần thay đổi
  if (currentHours != lastTimeParts[0])
  {
    // Xóa và vẽ lại phần giờ
    clearTimeChar(startX, TIME_START_Y);
    clearTimeChar(startX + 12, TIME_START_Y);
    drawTimeChar('0' + (currentHours / 10), startX, TIME_START_Y, COLOR_GREEN, 3);
    drawTimeChar('0' + (currentHours % 10), startX + 12, TIME_START_Y, COLOR_GREEN, 3);
    lastTimeParts[0] = currentHours;
  }

  if (currentMinutes != lastTimeParts[1])
  {
    // Xóa và vẽ lại phần phút
    clearTimeChar(startX + 36, TIME_START_Y);
    clearTimeChar(startX + 48, TIME_START_Y);
    drawTimeChar('0' + (currentMinutes / 10), startX + 36, TIME_START_Y, COLOR_GREEN, 3);
    drawTimeChar('0' + (currentMinutes % 10), startX + 48, TIME_START_Y, COLOR_GREEN, 3);
    lastTimeParts[1] = currentMinutes;
  }

  if (currentSeconds != lastTimeParts[2])
  {
    // Xóa và vẽ lại phần giây
    clearTimeChar(startX + 72, TIME_START_Y);
    clearTimeChar(startX + 84, TIME_START_Y);
    drawTimeChar('0' + (currentSeconds / 10), startX + 72, TIME_START_Y, COLOR_GREEN, 3);
    drawTimeChar('0' + (currentSeconds % 10), startX + 84, TIME_START_Y, COLOR_GREEN, 3);
    lastTimeParts[2] = currentSeconds;
  }

  // Vẽ dấu ':' một lần duy nhất (không thay đổi)
  static bool colonsDrawn = false;
  if (!colonsDrawn)
  {
    drawTimeChar(':', startX + 24, TIME_START_Y, COLOR_GREEN, 3);
    drawTimeChar(':', startX + 60, TIME_START_Y, COLOR_GREEN, 3);
    colonsDrawn = true;
  }
}

void clearTimeArea()
{
  // Chỉ clear khi thực sự cần (khi reset counter)
  fillArea(0, TIME_START_Y - 10, LCD_WIDTH - 1, TIME_START_Y + 25, COLOR_BLACK);
  // Reset cache
  lastTimeParts[0] = 0xFF;
  lastTimeParts[1] = 0xFF;
  lastTimeParts[2] = 0xFF;
  lastDisplayedTime = 0xFFFFFFFF;
}

// ================= HID FUNCTIONS =================
void startHIDSend(uint32_t value)
{
  // Format số với leading zeros
  sprintf(hidBuffer, "%0*lu", selectedDigitCount, value);
  isSendingHID = true;
  hidSendStartTime = millis();
  currentHIDDigit = 0;
  
  Serial.printf("Bắt đầu gửi HID: %s\n", hidBuffer);
}

void processHIDSend()
{
  if (!isSendingHID) return;

  uint32_t now = millis();
  
  if (currentHIDDigit < selectedDigitCount)
  {
    // Gửi từng chữ số với delay
    if (now - hidSendStartTime >= HID_DELAY_BETWEEN_DIGITS)
    {
      char digitChar = hidBuffer[currentHIDDigit];
      Keyboard.press(digitChar);
      delay(1);
      Keyboard.release(digitChar);
      
      Serial.printf("Đã gửi chữ số: %c\n", digitChar);
      
      currentHIDDigit++;
      hidSendStartTime = now;
    }
  }
  else
  {
    // Đã gửi xong tất cả chữ số, thêm ENTER và kết thúc
    if (now - hidSendStartTime >= HID_DELAY_AFTER_NUMBER)
    {
      Keyboard.write(KEY_RETURN);
      Serial.println("Đã gửi xong số, thêm ENTER");
      
      isSendingHID = false;
    }
  }
}

// ================= SD CARD FUNCTIONS =================
bool initializeSDCard() {
  Serial.println("Initializing SD card...");
  
  // Initialize SPI for SD card
  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  
  if (!SD.begin(SD_CS, sdSPI, 4000000)) {
    Serial.println("SD Card Mount Failed");
    return false;
  }

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return false;
  }

  Serial.printf("SD Card Type: ");
  if(cardType == CARD_MMC) Serial.println("MMC");
  else if(cardType == CARD_SD) Serial.println("SDSC");
  else if(cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");
  
  Serial.printf("SD Card Size: %lluMB\n", SD.cardSize() / (1024 * 1024));
  
  return true;
}

bool startUSBMSC() {
  if (!sdCardInitialized) return false;
  
  uint32_t sectorSize = 512;
  uint32_t sectorCount = SD.cardSize() / sectorSize;
  
  Serial.printf("Starting USB MSC with %lu sectors\n", sectorCount);

  // Configure MSC 
  msc.vendorID("T-ROOT");           // tối đa ~8 ký tự
  msc.productID("T-ROOT USB");      // tối đa ~16 ký tự
  msc.productRevision("1.0");  
  msc.onRead(onRead);
  msc.onWrite(onWrite);
  msc.onStartStop(onStartStop);
  msc.mediaPresent(true);
  msc.begin(sectorCount, sectorSize);

  return true;
}

// ================= BUTTON HANDLING - ĐÃ SỬA =================
void processSingleClick()
{
  if (selectingSpeed)
  {
    currentSpeedIndex = (currentSpeedIndex + 1) % speedPresetsCount;
    counterIntervalMs = speedPresets[currentSpeedIndex];
    drawSpeedSelection();
  }
  else if (!counterRunning)
  {
    // Nếu đang ở màn hình logo ban đầu, chuyển sang hiển thị T-ROOT và chọn số
    if (initialScreenOnly)
    {
      initialScreenOnly = false;
      maxValueForSelectedDigits = calculateMaxValue(selectedDigitCount);
      currentValue = 0;
      resetDigitsCache();
      clearNumberArea();
      
      // Xóa màn hình và hiển thị T-ROOT
      fillScreen(COLOR_BLACK);
      drawCenteredText("T-ROOT", 110, COLOR_GREEN, 3);
      drawNumberSelect(currentValue, true);
    }
    else
    {
      selectedDigitCount++;
      if (selectedDigitCount > 10)
        selectedDigitCount = 1;
      maxValueForSelectedDigits = calculateMaxValue(selectedDigitCount);
      currentValue = 0;
      resetDigitsCache();
      clearNumberArea();
      drawNumberSelect(currentValue, true);
    }
  }
  else
  {
    counterPaused = !counterPaused;
  }
}

void processDoubleClick()
{
  if (!counterRunning && !selectingSpeed)
  {
    selectingSpeed = true;
    currentSpeedIndex = 0;
    counterIntervalMs = speedPresets[currentSpeedIndex];
    drawSpeedSelection();
  }
  else if (selectingSpeed)
  {
    selectingSpeed = false;
    counterRunning = true;
    counterPaused = false;
    currentValue = 0;
    startTime = millis();
    resetDigitsCache();
    clearNumberArea();
    clearTimeArea(); // Clear time area when starting
    lastDisplayedTime = 0xFFFFFFFF;
    
    // Hiển thị số 0 ban đầu
    drawNumberAuto(currentValue, true);
    
    // Cập nhật thời gian ban đầu
    uint32_t remainingValues = maxValueForSelectedDigits - currentValue;
    uint32_t remainingTimeMs = remainingValues * counterIntervalMs;
    drawTimeOptimized(remainingTimeMs);
    
    // Bắt đầu gửi HID cho số đầu tiên
    startHIDSend(currentValue);
    
    Serial.printf("Bắt đầu chạy counter, số chữ số: %d\n", selectedDigitCount);
  }
  else
  {
    counterRunning = false;
    counterPaused = false;
    selectingSpeed = false;
    currentValue = 0;
    selectedDigitCount = 1;
    maxValueForSelectedDigits = calculateMaxValue(selectedDigitCount);
    resetDigitsCache();
    clearNumberArea();
    clearTimeArea(); // Clear time area when stopping
    drawNumberSelect(currentValue, true);
  }
}

void handleBootButton() {
  uint32_t now = millis();
  if (now - lastButtonCheck < DEBOUNCE_DELAY) return;
  lastButtonCheck = now;

  bool currentButtonState = digitalRead(BOOT_BUTTON_PIN);

  // Xử lý nhấn nút (cạnh xuống)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    buttonPressStartTime = now;
    buttonBeingHeld = true;
    processingLEDHold = false; // Reset cờ LED hold
    
    // Xử lý click
    if (!waitingSecondClick) {
      waitingSecondClick = true;
      clickCount = 1;
      lastClickTime = now;
    } else {
      if (now - lastClickTime <= DOUBLE_CLICK_TIME) {
        clickCount = 2;
      }
    }
  }

  // Xử lý thả nút (cạnh lên)
  if (currentButtonState == HIGH && lastButtonState == LOW) {
    buttonBeingHeld = false;
    
    // Nếu đang xử lý LED hold, bỏ qua xử lý click
    if (processingLEDHold) {
      processingLEDHold = false;
      ledHoldDetected = false;
    } 
    // Nếu không phải LED hold, xử lý click bình thường
    else if (clickCount == 2 && (now - lastClickTime) <= DOUBLE_CLICK_TIME) {
      waitingSecondClick = false;
      clickCount = 0;
      pendingSingleClick = false;
      processDoubleClick();
    } else if (clickCount == 1) {
      pendingSingleClick = true;
      pendingSingleClickTime = now;
    }
  }

  // Xử lý giữ nút cho LED (chỉ khi đang nhấn nút)
  if (buttonBeingHeld && !processingLEDHold) {
    if (now - buttonPressStartTime >= LED_HOLD_DURATION) {
      processingLEDHold = true;
      toggleLED();
    }
  }

  lastButtonState = currentButtonState;
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting T-ROOT Counter with SD Card MSC and LED Control...");

  // pins
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // LCD pins
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_DC, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_BL, LOW);

  // reset LCD
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(15);
  digitalWrite(LCD_RST, HIGH);
  delay(150);

  // SPI for LCD
  SPI.begin(LCD_SCLK, -1, LCD_MOSI, LCD_CS);
  SPI.setFrequency(40000000);

  // KHỞI TẠO LCD
  sendCmd(0x01);
  delay(150);
  sendCmd(0x11);
  delay(120);

  sendCmd(0x3A);
  sendData(0x55);
  delay(10);

  sendCmd(0x36);
  sendData(MADCTL);
  delay(10);

  sendCmd(0x21);
  delay(10);

  sendCmd(0x29);
  delay(120);

  fillScreen(COLOR_BLACK);
  delay(50);

  digitalWrite(LCD_BL, HIGH);

  // Khởi tạo LED RGB
  initLED();
  
  // Khởi tạo SD Card
  sdCardInitialized = initializeSDCard();
  
  // Khởi tạo USB với cả HID và MSC
  USB.begin();
  
  if (sdCardInitialized) {
    usbMSCStarted = startUSBMSC();
    if (usbMSCStarted) {
      Serial.println("USB MSC Started - Ready to use!");
    }
  }
  
  Keyboard.begin();
  delay(1000);

  maxValueForSelectedDigits = calculateMaxValue(selectedDigitCount);
  calculateCounterPosition(1);

  // initial UI - HIỂN THỊ LOGO KHI KHỞI ĐỘNG
  drawCenteredLogo(110, COLOR_GREEN, 5); // Logo scale 5x, vị trí Y = 110
  
  
  resetDigitsCache();
  currentValue = 0;

  lastUpdate = millis();
  startTime = millis();
}

// ================= LOOP =================
void loop()
{
  uint32_t now = millis();

  handleBootButton();  // Xử lý nút (cả click và hold)
  updateLED();         // Cập nhật màu LED nếu đang bật
  processHIDSend();

  // Xử lý click đơn (sau khi đã kiểm tra không phải double click)
  if (pendingSingleClick && now - pendingSingleClickTime > DOUBLE_CLICK_TIME)
  {
    pendingSingleClick = false;
    waitingSecondClick = false;
    clickCount = 0;
    processSingleClick();
  }

  if (counterRunning && !counterPaused && !isSendingHID)
  {
    if (currentValue < maxValueForSelectedDigits && now - lastUpdate >= counterIntervalMs)
    {
      lastUpdate = now;
      currentValue++;
      
      // Hiển thị số trước khi gửi HID
      drawNumberAuto(currentValue, false);
      
      // Bắt đầu gửi HID cho số mới
      startHIDSend(currentValue);
    }

    // Cập nhật thời gian mỗi giây
    static uint32_t lastTimeUpdate = 0;
    if (now - lastTimeUpdate >= 1000)
    {
      lastTimeUpdate = now;
      uint32_t remainingValues = maxValueForSelectedDigits - currentValue;
      uint32_t remainingTimeMs = remainingValues * counterIntervalMs;
      drawTimeOptimized(remainingTimeMs);
    }
  }

  delay(1);
}


 