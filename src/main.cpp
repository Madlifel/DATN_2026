#define BLYNK_TEMPLATE_ID "TMPL6tsQEYh5R"
#define BLYNK_TEMPLATE_NAME "SmartHydroponics"
#define BLYNK_AUTH_TOKEN "1Gk9pz_X3QobbMETF3KLhchpeCupJxzG"

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>
#include <time.h>

#define NTP_SERVER  "pool.ntp.org"
#define UTC_OFFSET  (7 * 3600)   // GMT+7 cho Việt Nam
// =====================================================================
// --- CẤU HÌNH CHÂN (PIN DEFINITIONS) ---
// =====================================================================
#define TdsSensorPin  32
#define VREF          3.3
#define ADC_RES       4095.0
#define SCOUNT        30
#define LED_PIN       25
#define LED_status    27
#define Button        33
#define Pump          26
#define I2C_SDA       21
#define I2C_SCL       22
#define LIGHT_ON_HOUR   8    // Bật lúc 08:00
#define LIGHT_OFF_HOUR  16   // Tắt lúc 16:00 (4 giờ chiều)

// =====================================================================
// --- KHỞI TẠO ĐỐI TƯỢNG ---
// =====================================================================
WiFiManager wm;
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
BH1750 lightMeter;
LiquidCrystal_I2C lcd(0x27, 16, 2);
BlynkTimer timer;

// =====================================================================
// --- GIỚI HẠN CẢNH BÁO ---
// =====================================================================
const float MAX_TEMP = 29.0;
const float Min_TEMP = 22.0;
const int   MAX_TDS  = 1000;
const int   Min_TDS  = 600;
const float MIN_LUX  = 10000.0;
const float MAX_LUX  = 20000.0;

// =====================================================================
// --- BIẾN TRẠNG THÁI ---
// =====================================================================
bool  autoMode  = false;
bool  pumpState = false;
int   ledPwm    = 0;

// Buffer TDS
int   analogBuffer[SCOUNT];
int   analogBufferTemp[SCOUNT];
int   analogBufferIndex = 0;
float averageVoltage    = 0;
float tdsValue          = 0;
float temperature       = 0;
float luxValue          = 0;
float bu                = 0.02;
// Nút nhấn
unsigned long buttonPressStart = 0;
bool          buttonHolding    = false;

// Chu kỳ bơm tự động
bool          pumpRunning        = false;
unsigned long previousMillisPump = 0;
const long    intervalOn         = 10000UL;
const long    intervalOff        = 20000UL;

// Chống spam thông báo Blynk
unsigned long lastNotifyTime = 0;
const long    notifyInterval = 300000UL;

// Cache LCD 
char prevHang0[17] = "";
char prevHang1[17] = "";
// Mode Auto/Manual
// =====================================================================
// --- THUẬT TOÁN LỌC TRUNG VỊ ---
// =====================================================================
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[SCOUNT];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];

  int bTemp;
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp       = bTab[i];
        bTab[i]     = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    return bTab[(iFilterLen - 1) / 2];
  else
    return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

// =====================================================================
// --- TÍNH GIÁ TRỊ TDS ---
// =====================================================================
float tinhTDS(float voltage, float temp) {
  // Nếu cảm biến lỗi, giả định nước ở 25°C để giữ cho phép tính TDS ổn định
  if (temp == DEVICE_DISCONNECTED_C || temp <= 0) {
      temp = 25.0; 
  }
  float compensationCoefficient = 1.0 + bu * (temp - 25.0);
  float compensationVoltage     = voltage / compensationCoefficient;
  float tds = (133.42 * pow(compensationVoltage, 3)
             - 255.86 * pow(compensationVoltage, 2)
             + 857.39 * compensationVoltage) * 0.5;
  return (tds < 0) ? 0 : tds;
}

// =====================================================================
// --- CẬP NHẬT TDS ---
// =====================================================================
void capnhatTDS() {
  static unsigned long analogSampleTimepoint = 0;
  if (millis() - analogSampleTimepoint > 40UL) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  static unsigned long printTimepoint = 0;
  if (millis() - printTimepoint > 800UL) {
    printTimepoint = millis();
    for (int i = 0; i < SCOUNT; i++) analogBufferTemp[i] = analogBuffer[i];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / ADC_RES;
    tdsValue       = tinhTDS(averageVoltage, temperature);
  }
}

// =====================================================================
// --- ĐIỀU KHIỂN BƠM TỰ ĐỘNG ---
// =====================================================================
void PumpAutoControl() {
  if (!autoMode) return;

  unsigned long currentMillis = millis();
  // Tính toán điều kiện môi trường an toàn
  bool envOK = (temperature >= Min_TEMP && temperature <= MAX_TEMP
                && tdsValue >= Min_TDS  && tdsValue <= MAX_TDS);

  if (pumpRunning) {
    // FIX: Nếu môi trường không còn an toàn HOẶC đã hết 10 giây thì dừng bơm ngay
    if (!envOK || (currentMillis - previousMillisPump >= intervalOn)) {
      pumpRunning        = false;
      pumpState          = false;
      previousMillisPump = currentMillis;
      digitalWrite(Pump, HIGH); // Tắt bơm
      
      if (!envOK) Serial.println("NGẮT KHẨN CẤP: Thông số vượt ngưỡng!");
      else Serial.println("Chu kỳ: TẮT BƠM (Hết 10s)");
      if (Blynk.connected()) Blynk.virtualWrite(V3, pumpState);
    }
  } else {
    // Chờ hết 20 giây nghỉ và môi trường phải OK mới được bật lại
    if (currentMillis - previousMillisPump >= intervalOff) {
      if (envOK) {
        pumpRunning        = true;
        pumpState          = true;
        previousMillisPump = currentMillis;
        digitalWrite(Pump, LOW); // Bật bơm
        Serial.println("Chu kỳ: BẬT BƠM");
        if (Blynk.connected()) Blynk.virtualWrite(V3, pumpState);
      }
    }
  }
}
// ==================== HÀM ĐỒNG BỘ HỜI GIAN ====================
void setupTime() {
  configTime(UTC_OFFSET, 0, NTP_SERVER);
  Serial.print("Đồng bộ NTP");

  unsigned long start = millis();
  while (time(nullptr) < 1577836800UL) {
    if (millis() - start >= 10000UL) {   // Timeout 10 giây
      Serial.println(" THẤT BẠI (sẽ thử lại sau)");
      return;
    }
    Serial.print(".");
    // Xử lý WiFi/yield trong lúc chờ, không block hệ thống
    yield();
    delay(200);  // ← vẫn cần delay nhỏ để NTP có thời gian phản hồi
  }
  Serial.println(" OK");
}
// ==================== HÀM KIỂM TRA KHUNG GIỜ ====================
bool isWithinSchedule() {
  time_t now = time(nullptr);
   // NTP chưa đồng bộ (time < năm 2020) → cho phép sáng mặc định
  if (now < 1577836800UL) {
    return true;
  }
  struct tm* t = localtime(&now);
  int hour = t->tm_hour;
  return (hour >= LIGHT_ON_HOUR && hour < LIGHT_OFF_HOUR);
}
// =====================================================================
// --- ĐIỀU CHỈNH ĐỘ SÁNG LED ---
// =====================================================================
void updateLedBrightness(float currentLux) {
  static int autoPwm = 0;
  const  int step    = 20;

  int pwmValue;
  // ── 1. Kiểm tra khung giờ ──────────────────────────
  if (autoMode && !isWithinSchedule()) {
    analogWrite(LED_PIN, 0);
    autoPwm = 0;
    if (Blynk.connected()) Blynk.virtualWrite(V4, 0);
    Serial.println("LED OFF: Ngoài khung giờ (Auto)");
    return;
  }
  // ── 2. Logic điều chỉnh độ sáng theo ánh sáng ──────
  if (autoMode) {
    if (currentLux < MIN_LUX) {
      autoPwm += step;
    } else if (currentLux > MAX_LUX) {
      autoPwm -= step;
    }
    autoPwm  = constrain(autoPwm, 0, 255);
    pwmValue = autoPwm;
    if (Blynk.connected()) Blynk.virtualWrite(V4, pwmValue);
  } else {
    autoPwm  = ledPwm; // Đồng bộ để khi chuyển Auto không bị giật
    pwmValue = ledPwm;
    if (Blynk.connected()) Blynk.virtualWrite(V4, pwmValue);
  }

  analogWrite(LED_PIN, pwmValue);
  Serial.printf("LED PWM: %d | Lux: %.0f (%s)\n",
                pwmValue, currentLux, autoMode ? "Auto" : "Manual");
}

// =====================================================================
// --- ĐIỀU KHIỂN THIẾT BỊ (TIMER 1.2 GIÂY) ---
// =====================================================================
void controlDevices() {
  PumpAutoControl();
  updateLedBrightness(luxValue);
}

// =====================================================================
// --- GỬI DỮ LIỆU LÊN BLYNK (TIMER 3 GIÂY) ---
// =====================================================================
void processAndSendData() {
  if (!Blynk.connected()) return;
   // Tự đồng bộ NTP nếu chưa có thời gian hợp lệ
  static bool ntpSynced = false;
  if (!ntpSynced && time(nullptr) > 1577836800UL) {
    ntpSynced = true;
    Serial.println("NTP đã đồng bộ thành công!");
  }
  if (!ntpSynced) {
    configTime(UTC_OFFSET, 0, NTP_SERVER); // Thử lại
  }
  // Gửi dữ liệu liên tục lên Blynk để vẽ biểu đồ
  Blynk.virtualWrite(V0, tdsValue);
  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, luxValue);
  Blynk.virtualWrite(V5, autoMode);

  Serial.printf("Data | Temp:%.1f°C TDS:%.0fppm Lux:%.0flx\n",
                temperature, tdsValue, luxValue);

  unsigned long currentMillis = millis();
  const unsigned long notifyInterval = 300000UL; // 5 phút khóa spam cho từng loại

  // Khai báo 3 biến mốc thời gian riêng biệt cho 3 cảm biến
  // Sử dụng từ khóa 'static' để biến không bị xóa sau khi thoát hàm
  static unsigned long lastTempNotify = 0;
  static unsigned long lastTdsNotify  = 0;
  static unsigned long lastLuxNotify  = 0;

  // -----------------------------------------------------------------
  // 1. KIỂM TRA NHIỆT ĐỘ (Chỉ kiểm tra nếu chưa từng báo hoặc đã hết 5 phút)
  // -----------------------------------------------------------------
  if (lastTempNotify == 0 || (currentMillis - lastTempNotify > notifyInterval)) {
    bool tempWarning = false;

    if (temperature > MAX_TEMP) {
      Blynk.logEvent("canh_bao_nhiet", "Cảnh báo: Nhiệt độ quá cao!");
      tempWarning = true;
    } else if (temperature < Min_TEMP) {
      Blynk.logEvent("canh_bao_nhiet", "Cảnh báo: Nhiệt độ quá thấp!");
      tempWarning = true;
    }

    if (tempWarning) {
      lastTempNotify = currentMillis; // Chỉ khóa riêng mục Nhiệt độ
    }
  }

  // -----------------------------------------------------------------
  // 2. KIỂM TRA TDS (Chạy hoàn toàn độc lập với Nhiệt độ)
  // -----------------------------------------------------------------
  if (lastTdsNotify == 0 || (currentMillis - lastTdsNotify > notifyInterval)) {
    bool tdsWarning = false;

    if (tdsValue > MAX_TDS) {
      Blynk.logEvent("canh_bao_tds", "Cảnh báo: TDS vượt ngưỡng!");
      tdsWarning = true;
    } else if (tdsValue < Min_TDS) {
      Blynk.logEvent("canh_bao_tds", "Cảnh báo: TDS quá thấp!");
      tdsWarning = true;
    }

    if (tdsWarning) {
      lastTdsNotify = currentMillis; // Chỉ khóa riêng mục TDS
    }
  }

  // -----------------------------------------------------------------
  // 3. KIỂM TRA ÁNH SÁNG (LUX)
  // -----------------------------------------------------------------
  if (lastLuxNotify == 0 || (currentMillis - lastLuxNotify > notifyInterval)) {
    bool luxWarning = false;

   if (isWithinSchedule()) {  // ← CHỈ cảnh báo lux khi đang trong khung giờ
    if (luxValue > MAX_LUX) {
      Blynk.logEvent("canh_bao_lux", "Cảnh báo: Ánh sáng vượt ngưỡng!");
      luxWarning = true;
    } else if (luxValue < MIN_LUX) {
      Blynk.logEvent("canh_bao_lux", "Cảnh báo: Ánh sáng quá thấp!");
      luxWarning = true;
    }
  }

    if (luxWarning) {
      lastLuxNotify = currentMillis; // Chỉ khóa riêng mục Ánh sáng
    }
  }
}

// =====================================================================
// --- NHẬN LỆNH TỪ BLYNK ---
// =====================================================================
BLYNK_WRITE(V3) {
  pumpState = param.asInt();
  if (!autoMode) {
    digitalWrite(Pump, pumpState ? LOW : HIGH);
    Serial.println(pumpState ? "Bơm: BẬT (thủ công)" : "Bơm: TẮT (thủ công)");
  }
}

BLYNK_WRITE(V4) {
  ledPwm = constrain(param.asInt(), 0, 255);
  if (!autoMode) {
    analogWrite(LED_PIN, ledPwm);
    Serial.printf("LED PWM (thủ công): %d\n", ledPwm);
  }
}

BLYNK_WRITE(V5) {
  autoMode = param.asInt();
  Serial.println(autoMode ? "Chế độ: TỰ ĐỘNG" : "Chế độ: THỦ CÔNG");
  if (!autoMode) {
    digitalWrite(Pump,   pumpState ? LOW : HIGH);
    analogWrite(LED_PIN, ledPwm);
  }
}

BLYNK_CONNECTED() {
  Serial.println("Đã kết nối Blynk! Đồng bộ trạng thái...");
  Blynk.syncVirtual(V3, V4, V5);
}

// =====================================================================
// --- HIỂN THỊ LCD ---
// =====================================================================
void displayLCD() {
  char h0[17], h1[17];
  bool showWarning = (millis() / 2000) % 2 == 0;

  // Hàng 0: Nhiệt độ & Lux
  if (temperature == DEVICE_DISCONNECTED_C) {
    snprintf(h0, sizeof(h0), "Loi cam bien T! ");
  } else if ((temperature > MAX_TEMP || temperature < Min_TEMP) && showWarning) {
    snprintf(h0, sizeof(h0), temperature > MAX_TEMP ? "NhietDo qua CAO!" : "NhietDo qua THAP");
  } else {
    snprintf(h0, sizeof(h0), "T:%-2d\xDF" "C L:%-5dLx ", (int)temperature, (int)luxValue);
  }

  // Hàng 1: TDS
  if ((tdsValue > MAX_TDS || tdsValue < Min_TDS) && showWarning) {
    snprintf(h1, sizeof(h1), tdsValue > MAX_TDS ? "TDS qua CAO!    " : "TDS qua THAP!   ");
  } else {
    snprintf(h1, sizeof(h1), "TDS:%-4d ppm    ", (int)tdsValue);
  }

  // Chỉ ghi khi thay đổi
  if (strcmp(h0, prevHang0) != 0) {
    lcd.setCursor(0, 0);
    lcd.print(h0);
    strncpy(prevHang0, h0, sizeof(prevHang0));
  }
  if (strcmp(h1, prevHang1) != 0) {
    lcd.setCursor(0, 1);
    lcd.print(h1);
    strncpy(prevHang1, h1, sizeof(prevHang1));
  }
}

// =====================================================================
// --- ĐỌC CẢM BIẾN & LCD (TIMER 1 GIÂY) ---
// =====================================================================
void docCamBienVaLCD() {
  /// 1. Đọc nhiệt độ (kết quả đã được yêu cầu từ 1 giây trước)
  temperature = sensors.getTempCByIndex(0);

  // 2. Yêu cầu cảm biến bắt đầu lấy mẫu mới cho 1 giây tiếp theo
  sensors.requestTemperatures(); 

  // 3. Đọc ánh sáng & Hiển thị
  luxValue = lightMeter.readLightLevel() * 2.2;
  displayLCD();
}

// =====================================================================
// --- SETUP ---
// =====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN,      OUTPUT);
  pinMode(Pump,         OUTPUT);
  pinMode(LED_status,   OUTPUT);
  pinMode(Button,       INPUT_PULLUP);
  pinMode(TdsSensorPin, INPUT);


  digitalWrite(LED_PIN,    LOW);
  digitalWrite(Pump,       HIGH);
  digitalWrite(LED_status, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures(); // Request đầu tiên

  // --- Kết nối WiFi ---
  wm.setConfigPortalTimeout(60);
  lcd.setCursor(0, 0); lcd.print("Connecting WiFi.");
  wm.setAPCallback([](WiFiManager* wm) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi:ThuyCanh");
    lcd.setCursor(0, 1); lcd.print("Pass:12345678");
  });

  bool res = wm.autoConnect("ThuyCanh", "12345678");

  if (!res) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Offline Mode!");
    lcd.setCursor(0, 1); lcd.print("No WiFi Connect.");
    Serial.println("Không có WiFi! Chuyển OFFLINE.");
    autoMode = true;
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_status, HIGH); delay(300);
      digitalWrite(LED_status, LOW);  delay(300);
    }
  } else {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi Connected!");
    Serial.println("Đã kết nối WiFi! CHẾ ĐỘ ONLINE.");
    digitalWrite(LED_status, HIGH);
    setupTime();
  }

  // --- Khởi tạo BH1750 ---
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 OK!");
  } else {
    Serial.println("Lỗi BH1750!");
  }

  // --- Cấu hình Blynk & Timer ---
  Blynk.config(BLYNK_AUTH_TOKEN);

  timer.setInterval(1000L,  docCamBienVaLCD);   // Đọc cảm biến & LCD
  timer.setInterval(1200L,  controlDevices);     // Điều khiển bơm & LED
  timer.setInterval(3000L,  processAndSendData); // Gửi Blynk
}

// =====================================================================
// --- LOOP ---
// =====================================================================
void loop() {
  // --- Xử lý nút nhấn (giữ 3 giây để reset WiFi) ---
  if (digitalRead(Button) == LOW) {
    if (!buttonHolding) {
      buttonHolding    = true;
      buttonPressStart = millis();
    } else if (millis() - buttonPressStart >= 3000) {
      buttonHolding = false;
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Configuring WiFi"); 
      Serial.println("Reset WiFi!");
      wm.resetSettings();
      delay(500);
      ESP.restart();
    }
  } else {
    buttonHolding = false; // Nhả tay -> reset bộ đếm
  }
  // --- Xử lý WiFi & Blynk ---
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_status, HIGH);

    // Kết nối lại Blynk nếu bị mất
    if (!Blynk.connected()) {
      Blynk.connect(500); // Thử nhanh, không block lâu
    }
    if (Blynk.connected()) {
      Blynk.run();
    }
  } else {
    // Mất WiFi: chuyển offline
    digitalWrite(LED_status, LOW);
    if (!autoMode) {
      autoMode = true;
      Serial.println("Mất WiFi! Chuyển OFFLINE.");
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("WiFi Lost!");
      lcd.setCursor(0, 1); lcd.print("Auto Mode ON");
      memset(prevHang0, 0, sizeof(prevHang0));
      memset(prevHang1, 0, sizeof(prevHang1));
      delay(2000);
    }
  }
  // --- Cập nhật TDS ---
  capnhatTDS();
  // --- Timer ---
  timer.run();
  delay(10);
}
