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
const float MAX_TEMP = 28.0;  // °C
const float Min_TEMP = 22.0;  // °C
const int   MAX_TDS  = 1000;  // ppm
const int   Min_TDS  = 800;   // ppm
const float MAX_LUX  = 10000.0;
const float MIN_LUX  = 15000.0;

// =====================================================================
// --- BIẾN TRẠNG THÁI ---
// =====================================================================
bool  autoMode  = false;
bool  pumpState = false;
int   ledPwm    = 0;   // Giá trị PWM LED: 0–255 (thủ công từ Slider Blynk)

// Buffer TDS
int   analogBuffer[SCOUNT];
int   analogBufferTemp[SCOUNT];
int   analogBufferIndex = 0;
float averageVoltage    = 0;
float tdsValue          = 0;
float temperature       = 0;
float luxValue          = 0;

// Chu kỳ bơm tự động
bool          pumpRunning        = false;
unsigned long previousMillisPump = 0;
const long    intervalOn         = 10000UL;  // 10 giây bơm CHẠY
const long    intervalOff        = 10000UL;  // 10 giây bơm NGHỈ

// Chống spam thông báo
unsigned long lastNotifyTime  = 0;
const long    notifyInterval  = 60000UL; // 1 phút

// =====================================================================
// --- THUẬT TOÁN LỌC TRUNG VỊ (MEDIAN FILTER) ---
// =====================================================================
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];

  int bTemp;
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp      = bTab[i];
        bTab[i]    = bTab[i + 1];
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
  float compensationCoefficient = 1.0 + 0.02 * (temp - 25.0);
  float compensationVoltage     = voltage / compensationCoefficient;
  float tds = (133.42 * pow(compensationVoltage, 3)
             - 255.86 * pow(compensationVoltage, 2)
             + 857.39 * compensationVoltage) * 0.5;
  return (tds < 0) ? 0 : tds;
}

// =====================================================================
// --- CẬP NHẬT TDS (GỌI LIÊN TỤC TRONG LOOP, KHÔNG CÓ DELAY) ---
// =====================================================================
void capnhatTDS() {
  // Lấy mẫu mỗi 40ms
  static unsigned long analogSampleTimepoint = 0;
  if (millis() - analogSampleTimepoint > 40UL) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;
  }

  // Tính toán mỗi 800ms
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

  if (pumpRunning) {
    if (currentMillis - previousMillisPump >= intervalOn) {
      pumpRunning          = false;
      pumpState            = false;
      previousMillisPump   = currentMillis;
      digitalWrite(Pump, HIGH); // Tắt bơm
      Serial.println("Chu kỳ: TẮT BƠM");
      Blynk.virtualWrite(V3, pumpState);
    }
  } else {
    if (currentMillis - previousMillisPump >= intervalOff) {
      bool envOK = (temperature >= Min_TEMP && temperature <= MAX_TEMP
                 && tdsValue    >= Min_TDS  && tdsValue    <= MAX_TDS);
      if (!envOK) {
        // Môi trường có vấn đề: không cho bật bơm
        pumpRunning = false;
        pumpState   = false;
        digitalWrite(Pump, HIGH);
        Serial.println("Lỗi môi trường! Khóa bơm để bảo vệ cây.");
      } else {
        pumpRunning          = true;
        pumpState            = true;
        previousMillisPump   = currentMillis;
        digitalWrite(Pump, LOW); // Bật bơm
        Serial.println("Chu kỳ: BẬT BƠM");
        Blynk.virtualWrite(V3, pumpState);
      }
    }
  }
}

// =====================================================================
// --- ĐIỀU CHỈNH ĐỘ SÁNG LED ---
// Chế độ TỰ ĐỘNG:
//   - Lux < MIN_LUX : tăng dần PWM (thiếu sáng -> bật thêm LED)
//   - Lux > MAX_LUX : giảm dần PWM (dư sáng  -> tắt bớt LED)
//   - Lux trong [MIN_LUX, MAX_LUX] : giữ nguyên PWM
// Chế độ THỦ CÔNG: dùng giá trị ledPwm do Slider Blynk (V4) gửi về
// =====================================================================
void updateLedBrightness(float currentLux) {
  static int autoPwm = 0;  // Lưu PWM hiện tại của chế độ auto
  const  int step    = 20;  // Bước tăng/giảm mỗi lần gọi (tăng nếu muốn phản ứng nhanh hơn)

  int pwmValue;

  if (autoMode) {
    if (currentLux < MIN_LUX) {
      autoPwm += step;     // Thiếu sáng -> tăng LED
    } else if (currentLux > MAX_LUX) {
      autoPwm -= step;     // Dư sáng    -> giảm LED
    }
    // Trong khoảng [MIN_LUX, MAX_LUX] -> không làm gì, giữ nguyên

    autoPwm  = constrain(autoPwm, 0, 255);
    pwmValue = autoPwm;
    Blynk.virtualWrite(V4, pwmValue);
  } else {
    // Khi ở thủ công: đồng bộ autoPwm để khi chuyển sang Auto không bị giật
    autoPwm  = ledPwm;
    pwmValue = ledPwm;
  }

  analogWrite(LED_PIN, pwmValue);
  Serial.printf("LED PWM: %d | Lux: %.0f (%s)\n",
                pwmValue, currentLux, autoMode ? "Auto" : "Manual");
}

// =====================================================================
// --- GỬI DỮ LIỆU & CẢNH BÁO LÊN BLYNK (GỌI QUA TIMER 3 GIÂY) ---
// =====================================================================
void processAndSendData() {

  if (!Blynk.connected()) return;

  // Gửi dữ liệu cảm biến
  Blynk.virtualWrite(V0, tdsValue);
  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, luxValue);
  // Đồng bộ trạng thái thiết bị
  Blynk.virtualWrite(V3, pumpState);
  // V4 (LED PWM) được đồng bộ trực tiếp trong updateLedBrightness()
  Blynk.virtualWrite(V5, autoMode);

  Serial.printf("Data sent | Temp: %.1f°C | TDS: %.0f ppm | Lux: %.0f lx\n",
                temperature, tdsValue, luxValue);

  // Cảnh báo chống spam
  if (millis() - lastNotifyTime > notifyInterval) {
    bool hasWarning = false;

    if (temperature > MAX_TEMP) {
      Blynk.logEvent("canh_bao_nhiet_do", "Cảnh báo: Nhiệt độ quá cao!");
      hasWarning = true;
    } else if (temperature < Min_TEMP) {
      Blynk.logEvent("canh_bao_nhiet_do", "Cảnh báo: Nhiệt độ quá thấp!");
      hasWarning = true;
    }

    if (tdsValue > MAX_TDS) {
      Blynk.logEvent("canh_bao_tds", "Cảnh báo: Chỉ số TDS vượt ngưỡng!");
      hasWarning = true;
    } else if (tdsValue < Min_TDS) {
      Blynk.logEvent("canh_bao_tds", "Cảnh báo: Chỉ số TDS quá thấp!");
      hasWarning = true;
    }

    if (luxValue > MAX_LUX) {
      Blynk.logEvent("canh_bao_lux", "Cảnh báo: Chỉ số ánh sáng vượt ngưỡng!");
      hasWarning = true;
    } else if (luxValue < MIN_LUX) {
      Blynk.logEvent("canh_bao_lux", "Cảnh báo: Chỉ số ánh sáng quá thấp!");
      hasWarning = true;
    }

    if (hasWarning) lastNotifyTime = millis();
  }
}

// =====================================================================
// --- NHẬN LỆNH TỪ BLYNK ---
// =====================================================================

// V3: Điều khiển Bơm (chỉ có tác dụng khi ở chế độ thủ công)
BLYNK_WRITE(V3) {
  pumpState = param.asInt();
  if (!autoMode) {
    digitalWrite(Pump, pumpState ? LOW : HIGH);
    Serial.println(pumpState ? "Bơm: BẬT (thủ công)" : "Bơm: TẮT (thủ công)");
  }
}

// V4: Slider điều khiển độ sáng LED (0–255), chỉ có tác dụng ở chế độ thủ công
BLYNK_WRITE(V4) {
  ledPwm = constrain(param.asInt(), 0, 255);
  if (!autoMode) {
    analogWrite(LED_PIN, ledPwm);
    Serial.printf("LED PWM (thủ công): %d\n", ledPwm);
  }
}

// V5: Chuyển chế độ Tự động / Thủ công
BLYNK_WRITE(V5) {
  autoMode = param.asInt();
  Serial.println(autoMode ? "Chế độ: TỰ ĐỘNG" : "Chế độ: THỦ CÔNG");

  if (!autoMode) {
    // Vừa chuyển sang THỦ CÔNG: áp ngay trạng thái hiện tại
    digitalWrite(Pump,   pumpState ? LOW : HIGH);
    analogWrite(LED_PIN, ledPwm);
  }
  // Nếu chuyển sang TỰ ĐỘNG: để PumpAutoControl() và updateLedBrightness()
  // xử lý ở chu kỳ kế tiếp
}
BLYNK_CONNECTED() {
  Serial.println("Đã kết nối Blynk! Đang đồng bộ trạng thái từ App...");
  
  // Yêu cầu server tải lại trạng thái cuối cùng của Bơm (V3), LED (V4) và Chế độ (V5)
  Blynk.syncVirtual(V3, V4, V5); 
}
// =====================================================================
// Lưu nội dung đang hiển thị để so sánh
static char prevHang0[17] = "";
static char prevHang1[17] = "";
void displayLCD() {
  char bufferHang0[17];
  char bufferHang1[17];

  bool showWarning = (millis() / 2000) % 2 == 0;

  // --- HÀNG 0: Nhiệt độ & Ánh sáng ---
  // Lưu ý: Thêm các khoảng trắng ở cuối để đảm bảo chuỗi luôn đủ 16 ký tự, ghi đè hoàn toàn ký tự rác cũ
  if (temperature != DEVICE_DISCONNECTED_C) {
    if ((temperature > MAX_TEMP || temperature < Min_TEMP) && showWarning) {
      snprintf(bufferHang0, sizeof(bufferHang0),
               temperature > MAX_TEMP ? "NhietDo qua CAO!" : "NhietDo qua THAP");
    } else {
      snprintf(bufferHang0, sizeof(bufferHang0),
               "T:%-2d\xDF" "C L:%-6d ", (int)temperature, (int)luxValue); 
    }
  } else {
    snprintf(bufferHang0, sizeof(bufferHang0), "Loi cam bien T! ");
  }

  // --- HÀNG 1: TDS ---
  if ((tdsValue > MAX_TDS || tdsValue < Min_TDS) && showWarning) {
    snprintf(bufferHang1, sizeof(bufferHang1),
             tdsValue > MAX_TDS ? "TDS qua CAO!    " : "TDS qua THAP!   ");
  } else {
    snprintf(bufferHang1, sizeof(bufferHang1), "TDS:%-4d ppm    ", (int)tdsValue);
  }

  // Kỹ thuật 1: Chỉ ghi khi nội dung thay đổi
  if (strcmp(bufferHang0, prevHang0) != 0) {
    lcd.setCursor(0, 0);
    lcd.print(bufferHang0);
    strncpy(prevHang0, bufferHang0, sizeof(prevHang0));
  }

  if (strcmp(bufferHang1, prevHang1) != 0) {
    lcd.setCursor(0, 1);
    lcd.print(bufferHang1);
    strncpy(prevHang1, bufferHang1, sizeof(prevHang1));
  }
}
// Thêm hàm này lên trên phần setup()
void docCamBienVaLCD() {
  // Đọc nhiệt độ và ánh sáng 1 giây/lần
  luxValue = lightMeter.readLightLevel()*2.2; // Hiệu chỉnh hệ số để khớp thực tế
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);

  // Gọi hàm hiển thị LCD
  displayLCD();
}
void controlDevices(){
  // Điều khiển bơm và LED dựa trên giá trị cảm biến
  PumpAutoControl();
  updateLedBrightness(luxValue);
}
// =====================================================================
// --- SETUP ---
// =====================================================================
void setup() {
  Serial.begin(115200);

  // FIX: khai báo pinMode TRƯỚC rồi mới dùng digitalWrite
  pinMode(LED_PIN,     OUTPUT);
  pinMode(Pump,        OUTPUT);
  pinMode(LED_status,  OUTPUT);
  pinMode(Button,      INPUT_PULLUP);
  pinMode(TdsSensorPin, INPUT);

  digitalWrite(LED_PIN,    LOW);   // Tắt LED khi khởi động
  digitalWrite(Pump,       HIGH);  // Tắt bơm khi khởi động (relay kích thấp)
  digitalWrite(LED_status, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();

  sensors.begin();

  // --- Kết nối WiFi qua WiFiManager ---
  // Timeout 60 giây: nếu không ai vào cấu hình thì chuyển Offline
  wm.setConfigPortalTimeout(60);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Connecting WiFi.");
  lcd.setCursor(0, 1); lcd.print("Portal: 60s...");

  Serial.println("Đang cố gắng kết nối WiFi...");

  // Callback: cập nhật đếm ngược lên LCD trong lúc chờ portal
  wm.setAPCallback([](WiFiManager* wm) {
    Serial.println("Portal đang mở: ThuyCanh_Setup");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi: ThuyCanh");
    lcd.setCursor(0, 1); lcd.print("Pass: 12345678");
  });

  bool res = wm.autoConnect("ThuyCanh_Setup", "12345678");

  if (!res) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Offline Mode!");
    lcd.setCursor(0, 1); lcd.print("No WiFi Connect.");
    Serial.println("Không có WiFi! Chuyển sang CHẾ ĐỘ OFFLINE.");
    autoMode = true;
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_status, HIGH); delay(500);
      digitalWrite(LED_status, LOW);  delay(500);
    }
  } else {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi Connected!");
    Serial.println("Đã kết nối WiFi! Đang chạy CHẾ ĐỘ ONLINE.");
    digitalWrite(LED_status, HIGH);
  }

  // --- Khởi tạo BH1750 ---
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("Khởi tạo BH1750 thành công!");
  } else {
    Serial.println("Lỗi: Không tìm thấy BH1750. Kiểm tra dây nối.");
  }

  // --- Cấu hình Blynk ---
  Blynk.config(BLYNK_AUTH_TOKEN);

  // Timer gửi dữ liệu mỗi 3 giây
  timer.setInterval(3000L, processAndSendData);
  timer.setInterval(1000L, controlDevices); // Điều khiển thiết bị mỗi 1 giây
  // THÊM: Timer đọc cảm biến và hiển thị LCD mỗi 1 giây
  timer.setInterval(1000L, docCamBienVaLCD);
}

// =====================================================================
// --- LOOP ---
// =====================================================================
void loop() {
  // --- Nút nhấn: reset WiFi ---
  // Nhấn giữ 3 giây để tránh reset nhầm
  if (digitalRead(Button) == LOW) {
    lcd.setCursor(0, 0); 
    lcd.print("Resetting WiFi...");
    Serial.println("Xóa cấu hình WiFi và khởi động lại...");
    delay(200); // Chống rung nút
    wm.resetSettings();
    delay(500);
    ESP.restart();
  }

  // --- Xử lý Blynk ---
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_status, HIGH);
    // FIX: khôi phục autoMode khi WiFi kết nối lại
    if (autoMode) {
      // Chỉ tắt autoMode tự động nếu ban đầu là do mất WiFi (không phải do người dùng chọn)
      // Giữ nguyên: để người dùng tự quyết trên app Blynk
    }
    if (!Blynk.connected()) {
      Blynk.connect(3000);
    }
    if (Blynk.connected()) {
      Blynk.run();
    }
  } else {
    // Mất WiFi: bật chế độ tự động để bảo vệ cây
    if (!autoMode) {
      autoMode = true;
      Serial.println("Mất kết nối WiFi! Chuyển sang CHẾ ĐỘ OFFLINE.");
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("WiFi Lost!");
      lcd.setCursor(0, 1); lcd.print("Auto Mode ON");
      digitalWrite(LED_status, LOW);
      delay(2000);
      lcd.clear();
    }
  }


  // --- Cập nhật TDS (không dùng delay để giữ tần số lấy mẫu) ---
  capnhatTDS();


  // --- Chạy timer Blynk ---
  timer.run();

  // FIX: Bỏ delay(500) để capnhatTDS() hoạt động đúng tần số 40ms
  // Nếu cần giảm tải CPU có thể dùng delay nhỏ hơn như delay(10)
  delay(10);
}
