/*
 * ===============================================
 * Arduino 멀티 디바이스 + LCD 통합 제어 (리팩토링 버전)
 * ===============================================
 *
 * [주요 개선 사항]
 * - 구조체를 사용한 센서 데이터 관리 (SensorData)
 * - 모든 매직 넘버를 상수로 정의
 * - loop() 함수를 3단계로 분리: readSensors() → processLogic() → controlActuators()
 * - 각 디바이스별 독립 함수로 분리하여 가독성 및 유지보수성 향상
 *
 * [작성자] POCHITA Team
 * [날짜] 2025-12-20
 * [버전] 3.0 - 코드 리팩토링 (구조체 + 상수화 + 함수 분리)
 */

// ========== 라이브러리 포함 ==========
#include <LED.h>
#include <RgbLed.h>
#include <FND.h>
#include "DCMotor.h"
#include "StepMotor.h"
#include "RgbLcd.h"
#include "Temperature.h"
#include "Gas.h"
#include "PIR.h"
#include <util/atomic.h>

// ========================================
// 상수 정의 (모든 매직 넘버 제거)
// ========================================

// 타이밍 상수
#define MAIN_CLOCK 100                    // 메인 루프 카운트
#define INTERVAL_SERIAL_PRINT 500         // 시리얼 출력 주기 (ms)
#define INTERVAL_LED_UPDATE 500           // LED 업데이트 주기 (ms)
#define INTERVAL_RGB_UPDATE 30            // RGB LED 업데이트 주기 (ms)
#define INTERVAL_FND_UPDATE 0             // FND 업데이트 (매 루프)
#define INTERVAL_MOTOR_UPDATE 1000        // DC 모터 업데이트 주기 (ms)
#define INTERVAL_VIBRATION_READ 100       // 진동 센서 읽기 주기 (ms)
#define INTERVAL_TEMP_GAS_READ 1000       // 온도/가스 센서 읽기 주기 (ms)
#define INTERVAL_MELODY_NOTE_GAP 50       // 멜로디 음표 사이 간격 (ms)
#define INTERVAL_MELODY_REPEAT 3000       // 멜로디 반복 대기 시간 (ms)
#define INTERVAL_LCD_DURATION 3000        // LCD 특수 화면 표시 시간 (ms)
#define INTERVAL_START_SCREEN 3000        // START 화면 표시 시간 (ms)
#define INTERVAL_BOOT_SCREEN 2000         // 부팅 화면 표시 시간 (ms)

// 센서 임계값
#define TEMP_DANGER_THRESHOLD 40.0        // 온도 위험 임계값 (℃)
#define GAS_DANGER_THRESHOLD 60           // 가스 위험 임계값 (%)
#define PIR_START_COUNT 10                // PIR 시스템 시작 카운트

// LED 설정
#define LED_COUNT 8                       // LED 개수
#define LED_MIN_INDEX 1                   // LED 최소 인덱스
#define LED_MAX_INDEX 8                   // LED 최대 인덱스

// FND 설정
#define FND_MIN_VALUE 0                   // FND 최소값
#define FND_MAX_VALUE 7777                // FND 최대값
#define FND_100_UNIT 100                  // FND 100 단위
#define FND_1000_UNIT 1000                // FND 1000 단위

// RGB LED 설정
#define RGB_STATE_COUNT 7                 // RGB 색상 개수
#define RGB_MAX_VALUE 255                 // RGB 최대값
#define RGB_MIN_VALUE 0                   // RGB 최소값

// 모터 상태
#define MOTOR_STATE_STOP 0                // 모터 정지
#define MOTOR_STATE_FORWARD 1             // 모터 정방향
#define MOTOR_STATE_REVERSE 2             // 모터 역방향

// LCD 모드
#define LCD_MODE_TEAM_NAME 0              // 팀 이름 표시
#define LCD_MODE_TEMP_GAS 1               // 온도/가스 표시
#define LCD_MODE_MELODY_INFO 2            // 곡 정보 표시
#define LCD_MODE_REALTIME 3               // 실시간 데이터 표시
#define LCD_MODE_GAS_DANGER 4             // 가스 위험 경고
#define LCD_MODE_TEMP_DANGER 5            // 온도 위험 경고

// 부저 주파수
#define BUZZER_FREQ_FND_100 1000          // FND 100 단위 비프음 (Hz)
#define BUZZER_FREQ_PIR_DETECT 1500       // PIR 감지 비프음 (Hz)
#define BUZZER_FREQ_SYSTEM_START 2000     // 시스템 시작 비프음 (Hz)
#define BUZZER_DURATION_SHORT 50          // 짧은 비프음 (ms)
#define BUZZER_DURATION_LONG 500          // 긴 비프음 (ms)

// 핀 번호 정의
#define PIN_INTERRUPT 48
#define PIN_LOOP 49
#define PIN_LED 23
#define PIN_RGB 24
#define PIN_MOTOR 25
#define PIN_SENSOR 26
#define PIN_VIBRATION 27
#define PIN_FND 28
#define PIN_MELODY 29
#define PIN_BUZZER 30

#define LED_PIN 36
#define RGB_R_PIN 4
#define RGB_G_PIN 5
#define RGB_B_PIN 6
#define FND_LATCH_PIN 38
#define FND_CLOCK_PIN 32
#define DCIN_A 7
#define DCIN_B 8
#define SM_DIR 74
#define SM_STEP 3
#define SM_MS1 41
#define SM_MS2 40
#define SM_SLEEP 38
#define BUZZER_PIN 11
#define PIEZO_SENSOR_PIN A1
#define PIR_ADC 39
#define TEMP_ADDR 72
#define GAS_ADC 54

// 음계 정의
#define B3  247
#define C4  262
#define CS4 277
#define D4  294
#define DS4 311
#define E4  330
#define F4  349
#define FS4 370
#define G4  392
#define GS4 415
#define A4  440
#define B4  494
#define REST 0

// ========================================
// 구조체 정의
// ========================================

// 센서 데이터 구조체
struct SensorData {
  float temperature;          // 온도 (℃)
  int gas;                    // 가스 농도 (%)
  int vibration;              // 진동 값 (0~1023)
  int pir_value;              // PIR 센서 값
  int pir_count;              // PIR 감지 카운트
};

// RGB LED 색상 구조체
struct RgbColor {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

// FND 상태 구조체
struct FndState {
  int value;                  // 현재 FND 값
  unsigned char direction;    // 증가(1)/감소(0)
  int prev_100_unit;          // 이전 100 단위 값
};

// LED 상태 구조체
struct LedState {
  unsigned char index;        // 현재 LED 인덱스
  volatile unsigned char direction_toggle;  // 방향 토글
  volatile unsigned char danger_toggle;     // 위험 모드 토글
};

// 타이머 카운터 구조체
struct TimerCounters {
  unsigned long last_serial_print;
  unsigned long last_led;
  unsigned long last_rgb;
  unsigned long last_motor;
  unsigned long last_vibration;
  unsigned long last_temp_gas;
  unsigned long last_melody;
};

// ========================================
// 전역 객체
// ========================================
LED myled;
RgbLed myrgbled;
FND myfnd;
DCMotor dcMotor;
StepMotor stepMotor;
RgbLcd lcd;
Temperature temper;
Gas gas;
PIR pir;

// ========================================
// 전역 변수
// ========================================

// 인터럽트 제어
volatile unsigned long count_interrupt = 0;
volatile unsigned int main_interval_count = 0;
volatile unsigned char pin_interrupt = 0;
volatile unsigned char pin_loop = 0;

// 센서 데이터
SensorData sensors = {0.0, 0, 0, 0, 0};

// 디바이스 상태
LedState led_state = {LED_MIN_INDEX, 0, 0};
FndState fnd_state = {FND_MIN_VALUE, 1, 0};
RgbColor rgb_color = {RGB_MAX_VALUE, RGB_MIN_VALUE, RGB_MIN_VALUE};
unsigned char rgb_state = 0;
unsigned char motor_state = MOTOR_STATE_STOP;

// 타이머
TimerCounters timers = {0, 0, 0, 0, 0, 0, 0};

// LCD 상태
unsigned char lcd_state = LCD_MODE_REALTIME;
unsigned char prev_lcd_state = 255;
unsigned long lcd_timer = 0;
bool flag_fnd_100 = false;
bool flag_fnd_1000 = false;

// 시스템 상태
bool system_started = false;
bool show_start_screen = false;
unsigned long start_screen_timer = 0;

// 멜로디
unsigned int melody_index = 0;
unsigned char melody_state = 0;
unsigned int melody_loop_count = 0;

// 디버그 핀 토글
unsigned char pin_led_toggle = 0;
unsigned char pin_rgb_toggle = 0;
unsigned char pin_motor_toggle = 0;
unsigned char pin_sensor_toggle = 0;
unsigned char pin_vibration_toggle = 0;
unsigned char pin_fnd_toggle = 0;
unsigned char pin_melody_toggle = 0;
unsigned char pin_buzzer_toggle = 0;

// 멜로디 데이터
int bassLine[] = {
  E4, B3, DS4, E4, FS4, REST, B3, REST,
  E4, B3, DS4, E4, FS4, REST, B3, REST,
  E4, DS4, E4, DS4, E4, B3, REST
};

int durations[] = {
  200, 200, 200, 200, 400, 100, 400, 200,
  200, 200, 200, 200, 400, 100, 400, 200,
  200, 200, 200, 200, 200, 400, 600
};

int noteCount = sizeof(bassLine) / sizeof(bassLine[0]);

// ========================================
// 인터럽트 및 타이머 함수
// ========================================

void setupTimer3(uint32_t freq, uint16_t prescaler) {
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= _BV(WGM32);
  OCR3A = 249;
  TCCR3B |= _BV(CS31) | _BV(CS30);
  TIMSK3 |= _BV(OCIE3A);
  interrupts();
}

void main_interrupt(void) {
  count_interrupt++;
  main_interval_count++;
  pin_interrupt = !pin_interrupt;
  digitalWrite(PIN_INTERRUPT, pin_interrupt);
}

ISR(TIMER3_COMPA_vect) {
  main_interrupt();
}

// ========================================
// 유틸리티 함수
// ========================================

unsigned long getCurrentCount() {
  unsigned long current;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    current = count_interrupt;
  }
  return current;
}

bool isDangerState() {
  return (sensors.temperature >= TEMP_DANGER_THRESHOLD) ||
         (sensors.gas >= GAS_DANGER_THRESHOLD);
}

// ========================================
// 센서 읽기 함수
// ========================================

void readSensors() {
  unsigned long current_count = getCurrentCount();

  // PIR 센서 (매 루프)
  int prev_pir = sensors.pir_value;
  sensors.pir_value = pir.readValue();

  // 상승 엣지 감지
  if (prev_pir == 0 && sensors.pir_value > 0 && !system_started && !show_start_screen) {
    sensors.pir_count++;
    tone(BUZZER_PIN, BUZZER_FREQ_PIR_DETECT, BUZZER_DURATION_SHORT);
    if (sensors.pir_count >= PIR_START_COUNT) {
      show_start_screen = true;
      start_screen_timer = current_count;
      tone(BUZZER_PIN, BUZZER_FREQ_SYSTEM_START, BUZZER_DURATION_LONG);
      lcd.clear();
    }
  }

  // 진동 센서 (100ms 주기)
  if (current_count - timers.last_vibration >= INTERVAL_VIBRATION_READ) {
    timers.last_vibration += INTERVAL_VIBRATION_READ;
    pin_vibration_toggle = !pin_vibration_toggle;
    digitalWrite(PIN_VIBRATION, pin_vibration_toggle);
    sensors.vibration = analogRead(PIEZO_SENSOR_PIN);
  }

  // 온도/가스 센서 (1초 주기)
  if (current_count - timers.last_temp_gas >= INTERVAL_TEMP_GAS_READ) {
    timers.last_temp_gas += INTERVAL_TEMP_GAS_READ;
    pin_sensor_toggle = !pin_sensor_toggle;
    digitalWrite(PIN_SENSOR, pin_sensor_toggle);
    sensors.temperature = temper.getTemperatureC();
    int raw_gas = gas.read();
    sensors.gas = map(raw_gas, 0, 1023, 0, 100);
  }
}

// ========================================
// 로직 처리 함수
// ========================================

void updateFndLogic() {
  if (fnd_state.direction == 1) {
    fnd_state.value++;
    if (fnd_state.value >= FND_MAX_VALUE) {
      fnd_state.value = FND_MAX_VALUE;
      fnd_state.direction = 0;
    }
  } else {
    if (fnd_state.value > 0) fnd_state.value--;
    if (fnd_state.value <= FND_MIN_VALUE) {
      fnd_state.value = FND_MIN_VALUE;
      fnd_state.direction = 1;
    }
  }
}

void updateRgbLogic() {
  if (isDangerState()) {
    rgb_color.r = RGB_MAX_VALUE;
    rgb_color.g = RGB_MIN_VALUE;
    rgb_color.b = RGB_MIN_VALUE;
    return;
  }

  switch(rgb_state) {
    case 0: rgb_color = {255, 0, 0}; rgb_state = 1; break;      // Red
    case 1: rgb_color = {0, 255, 0}; rgb_state = 2; break;      // Green
    case 2: rgb_color = {0, 0, 255}; rgb_state = 3; break;      // Blue
    case 3: rgb_color = {255, 255, 0}; rgb_state = 4; break;    // Yellow
    case 4: rgb_color = {255, 0, 255}; rgb_state = 5; break;    // Magenta
    case 5: rgb_color = {0, 255, 255}; rgb_state = 6; break;    // Cyan
    case 6: rgb_color = {255, 255, 255}; rgb_state = 0; break;  // White
  }
}

void updateLedLogic() {
  if (isDangerState()) {
    // 위험 모드: 전체 점멸
    led_state.danger_toggle = !led_state.danger_toggle;
    return;
  }

  // 정상 모드: 순차 점등
  if (led_state.direction_toggle == 0) {
    led_state.index++;
    if (led_state.index > LED_MAX_INDEX) {
      led_state.index = LED_MAX_INDEX;
      led_state.direction_toggle = 1;
    }
  } else {
    led_state.index--;
    if (led_state.index < LED_MIN_INDEX) {
      led_state.index = LED_MIN_INDEX;
      led_state.direction_toggle = 0;
    }
  }
}

void updateMotorLogic() {
  if (motor_state == MOTOR_STATE_STOP) {
    motor_state = MOTOR_STATE_FORWARD;
  } else if (motor_state == MOTOR_STATE_FORWARD) {
    motor_state = MOTOR_STATE_REVERSE;
  } else {
    motor_state = MOTOR_STATE_STOP;
  }
}

void updateLcdLogic() {
  unsigned long current_count = getCurrentCount();

  // 최우선 순위: 위험 상태
  if (sensors.temperature >= TEMP_DANGER_THRESHOLD) {
    lcd_state = LCD_MODE_TEMP_DANGER;
    return;
  } else if (sensors.gas >= GAS_DANGER_THRESHOLD) {
    lcd_state = LCD_MODE_GAS_DANGER;
    return;
  }

  // FND 1000 단위
  if (fnd_state.value % FND_1000_UNIT == 0 && fnd_state.value > 0) {
    if (!flag_fnd_1000) {
      lcd_state = LCD_MODE_TEAM_NAME;
      lcd_timer = current_count;
      flag_fnd_1000 = true;
    }
  } else {
    flag_fnd_1000 = false;
  }

  // FND 100 단위 (1000 단위가 아닐 때)
  if (fnd_state.value % FND_100_UNIT == 0 && fnd_state.value > 0 &&
      (fnd_state.value % FND_1000_UNIT != 0)) {
    if (!flag_fnd_100) {
      lcd_state = LCD_MODE_TEMP_GAS;
      lcd_timer = current_count;
      flag_fnd_100 = true;
    }
  } else {
    flag_fnd_100 = false;
  }

  // 타이머 기반 자동 복귀
  if (lcd_state == LCD_MODE_TEMP_DANGER) {
    if (sensors.temperature < TEMP_DANGER_THRESHOLD) lcd_state = LCD_MODE_REALTIME;
  } else if (lcd_state == LCD_MODE_GAS_DANGER) {
    if (sensors.gas < GAS_DANGER_THRESHOLD) lcd_state = LCD_MODE_REALTIME;
  } else if (lcd_state != LCD_MODE_REALTIME &&
             (current_count - lcd_timer >= INTERVAL_LCD_DURATION)) {
    lcd_state = LCD_MODE_REALTIME;
  }
}

void updateMelodyLogic() {
  unsigned long current_count = getCurrentCount();

  switch(melody_state) {
    case 0:  // 대기 상태
      if (melody_index < noteCount) {
        int freq = bassLine[melody_index];
        if (freq == REST) {
          noTone(SM_STEP);
        } else {
          pin_melody_toggle = !pin_melody_toggle;
          digitalWrite(PIN_MELODY, pin_melody_toggle);
          tone(SM_STEP, freq);
        }
        timers.last_melody = current_count;
        melody_state = 1;
      } else {
        // 멜로디 종료
        if (current_count - timers.last_melody >= INTERVAL_MELODY_REPEAT) {
          melody_loop_count++;
          lcd_state = LCD_MODE_MELODY_INFO;
          lcd_timer = current_count;
          melody_index = 0;
        }
      }
      break;

    case 1:  // 재생 중
      if (current_count - timers.last_melody >= durations[melody_index]) {
        noTone(SM_STEP);
        melody_index++;
        timers.last_melody = current_count;
        melody_state = (melody_index < noteCount) ? 2 : 0;
      }
      break;

    case 2:  // 간격 대기
      if (current_count - timers.last_melody >= INTERVAL_MELODY_NOTE_GAP) {
        melody_state = 0;
      }
      break;
  }
}

void processLogic() {
  unsigned long current_count = getCurrentCount();

  // FND 로직 (매 루프)
  updateFndLogic();

  // LED 로직 (500ms)
  if (current_count - timers.last_led >= INTERVAL_LED_UPDATE) {
    timers.last_led += INTERVAL_LED_UPDATE;
    pin_led_toggle = !pin_led_toggle;
    digitalWrite(PIN_LED, pin_led_toggle);
    updateLedLogic();
  }

  // RGB LED 로직 (30ms)
  if (current_count - timers.last_rgb >= INTERVAL_RGB_UPDATE) {
    timers.last_rgb += INTERVAL_RGB_UPDATE;
    pin_rgb_toggle = !pin_rgb_toggle;
    digitalWrite(PIN_RGB, pin_rgb_toggle);
    updateRgbLogic();
  }

  // DC 모터 로직 (1초)
  if (current_count - timers.last_motor >= INTERVAL_MOTOR_UPDATE) {
    timers.last_motor += INTERVAL_MOTOR_UPDATE;
    pin_motor_toggle = !pin_motor_toggle;
    digitalWrite(PIN_MOTOR, pin_motor_toggle);
    updateMotorLogic();
  }

  // LCD 로직
  updateLcdLogic();

  // 멜로디 로직
  updateMelodyLogic();
}

// ========================================
// 액추에이터 제어 함수
// ========================================

void controlLed() {
  if (isDangerState()) {
    if (led_state.danger_toggle == 0) {
      for (int i = LED_MIN_INDEX; i <= LED_MAX_INDEX; i++) myled.On(i);
    } else {
      for (int i = LED_MIN_INDEX; i <= LED_MAX_INDEX; i++) myled.Off(i);
    }
  } else {
    if (led_state.direction_toggle == 0) {
      myled.On(led_state.index);
    } else {
      myled.Off(led_state.index);
    }
  }
}

void controlRgb() {
  myrgbled.OnRgb(rgb_color.r, rgb_color.g, rgb_color.b);
}

void controlFnd() {
  pin_fnd_toggle = !pin_fnd_toggle;
  digitalWrite(PIN_FND, pin_fnd_toggle);
  myfnd.setAllNum(fnd_state.value);

  // 부저 제어
  int current_100 = fnd_state.value / FND_100_UNIT;
  if (current_100 != fnd_state.prev_100_unit) {
    pin_buzzer_toggle = !pin_buzzer_toggle;
    digitalWrite(PIN_BUZZER, pin_buzzer_toggle);
    tone(BUZZER_PIN, BUZZER_FREQ_FND_100, BUZZER_DURATION_SHORT);
    fnd_state.prev_100_unit = current_100;
  }
}

void controlMotor() {
  if (motor_state == MOTOR_STATE_FORWARD) {
    dcMotor.begin(DCIN_A, DCIN_B);
    dcMotor.Start();
  } else if (motor_state == MOTOR_STATE_REVERSE) {
    dcMotor.Stop();
    dcMotor.begin(DCIN_B, DCIN_A);
    dcMotor.Start();
  } else {
    dcMotor.Stop();
  }
}

void controlLcd() {
  if (lcd_state != prev_lcd_state) {
    prev_lcd_state = lcd_state;
    lcd.clear();

    switch(lcd_state) {
      case LCD_MODE_TEAM_NAME:
        lcd.onBacklightBlue(0); lcd.onBacklightRed(1); lcd.onBacklightGreen(1);
        lcd.setCursor(0, 0); lcd.print("   POCHITA   ");
        lcd.setCursor(0, 1); lcd.print("   TEAM!!!   ");
        break;

      case LCD_MODE_TEMP_GAS:
        lcd.onBacklightRed(0); lcd.onBacklightGreen(1); lcd.onBacklightBlue(0);
        lcd.setCursor(0, 0); lcd.print("Temp: ");
        lcd.print(sensors.temperature, 1); lcd.print(" C");
        lcd.setCursor(0, 1); lcd.print("Gas : ");
        lcd.print(sensors.gas); lcd.print(" %");
        break;

      case LCD_MODE_MELODY_INFO:
        lcd.onBacklightRed(0); lcd.onBacklightGreen(0); lcd.onBacklightBlue(1);
        lcd.setCursor(0, 0); lcd.print("Music:K.K.House");
        lcd.setCursor(0, 1); lcd.print("Play Count: ");
        lcd.print(melody_loop_count);
        break;

      case LCD_MODE_REALTIME:
        lcd.onBacklightRed(1); lcd.onBacklightGreen(1); lcd.onBacklightBlue(1);
        break;

      case LCD_MODE_GAS_DANGER:
        lcd.onBacklightRed(1); lcd.onBacklightGreen(0); lcd.onBacklightBlue(0);
        lcd.setCursor(0, 0); lcd.print("!! DANGER !!    ");
        lcd.setCursor(0, 1); lcd.print("GAS: ");
        lcd.print(sensors.gas); lcd.print("%      ");
        break;

      case LCD_MODE_TEMP_DANGER:
        lcd.onBacklightRed(1); lcd.onBacklightGreen(0); lcd.onBacklightBlue(0);
        lcd.setCursor(0, 0); lcd.print("!! DANGER !!    ");
        lcd.setCursor(0, 1); lcd.print("TEMP: ");
        lcd.print(sensors.temperature, 1); lcd.print("C    ");
        break;
    }
  }

  // 실시간 모드 업데이트
  if (lcd_state == LCD_MODE_REALTIME) {
    char motor_text[5];
    if (motor_state == MOTOR_STATE_STOP) strcpy(motor_text, "STOP");
    else if (motor_state == MOTOR_STATE_FORWARD) strcpy(motor_text, "FWD ");
    else strcpy(motor_text, "REV ");

    lcd.setCursor(0, 0); lcd.print("FND: ");
    lcd.print(fnd_state.value); lcd.print("        ");
    lcd.setCursor(0, 1); lcd.print("M:"); lcd.print(motor_text);
    lcd.print(" V:"); lcd.print(sensors.vibration); lcd.print("     ");
  }
}

void controlActuators() {
  unsigned long current_count = getCurrentCount();

  // LED 제어 (500ms마다)
  if (current_count - timers.last_led >= INTERVAL_LED_UPDATE) {
    controlLed();
  }

  // RGB LED 제어 (30ms마다)
  if (current_count - timers.last_rgb >= INTERVAL_RGB_UPDATE) {
    controlRgb();
  }

  // FND 제어 (매 루프)
  controlFnd();

  // DC 모터 제어 (1초마다)
  if (current_count - timers.last_motor >= INTERVAL_MOTOR_UPDATE) {
    controlMotor();
  }

  // LCD 제어
  controlLcd();
}

// ========================================
// 시리얼 출력
// ========================================

void serialPrint() {
  unsigned long current_count = getCurrentCount();

  if (current_count - timers.last_serial_print >= INTERVAL_SERIAL_PRINT) {
    timers.last_serial_print += INTERVAL_SERIAL_PRINT;
    digitalWrite(PIN_LOOP, pin_loop);

    if (!system_started && !show_start_screen) {
      Serial.print("PIR Motion Count: ");
      Serial.print(sensors.pir_count);
      Serial.println("/10");
      return;
    }

    Serial.print("Temp: "); Serial.print(sensors.temperature, 1);
    Serial.print("C | Gas: "); Serial.print(sensors.gas);
    Serial.print("% | Status: ");

    if (sensors.temperature >= TEMP_DANGER_THRESHOLD) {
      Serial.println("TEMP DANGER!");
    } else if (sensors.gas >= GAS_DANGER_THRESHOLD) {
      Serial.println("GAS DANGER!");
    } else {
      Serial.println("NORMAL");
    }
  }
}

// ========================================
// 시스템 시작 화면 처리
// ========================================

bool handleStartupScreens() {
  unsigned long current_count = getCurrentCount();

  // START!! 화면 표시
  if (show_start_screen) {
    lcd.onBacklightRed(0); lcd.onBacklightGreen(1); lcd.onBacklightBlue(0);
    lcd.setCursor(0, 0); lcd.print("                ");
    lcd.setCursor(0, 1); lcd.print("    START!!     ");

    if (current_count - start_screen_timer >= INTERVAL_START_SCREEN) {
      show_start_screen = false;
      system_started = true;
      lcd.clear();

      // 타이머 초기화
      timers.last_led = current_count;
      timers.last_rgb = current_count;
      timers.last_motor = current_count;
      timers.last_vibration = current_count;
      timers.last_temp_gas = current_count;
      timers.last_serial_print = current_count;
      timers.last_melody = current_count;
    }
    return true;
  }

  // 대기 화면
  if (!system_started) {
    lcd.setCursor(0, 0); lcd.print("Waiting Motion..");
    lcd.setCursor(0, 1); lcd.print("Count: ");
    lcd.print(sensors.pir_count); lcd.print("/10    ");
    return true;
  }

  return false;
}

// ========================================
// Setup 함수
// ========================================

void setup(void) {
  Serial.begin(115200);

  // 디버그 핀 설정
  pinMode(PIN_INTERRUPT, OUTPUT);
  pinMode(PIN_LOOP, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RGB, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_SENSOR, OUTPUT);
  pinMode(PIN_VIBRATION, OUTPUT);
  pinMode(PIN_FND, OUTPUT);
  pinMode(PIN_MELODY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // 타이머 설정
  setupTimer3(1000, 64);

  // 디바이스 초기화
  myled.begin(LED_PIN);
  myrgbled.begin(RGB_R_PIN, RGB_G_PIN, RGB_B_PIN);
  myfnd.begin(FND_LATCH_PIN, FND_CLOCK_PIN);
  dcMotor.begin(DCIN_A, DCIN_B);
  stepMotor.begin(SM_DIR, SM_STEP, SM_MS1, SM_MS2, SM_SLEEP);
  stepMotor.setStep(FULL_STEP);
  stepMotor.setDirection(0);
  stepMotor.on(1);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  temper.begin(TEMP_ADDR);
  gas.begin(GAS_ADC);
  pinMode(PIEZO_SENSOR_PIN, INPUT);
  pir.begin(PIR_ADC);

  lcd.begin(62, 63, 64, 65, 66, 67, 45, 44, 43, 42, 16, 2);
  lcd.clear();

  // 부팅 화면
  lcd.onBacklightBlue(0); lcd.onBacklightRed(1); lcd.onBacklightGreen(1);
  lcd.setCursor(0, 0); lcd.print("   POCHITA   ");
  lcd.setCursor(0, 1); lcd.print("  LOADING... ");
  delay(INTERVAL_BOOT_SCREEN);
  lcd.clear();

  lcd_state = LCD_MODE_REALTIME;
}

// ========================================
// Loop 함수 (3단계 구조)
// ========================================

void loop() {
  // 시스템 시작 화면 처리
  if (handleStartupScreens()) {
    // 타이밍 제어
    do {} while (main_interval_count < MAIN_CLOCK);
    main_interval_count = 0;
    pin_loop = !pin_loop;
    return;
  }

  // 시리얼 출력
  serialPrint();

  // === 3단계 구조 ===
  readSensors();       // 1단계: 센서 읽기
  processLogic();      // 2단계: 로직 처리
  controlActuators();  // 3단계: 액추에이터 제어

  // 메인 루프 타이밍 제어
  do {} while (main_interval_count < MAIN_CLOCK);
  main_interval_count = 0;
  pin_loop = !pin_loop;
}
