/*
 * ===============================================
 * Arduino 멀티 디바이스 + LCD 통합 제어 (최종 완성본)
 * ===============================================
 *
 * [주요 기능]
 * - PIR 모션 센서 기반 시스템 시작 (10회 감지 필요)
 * - LED 8개 순차 점멸 (500ms 간격, Ping-Pong 패턴)
 *   * 위험 상태 시: 모든 LED 동시 점멸 (빨간색)
 * - RGB LED 7색 순환 (30ms 간격)
 *   * 위험 상태 시: 빨간색 고정
 * - FND 0~7777 증가/감소 (매 루프마다)
 * - DC 모터 정지/정회전/역회전 순환 (1초 간격)
 * - 스테퍼 모터로 "K.K. House" 멜로디 재생
 * - LCD 디스플레이 6가지 모드 자동 전환
 * - 온도/가스/진동 센서 실시간 모니터링
 * - 부저 100 단위 알림음 + PIR 감지 알림
 * - 위험 감지 시스템 (온도 40°C 이상 또는 가스 80% 이상)
 *
 * [타이머]
 * - 인터럽트 주기: 1ms (Timer3 CTC 모드)
 * - 메인 루프 주기: 100ms
 * - 센서 읽기 주기: 온도/가스 1초, 진동 100ms, PIR 실시간
 *
 * [시작 순서]
 * 1. 부팅 화면: "POCHITA LOADING..." (주황색, 2초)
 * 2. PIR 대기: "Waiting Motion.. Count: X/10"
 * 3. 시작 화면: "START!!" (초록색, 3초)
 * 4. 정상 동작 시작
 *
 * [LCD 모드 및 우선순위]
 * - Mode 5 (빨간색): 온도 위험 (≥40°C) - 최우선 순위 1
 * - Mode 4 (빨간색): 가스 위험 (≥80%) - 최우선 순위 2
 * - Mode 0 (주황색): FND 1000 단위 시 팀 이름 표시 (3초)
 * - Mode 1 (초록색): FND 100 단위 시 온도/가스 표시 (3초)
 * - Mode 2 (파란색): 멜로디 완료 시 곡 정보 표시 (3초)
 * - Mode 3 (흰색): 실시간 FND/모터/진동 데이터 표시 (기본)
 *
 * [위험 감지 시스템]
 * - 온도 40°C 이상 또는 가스 80% 이상 감지 시:
 *   1. LCD: 빨간색 백라이트 + "!! DANGER !!" 메시지
 *   2. LED 8개: 모두 동시 점멸 (500ms 간격)
 *   3. RGB LED: 빨간색 고정
 *   4. 센서 값이 정상 범위로 복귀할 때까지 지속
 *
 * [시리얼 모니터]
 * - 모든 센서 데이터 실시간 출력 (100ms 간격)
 * - Time, FND, Motor, Vib, Temp, Gas, PIR, System, LCD 상태
 *
 * [하드웨어]
 * - Arduino Mega 2560 / Mega ADK
 * - LED 8개 (핀 36)
 * - RGB LED (핀 4, 5, 6)
 * - FND 7-Segment (핀 38, 32)
 * - DC 모터 (핀 7, 8)
 * - 스테퍼 모터 (핀 74, 3, 41, 40, 38)
 * - RGB LCD 16x2 (I2C 방식, 핀 62-67, 42-45)
 * - LM75 온도 센서 (I2C 주소 0x48)
 * - 가스 센서 (아날로그 핀 A0)
 * - 피에조 진동 센서 (아날로그 핀 A1)
 * - PIR 모션 센서 (핀 39)
 * - 부저 (핀 11)
 *
 * [참고]
 * - Timer3를 직접 설정하여 1ms 정밀 타이밍 구현
 * - Timer2는 비어있어 tone() 함수 사용 가능
 * - PIR 센서는 상승 엣지 감지로 중복 카운트 방지
 * - 위험 상태는 센서 값 기반 자동 복귀 (타이머 무시)
 *
 * [작성자] POCHITA Team
 * [날짜] 2025-12-15
 * [버전] 2.0 - PIR 시작 시스템 + 위험 감지 기능 추가
 */

// ========== 라이브러리 포함 ==========
#include <LED.h>           // LED 제어
#include <RgbLed.h>        // RGB LED 제어
#include <FND.h>           // 7-Segment 디스플레이
#include "DCMotor.h"       // DC 모터 제어
#include "StepMotor.h"     // 스테퍼 모터 제어
#include "RgbLcd.h"        // RGB LCD 제어
#include "Temperature.h"   // LM75 온도 센서
#include "Gas.h"           // 가스 센서
#include "PIR.h"           // PIR 모션 센서

// ========== 객체 생성 ==========
LED myled;                 // LED 제어 객체
RgbLed myrgbled;           // RGB LED 제어 객체
FND myfnd;                 // FND 제어 객체
DCMotor dcMotor;           // DC 모터 제어 객체
StepMotor stepMotor;       // 스테퍼 모터 제어 객체
RgbLcd lcd;                // RGB LCD 제어 객체
Temperature temper;        // 온도 센서 제어 객체
Gas gas;                   // 가스 센서 제어 객체
PIR pir;                   // PIR 모션 센서 제어 객체

// ========== 타이머 설정 ==========
#define MAIN_CLOCK 100     // 메인 루프 주기 (100ms)

// ========== 핀맵 정의 ==========
// 디버그 핀
#define PIN_INTERRUPT 48   // 인터럽트 토글 핀
#define PIN_LOOP 49        // 메인 루프 토글 핀

// LED 핀
#define LED_PIN 36         // LED 제어 핀

// RGB LED 핀
#define RGB_R_PIN 4        // RGB Red 핀
#define RGB_G_PIN 5        // RGB Green 핀
#define RGB_B_PIN 6        // RGB Blue 핀

// FND 핀
#define FND_LATCH_PIN 38   // FND Latch 핀
#define FND_CLOCK_PIN 32   // FND Clock 핀

// DC 모터 핀
#define DCIN_A 7           // DC 모터 입력 A
#define DCIN_B 8           // DC 모터 입력 B

// 스테퍼 모터 핀
#define SM_DIR  74         // 스테퍼 모터 방향 핀
#define SM_STEP  3         // 스테퍼 모터 스텝 핀 (멜로디 출력용)
#define SM_MS1  41         // 마이크로스텝 설정 1
#define SM_MS2  40         // 마이크로스텝 설정 2
#define SM_SLEEP  38       // 슬립 모드 제어 핀

// 부저 및 센서 핀
#define BUZZER_PIN 11      // 부저 핀
#define PIEZO_SENSOR_PIN A1   // 피에조 진동 센서 핀
#define PIR_ADC 39         // PIR 모션 센서 핀
#define TEMP_ADDR 72       // LM75 온도 센서 I2C 주소 (0x48 = 72)
#define GAS_ADC 54         // 가스 센서 아날로그 핀 (A0)

// ========== 음계 정의 (Hz) ==========
#define B3  247            // 시 (3옥타브)
#define C4  262            // 도 (4옥타브)
#define CS4 277            // 도#
#define D4  294            // 레
#define DS4 311            // 레#
#define E4  330            // 미
#define F4  349            // 파
#define FS4 370            // 파#
#define G4  392            // 솔
#define GS4 415            // 솔#
#define A4  440            // 라
#define B4  494            // 시 (4옥타브)
#define REST 0             // 쉼표 (무음)

// ========== 타이머 인터럽트 제어 변수 ==========
volatile unsigned long count_interrupt = 0;      // 인터럽트 카운터 (1ms 단위)
volatile unsigned int main_interval_count = 0;   // 메인 루프 간격 카운터
volatile unsigned char pin_interrupt = 0;        // 인터럽트 토글 상태
volatile unsigned char pin_loop = 0;             // 메인 루프 토글 상태
volatile unsigned char toggle = 0;               // LED 방향 전환용 토글

// ========== LED 제어 변수 ==========
unsigned char led_index = 1;                     // 현재 켜진 LED 인덱스 (1~8)

// ========== FND 제어 변수 ==========
int fnd = 0;                                     // FND 표시 값 (0~7777)
unsigned char fnd_direction = 1;                 // 증가(1)/감소(0) 방향

// ========== RGB LED 제어 변수 ==========
unsigned char rgb_r = 255;                       // Red 값 (0~255)
unsigned char rgb_g = 0;                         // Green 값 (0~255)
unsigned char rgb_b = 0;                         // Blue 값 (0~255)
unsigned char rgb_state = 0;                     // 현재 색상 상태 (0~6)

// ========== DC 모터 제어 변수 ==========
unsigned long motor_timer = 0;                   // 모터 타이머 (미사용)
unsigned char motor_state = 0;                   // 모터 상태: 0=정지, 1=정방향, 2=역방향

// ========== 타이밍 제어용 카운트 저장 변수 ==========
unsigned long last_led_count = 0;                // LED 마지막 업데이트 시간
unsigned long last_rgb_count = 0;                // RGB LED 마지막 업데이트 시간
unsigned long last_motor_count = 0;              // DC 모터 마지막 업데이트 시간

// ========== 멜로디 데이터 ==========
// "K.K. House" (동물의 숲 OST)
int bassLine[] = {
  E4, B3, DS4, E4, FS4, REST, B3, REST,
  E4, B3, DS4, E4, FS4, REST, B3, REST,
  E4, DS4, E4, DS4, E4, B3, REST
};

// 각 음표의 지속 시간 (ms)
int durations[] = {
  200, 200, 200, 200, 400, 100, 400, 200,
  200, 200, 200, 200, 400, 100, 400, 200,
  200, 200, 200, 200, 200, 400, 600
};

// ========== 멜로디 제어 변수 ==========
unsigned int melody_index = 0;                   // 현재 재생 중인 음표 인덱스
unsigned long last_melody_count = 0;             // 멜로디 마지막 업데이트 시간
unsigned char melody_state = 0;                  // 멜로디 상태: 0=대기, 1=재생중, 2=50ms간격
int noteCount = sizeof(bassLine) / sizeof(bassLine[0]);  // 전체 음표 개수
unsigned int melody_loop_count = 0;              // 멜로디 재생 횟수

// ========== LCD 디스플레이 제어 변수 ==========
unsigned char lcd_state = 3;                     // LCD 상태: 0=팀이름, 1=온도/가스, 2=곡정보, 3=실시간
unsigned char prev_lcd_state = 255;              // 이전 LCD 상태 (상태 변경 감지용)
unsigned long lcd_timer = 0;                     // LCD 화면 전환 타이머 시작 시간
const int LCD_DURATION = 3000;                   // LCD 특수 화면 표시 시간 (3초)
bool flag_fnd_100 = false;                       // FND 100 단위 이벤트 플래그
bool flag_fnd_1000 = false;                      // FND 1000 단위 이벤트 플래그
int lcd_scroll_pos = 0;                          // LCD 스크롤 위치
unsigned long last_lcd_scroll = 0;               // LCD 스크롤 마지막 업데이트 시간
unsigned long last_lcd_temp_gas_update = 0;      // 온도/가스 표시 마지막 업데이트 시간
bool show_temp_gas = false;                      // 온도/가스 표시 플래그

// ========== 센서 제어 변수 ==========
int prev_fnd_100 = 0;                            // 이전 FND 100 단위 값 (부저 제어용)
unsigned long last_temp_count = 0;               // 온도/가스 센서 마지막 읽기 시간
float current_temperature = 0.0;                 // 현재 온도 값 (℃)
int current_gas = 0;                             // 현재 가스 농도 (0~100%)
int vibration_value = 0;                         // 진동 센서 값
unsigned long last_vibration_count = 0;          // 진동 센서 마지막 읽기 시간

// ========== PIR 센서 제어 변수 ==========
int pir_count = 0;                               // PIR 감지 카운트
bool system_started = false;                     // 시스템 시작 플래그 (10번 이상 감지 시 true)
unsigned long last_pir_count = 0;                // PIR 센서 마지막 읽기 시간
int pir_value = 0;                               // PIR 센서 현재 값
unsigned long start_screen_timer = 0;            // START!! 화면 표시 시작 시간
bool show_start_screen = false;                  // START!! 화면 표시 플래그

// ========================================
// Timer3 설정 함수
// ========================================
// Timer3를 CTC 모드로 설정하여 지정된 주파수로 인터럽트 발생
// freq: 목표 주파수 (Hz)
// prescaler: 프리스케일러 값 (1, 8, 64, 256, 1024)
void setupTimer3(uint32_t freq, uint16_t prescaler) {
  noInterrupts();

  TCCR3A = 0;                // 비교 출력 모드 비활성
  TCCR3B = 0;                // 초기화

  // CTC 모드: WGM32 = 1 (WGM33:0 = 0100)
  TCCR3B |= _BV(WGM32);

  // OCR3A 계산: F_CPU = 16 MHz, prescaler = 64, freq = 1000 Hz
  // OCR3A = (F_CPU / (prescaler * freq)) - 1
  // OCR3A = (16000000 / (64 * 1000)) - 1 = 249
  OCR3A = 249;

  // prescaler = 64 설정 (CS31:CS30 = 1,1)
  TCCR3B |= _BV(CS31) | _BV(CS30);

  // OCR3A 비교 매치 인터럽트 허용
  TIMSK3 |= _BV(OCIE3A);

  interrupts();
}

// ========================================
// 타이머 인터럽트 서비스 루틴 (ISR)
// ========================================
// 1ms마다 호출되는 인터럽트 함수
// - count_interrupt: 전체 경과 시간 추적
// - main_interval_count: 메인 루프 타이밍 제어
void main_interrupt(void)
{
  count_interrupt++;                      // 전체 카운터 증가
  main_interval_count++;                  // 메인 루프 카운터 증가
  pin_interrupt = !pin_interrupt;         // 디버그 핀 토글

  digitalWrite(PIN_INTERRUPT, pin_interrupt);  // 오실로스코프 확인용
}

// Timer3 비교 매치 인터럽트 벡터
ISR(TIMER3_COMPA_vect) {
  main_interrupt();                       // 메인 인터럽트 함수 호출
}

// ========================================
// 초기화 함수
// ========================================
void setup(void)
{
  // 시리얼 통신 초기화
  Serial.begin(19200);

  // 디버그 핀 설정
  pinMode(PIN_INTERRUPT, OUTPUT);
  pinMode(PIN_LOOP, OUTPUT);

  // Timer3 설정 (1ms 주기, 1000Hz)
  setupTimer3(1000, 64);  // 1000Hz, prescaler=64

  // 각 디바이스 초기화
  myled.begin(LED_PIN);                   // LED 초기화
  myrgbled.begin(RGB_R_PIN, RGB_G_PIN, RGB_B_PIN);  // RGB LED 초기화
  myfnd.begin(FND_LATCH_PIN, FND_CLOCK_PIN);        // FND 초기화
  dcMotor.begin(DCIN_A, DCIN_B);          // DC 모터 초기화

  // 스테퍼 모터 초기화
  stepMotor.begin(SM_DIR, SM_STEP, SM_MS1, SM_MS2, SM_SLEEP);
  stepMotor.setStep(FULL_STEP);           // 풀스텝 모드 설정
  stepMotor.setDirection(0);              // 방향 설정 (0: 정방향)
  stepMotor.on(1);                        // 스테퍼 모터 활성화

  // 부저 초기화
  pinMode(BUZZER_PIN, OUTPUT);            // 부저 핀을 출력으로 설정
  digitalWrite(BUZZER_PIN, LOW);          // 부저 꺼짐 상태로 초기화

  // 센서 초기화
  temper.begin(TEMP_ADDR);                // LM75 온도 센서 초기화 (I2C 주소: 0x48)
  gas.begin(GAS_ADC);                     // 가스 센서 초기화
  pinMode(PIEZO_SENSOR_PIN, INPUT);       // 피에조 진동 센서 핀 입력 모드
  pir.begin(PIR_ADC);                     // PIR 모션 센서 초기화

  // LCD 초기화
  lcd.begin(62, 63, 64, 65, 66, 67, 45, 44, 43, 42, 16, 2);
  lcd.clear();

  // 부팅 시 "POCHITA" 전체 화면 표시 (주황색 백라이트)
  lcd.onBacklightBlue(0);    // Blue OFF
  lcd.onBacklightRed(1);     // Red ON
  lcd.onBacklightGreen(1);   // Green ON = 주황색

  lcd.setCursor(0, 0);
  lcd.print("   POCHITA   ");
  lcd.setCursor(0, 1);
  lcd.print("  LOADING... ");

  delay(2000);  // 2초간 표시
  lcd.clear();

  // 초기 LCD 모드 설정
  lcd_state = 3;  // 기본 화면(실시간 데이터)으로 시작
}

// ========================================
// 디버그 출력 함수
// ========================================
void test_print(void)
{
  digitalWrite(PIN_LOOP, pin_loop);       // 메인 루프 토글 핀 출력

  // 시리얼 모니터에 모든 센서 정보 출력
  Serial.print("\n=== SENSOR DATA ===");

  Serial.print(" | Time: ");
  Serial.print(count_interrupt);

  Serial.print(" | FND: ");
  Serial.print(fnd);

  // DC 모터 상태
  Serial.print(" | Motor: ");
  if (motor_state == 0) {
    Serial.print("STOP");
  } else if (motor_state == 1) {
    Serial.print("FWD");
  } else {
    Serial.print("REV");
  }

  // 진동 센서
  Serial.print(" | Vib: ");
  Serial.print(vibration_value);

  // 온도 센서
  Serial.print(" | Temp: ");
  Serial.print(current_temperature, 1);
  Serial.print("C");

  // 가스 센서
  Serial.print(" | Gas: ");
  Serial.print(current_gas);
  Serial.print("%");

  // PIR 센서
  Serial.print(" | PIR: ");
  if (pir_value > 0) {
    Serial.print("ON");
  } else {
    Serial.print("OFF");
  }
  Serial.print(" (Count: ");
  Serial.print(pir_count);
  Serial.print("/10)");

  // 시스템 상태
  Serial.print(" | System: ");
  if (system_started) {
    Serial.print("RUNNING");
  } else {
    Serial.print("WAITING");
  }

  // LCD 상태
  Serial.print(" | LCD: ");
  switch(lcd_state) {
    case 0: Serial.print("TEAM"); break;
    case 1: Serial.print("TEMP/GAS"); break;
    case 2: Serial.print("MELODY"); break;
    case 3: Serial.print("REALTIME"); break;
    case 4: Serial.print("GAS DANGER!"); break;
    case 5: Serial.print("TEMP DANGER!"); break;
    default: Serial.print("UNKNOWN"); break;
  }
}

// ========================================
// 메인 루프 함수
// ========================================
void loop()
{
  // ========== Output: 디버그 정보 출력 ==========
  test_print();

  // ========== Input: 입력 처리 (현재 미사용) ==========

  // ========== Calculation: 각 디바이스 제어 로직 ==========

  // ---------- PIR 센서 제어 (실시간) ----------
  // PIR 센서로 모션 감지하여 10번 이상 감지 시 시스템 시작
  int prev_pir_value = pir_value;
  pir_value = pir.readValue();

  // 이전 값이 0이고 현재 값이 1이면 감지됨 (상승 엣지)
  if (prev_pir_value == 0 && pir_value > 0 && !system_started && !show_start_screen) {
    pir_count++;
    tone(BUZZER_PIN, 1500, 50);  // 감지 시마다 짧은 비프음
    if (pir_count >= 10) {
      show_start_screen = true;          // START!! 화면 표시 플래그
      start_screen_timer = count_interrupt;  // 타이머 시작
      tone(BUZZER_PIN, 2000, 500);       // 시스템 시작 알림음
      lcd.clear();
    }
  }

  // START!! 화면 표시 (3초간)
  if (show_start_screen) {
    // 초록색 백라이트
    lcd.onBacklightRed(0);     // Red OFF
    lcd.onBacklightGreen(1);   // Green ON
    lcd.onBacklightBlue(0);    // Blue OFF

    lcd.setCursor(0, 0);
    lcd.print("                ");  // 첫 줄 비우기
    lcd.setCursor(0, 1);
    lcd.print("    START!!     ");   // 두 번째 줄에 START!!

    // 3초 경과 시 시스템 시작
    if (count_interrupt - start_screen_timer >= 3000) {
      show_start_screen = false;
      system_started = true;
      lcd.clear();
    }

    // 메인 루프 타이밍 제어
    do {
    } while (main_interval_count < MAIN_CLOCK);
    main_interval_count = 0;
    pin_loop = !pin_loop;
    return;  // START!! 화면 표시 중에는 나머지 로직 실행 안 함
  }

  // 시스템이 시작되지 않았으면 대기 화면만 표시
  if (!system_started) {
    lcd.setCursor(0, 0);
    lcd.print("Waiting Motion..");
    lcd.setCursor(0, 1);
    lcd.print("Count: ");
    lcd.print(pir_count);
    lcd.print("/10    ");

    // 메인 루프 타이밍 제어
    do {
    } while (main_interval_count < MAIN_CLOCK);
    main_interval_count = 0;
    pin_loop = !pin_loop;
    return;  // 시스템 시작 전까지 나머지 로직 실행 안 함
  }

  // ---------- LED 제어 (500ms 간격) ----------
  // 위험 상태(온도 40도 이상 또는 가스 80% 이상)일 때는 모든 LED 빨간색 점멸
  // 정상 상태일 때는 LED 1번부터 8번까지 순차적으로 켜고, 다시 8번부터 1번까지 끄기
  if (count_interrupt - last_led_count >= 500) {
    last_led_count = count_interrupt;

    // 위험 상태 체크
    bool is_danger = (current_temperature >= 40.0) || (current_gas >= 80);

    if (is_danger) {
      // 위험 상태: 모든 LED 8개 동시에 점멸 (ON/OFF 토글)
      if (toggle == 0) {
        for (int i = 1; i <= 8; i++) {
          myled.On(i);                    // 모든 LED 켜기
        }
        toggle = 1;
      } else {
        for (int i = 1; i <= 8; i++) {
          myled.Off(i);                   // 모든 LED 끄기
        }
        toggle = 0;
      }
    } else {
      // 정상 상태: 순차 점등
      // LED ON/OFF
      if (0 == toggle) {
        myled.On(led_index);              // 증가 방향: LED 켜기
      } else {
        myled.Off(led_index);             // 감소 방향: LED 끄기
      }

      // LED 인덱스 증가/감소
      if (0 == toggle) {
        led_index++;
        if (led_index > 8) {              // 8번 LED 도달 시
          led_index = 8;
          toggle = !toggle;               // 감소 모드로 전환
        }
      } else {
        led_index--;
        if (led_index < 1) {              // 1번 LED 도달 시
          led_index = 1;
          toggle = !toggle;               // 증가 모드로 전환
        }
      }
    }
  }

  // ---------- RGB LED 제어 (30ms 간격) ----------
  // 위험 상태일 때는 빨간색으로 고정
  // 정상 상태일 때는 7가지 색상을 순환: 빨강 → 초록 → 파랑 → 노랑 → 마젠타 → 시안 → 흰색
  if (count_interrupt - last_rgb_count >= 30) {
    last_rgb_count = count_interrupt;

    // 위험 상태 체크
    bool is_danger = (current_temperature >= 40.0) || (current_gas >= 80);

    if (is_danger) {
      // 위험 상태: 빨간색 고정
      rgb_r = 255;
      rgb_g = 0;
      rgb_b = 0;
    } else {
      // 정상 상태: 색상 순환
      switch(rgb_state) {
        case 0:  // 빨강 (Red)
          rgb_r = 255;
          rgb_g = 0;
          rgb_b = 0;
          rgb_state = 1;
          break;

        case 1:  // 초록 (Green)
          rgb_r = 0;
          rgb_g = 255;
          rgb_b = 0;
          rgb_state = 2;
          break;

        case 2:  // 파랑 (Blue)
          rgb_r = 0;
          rgb_g = 0;
          rgb_b = 255;
          rgb_state = 3;
          break;

        case 3:  // 노랑 (Yellow)
          rgb_r = 255;
          rgb_g = 255;
          rgb_b = 0;
          rgb_state = 4;
          break;

        case 4:  // 마젠타 (Magenta)
          rgb_r = 255;
          rgb_g = 0;
          rgb_b = 255;
          rgb_state = 5;
          break;

        case 5:  // 시안 (Cyan)
          rgb_r = 0;
          rgb_g = 255;
          rgb_b = 255;
          rgb_state = 6;
          break;

        case 6:  // 흰색 (White)
          rgb_r = 255;
          rgb_g = 255;
          rgb_b = 255;
          rgb_state = 0;                    // 다음은 빨강으로 순환
          break;
      }
    }

    myrgbled.OnRgb(rgb_r, rgb_g, rgb_b);  // RGB LED 출력
  }

  // ---------- FND 제어 (매 루프마다) ----------
  // FND 값을 0부터 7777까지 증가 후 다시 0까지 감소 반복
  if (fnd_direction == 1) {
    fnd++;
    if (fnd >= 7777) {
      fnd = 7777;
      fnd_direction = 0;                  // 감소 모드로 전환
    }
  } else {
    if (fnd > 0) {
      fnd--;
    }
    if (fnd <= 0) {
      fnd = 0;
      fnd_direction = 1;                  // 증가 모드로 전환
    }
  }
  myfnd.setAllNum(fnd);                   // FND에 숫자 표시

  // ---------- 부저 제어 ----------
  // FND 값이 100 단위로 변경될 때마다 비프음 출력
  int current_fnd_100 = fnd / 100;
  if (current_fnd_100 != prev_fnd_100) {
    tone(BUZZER_PIN, 1000, 50);           // 1000Hz, 50ms 비프음
    prev_fnd_100 = current_fnd_100;
  }

  // ---------- 진동 센서 읽기 (100ms 간격) ----------
  // 피에조 센서로 진동 감지
  if (count_interrupt - last_vibration_count >= 100) {
    last_vibration_count = count_interrupt;
    vibration_value = analogRead(PIEZO_SENSOR_PIN);  // 아날로그 값 읽기 (0~1023)
  }

  // ---------- DC 모터 제어 (1초 간격) ----------
  // 1초마다 모터를 정지 → 정방향 → 역방향 순환
  if (count_interrupt - last_motor_count >= 1000) {
    last_motor_count = count_interrupt;

    if (motor_state == 0) {
      // 정지 → 정방향
      dcMotor.begin(DCIN_A, DCIN_B);      // 정방향 핀 설정
      dcMotor.Start();                    // 모터 시작
      motor_state = 1;
    } else if (motor_state == 1) {
      // 정방향 → 역방향
      dcMotor.Stop();                     // 일단 정지
      dcMotor.begin(DCIN_B, DCIN_A);      // 역방향 핀 설정 (A, B 반대로)
      dcMotor.Start();                    // 모터 시작
      motor_state = 2;
    } else {
      // 역방향 → 정지
      dcMotor.Stop();                     // 모터 정지
      motor_state = 0;
    }
  }

  // ---------- 스테퍼 모터 멜로디 재생 ----------
  // "K.K. House"를 상태 머신 방식으로 재생
  switch(melody_state) {
    case 0:  // 대기 상태 - 새로운 음표 시작
      if (melody_index < noteCount) {
        int bassFreq = bassLine[melody_index];
        if (bassFreq == REST) {
          noTone(SM_STEP);                // 쉼표: 소리 끄기
        } else {
          tone(SM_STEP, bassFreq);        // 음표 재생
        }

        last_melody_count = count_interrupt;
        melody_state = 1;                 // 음표 재생 중 상태로 전환
      } else {
        // 멜로디 끝 - 3초 대기 후 처음부터 다시
        if (count_interrupt - last_melody_count >= 3000) {
          melody_loop_count++;            // 멜로디 재생 횟수 증가
          lcd_state = 2;                  // LCD 곡 정보 화면으로 전환
          lcd_timer = count_interrupt;    // LCD 타이머 시작
          melody_index = 0;               // 처음부터 다시 재생
        }
      }
      break;

    case 1:  // 음표 재생 중 - 지속 시간 체크
      if (count_interrupt - last_melody_count >= durations[melody_index]) {
        noTone(SM_STEP);                  // 음표 끄기
        melody_index++;                   // 다음 음표로 이동
        last_melody_count = count_interrupt;

        if (melody_index < noteCount) {
          melody_state = 2;               // 50ms 간격 대기 상태로 전환
        } else {
          melody_state = 0;               // 멜로디 종료, 대기 상태로
        }
      }
      break;

    case 2:  // 음표 사이 50ms 간격 대기
      if (count_interrupt - last_melody_count >= 50) {
        melody_state = 0;                 // 다음 음표 재생을 위해 대기 상태로
      }
      break;
  }

  // ---------- 센서 데이터 읽기 (1초 간격) ----------
  // 온도 센서와 가스 센서 값 읽기
  if (count_interrupt - last_temp_count >= 1000) {
    last_temp_count = count_interrupt;
    current_temperature = temper.getTemperatureC();  // 온도 읽기 (℃)
    int raw_gas = gas.read();                        // 가스 센서 아날로그 값
    current_gas = map(raw_gas, 0, 1023, 0, 100);     // 0~100% 변환
  }

  // ---------- LCD 화면 전환 이벤트 감지 ----------

  // 온도 40도 이상 → 온도 위험 경고 화면 (최우선 순위 1!)
  if (current_temperature >= 40.0) {
    lcd_state = 5;                        // 온도 위험 경고 화면으로 전환
  }
  // 가스 농도 80% 이상 → 가스 위험 경고 화면 (최우선 순위 2!)
  else if (current_gas >= 80) {
    lcd_state = 4;                        // 가스 위험 경고 화면으로 전환
  }
  // FND가 1000 단위일 때 → 팀 이름 표시 (우선순위 3)
  else if (fnd % 1000 == 0 && fnd > 0) {
    if (!flag_fnd_1000) {
      lcd_state = 0;                      // 팀 이름 화면으로 전환
      lcd_timer = count_interrupt;        // 3초 타이머 시작
      flag_fnd_1000 = true;
    }
  } else {
    flag_fnd_1000 = false;
  }

  // FND가 100 단위일 때 → 온도/가스 표시 (우선순위 중간)
  // (1000 단위가 아닐 때만 실행)
  if (fnd % 100 == 0 && fnd > 0 && (fnd % 1000 != 0)) {
    if (!flag_fnd_100) {
      lcd_state = 1;                      // 온도/가스 화면으로 전환
      lcd_timer = count_interrupt;        // 3초 타이머 시작
      flag_fnd_100 = true;
    }
  } else {
    flag_fnd_100 = false;
  }

  // ---------- LCD 화면 렌더링 ----------
  // 화면 모드가 변경되었을 때만 화면 갱신 (깜빡임 방지)
  if (lcd_state != prev_lcd_state) {
    prev_lcd_state = lcd_state;
    lcd.clear();

    switch(lcd_state) {
      case 0:  // 팀 이름 표시 (주황색 백라이트)
        lcd.onBacklightBlue(0);    // Blue OFF
        lcd.onBacklightRed(1);     // Red ON
        lcd.onBacklightGreen(1);   // Green ON

        lcd.setCursor(0, 0);
        lcd.print("   POCHITA   ");
        lcd.setCursor(0, 1);
        lcd.print("   TEAM!!!   ");
        break;

      case 1:  // 온도/가스 정보 표시 (초록색 백라이트)
        lcd.onBacklightRed(0);     // Red OFF
        lcd.onBacklightGreen(1);   // Green ON
        lcd.onBacklightBlue(0);    // Blue OFF

        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(current_temperature, 1);  // 소수점 1자리
        lcd.print(" C");
        lcd.setCursor(0, 1);
        lcd.print("Gas : ");
        lcd.print(current_gas);
        lcd.print(" %");
        break;

      case 2:  // 곡 정보 표시 (파란색 백라이트)
        lcd.onBacklightRed(0);     // Red OFF
        lcd.onBacklightGreen(0);   // Green OFF
        lcd.onBacklightBlue(1);    // Blue ON

        lcd.setCursor(0, 0);
        lcd.print("Music:K.K.House");
        lcd.setCursor(0, 1);
        lcd.print("Play Count: ");
        lcd.print(melody_loop_count);
        break;

      case 3:  // 실시간 데이터 표시 (흰색 백라이트)
        lcd.onBacklightRed(1);     // Red ON
        lcd.onBacklightGreen(1);   // Green ON
        lcd.onBacklightBlue(1);    // Blue ON

        // 스크롤 위치 초기화
        lcd_scroll_pos = 0;
        last_lcd_scroll = count_interrupt;
        break;

      case 4:  // 가스 위험 경고 표시 (빨간색 백라이트)
        lcd.onBacklightRed(1);     // Red ON
        lcd.onBacklightGreen(0);   // Green OFF
        lcd.onBacklightBlue(0);    // Blue OFF

        lcd.setCursor(0, 0);
        lcd.print("!! DANGER !!    ");
        lcd.setCursor(0, 1);
        lcd.print("GAS: ");
        lcd.print(current_gas);
        lcd.print("%      ");
        break;

      case 5:  // 온도 위험 경고 표시 (빨간색 백라이트)
        lcd.onBacklightRed(1);     // Red ON
        lcd.onBacklightGreen(0);   // Green OFF
        lcd.onBacklightBlue(0);    // Blue OFF

        lcd.setCursor(0, 0);
        lcd.print("!! DANGER !!    ");
        lcd.setCursor(0, 1);
        lcd.print("TEMP: ");
        lcd.print(current_temperature, 1);
        lcd.print("C    ");
        break;
    }
  }

  // 실시간 데이터 모드일 때는 FND, Motor, Vib만 표시
  if (lcd_state == 3) {
    // 모터 상태 텍스트
    char motor_text[5];
    if (motor_state == 0) {
      strcpy(motor_text, "STOP");
    } else if (motor_state == 1) {
      strcpy(motor_text, "FWD ");
    } else {
      strcpy(motor_text, "REV ");
    }

    // 첫 번째 줄: FND 값
    lcd.setCursor(0, 0);
    lcd.print("FND: ");
    lcd.print(fnd);
    lcd.print("        ");  // 나머지 공백

    // 두 번째 줄: Motor 상태, Vib 값
    lcd.setCursor(0, 1);
    lcd.print("M:");
    lcd.print(motor_text);
    lcd.print(" V:");
    lcd.print(vibration_value);
    lcd.print("     ");  // 나머지 공백
  }

  // LCD 자동 복귀 타이머 (3초 경과 시 실시간 데이터 화면으로 복귀)
  // 단, 위험 상태(case 4, 5)는 타이머 무시하고 센서 값이 정상 범위로 돌아와야 복귀
  if (lcd_state == 5) {
    // 온도가 40도 미만으로 내려가면 실시간 화면으로 복귀
    if (current_temperature < 40.0) {
      lcd_state = 3;
    }
  } else if (lcd_state == 4) {
    // 가스 농도가 80% 미만으로 내려가면 실시간 화면으로 복귀
    if (current_gas < 80) {
      lcd_state = 3;
    }
  } else if (lcd_state != 3 && (count_interrupt - lcd_timer >= LCD_DURATION)) {
    lcd_state = 3;
  }

  // ========== 메인 루프 타이밍 제어 ==========
  // 100ms 주기로 루프 실행 대기
  do {
    // 대기
  } while (main_interval_count < MAIN_CLOCK);

  main_interval_count = 0;                // 카운터 리셋
  pin_loop = !pin_loop;                   // 디버그 핀 토글
}
