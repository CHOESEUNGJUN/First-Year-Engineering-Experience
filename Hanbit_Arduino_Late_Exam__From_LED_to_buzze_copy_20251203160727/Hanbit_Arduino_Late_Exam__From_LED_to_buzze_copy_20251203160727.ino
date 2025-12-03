/*
 * ===============================================
 * Arduino 멀티 디바이스 제어 프로그램
 * ===============================================
 *
 * [기능]
 * - LED 8개 순차 점멸 (500ms 간격)
 * - RGB LED 7색 순환 (30ms 간격)
 * - FND 0~7777 증가/감소 (1ms 간격)
 * - DC 모터 ON/OFF 전환 (1초 간격)
 * - 스테퍼 모터로 "La Vie en Rose" 멜로디 재생
 *
 * [타이머]
 * - 인터럽트 주기: 1ms (Timer3 사용)
 * - 메인 루프 주기: 100ms
 *
 * [참고]
 * - MsTimer2 대신 Timer3를 직접 설정하여 사용
 * - Timer2는 비어있어 tone() 함수 사용 가능 (스테퍼 모터 멜로디 가능)
 */

// ========== 라이브러리 포함 ==========
// #include <MsTimer2.h>   // MsTimer2 대신 Timer3 직접 설정 사용
#include <LED.h>           // LED 제어
#include <RgbLed.h>        // RGB LED 제어
#include <FND.h>           // 7-Segment 디스플레이
#include "DCMotor.h"       // DC 모터 제어
#include "StepMotor.h"     // 스테퍼 모터 제어
#include "RgbLcd.h"        // RGB LCD 제어

// ========== 객체 생성 ==========
LED myled;                 // LED 제어 객체
RgbLed myrgbled;           // RGB LED 제어 객체
FND myfnd;                 // FND 제어 객체
DCMotor dcMotor;           // DC 모터 제어 객체
StepMotor stepMotor;       // 스테퍼 모터 제어 객체
RgbLcd lcd;                // RGB LCD 제어 객체


// ========== 타이머 설정 ==========
#define MAIN_CLOCK 100     // 메인 루프 주기 (100ms)
#define INT_CLOCK 1        // 인터럽트 주기 (1ms)

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
#define SM_STEP  3         // 스테퍼 모터 스텝 핀 (베이스 출력용)
#define SM_MS1  41         // 마이크로스텝 설정 1
#define SM_MS2  40         // 마이크로스텝 설정 2
#define SM_SLEEP  38       // 슬립 모드 제어 핀

// 부저 핀
#define BUZZER_PIN 11      // 부저 핀 (멜로디 출력용)

// ========== 음계 정의 (Hz) ==========
// 베이스용 (낮은 음역 - 스테퍼 모터)
#define C3  131            // 도 (3옥타브)
#define D3  147            // 레
#define E3  165            // 미
#define F3  175            // 파
#define G3  196            // 솔
#define A3  220            // 라
#define B3  247            // 시
#define C4  262            // 도 (4옥타브)
#define D4  294            // 레
#define E4  330            // 미
#define F4  349            // 파
#define G4  392            // 솔

// 멜로디용 (높은 음역 - 부저)
#define C5  523            // 도 (5옥타브)
#define D5  587            // 레
#define E5  659            // 미
#define F5  698            // 파
#define G5  784            // 솔
#define A5  880            // 라
#define B5  988            // 시
#define C6  1047           // 도 (6옥타브)
#define D6  1175           // 레
#define E6  1319           // 미
#define F6  1397           // 파
#define G6  1568           // 솔

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

// ========== 스테퍼 모터 & 부저 2파트 멜로디 데이터 ==========
// "La Vie en Rose" - 베이스 라인 (스테퍼 모터, 낮은 음역)
int bassLine[] = {
  // "Hold me close and hold me fast"
  C3, C3, C3, F3, F3, F3, F3, C3,
  C3, REST,

  // "The magic spell you cast"
  C3, C3, C3, G3, G3, G3, G3,
  C3, REST,

  // "This is la vie en rose"
  C4, C4, F3, F3, F3, F3, C3,
  C3, REST,

  // 후렴
  G3, G3, C4, C4, G3, G3,
  G3, REST,

  C4, C4, F3, F3, C3, C3,
  C3, REST
};

// "La Vie en Rose" - 멜로디 라인 (부저, 높은 음역)
int melodyLine[] = {
  // "Hold me close and hold me fast"
  C6, C6, D6, E6, E6, D6, C6, D6,
  C6, REST,

  // "The magic spell you cast"
  C6, C6, D6, E6, F6, E6, D6,
  C6, REST,

  // "This is la vie en rose"
  E6, F6, G6, F6, E6, D6, C6,
  C6, REST,

  // 후렴
  G5, A5, C6, C6, B5, A5,
  G5, REST,

  C6, D6, E6, E6, D6, C6,
  C6, REST
};

// 각 음표의 지속 시간 (ms) - 베이스와 멜로디 동일
int durations[] = {
  300, 300, 400, 500, 300, 300, 400, 500,
  800, 200,

  300, 300, 400, 500, 400, 300, 400,
  800, 200,

  400, 400, 500, 400, 400, 400, 400,
  800, 200,

  400, 400, 500, 300, 400, 400,
  800, 200,

  400, 400, 500, 300, 400, 400,
  1000, 200
};

// ========== 스테퍼 모터 & 부저 멜로디 제어 변수 ==========
unsigned int melody_index = 0;                   // 현재 재생 중인 음표 인덱스
unsigned long last_melody_count = 0;             // 멜로디 마지막 업데이트 시간
unsigned char melody_state = 0;                  // 멜로디 상태: 0=대기, 1=음표재생중, 2=50ms간격대기
int noteCount = sizeof(melodyLine) / sizeof(melodyLine[0]);  // 전체 음표 개수
unsigned int melody_loop_count = 0;              // 멜로디 재생 횟수

// 부저 직접 제어 변수 (tone() 함수는 한 번에 하나의 핀만 지원하므로 직접 제어)
volatile unsigned long last_buzzer_toggle = 0;   // 부저 마지막 토글 시간 (마이크로초)
volatile unsigned char buzzer_state = 0;         // 부저 출력 상태
volatile int current_buzzer_freq = 0;            // 현재 부저 주파수
volatile unsigned int buzzer_half_period = 0;    // 부저 반주기 (마이크로초)

// ========== LCD 디스플레이 제어 변수 ==========
unsigned char lcd_state = 0;                     // LCD 상태: 0=POCHITA, 1=시리얼정보, 2=플레이목록
unsigned char melody_started = 0;                // 멜로디 시작 여부 플래그
unsigned long melody_display_start = 0;          // 멜로디 표시 시작 시간
String songTitle = "La Vie en Rose";             // 재생 중인 곡 제목
unsigned char fnd_100_flag = 0;                  // FND 100 도달 플래그
unsigned long team_display_start = 0;            // 팀 이름 표시 시작 시간
unsigned char prev_lcd_state = 0;                // 이전 LCD 상태 (상태 변경 감지용)


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
// - 부저 제어: 고주파 토글 처리
void main_interrupt(void)
{
  count_interrupt++;                      // 전체 카운터 증가
  main_interval_count++;                  // 메인 루프 카운터 증가
  pin_interrupt = !pin_interrupt;         // 디버그 핀 토글

  digitalWrite(PIN_INTERRUPT, pin_interrupt);  // 오실로스코프 확인용

  // 부저 토글 제어 (고주파수를 위해 인터럽트 내부에서 처리)
  if (current_buzzer_freq > 0) {
    unsigned long current_micros = micros();
    if (current_micros - last_buzzer_toggle >= buzzer_half_period) {
      buzzer_state = !buzzer_state;
      digitalWrite(BUZZER_PIN, buzzer_state);
      last_buzzer_toggle = current_micros;
    }
  }
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
  // MsTimer2 대신 Timer3 직접 설정 사용
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
}


// ========================================
// 디버그 출력 함수
// ========================================
void test_print(void)
{
  digitalWrite(PIN_LOOP, pin_loop);       // 메인 루프 토글 핀 출력

  // 시리얼 모니터에 상태 정보 출력
  Serial.print("\nMain Clock: ");
  Serial.print(count_interrupt);
  Serial.print(" FND: ");
  Serial.print(fnd);

  // DC 모터 상태 출력
  Serial.print(" Motor: ");
  if (motor_state == 0) {
    Serial.print("STOP");
  } else if (motor_state == 1) {
    Serial.print("FWD");  // Forward (정방향)
  } else {
    Serial.print("REV");  // Reverse (역방향)
  }

  // 스테퍼 모터 멜로디 재생 횟수 출력
  Serial.print(" Melody: ");
  Serial.print(melody_loop_count);
  Serial.print(" times");
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

  // ---------- LED 제어 (500ms 간격) ----------
  // LED 1번부터 8번까지 순차적으로 켜고, 다시 8번부터 1번까지 끄기
  if (count_interrupt - last_led_count >= 500) {
    last_led_count = count_interrupt;

    // LED ON/OFF
    if (0 == toggle) {
      myled.On(led_index);                // 증가 방향: LED 켜기
    } else {
      myled.Off(led_index);               // 감소 방향: LED 끄기
    }

    // LED 인덱스 증가/감소
    if (0 == toggle) {
      led_index++;
      if (led_index > 8) {                // 8번 LED 도달 시
        led_index = 8;
        toggle = !toggle;                 // 감소 모드로 전환
      }
    } else {
      led_index--;
      if (led_index < 1) {                // 1번 LED 도달 시
        led_index = 1;
        toggle = !toggle;                 // 증가 모드로 전환
      }
    }
  }
  
  // ---------- RGB LED 제어 (30ms 간격) ----------
  // 7가지 색상을 순환: 빨강 → 초록 → 파랑 → 노랑 → 마젠타 → 시안 → 흰색
  if (count_interrupt - last_rgb_count >= 30) {
    last_rgb_count = count_interrupt;

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

    myrgbled.OnRgb(rgb_r, rgb_g, rgb_b);  // RGB LED 출력
  }
  
  // ---------- FND 제어 (1ms 간격) ----------
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
  
  // ---------- DC 모터 제어 (1초 간격) ----------
  // 1초마다 모터를 정방향 → 역방향 → 정지 순환
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

  // ---------- 부저 직접 제어는 인터럽트에서 처리 ----------
  // (부저 토글은 1ms 타이머 인터럽트 내부에서 고주파로 처리됨)

  // ---------- 스테퍼 모터(베이스) & 부저(멜로디) 2파트 재생 ----------
  // "La Vie en Rose"를 2파트로 동시 재생 (상태 머신 방식)
  // - 스테퍼 모터: 베이스 라인 (낮은 음역) - tone() 사용
  // - 부저: 멜로디 라인 (높은 음역) - 직접 제어
  switch(melody_state) {
    case 0:  // 대기 상태 - 새로운 음표 시작
      if (melody_index < noteCount) {
        // 베이스 라인 재생 (스테퍼 모터)
        int bassFreq = bassLine[melody_index];
        if (bassFreq == REST) {
          noTone(SM_STEP);                // 쉼표: 베이스 소리 끄기
        } else {
          tone(SM_STEP, bassFreq);        // 베이스 음표 재생
        }

        // 멜로디 라인 재생 (부저 - 직접 제어)
        int melodyFreq = melodyLine[melody_index];
        if (melodyFreq == REST) {
          current_buzzer_freq = 0;        // 쉼표: 주파수 0으로 설정
          digitalWrite(BUZZER_PIN, LOW);  // 부저 끄기
        } else {
          current_buzzer_freq = melodyFreq;
          buzzer_half_period = 1000000L / melodyFreq / 2;  // 반주기 계산 (마이크로초)
          last_buzzer_toggle = micros();
        }

        // 2번째 음표 시작 시 플레이 목록 표시 (5초간)
        if (melody_index == 1 && !melody_started) {
          melody_started = 1;
          lcd_state = 2;                  // 플레이 목록 모드로 전환
          melody_display_start = count_interrupt;
        }

        last_melody_count = count_interrupt;
        melody_state = 1;                 // 음표 재생 중 상태로 전환
      } else {
        // 멜로디 끝 - 3초 대기 후 처음부터 다시
        if (count_interrupt - last_melody_count >= 3000) {
          melody_index = 0;
          melody_loop_count++;            // 멜로디 재생 횟수 증가
          melody_started = 0;             // 다음 재생을 위해 플래그 리셋
        }
      }
      break;

    case 1:  // 음표 재생 중 - 지속 시간 체크
      if (count_interrupt - last_melody_count >= durations[melody_index]) {
        noTone(SM_STEP);                  // 베이스 음표 끄기
        current_buzzer_freq = 0;          // 부저 끄기
        digitalWrite(BUZZER_PIN, LOW);
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

  // ---------- LCD 디스플레이 제어 ----------

  // FND가 100의 배수에 도달하면 팀 이름 5초 표시
  if (fnd % 100 == 0 && fnd > 0 && !fnd_100_flag && lcd_state != 2) {
    fnd_100_flag = 1;
    lcd_state = 0;  // 팀 이름(POCHITA) 표시 모드
    team_display_start = count_interrupt;
  }

  // FND가 100의 배수가 아니면 플래그 리셋
  if (fnd % 100 != 0) {
    fnd_100_flag = 0;
  }

  // 팀 이름 표시 5초 경과 시 시리얼 정보로 전환
  if (lcd_state == 0 && team_display_start > 0 &&
      count_interrupt - team_display_start >= 5000) {
    lcd_state = 1;  // 시리얼 정보 표시 모드
    team_display_start = 0;
  }

  // 플레이 목록 표시 5초 경과 시 시리얼 정보로 전환
  if (lcd_state == 2 && melody_display_start > 0 &&
      count_interrupt - melody_display_start >= 5000) {
    lcd_state = 1;  // 시리얼 정보 표시 모드
    melody_display_start = 0;
  }

  // LCD 상태 변경 시 화면 업데이트
  if (lcd_state != prev_lcd_state) {
    prev_lcd_state = lcd_state;
    lcd.clear();

    switch(lcd_state) {
      case 0:  // POCHITA (팀 이름) 표시
        // 주황색 백라이트
        lcd.onBacklightBlue(0);    // Blue OFF
        lcd.onBacklightRed(1);     // Red ON
        lcd.onBacklightGreen(1);   // Green ON

        lcd.setCursor(0, 0);
        lcd.print("   POCHITA   ");
        lcd.setCursor(0, 1);
        lcd.print("   TEAM!!!   ");
        break;

      case 1:  // 시리얼 정보 표시
        // 흰색 백라이트 (Red + Green + Blue)
        lcd.onBacklightRed(1);     // Red ON
        lcd.onBacklightGreen(1);   // Green ON
        lcd.onBacklightBlue(1);    // Blue ON

        lcd.setCursor(0, 0);
        lcd.print("FND:");
        lcd.print(fnd);
        lcd.print(" Song:");
        lcd.print(melody_loop_count);
        lcd.setCursor(0, 1);
        lcd.print("Motor:");
        if (motor_state == 0) {
          lcd.print("STOP");
        } else if (motor_state == 1) {
          lcd.print("FWD");
        } else {
          lcd.print("REV");
        }
        break;

      case 2:  // 플레이 목록 표시
        // 파란색 백라이트
        lcd.onBacklightRed(0);     // Red OFF
        lcd.onBacklightGreen(0);   // Green OFF
        lcd.onBacklightBlue(1);    // Blue ON

        lcd.setCursor(0, 0);
        lcd.print("Now Playing:");
        lcd.setCursor(0, 1);
        lcd.print(songTitle);
        break;
    }
  }

  // 시리얼 정보 모드일 때는 매 루프마다 FND, Song, Motor 상태 업데이트
  if (lcd_state == 1) {
    lcd.setCursor(4, 0);
    lcd.print(fnd);
    lcd.print(" Song:");
    lcd.print(melody_loop_count);
    lcd.print("  ");  // 기존 숫자 지우기

    lcd.setCursor(6, 1);
    if (motor_state == 0) {
      lcd.print("STOP");
    } else if (motor_state == 1) {
      lcd.print("FWD ");
    } else {
      lcd.print("REV ");
    }
  }

  // ========== 메인 루프 타이밍 제어 ==========
  // 100ms 주기로 루프 실행 대기
  do {
    // 대기
  } while (main_interval_count < MAIN_CLOCK);

  main_interval_count = 0;                // 카운터 리셋
  pin_loop = !pin_loop;                   // 디버그 핀 토글
}