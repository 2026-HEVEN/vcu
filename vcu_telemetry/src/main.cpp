// ============================================================
//  VCU 텔레메트리 (4회차 스터디) — ESP32 TWAI
//  단일 컨트롤러 제어 + 텔레메트리(전압/전류/속도/온도) 수신
//
//  핵심: 핸드쉐이크 요청(0x55)과 Message I(텔레메트리)의 ID가 동일(0x1801D0EF)
//        → 상태(handshakeDone)로 구분해서 해석
//  배선: SN65HVD230 RXD→GPIO4, TXD→GPIO5 | 스로틀 SIG→GPIO34 (5V출력이면 분압)
// ============================================================
#include <Arduino.h>
#include "driver/twai.h"

// ---------------- 핀 설정 ----------------
#define RX_PIN 4
#define TX_PIN 5
const int throttlePin = 34; // ESP32 아날로그 입력 핀

// ------------- CAN 통신 ID 설정 -------------
const uint32_t CAN_TX_ID = 0x0C01EFD0;   // 제어 명령 / 핸드쉐이크 응답 송신 ID

// 🚨 핵심: 핸드쉐이크 요청과 Message I(전압/전류/속도)는 ID가 동일(0x1801D0EF)
const uint32_t CAN_ID_MSG1 = 0x1801D0EF; // 핸드쉐이크 요청 ID == 텔레메트리 ID
const uint32_t CAN_ID_MSG2 = 0x1802D0EF; // 수신: 온도

// ---------------- 상태 변수 ----------------
bool handshakeDone = false;
uint8_t lifeSignal = 0;
unsigned long lastSendTime = 0;
const int sendInterval = 50;  // 제어 명령 전송 주기 50ms

// ---------------- 설정 변수 ----------------
const int MAX_PHASE_CURRENT = 100;  // 최대 목표 상전류 100A
const int MAX_RPM = 4000;           // 최대 목표 속도 4000rpm

// 스로틀 캘리브레이션 (12비트 ADC 0~4095 기준) — 실측 후 조정
const int THR_DEADZONE = 720;   // 기존 10비트 180 × 4
const int THR_FULL     = 3400;  // 기존 10비트 850 × 4

void sendHandshakeResponse() {
  twai_message_t txHandshake;
  txHandshake.extd = 1; // 확장 프레임
  txHandshake.rtr = 0;  // 데이터 프레임
  txHandshake.identifier = CAN_TX_ID;
  txHandshake.data_length_code = 8;

  for (int i = 0; i < 8; i++) {
    txHandshake.data[i] = 0xAA;
  }

  twai_transmit(&txHandshake, pdMS_TO_TICKS(10));
}

void setup() {
  Serial.begin(115200);
  delay(2000); // 시리얼 포트 대기

  // 12비트 ADC (0~4095), 0~3.3V 범위
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // ==========================================
  // ESP32 TWAI(CAN) 드라이버 초기화
  // ==========================================
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS(); // 통신 속도 250kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // 모든 ID 수신 허용

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI(CAN) 드라이버 설치 완료");
  } else {
    Serial.println("TWAI(CAN) 드라이버 설치 실패!");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("TWAI(CAN) 통신 시작! 핸드쉐이크 대기 중...");
  } else {
    Serial.println("TWAI(CAN) 통신 시작 실패!");
    return;
  }
}

void loop() {
  // ==========================================
  // 1. CAN 메시지 수신 및 파싱 처리
  // ==========================================
  twai_message_t rxMsg;

  while (twai_receive(&rxMsg, 0) == ESP_OK) {
    uint32_t rxId = rxMsg.identifier;

    // ------------------------------------------------------------
    // (A) 0x1801D0EF — 상태에 따라 "핸드쉐이크" 또는 "텔레메트리"
    // ------------------------------------------------------------
    if (rxId == CAN_ID_MSG1) {

      // 핸드쉐이크(요청/재요청)는 8바이트 전부 0x55 → 텔레메트리보다 먼저 판별
      bool isHandshake = true;
      for (int i = 0; i < 8; i++) {
        if (rxMsg.data[i] != 0x55) { isHandshake = false; break; }
      }

      if (isHandshake) {
        if (!handshakeDone) {
          Serial.println("핸드쉐이크 요청 수신(0x55). 0xAA 응답 전송...");
          sendHandshakeResponse();
          handshakeDone = true;
          delay(5);
          lastSendTime = millis() - sendInterval; // 바로 제어 명령 전송되도록
          Serial.println(">>> 핸드쉐이크 완료! 제어 시작...");
        } else {
          // 이미 완료됐는데 다시 0x55 → 컨트롤러 재시작 → 재핸드쉐이크 대기
          Serial.println("WARNING: 컨트롤러 재시작 감지 (핸드쉐이크 재요청)");
          handshakeDone = false;
        }
      }
      // 핸드쉐이크 완료 후에만 텔레메트리로 해석
      else if (handshakeDone) {
        float busVoltage = ((rxMsg.data[1] << 8) | rxMsg.data[0]) * 0.1;
        float busCurrent = ((rxMsg.data[3] << 8) | rxMsg.data[2]) * 0.1 - 3200;
        int   speedRPM   = ((rxMsg.data[7] << 8) | rxMsg.data[6]) - 32000; // 1rpm/bit

        Serial.print("[상태] 전압(V): "); Serial.print(busVoltage, 1);
        Serial.print(" | 전류(A): ");    Serial.print(busCurrent, 1);
        Serial.print(" | 속도(RPM): ");  Serial.println(speedRPM);
      }
    }
    // ------------------------------------------------------------
    // (B) Message II (0x1802D0EF) - 온도
    // ------------------------------------------------------------
    else if (rxId == CAN_ID_MSG2) {
      int controllerTemp = rxMsg.data[0] - 40;
      int motorTemp      = rxMsg.data[1] - 40;

      Serial.print("[온도] 컨트롤러: "); Serial.print(controllerTemp);
      Serial.print("C | 모터: ");        Serial.print(motorTemp);
      Serial.println("C");
    }
  }

  // ==========================================
  // 2. 스로틀 제어 명령 전송 처리 (50ms 주기)
  // ==========================================
  if (handshakeDone) {
    unsigned long currentTime = millis();

    if (currentTime - lastSendTime >= sendInterval) {
      lastSendTime = currentTime;

      int sensorValue   = analogRead(throttlePin);
      int targetCurrent = 0;

      // 12비트 데드존/최대값 (실측 후 THR_DEADZONE / THR_FULL 조정)
      if (sensorValue > THR_DEADZONE) {
        targetCurrent = map(sensorValue, THR_DEADZONE, THR_FULL, 0, MAX_PHASE_CURRENT);
      }
      targetCurrent = constrain(targetCurrent, 0, MAX_PHASE_CURRENT);

      twai_message_t txControl;
      txControl.extd = 1;
      txControl.rtr = 0;
      txControl.identifier = CAN_TX_ID;
      txControl.data_length_code = 8;

      // Byte 0~1: 목표 상전류 (offset -3200A, 0.1A/bit, 리틀엔디안)
      uint16_t currentData = (targetCurrent + 3200) * 10;
      txControl.data[0] = currentData & 0xFF;
      txControl.data[1] = (currentData >> 8) & 0xFF;

      // Byte 2~3: 목표 속도 (offset -32000rpm, 1rpm/bit)
      uint16_t speedData = MAX_RPM + 32000;
      txControl.data[2] = speedData & 0xFF;
      txControl.data[3] = (speedData >> 8) & 0xFF;

      // Byte 4: 제어 명령 (Bit0=1 → RUNNING)
      txControl.data[4] = 0x01;
      txControl.data[5] = 0x00;
      txControl.data[6] = 0x00;

      // Byte 7: 생존 신호 (0~255 순환)
      txControl.data[7] = lifeSignal++;

      twai_transmit(&txControl, pdMS_TO_TICKS(10));

      // (참고) 디버깅 출력이 필요하면 주석 해제
      // Serial.print("Throttle ADC: "); Serial.print(sensorValue);
      // Serial.print(" | Target(A): ");  Serial.println(targetCurrent);
    }
  }
}
