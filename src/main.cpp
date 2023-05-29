#include <Arduino.h>
#include "m3508.h"
#include "mdds30.h"
#include "qei.hpp"
#include "esp_intr_alloc.h"
#include "udp.h"

#define rx 4
#define tx 5
#define CAN_SPEED 1000E3

// AMT102
#define TOP_A 25 // Blue
#define TOP_B 33 // Yellow
#define BOTTOM_A 32 // Green
#define BOTTOM_B 35 // White

#define LIMIT_PIN 26

uint8_t monitor_mac_addr[] = {0xe0, 0x5a, 0x1b, 0x75, 0x13, 0x24};

TaskHandle_t thp[2];

int id = 0x201;

int16_t count_T;
int16_t count_B;

int8_t headerByte = 0x55; // -128 ~ 127, 0x12 = 18
int8_t addressByte = 0x00; // -128 ~ 127, 0x01 = 1
int8_t commandByte1 = 0x00; // -128 ~ 127, -1 or 0 or 1
int8_t commandByte2 = 0x00; // -128 ~ 127, -1 or 0 or 1
int8_t commandByte3 = 0x00; // -128 ~ 127, -1 or 0 or 1
int8_t commandByte4 = 0x00; // 自動昇降
int8_t checksum = 0x00; // -128 ~ 127

int16_t mangle[4] = {0, 0, 0, 0};
int16_t mrpm[4] = {0, 0, 0, 0};
int16_t mtorque[4] = {0, 0, 0, 0};
int16_t current_data[4] = {0, 0, 0, 0};
uint8_t send_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int8_t updwn_cmd = 0; // 昇降, 指令値（-1, 0, 1)
int8_t rollr_cmd = 0; // 上下ローラ, 指令値（1，2，3)
int8_t rollr_spd_cmd = 0; // 上下ローラ, 速度増減指令値(-1, 0, 1)
int8_t auto_updwn_cmd = 0; // 自動昇降, 指令値（-1, 0, 1)
bool auto_updwn_flag = false; // 自動昇降, フラグ
int8_t rollr_val[2] = {0, 0}; // 上下ローラ, %指令値(-80 ~ 80)

int updwn_trgt = 500; // 昇降のターゲットRPM (500)
int rollr_rdsc[2] = {0, 0};
float updwn_rdsc = 0;
float updwn_angle = 0;
int trgt_angle = 0;

int16_t error = 0;
int16_t integral = 0;
int16_t derivative = 0;
int16_t prev_error = 0;

int16_t rollr_trgt = 0; // ローラのターゲットRPM

// PID: ローラ
int16_t rollr_error[2] = {0, 0};
int16_t rollr_integral[2] = {0, 0};
int16_t rollr_derivative[2] = {0, 0};
int16_t rollr_prev_error[2] = {0, 0};

float prev_time = 0;
float dt = 0;

float Kp = 60; // ~70
float Ki = 0;
float Kd = 10;

// todo 05/29
float rollr_Kp = 0.2; 
float rollr_Ki = 0.0018; // ~0.0019
float rollr_Kd = 0.05;

int core0a_free_stack = 0;
int core0b_free_stack = 0;
int core1b_free_stack = 0;
int core1m_free_stack = 0;

uint8_t data[13] = {0, 0, 0, 0, 0, 0, 0, 0};

void Core0a(void *args); // PS4 Controller
void Core0b(void *args);
void Core1b(void *args);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); // Serial2: RXD16, TXD17

  pinMode(LIMIT_PIN, INPUT_PULLUP);

  // AMT102
  qei_setup_x1(PCNT_UNIT_2, TOP_A, TOP_B); // TOP: Blue, Yellow
  qei_setup_x1(PCNT_UNIT_3, BOTTOM_A, BOTTOM_B); // BOTTOM: Green, White
  
  Serial2.write(0x80);
  for (int i = 0; i < 3000; i++) {
    Serial2.write(0x80);
    delay(1);
  }
  while (!can_init(rx, tx, CAN_SPEED));
  Serial.println("> CAN: Started.");

  while (!udp_init());
  Serial.println("> UDP: Started.");
  Serial.println("> UDP: My MacAddress: " + WiFi.macAddress());

  // UDP(ESPNOW): 接続したいESP32のMACアドレスを登録
  if (registerPeerInfo(monitor_mac_addr) == ESP_OK) {
    Serial.println("> UDP: Peer registered.");
  } else {
    Serial.println("> UDP: Peer not registered.");
  }

  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 3, &thp[0], 0); // (タスク名, タスクのサイズ, パラメータ, 優先度, タスクハンドル, コア番号)
  xTaskCreatePinnedToCore(Core0b, "Core0b", 4096, NULL, 3, &thp[1], 0);
  xTaskCreatePinnedToCore(Core1b, "Core1b", 4096, NULL, 3, &thp[2], 1); 
}

void loop() {
  m3508_read_data(0x201, mangle, mrpm, mtorque);
  dt = millis() - prev_time;
  prev_time = millis();

  // 自動昇降機構の制御
  if (auto_updwn_cmd == 1) {
    trgt_angle = 31.5;
    auto_updwn_flag = !auto_updwn_flag;
  } else if (auto_updwn_cmd == 2) {
    trgt_angle = 30.5;
    auto_updwn_flag = !auto_updwn_flag;
  } else if (auto_updwn_cmd == 3) {
    trgt_angle = 0;
    auto_updwn_flag = !auto_updwn_flag;
  } else if (auto_updwn_cmd == 4) {
    trgt_angle = 0;
    auto_updwn_flag = !auto_updwn_flag;
  }

  if (auto_updwn_flag == true) {
    float error_angle = trgt_angle - updwn_angle;
    // 差が1deg以下の時は制御しない
    if (abs(error_angle) < 1) {
      updwn_cmd = 0;
      auto_updwn_flag = false;
    } else {
      updwn_cmd = error_angle > 0 ? 1 : -1;
    }
  }

  if (updwn_cmd == -1 && updwn_angle < 1.7) {
    updwn_trgt = -150;
  } else {
    updwn_trgt = -500;
  }

  if (digitalRead(LIMIT_PIN) == HIGH && updwn_cmd == -1) {
    updwn_cmd = 0;
  }

  if (updwn_angle > 40 && updwn_cmd == 1) {
    updwn_cmd = 0;
  }

  if (rollr_cmd == 1) {
    rollr_trgt = 160 + rollr_spd_cmd * 5; // 32-33deg
  } else if (rollr_cmd == 2) {
    rollr_trgt = 225 + rollr_spd_cmd * 5; // 31deg
  } else if (rollr_cmd == 3) {
    rollr_trgt = 0 + rollr_spd_cmd * 5;
  } else {
    rollr_trgt = 0;
  }

  rollr_trgt = (rollr_trgt > 250) ? 250 : rollr_trgt;

  error = (updwn_cmd * updwn_trgt) - mrpm[0];
  integral += error * dt;
  derivative = (error - prev_error) / dt;
  prev_error = error;

  current_data[0] = Kp * error + Ki * integral + Kd * derivative;
  current_data[0] = constrain(current_data[0], -5000, 5000);

  for (int i = 0; i < 2; i++) {
    rollr_error[i] = rollr_trgt - rollr_rdsc[i];
    rollr_integral[i] += rollr_error[i] * dt;
    rollr_derivative[i] = (rollr_error[i] - rollr_prev_error[i]) / dt;
    rollr_prev_error[i] = rollr_error[i];
    rollr_val[i] = rollr_Kp * rollr_error[i] + rollr_Ki * rollr_integral[i] + rollr_Kd * rollr_derivative[i];
  }

  rollr_val[0] = constrain(rollr_val[0], -80, 80);
  rollr_val[1] = constrain(rollr_val[1], -80, 80);

  m3508_make_data(current_data, send_data);
  m3508_send_data(send_data);
  mdds30_control_motor(0x00, rollr_val[1], rollr_val[1]);

  delay(1);
  core1m_free_stack = uxTaskGetStackHighWaterMark(NULL);
}

void Core0a(void *args) {
  while (1) {
    pcnt_get_counter_value(PCNT_UNIT_2, &count_T);
    pcnt_get_counter_value(PCNT_UNIT_3, &count_B);
    rollr_rdsc[0] = ((count_T / 5.0) / 512.0) * 1000 * (2 * PI); // 実際のrad/s(上のモーター) 0001
    rollr_rdsc[1] = ((count_B / 5.0) / 512.0) * 1000 * (2 * PI); // 実際のrad/s(下のモーター) 0001
    updwn_rdsc = -mrpm[0] * 0.10472; // 実際のrad/s(昇降)
    if (digitalRead(LIMIT_PIN) == HIGH) {
      updwn_angle = 0;
    } else {
      updwn_angle += ((updwn_rdsc * 0.0500000) * 57.2957795) * 0.0002350; // rad to deg 
    }
    pcnt_counter_clear(PCNT_UNIT_2); 
    pcnt_counter_clear(PCNT_UNIT_3);
    delay(5);
    core0a_free_stack = uxTaskGetStackHighWaterMark(NULL);
  }
}

void Core0b(void *args) {
  while (1) {
    if (Serial2.available() && Serial2.read() == 0x55) {
      addressByte = Serial2.read();
      commandByte1 = Serial2.read();
      commandByte2 = Serial2.read();
      commandByte3 = Serial2.read();
      commandByte4 = Serial2.read();
      checksum = Serial2.read();
      if (checksum == headerByte + addressByte + commandByte1 + commandByte2 + commandByte3 + commandByte4) {
        updwn_cmd = commandByte1;
        rollr_cmd = commandByte2;
        rollr_spd_cmd = commandByte3;
        auto_updwn_cmd = commandByte4;
      }
      // Serial2を空にする
      while (Serial2.available()) {
        Serial2.read();
      }
    }
    delay(10);
    core0b_free_stack = uxTaskGetStackHighWaterMark(NULL);
  }
}

void Core1b(void *args) {
  while (1) {
    // rollr_rdsc[0] を data[0], data[1]
    data[0] = (uint8_t)(rollr_rdsc[0] >> 8) & 0xff; // 上位8bit
    data[1] = (uint8_t)rollr_rdsc[0] & 0xff; // 下位8bit
    // rollr_rdsc[1] を data[2], data[3]
    data[2] = (uint8_t)(rollr_rdsc[1] >> 8) & 0xff; // 上位8bit
    data[3] = (uint8_t)rollr_rdsc[1] & 0xff; // 下位8bit
    // updwn_angle 
    data[4] = (uint8_t)updwn_angle;
    // LIMIT_PIN
    data[5] = (uint8_t)digitalRead(LIMIT_PIN);
    // updwn_rdsc
    data[8] = (uint8_t)updwn_rdsc & 0xff; // 下位8bit
    data[12] = (uint8_t)dt;
    esp_now_send(monitor_mac_addr, data, sizeof(data)); // データを送信

    // Serial.print(dt);
    // Serial.print(", ");
    // Serial.print(updwn_cmd);
    // Serial.print(", ");
    // Serial.print(rollr_cmd);
    // Serial.print(", ");
    // Serial.print(rollr_spd_cmd);
    // Serial.print(" | ");
    // Serial.print(mrpm[0]);
    // Serial.print(" | ");
    // Serial.print(digitalRead(LIMIT_PIN));
    // Serial.print(" | ");
    // Serial.print(updwn_angle);
    // Serial.print(" | ");
    // Serial.print(updwn_rdsc);
    // Serial.print(" | ");
    Serial.print(rollr_rdsc[0]);
    // Serial.print(", ");
    // Serial.print(rollr_rdsc[1]);
    // Serial.print(" | ");
    // Serial.print(rollr_val[0]);
    // Serial.print(", ");
    // Serial.print(rollr_val[1]);
    // Serial.print(" | ");
    // Serial.print(core0a_free_stack);
    // Serial.print(", ");
    // Serial.print(core0b_free_stack);
    // Serial.print(", ");
    // Serial.print(core1b_free_stack);
    // Serial.print(", ");
    // Serial.print(core1m_free_stack);
    Serial.println();
    delay(50);
    // delay(10);
    core1b_free_stack = uxTaskGetStackHighWaterMark(NULL);
  }
}
