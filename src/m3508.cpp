#include "m3508.h"
#include <Arduino.h>
#include <CAN.h>

static uint16_t angle = 0;
static uint16_t rpm = 0;
static uint16_t torque = 0;

static uint8_t top = 0;
static uint8_t bottom = 0;
static int16_t data = 0;

// setup()で呼び出す
bool can_init(int rx, int tx, int speed) {
    CAN.setPins(rx, tx);
    if (!CAN.begin(speed)){
        return false;
    } else {
        // volatile uint32_t* pREG_IER = (volatile uint32_t*)0x3ff6b010;
        // *pREG_IER &= ~(uint8_t)0x10;
        return true;
    }
}

// M3508のデータを作る
void m3508_make_data(int16_t data_in[4], uint8_t data_out1[8]) {
    // 0x200用のデータを作成（ID1-4）
    for (int i = 0; i < 4; i++) {
        data_out1[0 + (i * 2)] = (data_in[i] >> 8) & 0xFF;
        data_out1[1 + (i * 2)] = data_in[i] & 0xFF;
    }
}

// M3508にデータを送る
bool m3508_send_data(uint8_t data_out1[8]) {
    bool success0x200 = false;
    CAN.beginPacket(0x200); // 0x200は，M3508のCAN ID(1-4)
    CAN.write(data_out1, 8); // 8 bytes
    if (CAN.endPacket()) success0x200 = true; // 送信
    return success0x200;
}

// M3508のデータを読む
void m3508_read_data(int id, int16_t mangle[4], int16_t mrpm[4], int16_t mtorque[4]) {
    if (CAN.parsePacket() && CAN.filter(id)) {
        angle = 0;
        rpm = 0;
        torque = 0;
        top = 0;
        bottom = 0;
        data = 0;
        long id = CAN.packetId();
        for (int i = 0; i < 4; i++){
            top = CAN.read();
            bottom = CAN.read();
            data = (top << 8) | bottom;
            if (i == 0) angle = (float)(data * 360 / 8192);
            if (i == 1) rpm = (float)(data / (3591 / 187));
            if (i == 2) torque = (float)(data * 100 / 8192);
            if (i == 3) data = 0;
        }
        mangle[id - 0x201] = angle;
        mrpm[id - 0x201] = rpm;
        mtorque[id - 0x201] = torque;
    }
}