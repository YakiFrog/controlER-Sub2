#include "mdds30.h"
#include <Arduino.h>

// MDDS30
static int headerByte = 0x55;
static int addressByte;
static int commandByte;
static int checksum;
static bool hardwareSerial = true;

// MDDS30に繋がれてるモータを制御する
void mdds30_control_motor(int id, signed int speed_left, signed int speed_right){
    // Left motor
    addressByte = id | 0b00000000;
    commandByte = map(speed_left, -100, 100, 0, 255);
    checksum = headerByte + addressByte + commandByte;
    if (hardwareSerial == true) {
        Serial2.write(headerByte);
        Serial2.write(addressByte);
        Serial2.write(commandByte);
        Serial2.write(checksum);
    }
    // Right motor
    addressByte = id | 0b00001000;
    commandByte = map(speed_right, -100, 100, 0, 255);
    checksum = headerByte + addressByte + commandByte;
    if (hardwareSerial == true) {
        Serial2.write(headerByte);
        Serial2.write(addressByte);
        Serial2.write(commandByte);
        Serial2.write(checksum);
    }
}