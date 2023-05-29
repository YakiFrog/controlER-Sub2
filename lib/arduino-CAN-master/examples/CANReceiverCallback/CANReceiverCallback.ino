// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Receiver Callback");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // register the receive callback
  CAN.onReceive(onReceive);
}

void loop() {
  // do nothing
}

void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received ");

  if (CAN.packetExtended()) { // 意味は，CAN packetのIDが拡張IDかどうかを判定する．
    Serial.print("extended "); // 拡張IDの場合，extendedと表示する．
  }

  if (CAN.packetRtr()) { // 意味は，CAN packetがRTRかどうかを判定する．
    // Remote transmission request, packet contains no data
    Serial.print("RTR "); // RTRとは，リモート送信要求のこと．
  }

  Serial.print("packet with id 0x"); // CAN packetのIDを表示する．
  Serial.print(CAN.packetId(), HEX); // CAN packetのIDを16進数で表示する．

  if (CAN.packetRtr()) { // CAN packetがRTRの場合，以下の処理を行う．
    Serial.print(" and requested length "); 
    Serial.println(CAN.packetDlc()); // CAN packetのDLCを表示する．(DLCは，データ長を表す．)
  } else { // CAN packetがRTRでない場合，以下の処理を行う．
    Serial.print(" and length ");
    Serial.println(packetSize); // CAN packetのサイズを表示する．

    // only print packet data for non-RTR packets // RTRでない場合，CAN packetのデータを表示する．
    while (CAN.available()) {
      Serial.print((char)CAN.read()); // CAN packetのデータを1byteずつ読み込み，char型に変換して表示する．
    }
    Serial.println();
  }

  Serial.println(); // これ入れると
}


