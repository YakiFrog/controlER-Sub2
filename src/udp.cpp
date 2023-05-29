#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "udp.h"

// setup()で呼び出す
bool udp_init() {
    WiFi.mode(WIFI_STA); // WiFiモードをステーションに設定(子機)
    WiFi.disconnect(); // WiFi接続を切断

    if (esp_now_init() == ESP_OK) return true;
    else return false;
}

// 送信側のMACアドレスを設定
esp_err_t registerPeerInfo(uint8_t *mac_addr) {
    esp_now_peer_info_t peerInfo; // 送信先のMACアドレスを設定するための構造体
    memset(&peerInfo, 0, sizeof(peerInfo)); // peerInfoを0で初期化
    memcpy(peerInfo.peer_addr, mac_addr, 6); // 送信先のMACアドレスを設定
    peerInfo.channel = 0; // チャンネルは0-13(0は自動, 1-13は固定)
    peerInfo.encrypt = false; // 暗号化しない
    peerInfo.ifidx = WIFI_IF_STA; // 送信インターフェースを設定(WIFI_IF_STA:ステーション WIFI_IF_AP:アクセスポイント)
    return esp_now_add_peer(&peerInfo); // 送信先のMACアドレスを設定
}

// 送信ステータスを表示するコールバック関数
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("送信先MACアドレス: ");
    for (uint8_t i = 0; i < 6; i++) {
    Serial.print(mac_addr[i], HEX);
    if (i < 5) {
        Serial.print(":");
    }
    }
    Serial.print(" 送信ステータス: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "成功" : "失敗");
}

// 受信したデータを表示するコールバック関数
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
    Serial.print("受信元MACアドレス: ");
    for (uint8_t i = 0; i < 6; i++) {
      Serial.print(mac_addr[i], HEX);
      if (i < 5) {
        Serial.print(":");
      }
    }
    Serial.print(" 受信データ: ");
    for (int i = 0; i < len; i++) {
      Serial.print(data[i]);
      if (i < len - 1) {
        Serial.print(",");
      }
    }

//   // 8bit * 8のデータを16bit * 4に変換
//   for (int i = 0; i < 4; i++) {
//     encoder_data[i] = data[i * 2] + data[i * 2 + 1] * 256;
//     //Serial.print(" " + String(encoder_data[i]));
//   }

    Serial.println();
}