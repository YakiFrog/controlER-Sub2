#ifndef __UDP_H__
#define __UDP_H__

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// setup()で呼び出す
bool udp_init();

// 送信側のMACアドレスを設定
esp_err_t registerPeerInfo(uint8_t *mac_addr);

// 送信ステータスを表示するコールバック関数
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status);

// 受信したデータを表示するコールバック関数
void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len);

#endif // __UDP_H__
