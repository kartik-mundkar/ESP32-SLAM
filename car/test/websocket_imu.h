#ifndef ESP32_SLAM_WEBSOCKET_IMU_H
#define ESP32_SLAM_WEBSOCKET_IMU_H

#include <WebSocketsServer.h>

class WebSocketIMU {
public:
    WebSocketIMU(uint16_t port);
    void begin();
    void loop();
    void broadcastData(String& jsonData);  // Now directly accepts JSON data

private:
    WebSocketsServer webSocket;
    void handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
};

#endif // WEBSOCKET_IMU_H
