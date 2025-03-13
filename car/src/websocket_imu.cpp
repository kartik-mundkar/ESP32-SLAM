#include "../include/websocket_imu.h"

WebSocketIMU::WebSocketIMU(uint16_t port) : webSocket(port) {}

void WebSocketIMU::begin() {
    webSocket.begin();
    webSocket.onEvent([this](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
        this->handleWebSocketEvent(num, type, payload, length);
    });
    Serial.println("WebSocket server started.");
}

void WebSocketIMU::loop() {
    webSocket.loop();
}

void WebSocketIMU::broadcastData(String& jsonData) {
    webSocket.broadcastTXT(jsonData);
}

void WebSocketIMU::handleWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.printf("[WebSocket] Client %u connected.\n", num);
            break;

        case WStype_DISCONNECTED:
            Serial.printf("[WebSocket] Client %u disconnected.\n", num);
            break;

        case WStype_TEXT:
            Serial.printf("[WebSocket] Received message: %s\n", payload);
            break;
    }
}
