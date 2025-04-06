#ifndef WEBSOCKET_HANDLER_H
#define WEBSOCKET_HANDLER_H

#include <WebSocketsServer.h>
#include "car.h"
#include "tasks.h"

// WebSocket event handlers
void handleCommandWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void handleDataWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

#endif