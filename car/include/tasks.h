#ifndef TASKS_H
#define TASKS_H

#include <WebSocketsServer.h>
#include "car.h"

// Initialize FreeRTOS tasks
void initTasks(Car *car, WebSocketsServer *commandWebSocket, WebSocketsServer *dataWebSocket);

#endif