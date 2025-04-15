#include "tasks.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// FreeRTOS queue for commands
QueueHandle_t commandQueue;

// Task: Receive commands from WebSocket
void receiveCommandTask(void *parameter) {
    WebSocketsServer *commandWebSocket = (WebSocketsServer *)parameter;
    while (true) {
        if (commandWebSocket != nullptr) {
            commandWebSocket->loop(); // Process WebSocket events
        } else {
            Serial.println("Error: Command WebSocket reference is null.");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Yield to other tasks
    }
}

// Task: Send sensor data to the server
void sendDataTask(void *parameter) {
    struct TaskParams {
        Car *car;
        WebSocketsServer *dataWebSocket;
    };
    TaskParams *params = (TaskParams *)parameter;
    Car *car = params->car;
    WebSocketsServer *dataWebSocket = params->dataWebSocket;

    while (true) {
        if (car != nullptr && dataWebSocket != nullptr) {
            String imuData = car->getCarData(0.1); // Get IMU and ultrasonic data
            dataWebSocket->broadcastTXT(imuData);  // Send data via WebSocket
        } else {
            Serial.println("Error: Car or Data WebSocket reference is null.");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Send data every 50 ms
    }
}

// Task: Handle car operations
void carTask(void *parameter) {
    Car *car = (Car *)parameter;
    while (true) {
        char command;
        if (commandQueue != nullptr && xQueueReceive(commandQueue, &command, portMAX_DELAY)) {
            if (car != nullptr) {
                car->drive(command); // Process the command
            } else {
                Serial.println("Error: Car reference is null.");
            }
        } else {
            Serial.println("Error: Command queue is null or failed to receive.");
        }
    }
}

// Initialize FreeRTOS tasks
void initTasks(Car *car, WebSocketsServer *commandWebSocket, WebSocketsServer *dataWebSocket) {
    // Create the command queue
    commandQueue = xQueueCreate(10, sizeof(char));
    if (commandQueue == nullptr) {
        Serial.println("Error: Failed to create command queue.");
        return;
    }

    // Create the tasks
    if (xTaskCreate(receiveCommandTask, "ReceiveCommandTask", 4096, commandWebSocket, 1, NULL) != pdPASS) {
        Serial.println("Error: Failed to create ReceiveCommandTask.");
    }

    struct TaskParams {
        Car *car;
        WebSocketsServer *dataWebSocket;
    };
    static TaskParams params = {car, dataWebSocket};

    if (xTaskCreate(sendDataTask, "SendDataTask", 4096, &params, 1, NULL) != pdPASS) {
        Serial.println("Error: Failed to create SendDataTask.");
    }

    if (xTaskCreate(carTask, "CarTask", 4096, car, 1, NULL) != pdPASS) {
        Serial.println("Error: Failed to create CarTask.");
    }
}