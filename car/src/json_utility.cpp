#include "../include/json_utility.h"

#define JSON_BUFFER_SIZE 512  // Buffer limit for the JSON document

String JsonUtility::createJson(std::initializer_list<std::pair<String, String>> stringData,
                               std::initializer_list<std::pair<String, int>> intData,
                               std::initializer_list<std::pair<String, float>> floatData,
                               std::initializer_list<std::pair<String, bool>> boolData) {
    StaticJsonDocument<JSON_BUFFER_SIZE> jsonDoc;

    // Adding string data
    for (const auto& entry : stringData) {
        jsonDoc[entry.first] = entry.second;
    }

    // Adding integer data
    for (const auto& entry : intData) {
        jsonDoc[entry.first] = entry.second;
    }

    // Adding float data
    for (const auto& entry : floatData) {
        jsonDoc[entry.first] = entry.second;
    }

    // Adding boolean data
    for (const auto& entry : boolData) {
        jsonDoc[entry.first] = entry.second;
    }

    // Buffer to store JSON output
    String jsonString;
    if (serializeJson(jsonDoc, jsonString) == 0) {
        return "{\"error\":\"JSON serialization failed\"}";  // Fallback for serialization error
    }

    return jsonString;
}
