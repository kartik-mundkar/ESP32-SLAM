#include "../include/json_utility.h"

#define JSON_BUFFER_SIZE 512 // Buffer limit for the JSON document

String JsonUtility::createJson(std::initializer_list<std::pair<String, float>> floatData,
                               std::initializer_list<std::pair<String, int>> intData,
                               std::initializer_list<std::pair<String, String>> stringData,
                               std::initializer_list<std::pair<String, bool>> boolData)
{
    JsonDocument jsonDoc;

    // Adding string data
    for (const auto &entry : stringData)
    {
        jsonDoc[entry.first] = entry.second;
    }

    // Adding integer data
    for (const auto &entry : intData)
    {
        jsonDoc[entry.first] = entry.second;
    }

    // Adding float data
    for (const auto &entry : floatData)
    {
        jsonDoc[entry.first] = entry.second;
    }

    // Adding boolean data
    for (const auto &entry : boolData)
    {
        jsonDoc[entry.first] = entry.second;
    }

    // Buffer to store JSON output
    String jsonString;
    if (serializeJson(jsonDoc, jsonString) == 0)
    {
        return "{\"error\":\"JSON serialization failed\"}"; // Fallback for serialization error
    }

    return jsonString;
}


String JsonUtility::appendJson(String json, std::initializer_list<std::pair<String, float>> floatData,
                               std::initializer_list<std::pair<String, int>> intData,
                               std::initializer_list<std::pair<String, String>> stringData,
                               std::initializer_list<std::pair<String, bool>> boolData)
{
    JsonDocument jsonDoc;

    // Handle empty JSON input
    if (json.length() == 0 || json == "{}") {
        jsonDoc.to<JsonObject>();
    } else {
        DeserializationError error = deserializeJson(jsonDoc, json);
        if (error) {
            return "{\"error\":\"Invalid JSON\"}"; 
        }
    }

    // Append values to JSON
    for (const auto &entry : stringData) {
        JsonArray tempArray = jsonDoc[entry.first].to<JsonArray>();  
        tempArray.add(entry.second);
    }

    for (const auto &entry : intData) {
        JsonArray tempArray = jsonDoc[entry.first].to<JsonArray>();  
        tempArray.add(entry.second);
    }

    for (const auto &entry : floatData) {
        JsonArray tempArray = jsonDoc[entry.first].to<JsonArray>();  
        tempArray.add(entry.second);
    }

    for (const auto &entry : boolData) {
        JsonArray tempArray = jsonDoc[entry.first].to<JsonArray>();  
        tempArray.add(entry.second);
    }

    // Serialize JSON
    String updatedJson;
    if (serializeJson(jsonDoc, updatedJson) == 0) {
        return "{\"error\":\"JSON serialization failed\"}";
    }

    return updatedJson;
}
