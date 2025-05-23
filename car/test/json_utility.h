#ifndef ESP32_SLAM_JSON_UTILITY_H
#define ESP32_SLAM_JSON_UTILITY_H

#include <ArduinoJson.h>
#include <Arduino.h>

class JsonUtility
{
public:
    static String createJson(std::initializer_list<std::pair<String, float>> floatData = {},
                             std::initializer_list<std::pair<String, int>> intData = {},
                             std::initializer_list<std::pair<String, String>> stringData = {},
                             std::initializer_list<std::pair<String, bool>> boolData = {});

    static String appendJson(String json, std::initializer_list<std::pair<String, float>> floatData = {},
                             std::initializer_list<std::pair<String, int>> intData = {},
                             std::initializer_list<std::pair<String, String>> stringData = {},
                             std::initializer_list<std::pair<String, bool>> boolData = {});
};

#endif // JSON_UTILITY_H
