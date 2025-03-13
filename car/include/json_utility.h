#ifndef JSON_UTILITY_H
#define JSON_UTILITY_H

#include <ArduinoJson.h>
#include <Arduino.h>

class JsonUtility {
    public:
        static String createJson(std::initializer_list<std::pair<String, String>> stringData = {},
                                 std::initializer_list<std::pair<String, int>> intData = {},
                                 std::initializer_list<std::pair<String, float>> floatData = {},
                                 std::initializer_list<std::pair<String, bool>> boolData = {});
    };
    

#endif // JSON_UTILITY_H
