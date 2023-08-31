#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

int main() {
    std::ifstream inputFile("../src/command.json");
    if (!inputFile.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        return 1;
    }

    Json::Value jsonData;
    Json::CharReaderBuilder reader;
    JSONCPP_STRING errs;
    Json::parseFromStream(reader, inputFile, &jsonData, &errs);

    for (const auto& item : jsonData) {
        std::cout << "Host: " << item["host"].asString() << std::endl;
        std::cout << "IP: " << item["ip"].asString() << std::endl;
        std::cout << "Pass: " << item["pass"].asString() << std::endl;
        
        std::cout << "Commands:" << std::endl;
        for (const auto& command : item["commands"]) {
            std::cout << "  Name: " << command["name"].asString() << std::endl;
            std::cout << "  Command: " << command["command"].asString() << std::endl;
        }
        std::cout << std::endl;
    }

    return 0;
}
