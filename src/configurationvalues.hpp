#ifndef CONFIGURATION_VALUES
#define CONFIGURATION_VALUES

#include <map>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class ConfigurationValues {
    public:
        ConfigurationValues(std::string cfp) noexcept
        : confMap{}
        {
            parseConfigurationFile(cfp);
            for (auto elem : confMap) {
                std::cout << elem.first << " " << elem.second << std::endl;
            }
        }
        ~ConfigurationValues() = default;

        std::map<std::string, double> confMap;

    private:
        void parseConfigurationFile(std::string confFilePath) noexcept {
            std::ifstream fileInput(confFilePath);
            std::string line;
            while (std::getline(fileInput, line)) {
                std::istringstream isLine(line);
                std::string key;
                if (std::getline(isLine, key, '=')) {
                    std::string value;
                    if (key[0] == '#') {
                        continue;
                    }
                    if (std::getline(isLine, value)) {
                        confMap[key] = std::stod(value);
                    }
                }
            }
        }


};

#endif