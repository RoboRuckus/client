/*
 * This file is licensed under the MIT Lesser General Public License Copyright (c) 2023 RoboRuckus Group
 *
 * Template for RoboRuckus game robot based on the 
 * ESP32 based Mbits board via Arduino platform.
 * https://www.elecrow.com/mbits.html
 *
 * This code is intended to work with the Ring:but Car v2
 * https://www.elecfreaks.com/ring-bit-car-v2-for-micro-bit.html
 * 
 * External libraries needed:
 * FastLED https://github.com/FastLED/FastLED
 * Temperature_LM75_Derived: https://github.com/jeremycole/Temperature_LM75_Derived <-- Not currently used
 * Tone32: https://github.com/lbernstone/Tone32 <-- Not currently used
 * MPU6050_tockn: https://github.com/Tockn/MPU6050_tockn
 * AsyncTCP: https://github.com/esphome/AsyncTCP
 * ESPAsyncWebServer: https://github.com/esphome/ESPAsyncWebServer
 * ESPAsyncWiFiManager: https://github.com/alanswx/ESPAsyncWiFiManager
 * ArduinoJson: https://github.com/bblanchon/ArduinoJson
 * ESP32Servo https://github.com/madhephaestus/ESP32Servo
 *
 * Contributors: Sam Groveman, Josh Buker
 */

#include <Arduino.h>
#include <Update.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <SPIFFS.h>
#include <Robot.hpp>

// Global function declarations (there's probably a better way to handle this)
void updateServerStart();
void updateServerStop();
void onUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

/// @brief Defines the command logic class
class CommandLogic {
    public:
        void begin(WifiCommunication* Wifi, RingBitRobot* Bot) {
            wifi = Wifi;
            bot = Bot;
        }

        /// @brief 
        /// @param message 
        void parseMessage(String message) {
            // Check if in setup mode
            if (!inSetupMode)
            {
                // Respond
                wifi->sendResponse(F("OK"));
            }
            // Check if game is already started
            if (started) 
            {
                // Parse movement instruction
                uint8_t movement = message[0] - '0';   // Convert char to int
                uint8_t magnitude = message[1] - '0';  // Convert char to int
                uint8_t lateralMove = message[2] - '0';  // Convert char to int
                if (lateralMove == 2)
                {
                    // Bot received reset command
                    started = false;
                    // Restart the update server
                    updateServerStart();
                    bot->reset();
                }
                else
                {
                    // Bot received move order
                    executeMove(movement, magnitude, lateralMove);
                }
            }
            // Check if in setup mode
            else if (inSetupMode)
            {
                setupMode(message);            
            }
            else
            {
                // Prase instruction, has the format instruction:message
                int instruction = message.substring(0, message.indexOf(':')).toInt();
                // Bot has been assigned to player
                if (instruction == 0)
                {
                // Parse message
                message = message.substring(message.indexOf(':') + 1);
                // Get assigned player
                bot->playerAssigned(message[0] - '0');
                // Get assigned bot number
                bot->botNum = message.substring(1, message.indexOf('\n'));
                // Stop the update server while robot is in the game and ready to play
                updateServerStop();
                started = true;
                }
                else if (instruction == 1)
                {
                // Enter setup mode
                inSetupMode = true;
                bot->showImage(Robot::images::Duck, (Robot::colors)bot->robotColor);
                }
            }
        }
    private:
        WifiCommunication* wifi;
        RingBitRobot* bot;
        bool inSetupMode = false;
        void setupMode() {}
}

class WifiCommunication {
    public:
        WiFiCommunication(){}

        /// @brief Initialize the WifiCommunication instance
        /// @param serverIP 
        /// @param serverPort 
        /// @param bot 
        void begin(IPAddress serverIP, int serverPort, Robot* bot) {}

        /// @brief Establishes the wifi communication
        /// @return 
        bool connect() {}

        /// @brief Send a new message to the game server
        /// @param message 
        /// @return 
        String sendMessage(String message) {}

        /// @brief Send the response for when we have received a message
        /// @param response 
        void sendResponse(String response) {
            client.print(response);
        }

        /// @brief Getter for bot IP address
        /// @return 
        IPAddress botIP() {
            return clientIP;
        }
    private:
        IPAddress clientIP;
        #define END_TRANSMISSION F("AK\n")
}

/// @brief Defines the communication class
class Communication {
    public:
        Communication(){};
        virtual void begin(CommandLogic* logic) {
            command = logic;
        }
        virtual void startup() = 0;
        virtual void loop() {
            customLogicHere();
            command->parseMessage(receiveMessage())
        }
        virtual bool messageWaiting();
        virtual String receiveMessage() = 0;
        virtual void sendMessage(String message) = 0;
    private:
        CommandLogic* command;
}

void loop() {
    loop();
}