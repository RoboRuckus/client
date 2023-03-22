#include <Robot.hpp>
// Don't include anything above this line

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <FastLED.h>
#include <MPU6050_tockn.h>
#include <ESP32Servo.h>

namespace RoboRuckus {

    /// @brief Defines the robot class
    class Robot {
        public:

            /// @brief Creates robot object
            Robot(){};
            
            void begin(bool calibrateMode) {
                Serial.println("Initializing robot");
                inCalibrateMode = calibrateMode;
                
                // Start LEDs
                FastLED.addLeds<WS2812B, 13, GRB>(leds, 25);

                // Don't use accessory header for LEDs if in calibrate mode
                if (!inCalibrateMode) {
                    FastLED.addLeds<WS2812B, FRONT_LEDS_PIN, GRB>(frontLED, NUM_FRONT_LEDS);
                }

                FastLED.   
                FastLED.setBrightness(10);
                FastLED.clear();
                delay(50);
                FastLED.show();

                // Configure sensor pin if in calibrate mode, just in case
                if (inCalibrateMode) {
                    pinMode(SENSOR_PIN, INPUT);
                }

                // Start IMU
                mpu6050.begin();
                // begin() is called a second time to avoid a bug where the gyro counts twice the angle expected
                delay(5);
                mpu6050.begin();
                // Gyro will be calibrated later when in calibration mode
                if (!inCalibrateMode){
                    calibrateGyro();
                }

                // Start servos
                // Allow allocation of all timers
                ESP32PWM::allocateTimer(0);
                ESP32PWM::allocateTimer(1);
                ESP32PWM::allocateTimer(2);
                ESP32PWM::allocateTimer(3);
                // Attach servos
                left.setPeriodHertz(50);  // Standard 50hz servo
                right.setPeriodHertz(50); 
                left.attach(LEFT_SERVO_PIN, 500, 2500); // pin, min pulse, max pulse
                right.attach(RIGHT_SERVO_PIN, 500, 2500);

                // Set default settings values
                robotName = "Test%20Bot";
                leftForwardSpeed = 165;
                rightForwardSpeed = 15;
                leftBackwardSpeed = 15;
                rightBackwardSpeed = 165;
                linearTime = 1200;
                drift = 5;
                driftBoost = 10;
                turnAngle = 90;
                robotColor = 0;

                loadSettingsFromFile();
            }

            void playerAssigned(int player) {}

            void turn (int direction, int magnitude) {}

            void driveForward(int magnitude) {}
            /// @brief 
            /// @param Magnitude 
            void driveBackward(int Magnitude) {}
            
            /// @brief 
            /// @param direction 
            /// @param magnitude 
            void lateralMove(int direction, int magnitude) {}

            /// @brief Calibrates the gyroscope offsets
            void calibrateGyro() {
                mpu6050.calcGyroOffsets(true, 2000, 1000);
                // Fix mpu6050 printing without an ending newline character
                Serial.println("");
            }

            /// @brief Called when a robot has new settings, should update robot movement parameter variables with new values.
            /// @param settings A comma separated list of new parameters.
            /// @param commit True when the settings should be saved to persistent storage like EEPROM.
            void saveSettings(String settings, bool commit) {
                if (settings != "") {
                    // Prase settings string
                    settings.trim();
                    String settings_clean = settings.substring(0, settings.length() - 1);

                    /* 
                    * Assign new values.
                    * The motor speed is set via the Servo library, which uses a value between 0-180.
                    * At the middle value, 90, the motors don't turn, above that value the motors
                    * turn one direction increasing in speed the further from 90. Below 90 is the same
                    * only the wheels turn the opposite direction. To let the user set a value from 0-90
                    * with 90 being the fastest, you need to subtract each value from 90 when receiving 
                    * or sending the value if the desired setting is below 90. For wheels that need to
                    * have values above 90, 90 must be added.
                    */
                    robotName = getValue(settings_clean, ',', 0);
                    leftForwardSpeed = getValue(settings_clean, ',', 1).toInt() + 90;
                    rightForwardSpeed = 90 - getValue(settings_clean, ',', 2).toInt();
                    leftBackwardSpeed = 90 - getValue(settings_clean, ',', 3).toInt();
                    rightBackwardSpeed = getValue(settings_clean, ',', 4).toInt() + 90;
                    linearTime = getValue(settings_clean, ',', 5).toInt();
                    drift = getValue(settings_clean, ',', 6).toInt();
                    driftBoost = getValue(settings_clean, ',', 7).toInt();
                    turnAngle = getValue(settings_clean, ',', 8).toFloat();
                    robotColor = getValue(settings_clean, ',', 9).toInt();
                } else {
                    // Save current settings
                    settings = robotName + "," + String(leftForwardSpeed - 90) + "," + String(90 - rightForwardSpeed) + "," + String(90 - leftBackwardSpeed) + "," + String(rightBackwardSpeed - 90) + "," + String(linearTime) + "," + String(drift) + "," + String (driftBoost) + "," + String(turnAngle) + "," + String(robotColor) + ":";
                }
                
                // Save values to file
                if (commit)
                {
                    Serial.println("Saving config: " + settings);
                    File configFile = SPIFFS.open("/robot_config.txt", "w");
                    if (!configFile) {
                    Serial.println("Failed to open config file for writing");
                    }
                    else
                    {
                    configFile.print(settings);
                    configFile.close();     
                    }
                }
            }

            /// @brief Called when the robot needs to load its settings.
            /// @return A JSON object of all the modifiable movement parameters.
            String loadSettingsFromRAM() {
                String content = "{\"name\": \"";
                content += robotName;
                content += "\", \"controls\": [{ \"name\": \"leftForwardSpeed\", \"displayname\": \"Left Forward Speed\", \"min\": 0, \"max\": 90, \"increment\": 1, \"current\": ";
                content += String(leftForwardSpeed - 90);
                content += "},{ \"name\": \"rightForwardSpeed\", \"displayname\": \"Right Forward Speed\", \"min\": 0, \"max\": 90, \"increment\": 1, \"current\": ";
                content += String(90 - rightForwardSpeed);
                content += "},{ \"name\": \"leftBackwardSpeed\", \"displayname\": \"Left Backward Speed\", \"min\": 0, \"max\": 90, \"increment\": 1, \"current\": ";
                content += String(90 - leftBackwardSpeed);
                content += "},{ \"name\": \"rightBackwardSpeed\", \"displayname\": \"Right Backward Speed\", \"min\": 0, \"max\": 90, \"increment\": 1, \"current\": ";
                content += String(rightBackwardSpeed - 90);
                content += "},{ \"name\": \"linearTime\", \"displayname\": \"Linear Movement Time\", \"min\": 500, \"max\": 2000, \"increment\": 10, \"current\": ";
                content += String(linearTime);
                content += "},{ \"name\": \"drift\", \"displayname\": \"Drift Limit\", \"min\": 0, \"max\": 15, \"increment\": 1, \"current\": ";
                content += String(drift);
                content += "},{ \"name\": \"driftBoost\", \"displayname\": \"Drift Boost\", \"min\": 0, \"max\": 20, \"increment\": 1, \"current\": ";
                content += String(driftBoost);
                content += "},{ \"name\": \"turnAngle\", \"displayname\": \"Turn Angle\", \"min\": 60, \"max\": 120, \"increment\": 0.5, \"current\": ";
                content += String(turnAngle);
                content += "},{ \"name\": \"robotColor\", \"displayname\": \"Color\", \"min\": 0, \"max\": 7, \"increment\": 1, \"current\": ";
                content += String(robotColor);
                content +="}]}";
                return content;
            }

        private:
            CRGBArray<25> leds;
            CRGBArray<NUM_FRONT_LEDS> frontLED;
            // Temperature sensor not currently used
            // Generic_LM75 Tmp75Sensor;
            MPU6050 mpu6050 = MPU6050(Wire);
            // Buzzer not currently used
            // int BUZZER_PIN = 33;
            // int BUZZER_CHANNEL = 0;
            Servo left, right;

            // Robot movement parameters 
            int leftForwardSpeed, rightForwardSpeed, rightBackwardSpeed, leftBackwardSpeed, linearTime, driftBoost, drift;
            float turnAngle;

            /// @brief Image maps for display. Binary maps for each row, 1 on, 0 off.
            uint8_t image_maps[15][5] = {
            {B00000,B00000,B00000,B00000,B00000}, // Clear
            {B00100,B01100,B00100,B00100,B01110}, // 1
            {B11100,B00010,B01100,B10000,B11110}, // 2
            {B11110,B00010,B00100,B10010,B01100}, // 3
            {B00110,B01010,B10010,B11111,B00010}, // 4
            {B11111,B10000,B11110,B00001,B11110}, // 5
            {B00010,B00100,B01110,B10001,B01110}, // 6
            {B11111,B00010,B00100,B01000,B10000}, // 7
            {B01110,B10001,B01110,B10001,B01110}, // 8
            {B01110,B10001,B01110,B00100,B01000}, // 9
            {B01010,B01010,B00000,B10001,B01110}, // Happy
            {B01010,B01010,B00000,B01110,B10001}, // Sad
            {B01010,B00000,B00100,B01010,B00100}, // Surprised
            {B01100,B11100,B01111,B01110,B00000}, // Duck
            {B00000,B00001,B00010,B10100,B01000}  // Check
            };

            /// @brief Color maps for display
            CRGB color_map[8] = {
            CRGB(255, 0, 0),    // Red
            CRGB(0, 255, 0),    // Green
            CRGB(0, 0, 255),    // Blue
            CRGB(255, 128, 0),  // Yellow
            CRGB(255, 0, 196),  // Purple
            CRGB(255, 96, 0),   // Orange
            CRGB(0, 196, 255),  // Cyan
            CRGB(144, 144, 128) // White
            };
            
            /// @brief Takes a string and splits it by a deliminator and returns substring at desired index
            /// @param data  The string to search over
            /// @param separator the separator (delimiter) to look for
            /// @param index The substring index to return
            /// @return The found substring
            String getValue(String data, char separator, int index) {
            int found = 0;
            // Create array to hold indices that bound the substring found between delimiters
            int strIndex[] = {0, -1};
            int maxIndex = data.length()-1;

            // Look for substring between delimiters
            for(int i=0; i <= maxIndex && found <= index; i++) {
                if(data.charAt(i) == separator || i == maxIndex) 
                {
                    found++;
                    strIndex[0] = strIndex[1] + 1;
                    strIndex[1] = (i == maxIndex) ? i + 1 : i;
                }
            }
            // If a substring was found at the desired index return it, else return an empty string
            return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
            }

            /// @brief Displays an image on the LED screen. Adapted from https://www.elecrow.com/wiki/index.php?title=Mbits#Use_with_Mbits-RGB_Matrix
            /// @param dat The binary encoding of LED statuses per row
            /// @param myRGBcolor The color to use
            void Display(uint8_t dat[], CRGB myRGBcolor) {
            for (int c = 0; c < 5; c++)
            {
                for (int r = 0; r < 5; r++)
                {
                if (bitRead(dat[c], r))
                {
                    // Set the LED color at the given column and row
                    leds[c * 5 + 4-r] = myRGBcolor;
                }
                }
            }
            FastLED.show();
            }

            /// @brief Show color on front LEDs
            /// @param myRGBcolor The color to use
            void showColor(CRGB myRGBcolor) {
            for (int i = 0; i < frontLED.len; i++)
            {
                frontLED[i] = myRGBcolor;
            }
            FastLED.show();
            }
            
            /// @brief Reads the line following state, returns a byte encoded with the reading
            /// @return 3 = both active (● ●)
            /// 2 = left active (● ◌)
            /// 1 = right active (◌ ●)
            /// 0 = neither active (◌ ◌)
            /// 4 = error with reading
            uint8_t trackingStatus() {
            int sensorValue = analogRead(SENSOR_PIN);
            Serial.print("Line sensor value: " + String(sensorValue) + " ");
            if (sensorValue < 200) {
                Serial.println("(● ●)");
                return 3;
            }
            else if (sensorValue >= 200 && sensorValue < 700) {
                Serial.println("(● ◌)");
                return 2;
            }
            else if (sensorValue >= 700 && sensorValue < 1200) {
                Serial.println("(◌ ●)");
                return 1;
            } else if (sensorValue >= 1200 && sensorValue < 1800) {
                Serial.println("(◌ ◌)");
                return 0;
            } else {
                Serial.println("Error");
                return 4;
            }
            }

            /// @brief Helper class for getting angle robot has turned.
            /// Used because the MPU6050 library gyroAngle can't
            /// be reset without calling begin() method again.    
            class GyroHelper {
            public:
                /// @brief  Initialize the helper using a the specific sensor
                /// @param Gyro The senor to use
                GyroHelper(MPU6050 &Gyro) : gyro(Gyro) {
                previousTime = millis();
                gyro.update();        
                }

                /// @brief Get the angle turned since last called
                /// @return Float of the degrees turned since last call
                float getAngle() {
                gyro.update();
                // Get rotation in deg/s
                float gyroX = gyro.getGyroX();
                // Calculate time since last call in seconds
                interval = (millis() - previousTime) * 0.001;
                previousTime = millis();
                // Calculate total degrees turned so far
                totalAngle += gyroX * interval;
                // Return total angle turned
                return totalAngle;
                }

            private:
                MPU6050 &gyro;
                long previousTime;
                float interval = 0;
                float totalAngle = 0;
            };

            /// @brief Load saved setings from file system
            void loadSettingsFromFile() {
                // Attempt to load saved values
                if (SPIFFS.begin()) {
                    Serial.println("Mounted file system");
                    if (SPIFFS.exists("/robot_config.txt")) 
                    {
                    // File exists, reading and loading
                    Serial.println("Reading robot config file");
                    File configFile = SPIFFS.open("/robot_config.txt", "r");
                    if (configFile) 
                    {
                        Serial.println("Opened robot config file, loading settings");
                        String settings = "";
                        // Read file
                        while(configFile.available())
                        {
                        settings += configFile.readString();
                        }
                        configFile.close();
                        Serial.println("Settings loaded: " + settings);
                        // Apply loaded settings
                        saveSettings(settings, false);            
                    } 
                    else 
                    {
                        Serial.println("Robot config file could not be opened");
                    }
                    }
                    else 
                    {
                    Serial.println("Robot config file not found");
                    }
                } 
                else 
                {
                    Serial.println("Failed to mount FS");
                    // Clean FS
                    Serial.println("Formatting SPIFFS, this will take a while, please wait...");
                    SPIFFS.format();
                    Serial.println("Done, rebooting");
                    ESP.restart();
                }
            }
    }
    }
