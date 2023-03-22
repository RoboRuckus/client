// Robot pin definitions
#define FRONT_LEDS_PIN  26
#define SENSOR_PIN      26
#define NUM_FRONT_LEDS  2
#define RIGHT_SERVO_PIN 25
#define LEFT_SERVO_PIN  32

namespace RoboRuckus {
  class Robot {
    // Robot variables
    int playerNumber = 0;
    String robotName, botNum;
    int robotColor = 0;
    bool inCalibrateMode = false;
    bool forwardCalComplete = false;

    // Variables for display
    enum class colors { Red, Green, Blue, Yellow, Purple, Orange, Cyan, White };
    enum class images { Clear, One, Two, Three, Four, Five, Six, Seven, Eight, Nine, Happy, Sad, Surprised, Duck, Check };
    images currentImage = images::Clear;
  }
}