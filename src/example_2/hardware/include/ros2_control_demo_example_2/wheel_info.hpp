#include <string>

struct WheelInfo {
    std::string name; // Name as passed in yaml file.
    double command; // Current commanded position.
    double velocity; // Current velocity command.
    double position; // Dummy value. TODO: Either put encoders in or remove. -njreichert
};
