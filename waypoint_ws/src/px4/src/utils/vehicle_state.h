#pragma once
#include <cstdint>

struct PositionNED
{
    double north{0.0};
    double east{0.0};
    double down{0.0};
};

class VehicleState
{
public:
    PositionNED position;

    double yaw{0.0};

    PositionNED hover_position;

    bool got_position{false};

    uint8_t arming_state{0};
};