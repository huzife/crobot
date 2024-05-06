#ifndef CROBOT_ROBOT_BASE_H
#define CROBOT_ROBOT_BASE_H

namespace crobot {

enum class Robot_Base_Type {
    ROBOT_BASE_2WD,
    ROBOT_BASE_3WO,
    ROBOT_BASE_4WD,
    ROBOT_BASE_4MEC
};

struct Robot_Base_2WD_Param {
    float radius;
    float separation;
};

struct Robot_Base_3WO_Param {
    float radius;
    float distance;
};

struct Robot_Base_4WD_Param {
    float radius;
    float separation;
};

struct Robot_Base_4MEC_Param {
    float radius;
    float distance_x;
    float distance_y;
};

}

#endif // CROBOT_ROBOT_BASE_H
