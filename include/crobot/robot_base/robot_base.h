#ifndef CROBOT_ROBOT_BASE_H
#define CROBOT_ROBOT_BASE_H

namespace crobot {

enum class Robot_Base_Type {
    ROBOT_BASE_2WD,
    ROBOT_BASE_3WO,
    ROBOT_BASE_4WD
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

}

#endif // CROBOT_ROBOT_BASE_H
