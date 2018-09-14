#include <stdlib.h>
#include <cstdio>
#include "LocalMap.h"

int main() {
    LocalMap* lm = new LocalMap(500, 500);
    RobotPos* pos = lm->getPos();
    printf("%f %f %f\n", pos->x, pos->y, pos->angle);
    lm->updateRobotPosition(40, 117);
    pos = lm->getPos();
    printf("%f %f %f\n", pos->x, pos->y, pos->angle);
    lm->updateRobotPosition(315, 315);
    pos = lm->getPos();
    printf("%f %f %f\n", pos->x, pos->y, pos->angle);
    return 0;
}