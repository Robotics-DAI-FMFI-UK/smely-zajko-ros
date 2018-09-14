#include "LocalMap.h"

// map constants
const int gridSize = 10;
const int gridWidth = 120;
const int gridHeight = 120;
const int mapWidth = gridWidth * gridSize;
const int mapHeight = gridHeight * gridSize;

// robot constants
const double wheelCircumference = 47.8779;
const int ticksPerRotation = 36;
const double wheelDistance = 52;
const int hokuyoOffset = 15;
const int sensorCutoff = 600;

// calculation constants
const int minUpdateDist = 10;
const double straightMovementThreshold = 0.07;

// calculation utilities
int LocalMap::clamp(int val, int max) {
    val = val % max;
    return val < 0? val + max : val;
}
double LocalMap::rescale(double val, double oldMax, double newMax) {
    return val * (newMax / oldMax);
}
int LocalMap::clampGridX(int x) {return clamp(x, gridWidth);}
int LocalMap::clampGridY(int y) {return clamp(y, gridHeight);}
int LocalMap::clampGuiX(int x) {return clamp(x, this->guiWidth);}
int LocalMap::clampGuiY(int y) {return clamp(y, this->guiHeight);}
int LocalMap::map2gridX(double x) {return clampGridX(round(rescale(x, mapWidth, gridWidth)));}
int LocalMap::map2gridY(double y) {return clampGridY(round(rescale(y, mapHeight, gridHeight)));}
int LocalMap::map2guiX(double x) {return clampGuiX(round(rescale(x, mapWidth, guiWidth)));}
int LocalMap::map2guiY(double y) {return clampGuiY(guiHeight - round(rescale(y, mapHeight, guiHeight)));}

LocalMap::LocalMap(int guiWidth, int guiHeight, ros::Publisher publisher) {
    srand(time(NULL));
    this->guiWidth = guiWidth;
    this->guiHeight = guiHeight;
    matrix = new double* [gridWidth];
    for (int i = 0; i < gridWidth; i++) {
        matrix[i] = new double[gridHeight];
        for (int j = 0; j < gridHeight; j++) {
            matrix[i][j] = 0.0;//((double) i + j) / (gridHeight + gridWidth);
        }
    }
    posX = 0;
    posY = 0;
    angle = 0;
    prevTicksL = 0;
    prevTicksR = 0;
    lastTicksL = 0;
    lastTicksR = 0;
    bestHeading = 0;
    compassHeading = 0;
    currWayHeading = 0;
    nextWayHeading = 0;

    this->publisher = publisher;
}

void LocalMap::doUpdate() {
    updateRobotPosition_(lastTicksL, lastTicksR, true);
}

void LocalMap::updateRobotPosition(long L, long R) {
    lastTicksL = L;
    lastTicksR = R;
    updateRobotPosition_(L, R, false);
}

void LocalMap::updateRobotPosition_(long L, long R, bool force) {
    L = -L;
    R = -R;

    double dL = wheelCircumference * (prevTicksL - L) / ticksPerRotation;
    double dR = wheelCircumference * (prevTicksR - R) / ticksPerRotation;
    
    if (!force && (fabs(dL) + fabs(dR) < minUpdateDist)) return; // don't update on small changes
    
    prevTicksL = L;
    prevTicksR = R;
    
    double d = (dL + dR) / 2;
    double turnRatio = 2;
    if ((fabs(dL) >= 1.0) || (fabs(dR) >= 1.0)) {
        turnRatio = dL / dR;
    }
    double newX, newY, newAngle;
    if (fabs(turnRatio - 1.0) < straightMovementThreshold) { // straight
        newX = posX + d * sin(angle);
        newY = posY + d * cos(angle);
        newAngle = angle;
    } else if ((dL * dR < 0) && (fabs(fabs(dL) - fabs(dR)) < 0.3)) { // rotate along center
        newAngle = angle + dL / (2.0 * wheelDistance);
    } else if (dL != dR) { // circular trajectory
        int centerRight = 1;
        double r1;
        if (fabs(dR) > fabs(dL)) {
            centerRight = -1;
            r1 = wheelDistance * dL / (dR - dL);
        } else {
            r1 = wheelDistance * dR / (dL - dR);
        }
        double r = r1 + wheelDistance / 2.0;
        double beta = d / r;
        double cX = posX + r * sin(angle + centerRight * pi / 2.0);
        double cY = posY + r * cos(angle + centerRight * pi / 2.0);
        newX = cX + r * sin(angle - centerRight * pi / 2.0 + centerRight * beta);
        newY = cY + r * cos(angle - centerRight * pi / 2.0 + centerRight * beta);
        newAngle = angle + beta * centerRight;
    } else { // not moving
        newX = posX;
        newY = posY;
        newAngle = angle;
    }
    // normalize new position data into map
    newX = newX > mapWidth? newX - mapWidth : newX;
    posX = newX < 0? newX + mapWidth : newX;
    newY = newY > mapHeight? newY - mapHeight : newY;
    posY = newY < 0? newY + mapHeight : newY;
    newAngle = newAngle > 2*pi? newAngle - 2*pi : newAngle;
    angle = newAngle < 0? newAngle + 2*pi : newAngle;

    applyHokuyoData();
    applyImage();
    applyCompassHeading();
    findBestHeading();

    std_msgs::Float64 msg;
    msg.data = bestHeading;
    publisher.publish(msg);
}

cv::Scalar LocalMap::getMatrixColor(int x, int y) {
    double val = matrix[clampGridX(x)][clampGridY(y)];
    return cv::Scalar((1-val)*255 , 255 - 100*val, (1-val)*255);
}

cv::Mat LocalMap::getGui() {
    cv::Mat result(guiWidth, guiHeight, CV_8UC3, cv::Scalar(255, 255, 255));

    int guiShiftX = guiWidth / 2 - map2guiX(posX);
    int guiShiftY = guiHeight / 2 - map2guiY(posY);

    // draw matrix
    for (int x = 0; x < gridWidth; x++) {
        for (int y = 0; y < gridHeight; y++) {
            cv::Point a(clampGuiX(map2guiX(x * gridSize) + guiShiftX), clampGuiY(map2guiY(y * gridSize) + guiShiftY));
            cv::Point b(clampGuiX(map2guiX((x+1) * gridSize - 1) + guiShiftX), clampGuiY(map2guiY((y+1) * gridSize - 1) + guiShiftY));
            if (abs(a.x - b.x) <= map2guiX(gridSize) && abs(a.y - b.y) <= map2guiX(gridSize)) {
                cv::rectangle(result, a, b, getMatrixColor(x, y), CV_FILLED);
            }
        }
    }

    // draw robot
    cv::Point a(clampGuiX(map2guiX(posX) + guiShiftX), clampGuiY(map2guiY(posY) + guiShiftY));
    cv::Point b = a + cv::Point(20 * sin(angle), -20 * cos(angle));
    cv::arrowedLine(result, a, b, cv::Scalar(0, 255, 0), 2, CV_AA, 0, 0.2);

    // draw target directions
    if (wayEndDistance < sensorCutoff) { // draw next heading when close
        b = a + cv::Point(10 * sin(angle - compassHeading + currWayHeading), -10 * cos(angle - compassHeading + currWayHeading));
        cv::Point c = b + cv::Point(10 * sin(angle - compassHeading + nextWayHeading), -10 * cos(angle - compassHeading + nextWayHeading));
        cv::line(result, a, b, cv::Scalar(255, 0, 0));
        cv::arrowedLine(result, b, c, cv::Scalar(255, 0, 0));
    } else {
        b = a + cv::Point(20 * sin(angle - compassHeading + currWayHeading), -20 * cos(angle - compassHeading + currWayHeading));
        cv::arrowedLine(result, a, b, cv::Scalar(255, 0, 0));
    }

    // draw north
    b = a + cv::Point(30 * sin(angle - compassHeading), -30 * cos(angle - compassHeading));
    cv::arrowedLine(result, a, b, cv::Scalar(0, 0, 255));

    // draw best heading
    b = a + cv::Point(100 * sin(bestHeading), -100 * cos(bestHeading));
    cv::arrowedLine(result, a, b, cv::Scalar(0, 0, 0), 2, CV_AA, 0, 0.2);

    return result;
}

RobotPos* LocalMap::getPos() {
    RobotPos* pos = new RobotPos();
    pos->x = posX;
    pos->y = posY;
    pos->angle = 180 * angle / pi;
    return pos;
}

void LocalMap::setHokuyoData(int* rays) {
    memcpy(hokuyo, rays, sizeof(int) * 1081);
    validHokuyo = true;
}

void LocalMap::applyHokuyoData() {
    if (!validHokuyo) return;

    // decay values
    for (int i = 0; i < gridWidth; i++) {
        for (int j = 0; j < gridHeight; j++) {
            matrix[i][j] *= 0.90;
        }
    }

    // sensor in map
    double sensorX = posX + hokuyoOffset * sin(angle);
    double sensorY = posY + hokuyoOffset * cos(angle);

    // iterate valid rays
    for (int i = 180; i <= 900; i++) {
        // calculate ray angle
        double rayAngle = ((-(double) i) / 4 + 135) * (pi / 180);

        // clamp ray length
        double rayLen = ((double) hokuyo[i]) / 10;
        if (rayLen > sensorCutoff) {
            continue; // skip too long rays
        }

        // ray end point
        double rayX = sensorX + rayLen * sin(angle + rayAngle);
        double rayY = sensorY + rayLen * cos(angle + rayAngle);

        // end point in grid
        int gX = map2gridX(rayX);
        int gY = map2gridY(rayY);

        // mark as obstacle (also mark 8-neighborhood)
        matrix[clampGridX(gX-1)][clampGridY(gY-1)] = (matrix[clampGridX(gX-1)][clampGridY(gY-1)] + 2) / 3;
        matrix[clampGridX(gX-1)][gY] = (matrix[clampGridX(gX-1)][gY] + 2) / 3;
        matrix[clampGridX(gX-1)][clampGridY(gY+1)] = (matrix[clampGridX(gX-1)][clampGridY(gY+1)] + 2) / 3;
        matrix[gX][clampGridY(gY-1)] = (matrix[gX][clampGridY(gY-1)] + 2) / 3;
        matrix[gX][gY] = (matrix[gX][gY] + 2) / 3;
        matrix[gX][clampGridY(gY+1)] = (matrix[gX][clampGridY(gY+1)] + 2) / 3;
        matrix[clampGridX(gX+1)][clampGridY(gY-1)] = (matrix[clampGridX(gX+1)][clampGridY(gY-1)] + 2) / 3;
        matrix[clampGridX(gX+1)][gY] = (matrix[clampGridX(gX+1)][gY] + 2) / 3;
        matrix[clampGridX(gX+1)][clampGridY(gY+1)] = (matrix[clampGridX(gX+1)][clampGridY(gY+1)] + 2) / 3;

        // mark points on ray as empty
        for (int j = 0; j < rayLen; j += 10) {
            double p = ((double) j) / rayLen;
            double pX = sensorX + p * (rayX - sensorX);
            double pY = sensorY + p * (rayY - sensorY);
            int gX = map2gridX(pX);
            int gY = map2gridY(pY);
            // don't overwrite obstacles found by previous rays from this batch
            if (matrix[gX][gY] < 1.0) matrix[gX][gY] = matrix[gX][gY] / 3;
        }
    }
}

void LocalMap::setGlobalMapData(double currHeading, double nextHeading, double distance) {
    currWayHeading = currHeading;
    nextWayHeading = nextHeading;
    wayEndDistance = distance * 100000;
}

void LocalMap::setCompassHeading(double heading) {
    compassHeading_ = heading;
}

void LocalMap::applyCompassHeading() {
    compassHeading = compassHeading_;
}

void LocalMap::setImageData(unsigned char* data) {
    for (int i = 0; i < 3600; i++) {
        cameraData[i%60][i/60] = data[i];
    }
    validImage = true;
}

void LocalMap::applyImage() {
    if (!validImage) return;
    double r11 = cos(angle);
    double r12 = -sin(angle);
    double r21 = sin(angle);
    double r22 = cos(angle);

    bool** done = new bool* [gridWidth];
    for (int i = 0; i < gridWidth; i++) {
        done[i] = new bool[gridHeight];
        for (int j = 0; j < gridHeight; j++) {
            done[i][j] = false;
        }
    }

    for (int x = -30; x < 30; x++) {
        for (int y = 0; y < 60; y++) {
            // rotate
            double rX = -((double) x*10) * r11 - ((double) y*10) * r12;
            double rY = ((double) x*10) * r21 + ((double) y*10) * r22;
            // move to robot
            int gX = map2gridX(rX + posX);
            int gY = map2gridY(rY + posY);
            if (!done[gX][gY]) {
                done[gX][gY] = true;
                double val = (double) cameraData[30 - x][y] / 255.0;
                if (val > 0.0001) {
                    matrix[gX][gY] = (matrix[gX][gY] + 2 * val) / 3;
                }
            }
        }
    }
}

double calcDist(double x1, double y1, double x2, double y2, double x, double y) {
    double a, b, c;
    if (y1 == 0 && y2 == 0) return y; // x axis
    if (x1 == 0 && x2 == 0) return x; // y axis
    if (x1 == 0 && y1 == 0) { // p1 is origin
        a = 1;
        b = -x2 / y2;
        c = 0;
    } else if (x2 == 0 && y2 == 0) { // p2 is origin
        a = 1;
        b = -x1 / y1;
        c = 0;
    } else if (x1 != 0 && y1 != 0 && x2 != 0 && y2 != 0 && (x1 / x2 - y1 / y2) < 0.0001) { // passes through origin
        a = 1;
        b = -x1 / y1;
        c = 0;
    } else { // doesn't pass through origin
        c = 1;
        if (x1 == 0) {
            b = -1 / y1;
            a = (y2 / y1 - 1) / x2;
        } else {
            b = (x2 - x1) / (x1 * y2 - x2 * y1);
            a = - (1 + b * y1) / x1;
        }
    }
    return fabs(a * x + b * y + c) / sqrt(a * a + b * b);
}

void LocalMap::findBestHeading() {
    double scores[360];
    // check directions in 1 degree intervals
    for (int i = 0; i < 360; i++) {
        double dir = ((double) i) * (pi / 180);

        // score of path (rectangle)
        scores[i] = 0;

        // rectangle width will be 1.2*wheelDistance
        // rectangle length will be sensorCutoff
        double endX = posX + sensorCutoff * sin(dir);
        double endY = posY + sensorCutoff * cos(dir);
        /* c--end--d
           |   |   |
           a--pos--b */
        double aX = posX + 0.6 * wheelDistance * sin(dir + (pi / 2));
        double aY = posY + 0.6 * wheelDistance * cos(dir + (pi / 2));
        double bX = posX + 0.6 * wheelDistance * sin(dir - (pi / 2));
        double bY = posY + 0.6 * wheelDistance * cos(dir - (pi / 2));
        double cX = endX + 0.6 * wheelDistance * sin(dir + (pi / 2));
        double cY = endY + 0.6 * wheelDistance * cos(dir + (pi / 2));
        double dX = endX + 0.6 * wheelDistance * sin(dir - (pi / 2));
        double dY = endY + 0.6 * wheelDistance * cos(dir - (pi / 2));

        // calculate bounding box
        int left = map2gridX(std::min({aX, bX, cX, dX})) - gridWidth;
        int right = map2gridX(std::max({aX, bX, cX, dX}));
        int bot = map2gridY(std::min({aY, bY, cY, dY})) - gridHeight;
        int top = map2gridY(std::max({aY, bY, cY, dY}));

        // iterate over bounding box
        for (int x = left; x <= right; x++) {
            double pX = gridSize * x;
            for (int y = bot; y <= top; y++) {
                double pY = gridSize * y;
                double lineDist = calcDist(posX, posY, endX, endY, pX, pY);
                double pointDist = sqrt(pow(posX - pX, 2) + pow(posY - pY, 2));
                if (lineDist < 0.6 * wheelDistance && pointDist < sensorCutoff) {
                    // contribution to score weighted by distance
                    scores[i] += (1-matrix[clampGridX(x)][clampGridY(y)]) * (1 - pointDist);
                }
            }
        }

        // reduce score of paths in wrong direction
        double target;
        if (wayEndDistance < sensorCutoff) {
            target = angle - compassHeading + currWayHeading;
        } else {
            target = angle - compassHeading + nextWayHeading;
        }
        double diff = target - dir;
        diff = fmod(fmod((diff + pi), 2*pi) + 2*pi, 2*pi) - pi;
        scores[i] *= 1 - (diff / pi);
    }
    int best = 0;
    // find path with highest score
    for (int i = 0; i < 360; i++) {
        if (scores[i] > scores[best]) best = i;
    }
    bestHeading = best * (pi / 180);
}

double LocalMap::getHeading() {
    return bestHeading;
}