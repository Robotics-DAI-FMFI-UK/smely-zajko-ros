#include "LocalMap.h"

#define IGNORE_TOO_LONG_RAY 3000

#define GOING_WRONG 10000

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
const double rpLidarAngle = 10 * pi / 180; // TODO measure
const double rpLidarHeight = 70; // TODO measure

// calculation constants
const int minUpdateDist = 10;
const double straightMovementThreshold = 0.07;


static int transmitting_going_wrong = 0;

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
int LocalMap::map2gridX_(double x) {return round(rescale(x, mapWidth, gridWidth));}
int LocalMap::map2gridY_(double y) {return round(rescale(y, mapHeight, gridHeight));}
int LocalMap::map2guiX(double x) {return clampGuiX(round(rescale(x, mapWidth, guiWidth)));}
int LocalMap::map2guiY(double y) {return clampGuiY(guiHeight - round(rescale(y, mapHeight, guiHeight)));}

// 0 threshold
const double eps = 0.0001;
bool isZero(double d) {return fabs(d) < eps;}

LocalMap::LocalMap(int guiWidth, int guiHeight, ros::Publisher publisher) {
    srand(time(NULL));
    this->guiWidth = guiWidth;
    this->guiHeight = guiHeight;
    matrix = new double* [gridWidth];
    matrix_cam = new double* [gridWidth];
    for (int i = 0; i < gridWidth; i++) {
        matrix[i] = new double[gridHeight];
        matrix_cam[i] = new double[gridHeight];
        for (int j = 0; j < gridHeight; j++) {
            matrix[i][j] = 0.0;//((double) i + j) / (gridHeight + gridWidth);
            matrix_cam[i][j] = 0.0;
        }
    }

    mask_val = new double* [gridWidth];
    depth_mask_val = new double* [gridWidth];
    mask_count = new int* [gridWidth];    
    depth_mask_count = new int* [gridWidth];    
    for (int i = 0; i < gridWidth; i++) {
        mask_val[i] = new double[gridHeight];
        depth_mask_val[i] = new double[gridHeight];
        mask_count[i] = new int[gridHeight];
        depth_mask_count[i] = new int[gridHeight];
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

int intmin(int a, int b) { if (a < b) return a; else return b; }
int intmax(int a, int b) { if (a > b) return a; else return b; }

// this erases all pixels that are further or equal than 1/2 of grid size
// i.e. we keep only a circle in a distance < 1/2 of a grid
void LocalMap::eraseAustralia()
{
    for (int x = 0; x < gridWidth; x++)
      for (int y = 0; y < gridHeight; y++)
      {
        // calculate distance of pixel [x,y] - first normalize x,y distances to 0-1 range (1=grid size)
        int left = intmin(x, posX);
        int right = intmax(x, posX);
        int gridXDistance = 1 + intmin(right - left, (gridWidth - right) + left);   // 1+ to catch "borders"

        int bottom = intmin(y, posY);
        int top = intmax(y, posY);
        int gridYDistance = 1 + intmin(top - bottom, (gridHeight - top) + bottom);
        
        double gridXDist = gridXDistance / (double)gridWidth;
        double gridYDist = gridYDistance / (double)gridHeight;

        if (gridXDist * gridXDist + gridYDist * gridYDist > 0.25)  // (0.5 ^ 2)
        {
          matrix[x][y] = 0.0;
          matrix_cam[x][y] = 0.0;
        }
      } 
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
   while (newX > mapWidth) newX = newX - mapWidth;    
    while (newX < 0) newX = newX + mapWidth;    
    posX = newX;
    
    while (newY > mapHeight) newY = newY - mapHeight;    
    while (newY < 0) newY = newY + mapHeight;    
    posY = newY;

    while (newAngle > 2 * pi) newAngle -= 2 * pi;
    while (newAngle < 0) newAngle += 2 * pi;
    angle = newAngle;
 
    eraseAustralia();
    decayMap();
    applyHokuyoData();
    applyRpLidarData();
    applyDepthMap();
    applyImage();
    applyCompassHeading();
    findBestHeading();

    std_msgs::Float64 msg;
    msg.data = getHeading();
    publisher.publish(msg);
}

void LocalMap::setPose(double x, double y, double a) {
    double newX = x, newY = y, newAngle = a;

    // normalize new position data into map
    while (newX > mapWidth) newX = newX - mapWidth;    
    while (newX < 0) newX = newX + mapWidth;    
    posX = newX;
    
    while (newY > mapHeight) newY = newY - mapHeight;    
    while (newY < 0) newY = newY + mapHeight;    
    posY = newY;

    while (newAngle > 2 * pi) newAngle -= 2 * pi;
    while (newAngle < 0) newAngle += 2 * pi;
    angle = newAngle;
 
    decayMap();
    applyHokuyoData();
    applyRpLidarData();
    applyDepthMap();
    applyImage();
    applyCompassHeading();
    findBestHeading();

    std_msgs::Float64 msg;
    msg.data = getHeading();
    publisher.publish(msg);

}

// representation:
//   matrix[x][y]       0 => free       1 => obstacle
//   cam_matrix[x][y]   0 => off-road   1 => road

cv::Scalar LocalMap::getMatrixColor(int x, int y) {
    double val = matrix[clampGridX(x)][clampGridY(y)];
    double val_cam = matrix_cam[clampGridX(x)][clampGridY(y)];
    return cv::Scalar(128, (1-val)*255, (val_cam)*255);
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

    // draw robot (black 100)
    cv::Point a(clampGuiX(map2guiX(posX) + guiShiftX), clampGuiY(map2guiY(posY) + guiShiftY));
    cv::Point b = a + cv::Point(100 * sin(angle), -100 * cos(angle));
    cv::arrowedLine(result, a, b, cv::Scalar(0, 0, 0), 2, CV_AA, 0, 0.2);

    // draw target directions (blue 30-30 / 60)
    if (wayEndDistance < sensorCutoff) { // draw next heading when close
        b = a + cv::Point(30 * sin(angle - compassHeading + currWayHeading), -30 * cos(angle - compassHeading + currWayHeading));
        cv::Point c = b + cv::Point(30 * sin(angle - compassHeading + nextWayHeading), -30 * cos(angle - compassHeading + nextWayHeading));
        cv::line(result, a, b, cv::Scalar(255, 0, 0), 2);
        cv::arrowedLine(result, b, c, cv::Scalar(255, 0, 0), 2);
    } else {
        b = a + cv::Point(60 * sin(angle - compassHeading + currWayHeading), -60 * cos(angle - compassHeading + currWayHeading));
        cv::arrowedLine(result, a, b, cv::Scalar(255, 0, 0), 2);
    }

    // draw north (thin red 50)
    b = a + cv::Point(50 * sin(angle - compassHeading), -50 * cos(angle - compassHeading));
    cv::arrowedLine(result, a, b, cv::Scalar(0, 0, 255));

    // draw best heading (red 90)
    b = a + cv::Point(90 * sin(bestHeading), -90 * cos(bestHeading));
    cv::arrowedLine(result, a, b, cv::Scalar(0, 0, 255), 2, CV_AA, 0, 0.2);

    // draw direction scores
    for (int i = 0; i < 360; i++) {
        double dir = (double) i * (pi / 180);
//        printf("%lf\n", scores[i]);
        b = a + cv::Point(100 * scores[i] * sin(dir), -100 * scores[i] * cos(dir));
        cv::circle(result, b, 1, cv::Scalar(255, 0, 0), CV_FILLED);
    }

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

void LocalMap::setRpLidarData(int rays, double *distances, double *angles) {
    rpRays = rays;
    memcpy(rpDistances, distances, sizeof(double) * rpRays);
    memcpy(rpAngles, angles, sizeof(double) * rpRays);
    validRpLidar = true;
//    for (int i = 0; i < rpRays; i++) {
//        printf("%f\t%f\n", rpDistances[i], rpAngles[i]);
//    }
}

void LocalMap::decayMap() {
    for (int i = 0; i < gridWidth; i++) {
        for (int j = 0; j < gridHeight; j++) {
            //matrix[i][j] *= 0.99; //0.85;
            matrix[i][j] *= 0.99; //0.85;
            matrix_cam[i][j] *= 1.0;
        }
    }
}

double doublemin(double a, double b) { if (a < b) return a; else return b; }

// first pass - clear 
void LocalMap::applyRay_clear_and_mark(double sensorX, double sensorY, double rayAngle, double rayLen) {
    // ray end point
    //double rayX = sensorX + rayLen * sin(angle + rayAngle);
    //double rayY = sensorY + rayLen * cos(angle + rayAngle);
    double rayX = sensorX + sensorCutoff * sin(angle + rayAngle);
    double rayY = sensorY + sensorCutoff * cos(angle + rayAngle);

    // mark points on ray as empty
    double q = 2.0/3;
    for (int j = 0; j < sensorCutoff; j += 10) {
        double qq = 1.0 - q;
        double p = ((double) j) / sensorCutoff;
        double pX = sensorX + p * (rayX - sensorX);
        double pY = sensorY + p * (rayY - sensorY);
        int gX = map2gridX(pX);
        int gY = map2gridY(pY);
        
        if (j > rayLen)
        {
			// mark as obstacle (also mark 8-neighborhood)
			matrix[clampGridX(gX-1)][clampGridY(gY-1)] = doublemin(1.0, qq * matrix[clampGridX(gX-1)][clampGridY(gY-1)] + q);
			matrix[clampGridX(gX-1)][gY] = doublemin(1.0, qq * (matrix[clampGridX(gX-1)][gY]) + q);
			matrix[clampGridX(gX-1)][clampGridY(gY+1)] = doublemin(1.0, qq * matrix[clampGridX(gX-1)][clampGridY(gY+1)] + q);
			matrix[gX][clampGridY(gY-1)] = doublemin(1.0, qq * matrix[gX][clampGridY(gY-1)] + q);
			matrix[gX][gY] = doublemin(1.0, qq * matrix[gX][gY] + q);
			matrix[gX][clampGridY(gY+1)] = doublemin(1.0, qq * matrix[gX][clampGridY(gY+1)] + q);
			matrix[clampGridX(gX+1)][clampGridY(gY-1)] = doublemin(1.0, qq * matrix[clampGridX(gX+1)][clampGridY(gY-1)] + q);
			matrix[clampGridX(gX+1)][gY] = doublemin(1.0, qq * matrix[clampGridX(gX+1)][gY] + q);
			matrix[clampGridX(gX+1)][clampGridY(gY+1)] = doublemin(1.0, qq * matrix[clampGridX(gX+1)][clampGridY(gY+1)] + q);
                        q *= 0.5;   // pixels behind the detected obstacles are not marked as obstacle so much
		}
		else
		{
        // don't overwrite obstacles found by previous rays from this batch
        // ???if (matrix[gX][gY] < 1.0) 
          matrix[gX][gY] = matrix[gX][gY] / 3;
	    }
    }
}

void LocalMap::applyHokuyoData() {
    if (!validHokuyo) return;

    // sensor in map
    double sensorX = posX + hokuyoOffset * sin(angle);
    double sensorY = posY + hokuyoOffset * cos(angle);

    // iterate valid rays   
    for (int i = 180; i <= 900; i++) {
        // calculate ray angle
        double rayAngle = ((-(double) i) / 4 + 135) * (pi / 180);

        // clamp ray length
        double rayLen = ((double) hokuyo[i]) / 10;
        //if (rayLen > sensorCutoff) {
        if (rayLen > IGNORE_TOO_LONG_RAY) {
            continue; // skip too long rays
        }

        applyRay_clear_and_mark(sensorX, sensorY, rayAngle, rayLen);
    }
}

void LocalMap::applyRpLidarData() {
    if (!validRpLidar) return;

    for (int i = 0; i < rpRays; i++) {
        // find hit coords in lidar plane
        double lX = sin(rpAngles[i]) * rpDistances[i];
        double lY = cos(rpAngles[i]) * rpDistances[i];

        // compensate angle
        double hX = lX * sin(rpLidarAngle);
        double z = lX * cos(rpLidarAngle);

        if (abs(z) > rpLidarHeight) // skip rays that would hit undeground / too high
            continue;

        // calculate new length and angle
        double rayAngle = atan2(hX, lY);
        double rayLen = sqrt(hX*hX + lY*lY) / 10;

        // clamp ray length
        if (rayLen > sensorCutoff) {
            continue; // skip too long rays
        }

        applyRay_clear_and_mark(posX, posY, rayAngle, rayLen);
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

void LocalMap::setDepthMap(unsigned char *data) 
{
    return;
/*
    for (int i = 0; i < 3600; i++) {
        depthMap[i%60][i/60] = data[i];
    }
    validDepthMap = true; */
}

void LocalMap::applyDepthMap()
{
    if (!validDepthMap) return;
    double r11 = cos(angle);
    double r12 = -sin(angle);
    double r21 = sin(angle);
    double r22 = cos(angle);

    FILE *f = fopen("depth.log", "w+");

    for (int i = 0; i < gridWidth; i++) {
        for (int j = 0; j < gridHeight; j++) {
            depth_mask_val[i][j] = 0.0;
            depth_mask_count[i][j] = 0;
        }
    }

    for (int x = -30; x < 30; x++) {
        for (int y = 0; y < 60; y++) {
            // rotate
            fprintf(f, "%d", depthMap[30 - x][y]);
            double rX = -((double) x*10) * r11 - ((double) y*10) * r12;
            double rY = ((double) x*10) * r21 + ((double) y*10) * r22;
            // move to robot
            int gX = map2gridX(rX + posX);
            int gY = map2gridY(rY + posY);
            if (depthMap[30 - x][y] == 1)
            {
              depth_mask_val[gX][gY] ++;
              depth_mask_count[gX][gY] ++;            
            }
            else if (depthMap[30 - x][y] == 2)
            {
              depth_mask_count[gX][gY] ++;
            }
        }
        fprintf(f, "\n");
    }
    fclose(f);

    for (int x = -30; x < 30; x++) {
        for (int y = 0; y < 60; y++) {
            // rotate
            double rX = -((double) x*10) * r11 - ((double) y*10) * r12;
            double rY = ((double) x*10) * r21 + ((double) y*10) * r22;
            // move to robot
            int gX = map2gridX(rX + posX);
            int gY = map2gridY(rY + posY);
            
            if (depth_mask_count[gX][gY])
            {
              double val = depth_mask_val[gX][gY] / depth_mask_count[gX][gY];
              matrix[gX][gY] = (matrix[gX][gY] + 2 * val) / 3;
            }
        }
    }
}

void LocalMap::applyImage() {
    if (!validImage) return;
    double r11 = cos(angle);
    double r12 = -sin(angle);
    double r21 = sin(angle);
    double r22 = cos(angle);

    for (int i = 0; i < gridWidth; i++) {
        for (int j = 0; j < gridHeight; j++) {
            mask_val[i][j] = 0.0;
            mask_count[i][j] = 0;
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
            if (cameraData[30 - x][y] > 0)
            {
              double val = (double) cameraData[30 - x][y] / 255.0;
            
              mask_val[gX][gY] += val;
              mask_count[gX][gY] ++;            
            }
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
            
            if (mask_count[gX][gY])
            {
              double val = mask_val[gX][gY] / mask_count[gX][gY];
              matrix_cam[gX][gY] = (matrix_cam[gX][gY] + 2 * val) / 3;
            }
        }
    }

    /* previous version:
     
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
                if (!isZero(val)) {
                    matrix_cam[gX][gY] = (matrix_cam[gX][gY] + 2 * val) / 3;
                }
            }
        }
    } */
}

double calcDist(double x1, double y1, double x2, double y2, double x, double y) {
    double a, b, c;
    if (isZero(y1) && isZero(y2)) return y; // x axis
    if (isZero(x1) && isZero(x2)) return x; // y axis
    if (isZero(x1) && isZero(y1)) { // p1 is origin
        a = 1;
        b = -x2 / y2;
        c = 0;
    } else if (isZero(x2) && isZero(y2)) { // p2 is origin
        a = 1;
        b = -x1 / y1;
        c = 0;
    } else if (!isZero(x1) && !isZero(y1) && !isZero(x2) && !isZero(y2) && isZero(x1 / x2 - y1 / y2)) { // passes through origin
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

double angleDiffAbs(double a, double b) {
    a = fmod(a, 2*pi);
    a = a < 0? a + 2*pi : a;
    b = fmod(b, 2*pi);
    b = b < 0? b + 2*pi : b;
    double d = fabs(a - b);
    if (d < pi) return d;
    else return 2*pi - d;
}

double angleDiffDir(double a, double b) {
    double d = b - a;
    if (d > pi) d -= 2*pi;
    if (d < -pi) d += 2*pi;
    return d;
}

double angleInterpolate(double a, double b, double p) {
    double d = angleDiffDir(a, b);
    double i = a + p * d;
    return i < 0 ? i + 2*pi : i;
}

const double pathWidth = wheelDistance * 2;

void LocalMap::findBestHeading() {
    static int going_wrong = 0;
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
        double aX = posX + pathWidth* sin(dir + (pi / 2));
        double aY = posY + pathWidth* cos(dir + (pi / 2));
        double bX = posX + pathWidth* sin(dir - (pi / 2));
        double bY = posY + pathWidth* cos(dir - (pi / 2));
        double cX = endX + pathWidth* sin(dir + (pi / 2));
        double cY = endY + pathWidth* cos(dir + (pi / 2));
        double dX = endX + pathWidth* sin(dir - (pi / 2));
        double dY = endY + pathWidth* cos(dir - (pi / 2));

        // calculate bounding box
        int left = map2gridX_(std::min({aX, bX, cX, dX}));
        int right = map2gridX_(std::max({aX, bX, cX, dX}));
        int bot = map2gridY_(std::min({aY, bY, cY, dY}));
        int top = map2gridY_(std::max({aY, bY, cY, dY}));

        int count = 0;
        // iterate over bounding box
        for (int x = left; x <= right; x++) {
            double pX = gridSize * x;
            for (int y = bot; y <= top; y++) {
                double pY = gridSize * y;
                double lineDist = calcDist(posX, posY, endX, endY, pX, pY);
                double pointDist = sqrt(pow(posX - pX, 2) + pow(posY - pY, 2));
                if (lineDist <= pathWidth&& pointDist < sensorCutoff) {
                    double v = (1 - matrix[clampGridX(x)][clampGridY(y)]) * matrix_cam[clampGridX(x)][clampGridY(y)];
                    // contribution to score weighted by distance
                    scores[i] += v * (1 - pointDist / sensorCutoff / 1.2);//(1 / pow(2, pointDist));
                    count++;
                }
            }
        }
        // normalize score for paths with more valid grid squares
        scores[i] /= (double) count;

        // reduce score of paths in wrong direction
        double target;
        if (wayEndDistance > sensorCutoff/2.4) {
            target = angle - compassHeading + currWayHeading;
        } else {
            target = angleInterpolate(currWayHeading, nextWayHeading, 1 - wayEndDistance / (sensorCutoff/2.4));
        }
        double diff = angleDiffAbs(target, dir);
        //scores[i] *= 1 - (diff / pi);
        scores[i] *= 0.5 + (1 - (diff / pi)) / 2;
    }
    int best = 0;
    double bestScore = scores[0];
    // find path with highest score
    for (int i = 1; i < 360; i++) {		
        if (scores[i] > scores[best]) {
            best = i;
            bestScore = scores[i];
        }
    }

    bestHeading = ((double) best) * (pi / 180);

    if (angleDiffAbs(bestHeading, currWayHeading) > 150 / 180.0 * pi)
    { 
      going_wrong++;
      if (going_wrong > 100) transmitting_going_wrong = 1;
      if (going_wrong % 20 == 0) printf("wrong %d\n", going_wrong);
    }
    else if (going_wrong) going_wrong--;

    if (angleDiffAbs(bestHeading, currWayHeading) < 30 / 180.0 * pi)
      if (transmitting_going_wrong) 
    {
      transmitting_going_wrong = 0;
      going_wrong = 0;
    }

    // rescale scores
    for (int i = 0; i < 360; i++) {
        scores[i] = scores[i] / bestScore;
    }
}

double LocalMap::getHeading() {
		
	static double last_d = 0;
	static double last_returned_d = 0;
	
    double d = bestHeading - angle;
    while (d > pi) d -= 2*pi;
    while (d < -pi) d += 2*pi;
    
    // do not return change by more than 30 degrees per iteration
    if (angleDiffAbs(last_d, d) < 30 / 180.0 * pi)
	   last_returned_d = d;
	
	last_d = d;
	
    if (transmitting_going_wrong) return GOING_WRONG;
    return last_returned_d;
}
