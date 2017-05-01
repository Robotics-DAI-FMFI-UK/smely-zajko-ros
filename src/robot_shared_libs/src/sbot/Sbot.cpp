#include "Sbot.h"

int validate(const char *devName) {
    // char *command="plink /dev/ttyUSB0 -serial -sercfg 115200,N,n,8,1";
    char command[128];


    sprintf(command, "plink %s -serial -sercfg 115200,N,n,8,1", devName);

    FILE *f;

    if (!(f = popen(command, "r"))) {
        // If fpipe is NULL
        return 0;
    }
    if (f < 0)
        return 0;

    char b[128];

    for (int i = 0; i < 10; i++) {

        fgets(b, sizeof b, f);

        //@0 -6 0 0 0 0 200 108 200
        std::istringstream is(b);
        if (!is.eof()) {
            char c;
            int n;

            is >> c;
            if (is.fail() || c != '@') {
                continue;
            }

            is >> n;
            if (is.fail()) {
                continue;
            }
            is >> n;
            if (is.fail()) {
                continue;
            }
            is >> n;
            if (is.fail()) {
                continue;
            }
            is >> n;
            if (is.fail()) {
                continue;
            }
            is >> n;
            if (is.fail()) {
                continue;
            }
            is >> n;
            if (is.fail()) {
                continue;
            }
            is >> n;
            if (is.fail()) {
                continue;
            }
            is >> n;
            if (is.fail()) {
                continue;
            }

            // printf("%s", b );
            pclose(f);
            return 1;
        }
    }

    pclose(f);

    return 0;
}

void Sbot::init() {
    if (!validate("/dev/sbot")) {
        ROS_ERROR("Sbot is not initialized");
        return;
    }
// char *command="plink /dev/ttyUSB0 -serial -sercfg 115200,N,n,8,1";

    if (pipe(fdR) < 0) {
        ROS_ERROR("pipe2()");
        return;
    }
    if (!pipe(fdW) < 0) {
        ROS_ERROR("pipe2()");
        return;
    }

    if ((child = fork()) == 0) {
        /* child */

        close(0);
        close(1);
        dup2(fdR[0], 0);
        dup2(fdW[1], 1);
        close(fdR[0]);
        close(fdR[1]);
        close(fdW[0]);
        close(fdW[1]);

        // if (execl("/usr/bin/plink", "/usr/bin/plink", "/dev/ttyUSB0",
        // "-serial", "-sercfg", "115200,N,n,8,1", NULL) < 0) {
        if (execl("/usr/bin/plink", "/usr/bin/plink", "/dev/sbot", "-serial",
                  "-sercfg", "115200,N,n,8,1", NULL) < 0) {
            ROS_ERROR("child execl()");
            return;
        }
    }

    if (child < 0) {
        ROS_ERROR("fork()");
        return;
    }

    /* parent */
    close(fdR[0]);
    close(fdW[1]);

    pthread_mutex_init(&m_read, NULL);
};

void Sbot::readData() {
    char line[1024];
    int printingDebugging = 1;
    // wait for '@'
    unsigned char ch;
    int numRead;
    int printing = 0;
    int lineIndex = 1;

    do {
        if (read(fdW[0], &ch, 1) < 0) {
            ROS_ERROR("read()");
            return;
        }
        if (ch == '!')
            printing = printingDebugging;
        if ((printing) && (ch == 13))
            printing = 0;
        if (printing)
            printf("%c", ch);
    } while (ch != '@');
    line[0] = '@';

    do {
        if ((numRead = read(fdW[0], line + lineIndex, 1)) < 0) {
            ROS_ERROR("read()");
            return;
        }
        lineIndex += numRead;
        if (lineIndex > 1023) {
            break;
        }
    } while (line[lineIndex - 1] != '\n');

    if (lineIndex > 1023) {
        return;
    }
    line[lineIndex] = '\0';
    sscanf(line, "@ %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &result.lstep,
           &result.rstep, &result.lspeed, &result.rspeed, &result.blocked,
           &result.obstacle, &result.distRR, &result.distFR, &result.distM,
           &result.distFL, &result.distRL, &result.time);
    if ((result.distRL < 35) && (result.distRL > 14))
        result.away_from_left = 1;
    else
        result.away_from_left = 0;
    if ((result.distRR < 35) && (result.distRR > 14))
        result.away_from_right = 1;
    else
        result.away_from_right = 0;
};

message_types::SbotMsg Sbot::getData() {

    return result;
};