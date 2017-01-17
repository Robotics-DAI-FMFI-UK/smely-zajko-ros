#include "Sbot.h"

void Sbot::init() {
// char *command="plink /dev/ttyUSB0 -serial -sercfg 115200,N,n,8,1";

    if (pipe(fdR) < 0) {
        ROS_ERROR("pipe2()");
        exit(-1);
    }
    if (!pipe(fdW) < 0) {
        ROS_ERROR("pipe2()");
        exit(-1);
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
            perror("child execl()");
            exit(-1);
        }
    }

    if (child < 0) {
        perror("fork()");
        exit(-1);
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
        if ((numRead = read(fdW[0], &ch, 1)) < 0) {
            perror("read()");
            exit(-1);
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
            perror("read()");
            exit(-1);
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

    message_types::SbotMsg result;

    int blocked, obstacle;
/*
    sscanf(line, "@ %d %d %d %d %d %d %d %d %d %d %d", &result->lstep,
           &result->rstep, &result->lspeed, &result->rspeed, &blocked,
           &obstacle, &result->distRR, &result->distFR, &result->distM,
           &result->distFL, &result->distRL);

    result->blocked = (blocked);
    result->obstacle = (obstacle);

    if ((result->distRL < 35) && (result->distRL > 14))
        away_from_left = 1;
    else
        away_from_left = 0;
    if ((result->distRR < 35) && (result->distRR > 14))
        away_from_right = 1;
    else
        away_from_right = 0;

    pthread_mutex_lock(&m_read);
    SbotThread::data = *result;
    pthread_mutex_unlock(&m_read);
    */
    //delete result;
};

message_types::SbotMsg Sbot::getData() {

};