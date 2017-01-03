#include "Hokuyo.h"

void Hokuyo::init() {
    data = new int[RANGE_DATA_COUNT];

    struct sockaddr_in remoteaddr;

    remoteaddr.sin_family = AF_INET;
    remoteaddr.sin_addr.s_addr = inet_addr("192.168.0.1");
    remoteaddr.sin_port = htons(10940);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);


    if (sockfd < 0) {
        ROS_ERROR("ERROR opening Hokuyo socket");

        return;
    }

    if (connect(sockfd, (struct sockaddr *) &remoteaddr, sizeof(remoteaddr)) < 0) {
        ROS_ERROR("ERROR connecting Hokuyo");

        return;
    }

    ROS_INFO("Hokuyo connected\n");

    pthread_mutex_init(&m_read, NULL);

    char *start_measurement = (char *) "BM\n";

    if (write(sockfd, start_measurement, strlen(start_measurement)) < 0) {
        ROS_ERROR("ERROR writing to Hokuyo\n");

        return;
    }

    unsigned char x[2];
    int cnt = 0;
    do {
        if (read(sockfd, x + ((cnt++) % 2), 1) < 0) {
            ROS_ERROR("ERROR reading response from Hokuyo");

            break;
        }
    } while ((x[0] != 10) || (x[1] != 10));
}

void Hokuyo::readData() {
    char *request_measurement = (char *) "GD0000108000\n";

    if (write(sockfd, request_measurement, strlen(request_measurement)) < 0) {
        ROS_ERROR("ERROR writing to Hokuyo\n");
    }

    int readptr = 0;
    do {
        int nread = read(sockfd, read_buf + readptr, BUFFER_SIZE - readptr);
        if (nread < 0) {
            ROS_ERROR("Problem reading from Hokuyo");

            break;
        }
        readptr += nread;

        if (readptr < 2)
            continue;
    } while ((read_buf[readptr - 1] != 10) || (read_buf[readptr - 2] != 10));

    int searchptr = 0;
    for (int i = 0; i < 3; i++) {
        while ((read_buf[searchptr] != 10) && (searchptr < readptr)) {
            searchptr++;
        }
        searchptr++;
    }

    if (readptr - searchptr != 103 + RANGE_DATA_COUNT * 3) {
        ROS_INFO("Hokuyo returned packet of unexpected size, I will ignore "
                         "it size=%d\n",
                 readptr - searchptr);

        return;
    }

    int beam_index = RANGE_DATA_COUNT - 1;
    readptr = searchptr;
    pthread_mutex_lock(&m_read);
    while (beam_index >= 0) {
        int pos = (searchptr - readptr) % 66;
        if (pos == 62) {
            data[beam_index] = ((read_buf[searchptr] - 0x30) << 12) |
                               ((read_buf[searchptr + 1] - 0x30) << 6) |
                               (read_buf[searchptr + 4] - 0x30);
            searchptr += 5;
        } else if (pos == 63) {
            data[beam_index] = ((read_buf[searchptr] - 0x30) << 12) |
                               ((read_buf[searchptr + 3] - 0x30) << 6) |
                               (read_buf[searchptr + 4] - 0x30);
            searchptr += 5;
        } else {
            if (pos == 64) {
                searchptr += 2;
            }
            data[beam_index] = ((((int) read_buf[searchptr]) - 0x30) << 12) |
                               ((((int) read_buf[searchptr + 1]) - 0x30) << 6) |
                               (((int) read_buf[searchptr + 2]) - 0x30);
            searchptr += 3;
        }

        beam_index--;
    }
    pthread_mutex_unlock(&m_read);
}

Int32MultiArray Hokuyo::getData() {

    Int32MultiArray result;

    pthread_mutex_lock(&m_read);
    for (int i = 0; i < RANGE_DATA_COUNT; i++) {
        result.data.push_back(data[i]);
    }
    pthread_mutex_unlock(&m_read);

    return result;
}
