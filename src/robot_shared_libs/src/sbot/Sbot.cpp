#include "Sbot.h"

static int validate(const char *devName) {
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

void *base_module_thread(void *arg)
{
    Sbot *s = (Sbot*) arg;
    char line[1024];
    int printingDebugging = 1;
    // wait for '@'
    unsigned char ch;
    int numRead;
    int printing = 0;

    while (s->running)
    {
      int lineIndex = 1;
      do {
          if (read(s->fdW[0], &ch, 1) < 0) {
              ROS_ERROR("read()");
              return 0;
          }
          if (ch == '!')
              printing = printingDebugging;
          //if ((printing) && (ch == 13))
           //   printing = 0;
           if (printing)
               printf("%c", ch);
      } while (ch != '@');
      line[0] = '@';
  
      do {
          if ((numRead = read(s->fdW[0], line + lineIndex, 1)) < 0) {
              ROS_ERROR("read()");
              return 0;
          }
          lineIndex += numRead;
          if (lineIndex > 1023) {
              break;
          }
      } while (line[lineIndex - 1] != '\n');
  
      if (lineIndex > 1023) {
          return 0;
      }
      line[lineIndex] = '\0';


      pthread_mutex_lock(&(s->m_read));

      sscanf(line, "@ %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &(s->result.lstep),
             &(s->result.rstep), &(s->result.lspeed), &(s->result.rspeed), &(s->result.blocked),
             &(s->result.obstacle), &(s->result.distRR), &(s->result.distFR), &(s->result.distM),
             &(s->result.distFL), &(s->result.distRL), &(s->result.payload), 
             &(s->result.rear_obstacle),&(s->result.time));
      if (((s->result.distRL) < 35) && ((s->result.distRL) > 14))
          s->result.away_from_left = 1;
      else
          s->result.away_from_left = 0;
      if ((s->result.distRR < 35) && (s->result.distRR > 14))
          s->result.away_from_right = 1;
      else
          s->result.away_from_right = 0;

      pthread_mutex_unlock(&(s->m_read));
    }
    return 0;
}

int Sbot::init() 
{
    if (!validate("/dev/sbot")) {
        ROS_ERROR("Sbot is not initialized");
        return 1;
    }
// char *command="plink /dev/ttyUSB0 -serial -sercfg 115200,N,n,8,1";

    if (pipe(fdR) < 0) {
        ROS_ERROR("pipe2()");
        return 2;
    }
    if (!pipe(fdW) < 0) {
        ROS_ERROR("pipe2()");
        return 3;
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
            return 4;
        }
    }

    if (child < 0) {
        ROS_ERROR("fork()");
        return 5;
    }

    /* parent */
    close(fdR[0]);
    close(fdW[1]);

    pthread_mutex_init(&m_read, NULL);
    
    pthread_t t;

    running = 1;
    if (pthread_create(&t, 0, base_module_thread, this) != 0)
    {
      ROS_ERROR("sbot pthread_create()");
      return 4;
    }

    return 0;
}


void Sbot::shutdown()
{
    running = 0;
}

void Sbot::getData(message_types::SbotMsg *res) 
{
    pthread_mutex_lock(&m_read);
    *res = result;
    pthread_mutex_unlock(&m_read);
}
