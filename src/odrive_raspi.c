#include "odrive_raspi.h"

#define speed B115200
struct Settings *sets;
int32_T f;

#ifdef __cplusplus
extern "C" {
#endif

void initialize(struct Settings *settings)
{
    sets = settings;
    
    uint8_T portName[64];
    if (settings->isPort)
    {
        strcpy(portName, settings->portName);
    }
    else
    {
        uint8_T ret = detectOdrivePort(settings->serial, portName);
        if (ret == 0)
        {
            printf("\nThe ODrive of serial number '%s' can't be found.\n", settings->serial);
            printf("Check the serial number or connection of the ODrive.\n");
            exit(1);
        }
    }
    
    f = openSerialPort(portName);
    if (f == -1)
    {
        fprintf(stderr, "Failed to open the serial port: %s\n", strerror(errno));
		exit(1);
    }
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        if (settings->isAxis[i])
        {
            startupSequence(f, i);
        }
    }
    
    waitSetupStatus(f, settings);
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        if (settings->isAxis[i])
        {
            setConfiguration(f, i, settings);
        }
    }
}

void step(struct Data *data)
{
    uint8_T send[64];
    uint8_T receive[64];
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        if (sets->isAxis[i])
        {
            sprintf(send, "%s %d", "u", i);
            sendMessage(f, send);
            
            if (sets->isExternal[i])
            {
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.pos_gain", data->posGain[i]);
                sendMessage(f, send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_gain", data->velGain[i]);
                sendMessage(f, send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_integrator_gain", data->velIntegratorGain[i]);
                sendMessage(f, send);
                
                if (data->velIntegratorCurrentTrigger[i])
                {
                    sprintf(send, "%s%d%s %f", "w axis", i, ".controller.vel_integrator_current", data->velIntegratorCurrentRef[i]);
                    sendMessage(f, send);
                }
                
                if (sets->velRampEnable[i])
                {
                    sprintf(send, "%s%d%s %d", "w axis", i, ".controller.vel_ramp_enable", data->velRampEnable[i]);
                    sendMessage(f, send);
                }
                
                sprintf(send, "%s%d%s %d", "w axis", i, ".controller.config.vel_ramp_rate", data->velRampRate[i]);
                sendMessage(f, send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_limit", data->velLimit[i]);
                sendMessage(f, send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_limit_tolerance", data->velLimitTolerance[i]);
                sendMessage(f, send);
            }
            
            switch (sets->controlMode[i])
            {
                case CTRL_MODE_POSITION_CONTROL:
                    sprintf(send, "%s %d %f %f %f", "p", i, data->posSetpoint[i], data->velSetpoint[i], data->currentSetpoint[i]);
                    sendMessage(f, send);
                    break;
                    
                case CTRL_MODE_VELOCITY_CONTROL:
                    if (sets->velRampEnable[i])
                    {
                        sprintf(send, "%s%d%s %d", "w axis", i, ".controller.vel_ramp_target", data->velRampTarget[i]);
                        sendMessage(f, send);
                    }
                    else
                    {
                        sprintf(send, "%s %d %f %f", "v", i, data->velSetpoint[i], data->currentSetpoint[i]);
                        sendMessage(f, send);
                    }
                    break;
                    
                case CTRL_MODE_CURRENT_CONTROL:
                    sprintf(send, "%s %d %f %f", "c", i, data->currentSetpoint[i]);
                    sendMessage(f, send);
                    break;
                default:
                    break;
            }
            
            real32_T pos;
            real32_T vel;
            sprintf(send, "%s %d", "f", i);
            sendMessage(f, send);
            receiveMessage(f, receive, sizeof(receive));
            sscanf(receive, "%f %f", &pos, &vel);
            data->actualPosition[i] = pos;
            data->actualVelocity[i] = vel;
            
            real32_T cur;
            sprintf(send, "%s%d%s", "r axis", i, ".motor.current_control.Iq_measured");
            sendMessage(f, send);
            receiveMessage(f, receive, sizeof(receive));
            sscanf(receive, "%f", &cur);
            data->actualCurrent[i] = cur;
            
            real32_T velcur;
            sprintf(send, "%s%d%s", "r axis", i, ".controller.vel_integrator_current");
            sendMessage(f, send);
            receiveMessage(f, receive, sizeof(receive));
            sscanf(receive, "%f", &velcur);
            data->velIntegratorCurrentAct[i] = velcur;
            
            real32_T err;
            sprintf(send, "%s%d%s", "r axis", i, ".error");
            sendMessage(f, send);
            receiveMessage(f, receive, sizeof(receive));
            sscanf(receive, "%f", &err);
            data->error[i] = err;
            
            if (err)
            {
                terminate();
            }
        }
    }
}

void terminate()
{
    uint8_T send[64];
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        sprintf(send, "%s%d%s %d", "w axis", i, ".requested_state", AXIS_STATE_IDLE);
        sendMessage(f, send);
    }
        
    fclose((FILE*)f);
}

uint8_T detectOdrivePort(uint8_T *serial, uint8_T *portName)
{
    uint8_T numDetect = 0;
    struct dirent **ttyList;
    int ret = scandir("/sys/class/tty", &ttyList, NULL, NULL);
    
    for (uint8_T i = 0; i < ret; ++i)
    {
        struct stat st;        
        uint8_T fullPath[256];
        sprintf(fullPath, "%s%s%s", "/sys/class/tty/", ttyList[i]->d_name, "/device");
        
        if (stat(fullPath, &st) == 0)
        {
            uint8_T cmdline[256];
            sprintf(cmdline, "%s%s%s%s%s", "udevadm info /dev/", ttyList[i]->d_name, 
                                           " | grep '", serial, "'");
            FILE *fp = popen(cmdline, "r");
            uint8_T buf[256];
            
            if (fgets(buf, sizeof(buf), fp) != NULL)
            {
                sprintf(portName, "%s%s", "/dev/", ttyList[i]->d_name);
                ++numDetect;
            }
            
            pclose(fp);
        }
    }
    
    free(ttyList);
    return numDetect;
}

int32_T openSerialPort(uint8_T *portName)
{
    int32_T fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
    
    struct termios tty;
    
    int16_T getStatus = tcgetattr(fd, &tty);
    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);
    
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 /* 8-bit characters */
    tty.c_cflag &= ~PARENB;             /* no parity bit */
    tty.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */
    
    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    
    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    int32_T setStatus = tcsetattr(fd, TCSANOW, &tty);
    
    if (fd == -1 || getStatus == -1 || setStatus == -1)
    {
        return -1;
    }
    else
    {
        return (int32_T) fdopen(fd, "r+");
    }
}

void startupSequence(int32_T f, uint8_T axis)
{
    uint8_T send[64];
    uint8_T receive[64];
    int32_T value;
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".motor.error", false);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".encoder.error", false);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.error", false);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".error", false);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s", "r axis", axis, ".motor.is_calibrated");
    sendMessage(f, send);
    receiveMessage(f, receive, sizeof(receive));
    sscanf(receive, "%d", &value);
    
    if (value == 0)
    {
        sprintf(send, "%s%d%s %d", "w axis", axis, ".config.startup_motor_calibration", true);
        sendMessage(f, send);
    }
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".config.startup_encoder_index_search", true);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s", "r axis", axis, ".encoder.is_ready");
    sendMessage(f, send);
    receiveMessage(f, receive, sizeof(receive));
    sscanf(receive, "%d", &value);
    
    if (value == 0)
    {
        sprintf(send, "%s%d%s %d", "w axis", axis, ".config.startup_encoder_offset_calibration", true);
        sendMessage(f, send);
    }
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".requested_state", AXIS_STATE_STARTUP_SEQUENCE);
    sendMessage(f, send);
}

void waitSetupStatus(int32_T f, struct Settings *settings)
{
    for (uint8_T i = 0; i < 2; ++i)
    {
        if (settings->isAxis[i])
        {
            int32_T isMotorCalib = 0;
            int32_T isEncoder1Calib = 0;
            while(isMotorCalib == 0 || isEncoder1Calib == 0)
            {
                uint8_T send[64];
                uint8_T receive[64];
                
                sprintf(send, "%s%d%s", "r axis", i, ".motor.is_calibrated");
                sendMessage(f, send);
                receiveMessage(f, receive, sizeof(receive));
                sscanf(receive, "%d", &isMotorCalib);
                
                sprintf(send, "%s%d%s", "r axis", i, ".encoder.is_ready");
                sendMessage(f, send);
                receiveMessage(f, receive, sizeof(receive));
                sscanf(receive, "%d", &isEncoder1Calib);
            }
        }
    }
}

void setConfiguration(int32_T f, uint8_T axis, struct Settings *settings)
{
    uint8_T send[64];
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".requested_state", AXIS_STATE_IDLE);
    sendMessage(f, send);
    
    struct timespec ts1 = {0, 10 * 1000000};
    nanosleep(&ts1, &ts1);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.config.control_mode", settings->controlMode[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.pos_gain", settings->posGain[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_gain", settings->velGain[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_integrator_gain", settings->velIntegratorGain[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_limit", settings->velLimit[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_limit_tolerance", settings->velLimitTolerance[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_ramp_rate", settings->velRampRate[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.config.setpoints_in_cpr", settings->setPointsInCpr[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.vel_ramp_enable", settings->velRampEnable[axis]);
    sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".requested_state", AXIS_STATE_CLOSED_LOOP_CONTROL);
    sendMessage(f, send);
    
    struct timespec ts2 = {0, 10 * 1000000};
    nanosleep(&ts2, &ts2);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".config.watchdog_timeout", settings->watchdogTimeout[axis]);
    sendMessage(f, send);
}

void sendMessage(int32_T f, uint8_T *message)
{
    FILE *fp = (FILE *)f;
    fprintf(fp, "%s\n", message);
}

void receiveMessage(int32_T f, uint8_T *message, uint8_T len)
{
    FILE *fp = (FILE *)f;
    fgets(message, len, fp);
}

#ifdef __cplusplus
}
#endif