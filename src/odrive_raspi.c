#include "odrive_raspi.h"

#define speed B115200
#define maxOdrives 16

struct odrive_Settings *od_sets[maxOdrives];
int32_T od_f[maxOdrives];
pthread_t od_thread[maxOdrives];
struct odrive_Data *od_pdata[maxOdrives];

#ifdef __cplusplus
extern "C" {
#endif

int8_T odrive_initialize(struct odrive_Settings *settings)
{
    static int8_T threadID = -1;
    ++threadID;
    
    od_sets[threadID] = settings;
    
    uint8_T portName[64];
    if (settings->isPort)
    {
        strcpy(portName, settings->portName);
    }
    else
    {
        uint8_T ret = odrive_detectOdrivePort(settings->serial, portName);
        if (ret == 0)
        {
            printf("\nThe ODrive of serial number '%s' can't be found.\n", settings->serial);
            printf("Check the serial number or connection of the ODrive.\n");
            exit(1);
        }
    }
    
    od_f[threadID] = odrive_openSerialPort(portName);
    if (od_f[threadID] == -1)
    {
        fprintf(stderr, "Failed to open the serial port: %s\n", strerror(errno));
		exit(1);
    }
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        if (settings->isAxis[i])
        {
            odrive_startupSequence(od_f[threadID], i);
        }
    }
    
    odrive_waitSetupStatus(od_f[threadID], settings);
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        if (settings->isAxis[i])
        {
            odrive_setConfiguration(od_f[threadID], i, settings);
        }
    }
    
    od_pdata[threadID] = (struct odrive_Data *)malloc(sizeof(struct odrive_Data));
    for (uint8_T i = 0; i < 2; ++i)
    {
        od_pdata[threadID]->error[i] = 0;
        od_pdata[threadID]->actualPosition[i] = 0;
        od_pdata[threadID]->actualVelocity[i] = 0;
        od_pdata[threadID]->actualCurrent[i] = 0;
        od_pdata[threadID]->velIntegratorCurrentAct[i] = 0;
    }
    
    printf("%s : %s : %d: %d\n", settings->serial, portName, od_f[threadID], threadID);
    
    return threadID;
}

void odrive_step(struct odrive_Data *data, int8_T th)
{
    pthread_join(od_thread[th], NULL);
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        od_pdata[th]->th = th;
        data->error[i] = od_pdata[th]->error[i];
        od_pdata[th]->posSetpoint[i] = data->posSetpoint[i];
        od_pdata[th]->velSetpoint[i] = data->velSetpoint[i];
        od_pdata[th]->currentSetpoint[i] = data->currentSetpoint[i];
        od_pdata[th]->posGain[i] = data->posGain[i];
        od_pdata[th]->velGain[i] = data->velGain[i];
        od_pdata[th]->velIntegratorGain[i] = data->velIntegratorGain[i];
        od_pdata[th]->velIntegratorCurrentRef[i] = data->velIntegratorCurrentRef[i];
        od_pdata[th]->velIntegratorCurrentTrigger[i] = data->velIntegratorCurrentTrigger[i];
        od_pdata[th]->velRampEnable[i] = data->velRampEnable[i];
        od_pdata[th]->velRampTarget[i] = data->velRampTarget[i];
        od_pdata[th]->velRampRate[i] = data->velRampRate[i];
        od_pdata[th]->velLimit[i] = data->velLimit[i];
        od_pdata[th]->velLimitTolerance[i] = data->velLimitTolerance[i];
        data->actualPosition[i] = od_pdata[th]->actualPosition[i];
        data->actualVelocity[i] = od_pdata[th]->actualVelocity[i];
        data->actualCurrent[i] = od_pdata[th]->actualCurrent[i];
        data->velIntegratorCurrentAct[i] = od_pdata[th]->velIntegratorCurrentAct[i];
    }
    
    pthread_create(&od_thread[th], NULL, (void *)odrive_tic, (void *)od_pdata[th]);
}

void odrive_terminate(int8_T th)
{
    pthread_join(od_thread[th], NULL);
    
    uint8_T send[64];
    for (uint8_T i = 0; i < 2; ++i)
    {
        sprintf(send, "%s%d%s %d", "w axis", i, ".requested_state", AXIS_STATE_IDLE);
        odrive_sendMessage(od_f[th], send);
    }
        
    fclose((FILE*)od_f[th]);
}

void *odrive_tic(void *pdata)
{
    struct odrive_Data *data = (struct odrive_Data *)pdata;
    uint8_T th = data->th;
    
    uint8_T send[64];
    uint8_T receive[64];
    
    for (uint8_T i = 0; i < 2; ++i)
    {
        if (od_sets[th]->isAxis[i])
        {
            sprintf(send, "%s %d", "u", i);
            odrive_sendMessage(od_f[th], send);
            
            if (od_sets[th]->isExternal[i])
            {
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.pos_gain", data->posGain[i]);
                odrive_sendMessage(od_f[th], send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_gain", data->velGain[i]);
                odrive_sendMessage(od_f[th], send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_integrator_gain", data->velIntegratorGain[i]);
                odrive_sendMessage(od_f[th], send);
                
                if (data->velIntegratorCurrentTrigger[i])
                {
                    sprintf(send, "%s%d%s %f", "w axis", i, ".controller.vel_integrator_current", data->velIntegratorCurrentRef[i]);
                    odrive_sendMessage(od_f[th], send);
                }
                
                if (od_sets[th]->velRampEnable[i])
                {
                    sprintf(send, "%s%d%s %d", "w axis", i, ".controller.vel_ramp_enable", data->velRampEnable[i]);
                    odrive_sendMessage(od_f[th], send);
                }
                
                sprintf(send, "%s%d%s %d", "w axis", i, ".controller.config.vel_ramp_rate", data->velRampRate[i]);
                odrive_sendMessage(od_f[th], send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_limit", data->velLimit[i]);
                odrive_sendMessage(od_f[th], send);
                
                sprintf(send, "%s%d%s %f", "w axis", i, ".controller.config.vel_limit_tolerance", data->velLimitTolerance[i]);
                odrive_sendMessage(od_f[th], send);
            }
            
            switch (od_sets[th]->controlMode[i])
            {
                case CTRL_MODE_POSITION_CONTROL:
                    sprintf(send, "%s %d %f %f %f", "p", i, data->posSetpoint[i], data->velSetpoint[i], data->currentSetpoint[i]);
                    odrive_sendMessage(od_f[th], send);
                    break;
                    
                case CTRL_MODE_VELOCITY_CONTROL:
                    if (od_sets[th]->velRampEnable[i])
                    {
                        sprintf(send, "%s%d%s %d", "w axis", i, ".controller.vel_ramp_target", data->velRampTarget[i]);
                        odrive_sendMessage(od_f[th], send);
                    }
                    else
                    {
                        sprintf(send, "%s %d %f %f", "v", i, data->velSetpoint[i], data->currentSetpoint[i]);
                        odrive_sendMessage(od_f[th], send);
                    }
                    break;
                    
                case CTRL_MODE_CURRENT_CONTROL:
                    sprintf(send, "%s %d %f %f", "c", i, data->currentSetpoint[i]);
                    odrive_sendMessage(od_f[th], send);
                    break;
                default:
                    break;
            }
            
            real32_T pos;
            real32_T vel;
            sprintf(send, "%s %d", "f", i);
            odrive_sendMessage(od_f[th], send);
            odrive_receiveMessage(od_f[th], receive, sizeof(receive));
            sscanf(receive, "%f %f", &pos, &vel);
            data->actualPosition[i] = pos;
            data->actualVelocity[i] = vel;
            
            real32_T cur;
            sprintf(send, "%s%d%s", "r axis", i, ".motor.current_control.Iq_measured");
            odrive_sendMessage(od_f[th], send);
            odrive_receiveMessage(od_f[th], receive, sizeof(receive));
            sscanf(receive, "%f", &cur);
            data->actualCurrent[i] = cur;
            
            real32_T velcur;
            sprintf(send, "%s%d%s", "r axis", i, ".controller.vel_integrator_current");
            odrive_sendMessage(od_f[th], send);
            odrive_receiveMessage(od_f[th], receive, sizeof(receive));
            sscanf(receive, "%f", &velcur);
            data->velIntegratorCurrentAct[i] = velcur;
            
            real32_T err;
            sprintf(send, "%s%d%s", "r axis", i, ".error");
            odrive_sendMessage(od_f[th], send);
            odrive_receiveMessage(od_f[th], receive, sizeof(receive));
            sscanf(receive, "%f", &err);
            data->error[i] = err;
            
//             if (err)
//             {
//                 odrive_terminate();
//             }
        }
    }
}

uint8_T odrive_detectOdrivePort(uint8_T *serial, uint8_T *portName)
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

int32_T odrive_openSerialPort(uint8_T *portName)
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
    
    int32_T od_setstatus = tcsetattr(fd, TCSANOW, &tty);
    
    if (fd == -1 || getStatus == -1 || od_setstatus == -1)
    {
        return -1;
    }
    else
    {
        return (int32_T) fdopen(fd, "r+");
    }
}

void odrive_startupSequence(int32_T f, uint8_T axis)
{
    uint8_T send[64];
    uint8_T receive[64];
    int32_T value;
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".motor.error", false);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".encoder.error", false);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.error", false);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".error", false);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s", "r axis", axis, ".motor.is_calibrated");
    odrive_sendMessage(f, send);
    odrive_receiveMessage(f, receive, sizeof(receive));
    sscanf(receive, "%d", &value);
    
    if (value == 0)
    {
        sprintf(send, "%s%d%s %d", "w axis", axis, ".config.startup_motor_calibration", true);
        odrive_sendMessage(f, send);
    }
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".config.startup_encoder_index_search", true);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s", "r axis", axis, ".encoder.is_ready");
    odrive_sendMessage(f, send);
    odrive_receiveMessage(f, receive, sizeof(receive));
    sscanf(receive, "%d", &value);
    
    if (value == 0)
    {
        sprintf(send, "%s%d%s %d", "w axis", axis, ".config.startup_encoder_offset_calibration", true);
        odrive_sendMessage(f, send);
    }
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".requested_state", AXIS_STATE_STARTUP_SEQUENCE);
    odrive_sendMessage(f, send);
}

void odrive_waitSetupStatus(int32_T f, struct odrive_Settings *settings)
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
                odrive_sendMessage(f, send);
                odrive_receiveMessage(f, receive, sizeof(receive));
                sscanf(receive, "%d", &isMotorCalib);
                
                sprintf(send, "%s%d%s", "r axis", i, ".encoder.is_ready");
                odrive_sendMessage(f, send);
                odrive_receiveMessage(f, receive, sizeof(receive));
                sscanf(receive, "%d", &isEncoder1Calib);
            }
        }
    }
}

void odrive_setConfiguration(int32_T f, uint8_T axis, struct odrive_Settings *settings)
{
    uint8_T send[64];
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".requested_state", AXIS_STATE_IDLE);
    odrive_sendMessage(f, send);
    
    struct timespec ts1 = {0, 10 * 1000000};
    nanosleep(&ts1, &ts1);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.config.control_mode", settings->controlMode[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.pos_gain", settings->posGain[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_gain", settings->velGain[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_integrator_gain", settings->velIntegratorGain[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_limit", settings->velLimit[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_limit_tolerance", settings->velLimitTolerance[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".controller.config.vel_ramp_rate", settings->velRampRate[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.config.setpoints_in_cpr", settings->setPointsInCpr[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".controller.vel_ramp_enable", settings->velRampEnable[axis]);
    odrive_sendMessage(f, send);
    
    sprintf(send, "%s%d%s %d", "w axis", axis, ".requested_state", AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrive_sendMessage(f, send);
    
    struct timespec ts2 = {0, 10 * 1000000};
    nanosleep(&ts2, &ts2);
    
    sprintf(send, "%s%d%s %f", "w axis", axis, ".config.watchdog_timeout", settings->watchdogTimeout[axis]);
    odrive_sendMessage(f, send);
}

void odrive_sendMessage(int32_T f, uint8_T *message)
{
    FILE *fp = (FILE *)f;
    fprintf(fp, "%s\n", message);
}

void odrive_receiveMessage(int32_T f, uint8_T *message, uint8_T len)
{
    FILE *fp = (FILE *)f;
    fgets(message, len, fp);
}

#ifdef __cplusplus
}
#endif