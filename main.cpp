#include "serialport.h"
#include <sys/stat.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <iomanip>
#include <string>
#include <thread>
#include "fmi_imu.h"

using namespace std;
SerialPort ImuUart;

//Send bytes by uart
void UartSend(uint8_t* buffer,int length)
{
    ImuUart.Write(buffer,length);
}

//Receive bytes by uart, please make sure that the driver of UART has
//its own receive buffer.
int UartRecv(uint8_t* buffer,int size,int timeOut)
{
    return ImuUart.Read(buffer,size,timeOut);
}

int main(int, char**)
{
    if(ImuUart.Open("/dev/ttyUSB0",115200) == false)
    {
        QPrint(0,"/dev/ttyUSB0 Open Fail\r\n");
        return -1;
    }

    ImuCfg_t cfg;

    //Valid PPS edge of Ublox's F9P is rising, let IMU know it.
    cfg.ppsEdge = PPSRising;

    //Config the gnss type, and you need to send GGA RMC GST at least 5Hz
    //to IMU's UART2
    cfg.gnssType = GnssOEM;

    //Config the work mode of IMU as tilt survey mode.
    cfg.workMode = ModeSurvey152;
    cfg.highRate = false;

    //Config the lever-arm,these value should be modify
    //depend on your survey rtk device. More detail on user manual.
    cfg.leverArm[0] = 0;
    cfg.leverArm[1] = 0;
    cfg.leverArm[2] = 0;

    //Config the install angle,these value should be modify
    //depend on your survey rtk device. More detail on user manual.
    cfg.install[0] = 90;
    cfg.install[1] = 90;
    cfg.install[2] = 90;

    FMIIMU imu(UartSend,UartRecv);
    imu.EnableLog("imulog.txt");

    if(imu.Init(cfg) == false) QPrint("Imu Configuration Fail\r\n");
    if(imu.SetPoleLength(1.85) == false) QPrint("Pole Length Configuration Fail\r\n");

    imu.GetConfiguration();

    if(imu.SetMsgOutput(MsgNavi,ImuUart1,1) == false) QPrint("MsgNavi Configuration Fail\r\n");
    if(imu.SetMsgOutput(MsgGnss,ImuUart1,1) == false) QPrint("MsgGnss Configuration Fail\r\n");
    //if(imu.SetMsgOutput(MsgMems,ImuUart1,100) == false) QPrint("MsgMems Configuration Fail\r\n");
    if(imu.SetMsgOutput(MsgEuler,ImuUart1,5) == false) QPrint("MsgEuler Configuration Fail\r\n");

    if(imu.TriPoleCalibration(1.85f) == false) QPrint("TriPoleCalibration Fail\r\n");

    while(1)
    {
        if(imu.MsgRecvUpdate() == true)
        {
            NaviData_t naviData;
            while(imu.GetData(naviData) == true)
            {
                const float Rad2Deg = 180 / 3.1415926f;
                QPrint("Navi:%09.2f,%.3f,%.3f,%.3f\r\n",naviData.utc,naviData.euler[0] * Rad2Deg
                                                    ,naviData.euler[1] * Rad2Deg,naviData.euler[2] * Rad2Deg);
            }

            GnssData_t gnssData;
            while(imu.GetData(gnssData) == true)
            {
                QPrint("Gnss:%09.2f,%.9f,%.9f,%.3f\r\n",gnssData.utc,gnssData.llh[0],gnssData.llh[1],gnssData.llh[2]);
            }

            MemsData_t memsData;
            while(imu.GetData(memsData) == true)
            {
                QPrint("Mems:%09.2f,%.3f,%.3f,%.3f\r\n",memsData.utc,memsData.acc[0],memsData.acc[1],memsData.acc[2]);
            }

            CaliData_t caliData;
            while(imu.GetData(caliData) == true)
            {
                QPrint("Cali:%d,%.3f\r\n",caliData.yawTarget,caliData.yawRealTime);
            }

            EulerData_t eulerData;
            while(imu.GetData(eulerData) == true)
            {
                QPrint("Euler:%09.2f,%.3f,%.3f,%.3f\r\n",eulerData.utc,eulerData.euler[0],eulerData.euler[1],eulerData.euler[2]);
            }
        }
    }

    return 0;
}
