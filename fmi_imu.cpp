#include "fmi_imu.h"
#include <stdarg.h>
#include <string.h>

FMIIMU::FMIIMU(UartSendApi send,UartRecvApi recv):UartSend(send),UartRecv(recv)
{

}

bool FMIIMU::Init(ImuCfg_t &cfg)
{
    bool success = true;

    SendCommand("AT+OUTPUT_DISABLE=UART1\r\n");
    SendCommand("AT+OUTPUT_DISABLE=UART3\r\n");

    if(cfg.ppsEdge == PPSRising) success &= SendCommand("AT+SET_PPS_EDGE=RISING\r\n");
    else success &= SendCommand("AT+SET_PPS_EDGE=FALLING\r\n");

    success &= SendCommand("AT+WORK_MODE=%d\r\n",cfg.workMode);
    success &= SendCommand("AT+INSTALL_ANGLE=%d,%d,%d\r\n",cfg.install[0],cfg.install[1],cfg.install[2]);
    success &= SendCommand("AT+LEVER_ARM2=%.3f,%.3f,%.3f\r\n",cfg.leverArm[0],cfg.leverArm[1],cfg.leverArm[2]);

    if((cfg.workMode == ModeVehicleDR) && (cfg.highRate == true))
        success &= SendCommand("AT+HIGH_RATE=ENABLE\r\n");

    switch (cfg.gnssType)
    {
        case GnssUniCore:
            success &= SendCommand("AT+GNSS_CARD=UNICORE\r\n");
            break;
        case GnssNovtel:
            success &= SendCommand("AT+GNSS_CARD=NOVTEL\r\n");
            break;
        case GnssHemi:
            success &= SendCommand("AT+GNSS_CARD=HEMI\r\n");
            break;
        default:
            success &= SendCommand("AT+GNSS_CARD=OEM\r\n");
            break;
    }

    success &= SendCommand("AT+SAVE_ALL\r\n");
    // success &= SendCommand("AT+UART_BRIDGE=UART2,UART1\r\n");
    m_imuCfg = cfg;
    return success;
}

void FMIIMU::GetConfiguration()
{
    char rxBuffer[2048] = {'\0'};

    UartSend((uint8_t*)"AT+VERSION\r\n",12);
    UartRecv((uint8_t*)rxBuffer,sizeof(rxBuffer),200);
    Log(false,"%s",(char*)rxBuffer);

    UartSend((uint8_t*)"AT+READ_PARA=ALL\r\n",18);
    UartRecv((uint8_t*)rxBuffer,sizeof(rxBuffer),200);
    Log(false,"%s",(char*)rxBuffer);
}

bool FMIIMU::SetPoleLength(float length)
{
    return SendCommand("AT+CLUB_VECTOR=0,0,%.3f\r\n",length);
}

bool FMIIMU::TriPoleCalibration(float length)
{
    if(SendCommand("AT+CLUB_VECTOR=0,0,%.3f\r\n",length) == false) return false;
    if(SendCommand("AT+AUTO_FIX=DISABLE\r\n",length) == false) return false;
    if(SendCommand("AT+FIX_ANGLE=0,0,0\r\n",length) == false) return false;
    if(SendCommand("AT+AUTO_FIX=ENABLE\r\n",length) == false) return false;

    return true;
}

bool FMIIMU::SetMsgOutput(MsgType type,ImuUart port,int rate)
{
    switch (type)
    {
        case MsgNavi:
            if(m_imuCfg.workMode == ModeSurvey152 || m_imuCfg.workMode == ModeSurvey408)
            {
                if(rate > 0) return SendCommand("AT+NAVI_OUTPUT=UART%d,ON\r\n",port);
                else return SendCommand("AT+NAVI_OUTPUT=UART%d,OFF\r\n",port);
            }
            else if(m_imuCfg.workMode == ModeVehicleDR)
            {
                if(rate > 0 && m_imuCfg.highRate == false) return SendCommand("AT+NAVI_OUTPUT=UART%d,ON\r\n",port);
                else if(rate > 0) return SendCommand("AT+NAVI_OUTPUT=UART%d,%d\r\n",port,rate);
                else return SendCommand("AT+NAVI_OUTPUT=UART%d,OFF\r\n",port);
            }
            else
            {
                if(rate > 0) return SendCommand("AT+NAVI_OUTPUT=UART%d,%d\r\n",port,rate);
                else return SendCommand("AT+NAVI_OUTPUT=UART%d,OFF\r\n",port);
            }
            break;

        case MsgGnss:
            if(rate > 0) return SendCommand("AT+GNSS_OUTPUT=UART%d,ON\r\n",port);
            else return SendCommand("AT+GNSS_OUTPUT=UART%d,OFF\r\n",port);
            break;

        case MsgMems:
            if(rate > 0) return SendCommand("AT+MEMS_OUTPUT=UART%d,%d\r\n",port,rate);
            else return SendCommand("AT+MEMS_OUTPUT=UART%d,OFF\r\n",port);
            break;

        case MsgEuler:
            if(rate > 0) return SendCommand("AT+EULER_OUTPUT=UART%d,%d\r\n",port,rate);
            else return SendCommand("AT+EULER_OUTPUT=UART%d,OFF\r\n",port);
            break;

        default:
            break;
    }

    return false;
}

bool FMIIMU::EnableLog(const char* file)
{
    m_logFile.open(file,std::ios::out | std::ios::binary);
    return m_logFile.is_open();
}

bool FMIIMU::MsgRecvUpdate()
{
    uint8_t buffer[512];
    memset(buffer,0,sizeof(buffer));
    int length = UartRecv(buffer,sizeof(buffer),10);
    if(length <= 0) return false;

    Log(true,(char*)buffer,length);
    Append(buffer,length);

    return true;
}

bool FMIIMU::SendCommand(const char *format,...)
{
    char txBuffer[128];
    va_list p;
    va_start(p, format);
    int length = vsnprintf(txBuffer,sizeof(txBuffer),format, p);
    va_end(p);

    int retry = 3;
    while(retry-- > 0)
    {
        UartSend((uint8_t*)txBuffer,length);

        uint8_t rxBuffer[1024];
        length = UartRecv(rxBuffer,sizeof(rxBuffer),100);
        if(length <= 0) continue;
        if(StrStrBin(rxBuffer,"OK",length) >= 0)
        {
            Log(false,"Cmd Cfg Pass:%s",txBuffer);
            return true;
        }
    }

    Log(false,"Cmd Cfg Fail:%s",txBuffer);
    return false;
}

void FMIIMU::Append(uint8_t *data, int length)
{
    //append binary to the end of buffer
    vector<uint8_t> tmp(length);
    memcpy(tmp.data(),data,length);

    m_buffer.insert(m_buffer.end(),tmp.begin(),tmp.end());
    if(m_buffer.size() > 2048)
    {
        int remove = m_buffer.size() - 2048;
        m_buffer.erase(m_buffer.begin(),m_buffer.begin() + remove);
    }
    //when we got ascii msg header, but there are not enough bytes in buffer.
    //we will stop binary msg decode, until we got ascii msg tail or there are
    //too much bytes in buffer.
    if(AscAnalysis() == false) BinAnalysis();
}

bool FMIIMU::AscAnalysis()
{
    if(DecCaliMsg() == true) return true;
    ////some others ascii message can be decoded here.

    return false;
}

bool FMIIMU::DecCaliMsg()
{
    int offset = 0;
    while(1)
    {
        string msg;
        MsgType type = SyncAscHeader(offset,msg,"FixRls");
        if(type == MsgNone) return false;
        if(type == MsgEmpty) return true;   //exit, when bytes in buffer is not enough.

        CaliData_t caliMsg;
        const char format[] = "FixRls:%d,%d,%d,%d,%d,%d,%f,%f,%x %x %x %x";

        int count = sscanf(msg.c_str(),format,&caliMsg.yawTarget,&caliMsg.index,
                                               caliMsg.sampleCount,caliMsg.sampleCount + 1,
                                               caliMsg.sampleCount + 2,caliMsg.sampleCount + 3,
                                              &caliMsg.yawRealTime,&caliMsg.tiltRealTime,
                                               caliMsg.sampleIndicate,caliMsg.sampleIndicate + 1,
                                               caliMsg.sampleIndicate + 2,caliMsg.sampleIndicate + 3);


        if(count == 12) m_caliBuffer.push_back(caliMsg);

    }

    return false;
}

void FMIIMU::BinAnalysis()
{
    while(1)
    {
        //find msg header
        switch (SyncBinHeader())
        {
            //exit, when bytes in buffer is not enough.
            case MsgEmpty:
                return;

            case MsgNavi:
                DecNaviMsg();
                break;

            case MsgGnss:
                DecGnssMsg();
                break;

            case MsgMems:
                DecMemsMsg();
                break;
            
            case MsgEuler:
                DecEulerMsg();
                break;

            default:
                m_buffer.erase(m_buffer.begin());
                break;
        }
    }
}

bool FMIIMU::GetData(NaviData_t &naviData)
{
    if(m_naviBuffer.size() > 0)
    {
        naviData = m_naviBuffer.front();
        m_naviBuffer.pop_front();
        return true;
    }

    return false;
}

bool FMIIMU::GetData(GnssData_t &gnssData)
{
    if(m_gnssBuffer.size() > 0)
    {
        gnssData = m_gnssBuffer.front();
        m_gnssBuffer.pop_front();
        return true;
    }

    return false;
}

bool FMIIMU::GetData(MemsData_t &memsData)
{
    if(m_memsBuffer.size() > 0)
    {
        memsData = m_memsBuffer.front();
        m_memsBuffer.pop_front();
        return true;
    }

    return false;
}

bool FMIIMU::GetData(CaliData_t &caliData)
{
    if(m_caliBuffer.size() > 0)
    {
        caliData = m_caliBuffer.front();
        m_caliBuffer.pop_front();
        return true;
    }

    return false;
}

bool FMIIMU::GetData(EulerData_t &eulerData)
{
    if(m_eulerBuffer.size() > 0)
    {
        eulerData = m_eulerBuffer.front();
        m_eulerBuffer.pop_front();
        return true;
    }

    return false;
}

bool FMIIMU::DecNaviMsg()
{
    NaviData_t naviData;
    bool error = false;
    ConvertAndRemove<uint32_t>(error);

    naviData.utc    = ConvertAndRemove<double>(error);
    naviData.llh[0] = ConvertAndRemove<double>(error);
    naviData.llh[1] = ConvertAndRemove<double>(error);
    naviData.llh[2] = ConvertAndRemove<double>(error);
    naviData.vel[0] = ConvertAndRemove<float>(error);
    naviData.vel[1] = ConvertAndRemove<float>(error);
    naviData.vel[2] = ConvertAndRemove<float>(error);
    naviData.euler[0] = ConvertAndRemove<float>(error);
    naviData.euler[1] = ConvertAndRemove<float>(error);
    naviData.euler[2] = ConvertAndRemove<float>(error);
    naviData.quality  = ConvertAndRemove<float>(error);
    naviData.accBias[0] = ConvertAndRemove<float>(error);
    naviData.accBias[1] = ConvertAndRemove<float>(error);
    naviData.accBias[2] = ConvertAndRemove<float>(error);
    naviData.gyrBias[0] = ConvertAndRemove<float>(error);
    naviData.gyrBias[1] = ConvertAndRemove<float>(error);
    naviData.gyrBias[2] = ConvertAndRemove<float>(error);
    naviData.temperature = ConvertAndRemove<float>(error);
    naviData.status = ConvertAndRemove<uint32_t>(error);

    ConvertAndRemove<uint32_t>(error);
    if(error == false) m_naviBuffer.push_back(naviData);
    else if(m_buffer.size() > 0) m_buffer.erase(m_buffer.begin());

    return !error;
}

bool FMIIMU::DecGnssMsg()
{
    GnssData_t gnssData;
    bool error = false;
    ConvertAndRemove<uint32_t>(error);

    gnssData.utc    = ConvertAndRemove<double>(error);
    gnssData.llh[0] = ConvertAndRemove<double>(error);
    gnssData.llh[1] = ConvertAndRemove<double>(error);
    gnssData.llh[2] = ConvertAndRemove<double>(error);
    gnssData.vel[0] = ConvertAndRemove<float>(error);
    gnssData.vel[1] = ConvertAndRemove<float>(error);
    gnssData.vel[2] = ConvertAndRemove<float>(error);
    gnssData.llhCov[0] = ConvertAndRemove<float>(error);
    gnssData.llhCov[1] = ConvertAndRemove<float>(error);
    gnssData.llhCov[2] = ConvertAndRemove<float>(error);
    gnssData.reserved[0] = ConvertAndRemove<float>(error);
    gnssData.reserved[1] = ConvertAndRemove<float>(error);
    gnssData.heading     = ConvertAndRemove<float>(error);
    gnssData.hrms = ConvertAndRemove<float>(error);
    gnssData.vrms = ConvertAndRemove<float>(error);
    gnssData.hdop = ConvertAndRemove<float>(error);
    gnssData.vdop = ConvertAndRemove<float>(error);
    gnssData.sats = ConvertAndRemove<uint8_t>(error);
    gnssData.sol  = ConvertAndRemove<uint8_t>(error);
    gnssData.ageDiff = ConvertAndRemove<uint8_t>(error);

    ConvertAndRemove<uint32_t>(error);
    if(error == false) m_gnssBuffer.push_back(gnssData);
    else if(m_buffer.size() > 0) m_buffer.erase(m_buffer.begin());

    return !error;
}

bool FMIIMU::DecMemsMsg()
{
    MemsData_t memsData;
    bool error = false;
    ConvertAndRemove<uint32_t>(error);

    memsData.utc    = ConvertAndRemove<double>(error);
    memsData.acc[0] = ConvertAndRemove<float>(error);
    memsData.acc[1] = ConvertAndRemove<float>(error);
    memsData.acc[2] = ConvertAndRemove<float>(error);
    memsData.gyr[0] = ConvertAndRemove<float>(error);
    memsData.gyr[1] = ConvertAndRemove<float>(error);
    memsData.gyr[2] = ConvertAndRemove<float>(error);
    memsData.mag[0] = ConvertAndRemove<float>(error);
    memsData.mag[1] = ConvertAndRemove<float>(error);
    memsData.mag[2] = ConvertAndRemove<float>(error);

    ConvertAndRemove<uint32_t>(error);
    if(error == false) m_memsBuffer.push_back(memsData);
    else if(m_buffer.size() > 0) m_buffer.erase(m_buffer.begin());

    return !error;
}

bool FMIIMU::DecEulerMsg()
{
    EulerData_t eulerData;
    bool error = false;
    ConvertAndRemove<uint32_t>(error);

    eulerData.utc    = ConvertAndRemove<double>(error);
    eulerData.euler[0] = ConvertAndRemove<float>(error);
    eulerData.euler[1] = ConvertAndRemove<float>(error);
    eulerData.euler[2] = ConvertAndRemove<float>(error);
    eulerData.euler_var[0] = ConvertAndRemove<float>(error);
    eulerData.euler_var[1] = ConvertAndRemove<float>(error);
    eulerData.euler_var[2] = ConvertAndRemove<float>(error);
    eulerData.euler_var[3] = ConvertAndRemove<float>(error);
    eulerData.euler_var[4] = ConvertAndRemove<float>(error);
    eulerData.euler_var[5] = ConvertAndRemove<float>(error);

    ConvertAndRemove<uint32_t>(error);
    if(error == false) m_eulerBuffer.push_back(eulerData);
    else if(m_buffer.size() > 0) m_buffer.erase(m_buffer.begin());

    return !error;
}

MsgType FMIIMU::SyncAscHeader(int &offset,string &msg,const string& header)
{
    size_t headerLength = header.length();
    if(offset >= (int)m_buffer.size()) return MsgNone;
    if(m_buffer.size() < headerLength) return MsgNone;

    int headerIndex = StrStrBin(m_buffer.data(),header.c_str(),m_buffer.size(),offset);
    if(headerIndex < 0) return MsgNone;

    int tailIndex = StrStrBin(m_buffer.data(),"\r\n",m_buffer.size(),headerIndex);
    if((tailIndex < 0) && (m_buffer.size() < (size_t)(headerIndex + 512))) return MsgEmpty;
    if(tailIndex < 0) return MsgNone;

    for(int i = headerIndex;i < tailIndex;i++)
    {
        uint8_t byte = m_buffer[i];
        if(isascii(byte) == 0) return MsgWrong;
        msg.push_back(byte);
    }

    tailIndex += 2;
    offset = tailIndex;
    return MsgAscii;
}

MsgType FMIIMU::SyncBinHeader()
{
    if(m_buffer.size() < 4) return MsgEmpty;
    if(m_buffer[0] != 'f') return MsgNone;
    if(m_buffer[1] != 'm') return MsgNone;
    if(m_buffer[2] != 'i') return MsgNone;

    switch(m_buffer[3])
    {
        case 'n':
            if(m_buffer.size() < 100) return MsgEmpty;
            if(CheckSum(100) == false) return MsgNone;
            return MsgNavi;

        case 'g':
            if(m_buffer.size() < 95) return MsgEmpty;
            if(CheckSum(95) == false) return MsgNone;
            return MsgGnss;

        case 'm':
            if(m_buffer.size() < 52) return MsgEmpty;
            if(CheckSum(52) == false) return MsgNone;
            return MsgMems;

        case 'e':
            if(m_buffer.size() < 52) return MsgEmpty;
            if(CheckSum(52) == false) return MsgNone;
            return MsgEuler;

        default: return MsgNone;
    }

    return MsgNone;
}

bool FMIIMU::CheckSum(int msgLength)
{
    if((int)m_buffer.size() < msgLength) return false;

    uint16_t checkSum1 = 0;
    uint16_t checkSum2 = m_buffer[msgLength - 4] | (m_buffer[msgLength - 3] << 8);
    uint16_t msgEnd = m_buffer[msgLength - 2] | (m_buffer[msgLength - 1] << 8);

    for(int i = 0;i < msgLength - 4;i++)
    {
        checkSum1 += m_buffer[i];
    }

    if(checkSum1 != checkSum2) return false;
    if(msgEnd != 0X6465) return false;
    return true;
}

int FMIIMU::StrStrBin(uint8_t *bytes, const char *string, int length,int offset)
{
    int strLength = strlen(string);
    length -= strLength;

    for(int i = offset;i < length;i++)
    {
        bool got = true;
        for(int j = 0;j < strLength;j++)
        {
            if(bytes[i + j] != string[j])
            {
                got = false;
                break;
            }
        }

        if(got == true) return i;
    }

    return -1;
}

void FMIIMU::Log(bool binMsg, const char *format, ...)
{
    char buffer[2048];
    if(!binMsg)
    {
        va_list p;
        va_start(p, format);
        int length = vsnprintf(buffer,sizeof(buffer),format, p);
        va_end(p);

        if(length <= 0) return;

        std::cout << buffer << std::flush;
        if(m_logFile.is_open()) m_logFile.write(buffer,length);
    }
    else
    {
        va_list p;
        va_start(p, format);
        int length = va_arg(p,int);
        va_end(p);

        if(length <= 0) return;
        if(m_logFile.is_open()) m_logFile.write(format,length);
    }
}
