#ifndef __FMI_IMU_H__
#define __FMI_IMU_H__
#include <vector>
#include <deque>
#include <fstream>
#include <iostream>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "fmi_def.h"

using namespace std;

class FMIIMU
{
public:
    FMIIMU(UartSendApi,UartRecvApi);

    bool Init(ImuCfg_t&);
    bool EnableLog(const char*);
    bool SetMsgOutput(MsgType,ImuUart,int);
    void GetConfiguration();
    bool SetPoleLength(float);
    bool TriPoleCalibration(float);

    bool MsgRecvUpdate();
    bool GetData(NaviData_t&);
    bool GetData(GnssData_t&);
    bool GetData(MemsData_t&);
    bool GetData(CaliData_t&);
	bool GetData(EulerData_t&);
private:
    void Log(bool,const char* format,...);
    bool SendCommand(const char* format,...);
    void Append(uint8_t*,int);
    void BinAnalysis();
    bool AscAnalysis();
    bool CheckSum(int);
    bool DecNaviMsg();
    bool DecGnssMsg();
    bool DecMemsMsg();
    bool DecEulerMsg();
    bool DecCaliMsg();
    MsgType SyncBinHeader();
    MsgType SyncAscHeader(int&,string&,const string&);

    int StrStrBin(uint8_t*,const char*,int,int offset = 0);
    template<typename DataType>
    DataType ConvertAndRemove(bool& error)
    {
        DataType data = 0;
        if(m_buffer.size() < sizeof(data)) error = true;
        else
        {
            memcpy(&data,m_buffer.data(),sizeof(data));
            m_buffer.erase(m_buffer.begin(),m_buffer.begin() + sizeof(data));
        }

        return data;
    }
private:
    vector<uint8_t> m_buffer;
    deque<NaviData_t> m_naviBuffer;
    deque<GnssData_t> m_gnssBuffer;
    deque<MemsData_t> m_memsBuffer;
    deque<CaliData_t> m_caliBuffer;
	deque<EulerData_t> m_eulerBuffer;

    UartSendApi UartSend;
    UartRecvApi UartRecv;

    ofstream m_logFile;
    ImuCfg_t m_imuCfg;
};




#endif
