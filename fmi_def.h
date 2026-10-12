#ifndef __FMI_DEF_H__
#define __FMI_DEF_H__
#include <stdint.h>

enum ImuWorkMode
{
    ModeSurvey152 = 152,
    ModeSurvey408 = 408,
    ModeAgHeading = 72,
    ModeVehicleDR = 13
};

enum GnssType
{
    GnssUniCore,
    GnssNovtel,
    GnssHemi,
    GnssOEM
};

enum PPSEdge
{
    PPSRising,
    PPSFalling
};

enum ImuUart
{
    ImuUart1 = 1,
    ImuUart3 = 3,
};

enum MsgType
{
    MsgNavi,
    MsgMems,
    MsgGnss,
    MsgEuler,
    MsgAscii,
    MsgNone,
    MsgEmpty,
    MsgWrong,
};

typedef struct
{
    double utc;
    double llh[3];
    float  vel[3];
    float  euler[3];
    float  quality;
    float  accBias[3];
    float  gyrBias[3];
    float  temperature;
    uint32_t status;
}NaviData_t;

typedef struct
{
    double utc;
    double llh[3];
    float  vel[3];
    float  llhCov[3];
    float  reserved[2];
    float  heading;
    float  hrms;
    float  vrms;
    float  hdop;
    float  vdop;
    uint8_t sats;
    uint8_t sol;
    uint8_t ageDiff;
}GnssData_t;

typedef struct
{
    double utc;
    double llh[3];
    float  acc[3];
    float  gyr[3];
    float  mag[3];
}MemsData_t;

typedef struct
{
    int index;
    int yawTarget;
    float yawRealTime;
    float tiltRealTime;
    uint32_t sampleCount[4];
    uint32_t sampleIndicate[4];
}CaliData_t;

typedef struct
{
    double utc;
	float  euler[3];
	float  euler_var[6];
}EulerData_t;

typedef struct
{
    ImuWorkMode workMode;
    GnssType gnssType;
    PPSEdge ppsEdge;
    float leverArm[3];
    int install[3];
    bool highRate;
}ImuCfg_t;

typedef void (*UartSendApi)(uint8_t*,int);
typedef int  (*UartRecvApi)(uint8_t*,int,int);

#endif
