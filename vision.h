#ifndef VISION_H
#define VISION_H

#include <string.h>
#include <stdio.h>
#include "straightlinetrajectorygenerator.h"

#pragma pack(push, 1)
struct DCMEASURECMD
{
    unsigned char 	ucHead_01;
    unsigned char 	ucHead_02;
    unsigned char 	ucCommand;
    unsigned char 	ucLen;
    unsigned char ucOthers[3];	    // 预留
    unsigned char ucCheckSum;	    // 校验和
};
struct MPOSE
{
    unsigned char 	ucTrust;
    int    Tx;
    int    Ty;
    int    Tz;
    int    Ax;
    int    Ay;
    int    Az;
};
struct DCMEASUREDATA
{
    unsigned char 	ucHead_01;
    unsigned char 	ucHead_02;
    unsigned char 	ucCommand;
    unsigned char 	ucLen;
    MPOSE           camPose;
    unsigned short usCostTime;	// ms
    unsigned char  ucIdx;
    unsigned char  ucOthers[3];	// 预留
    unsigned char  ucCheckSum;       // 校验和
};
#pragma pack(pop)

bool char2DCMEASUREDATA(const unsigned char msg[36], DCMEASUREDATA* );
void printDCMEASUREDATA(const DCMEASUREDATA& data);
int parsingDCMEASUREDATA(const unsigned char msg[36], Pose& pose, double& roll, double& pitch, double& yaw); //roll: deg

#endif // VISION_H
