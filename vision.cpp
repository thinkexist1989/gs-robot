#include "vision.h"

bool char2DCMEASUREDATA(const unsigned char msg[36], DCMEASUREDATA* userData)
{
    if(msg == nullptr){
        return false;
    }
    memcpy(userData, msg, sizeof (DCMEASUREDATA));
    return true;
}

void printDCMEASUREDATA(const DCMEASUREDATA &data)
{
    printf("位:%f %f %f\n", data.camPose.Tx/100.0, data.camPose.Ty/100.0, data.camPose.Tz/100.0);
    printf("姿:%f %f %f\n", data.camPose.Ax/100.0, data.camPose.Ay/100.0, data.camPose.Az/100.0);
}

int parsingDCMEASUREDATA(const unsigned char msg[36], Pose& pose, double& roll, double& pitch, double& yaw){
    if(msg == nullptr){
        return -1;
    }
    unsigned char sum = 0;  // 累加和
    for(unsigned long i = 2; i < sizeof (DCMEASUREDATA)-1; i++)
    {
        sum = sum + msg[i];
    }
    if(sum != msg[sizeof(DCMEASUREDATA)-1]){
        return -2;
    }
    DCMEASUREDATA userData;
    char2DCMEASUREDATA(msg, &userData);
    printDCMEASUREDATA(userData);
    if(userData.ucHead_01!=0x1B || userData.ucHead_02!=0x90 || userData.ucCommand!=0x27){ // head order
        return -3;
    }
    if(userData.camPose.ucTrust != 0x55){  // 置信度
        return -4;
    }
    pose.x = userData.camPose.Tx /100.0/1000.0;
    pose.y = userData.camPose.Ty /100.0/1000.0;
    pose.z = userData.camPose.Tz /100.0/1000.0;
    roll   = userData.camPose.Ax /100.0;
    pitch  = userData.camPose.Ay /100.0;
    yaw    = userData.camPose.Az /100.0;
    KDL::Rotation rot = KDL::Rotation::RPY(roll * KDL::deg2rad, pitch * KDL::deg2rad, yaw * KDL::deg2rad);
    double x,y,z,w;
    rot.GetQuaternion(x,y,z,w);
    pose.xx = x;
    pose.yy = y;
    pose.zz = z;
    pose.ww = w;
    return 1;
}
