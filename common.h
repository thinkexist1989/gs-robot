#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <array>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <chrono>

struct RobotParam
{
    char cmd[4];//'P'
    float x;
    float y;
    float z;
    //姿态，欧拉角
    float alf;
    float beta;
    float gama;
};

template <class T>
int getArrayLen(T&array)
{
    return sizeof(array) / sizeof(array[0]);
}

inline bool is_little_endian()
{
    bool islittle = true;
    int a = 0x1234;
    char b =  *(char *)&a;  //通过将int强制类型转换成char单字节，通过判断起始存储位置。即等于 取b等于a的低地址部分
    if(b == 0x12)
    {
        islittle = false;
    } else {
        islittle = true;
    }
    return islittle;
}
int little2bigi(int le);
short little2bigs(short num);
int big2littlei(int be);
short big2littles(short be);
unsigned short big2littleus(short be);
short checksum_char_short(const char* co, const int& start, const int& end);
int fouruchar_to_double(const unsigned char* res, double& out);
int three_uchar_to_double(const unsigned char* res, double& out);
int twouchar_to_un_double(const unsigned char* res, double& out);
int twouchar_to_double(const unsigned char* res, double& out);
int Ascii2Hex(char* ascii, char* hex);
std::time_t getTimeStamp();

#endif // COMMON_H
