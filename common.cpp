#include "common.h"
#include <cmath>

//小端转大端
int little2bigi(int le) {
    int a = (le>>24)&0xff;
    int b = (le>>16)&0xff;
    int c = (le>> 8)&0xff;
    int d = le &0xff;
    int e = (d<<24) + (c<<16) + (b<<8) + a;
    return e;
}

//大端转小端
int big2littlei(int be)
{
    return ((be >> 24) &0xff )
        | ((be >> 8) & 0xFF00)
        | ((be << 8) & 0xFF0000)
        | ((be << 24));
}

//小端转大端
short little2bigs(short num)
{
    short a = (num>>8)&0xff;
    short b = num&0xff;
    short c = b*256+a;
    return c;
}

//大端转小端
unsigned short big2littleus(short be)
{
    unsigned short a = be&0xff;
    unsigned short b = (be>>8)&0xff;
    unsigned short c = a*256+b;
    return c;
}

//大端转小端
short big2littles(short be)
{
    short swapped = (be << 8) | (be >> 8);
    return swapped;
}

//校验和，包含首尾
short checksum_char_short(const char* co, const int& start, const int& end)
{
    short sum = 0;
    for(int i = start; i<=end; i++)
    {
        sum = sum+(unsigned char)co[i];
    }
    if(is_little_endian()){
        sum = little2bigs(sum);
    }
    return sum;
}

int fouruchar_to_double(const unsigned char* res, double& out){
    bool li = is_little_endian();
    int i_res = 0x00;
    memcpy(&i_res, res, 4);
    if(li){
        i_res = big2littlei(i_res);
        out = double(i_res);
    } else {
        out = double(i_res);
    }
    return 1;
}

int three_uchar_to_double(const unsigned char* res, double& out){
    bool li = is_little_endian();
    int i_res = 0x00;
    memcpy(&i_res, res, 4);
    if(li){
        i_res = big2littlei(i_res);
    }
    i_res = i_res<<4; //先左移
    i_res = i_res>>4; //算数右移
    out = double(i_res);
    return 1;
}

int twouchar_to_un_double(const unsigned char* res, double& out){
    bool li = is_little_endian();
    unsigned short i_res = 0x00;
    memcpy(&i_res, res, 2);
    if(li){
        i_res = big2littleus(i_res);
        out = double(i_res);
    } else {
        out = double(i_res);
    }
    return 1;
}

int twouchar_to_double(const unsigned char* res, double& out){
    bool li = is_little_endian();
    if(li){
        unsigned char a = res[0];
        unsigned char b = res[1];
        unsigned char c[2] = {b,a};
        short i_res = 0x00;
        memcpy(&i_res, c, 2);
        out = double(i_res);
    } else {
        short i_res = 0x00;
        memcpy(&i_res, res, 2);
        out = double(i_res);
    }
    return 1;
}

int Ascii2Hex(char* ascii, char* hex)
{
    int i, len = strlen(ascii);
    char chHex[] = "0123456789ABCDEF";
    for (i = 0; i < len; i++)
    {
        hex[i * 3] = chHex[((unsigned char)ascii[i]) >> 4];
        hex[i * 3 + 1] = chHex[((unsigned char)ascii[i]) & 0xf];
        hex[i * 3 + 2] = ' ';
    }
    hex[len * 3] = '\0';
    return len * 3;
}

// 时间戳函数 单位：ms
std::time_t getTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());//获取当前时间点
    std::time_t timestamp =  tp.time_since_epoch().count(); //计算距离1970-1-1,00:00的时间长度
    return timestamp;
}
