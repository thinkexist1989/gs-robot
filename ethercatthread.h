#ifndef ETHERCATTHREAD_H
#define ETHERCATTHREAD_H

#include <QThread>
#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <vector>

// C标准库头文件
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cmath>
#include <cerrno>

// Linux/POSIX系统头文件
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <pthread.h>
#include <termios.h>

// EtherCAT库
#ifdef __cplusplus
extern "C" {
#endif
#include "ethercat.h"
#ifdef __cplusplus
}
#endif

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define EEP_MAN_SYNAPTICON (0x000022d2)
#define EEP_ID_SYNAPTICON (0x00000201)
#define stack64k (64 * 1024)
#define EC_CYCLETIME 10000000
#define EC_CYCLETIME_US (EC_CYCLETIME / 1000)  //10ms
#define CNT_PER_CYCLE 65536       //16位电机
#define BIG_JOINT_DACC 161
#define SMALL_JOINT_DACC 121
#define CNT_PER_DEBUG 200        //csp模式，每个周期给目标位置加一点
// 控制模式定义
#define MODE_PT 0xFF
#define MODE_PP 0x01
#define MODE_CSP 0x08
#define MODE_CSV 0x09
#define MODE_CST 0x0A
#define CONTROL_MODE MODE_CSP

#pragma pack(push, 1)
struct M4313_RxPDO
{
    int16_t para1;   // 0x7010:01
    int16_t para2;   // 0x7010:02
    int16_t para3;   // 0x7010:03
};
struct M4313_TxPDO   // 六维力/力矩
{
    uint16_t data_no; // 0x6030:01
    float fx;         // 0x6030:02
    float fy;         // 0x6030:03
    float fz;         // 0x6030:04
    float mx;         // 0x6030:05
    float my;         // 0x6030:06
    float mz;         // 0x6030:07
};
#pragma pack(pop)

typedef enum CMD_TYPE {
    STATE_RESET = 0x00,
    STATE_1,   //  初始状态
    STATE_2,
    STATE_3,
    STATE_4,
    STATE_5,
    STATE_6,
    STATE_7,
    STATE_RUN,
    STATE_DISABLE,
    STATE_IDLE              //空指令 什么也不做
} CMD_TYPE;

typedef void(*cmd_callback)(void*);

typedef struct ServoCommand
{
    uint8           slave_index; // slave_index 0~5 for joints 1~6. slave_index 6 for all joints.
    CMD_TYPE		cmd;
    CMD_TYPE		next_cmd;
    cmd_callback	back;
    void*			callbackpara;
} ServoCommand;

#pragma pack(push, 1)
struct inputs
{
    uint16_t statusword;
    int32_t position;
    int32_t velocity;
    int16_t torque;
    int8_t ModeOp;
    int8_t reserved;
};
typedef struct inputs inputs_t;

struct outputs
{
    uint16_t controlword;
    int32_t position;
    int32_t velocity;
    int16_t torque;
    int8_t ModeOp;
    int8_t reserved;
};
typedef struct outputs outputs_t;
#pragma pack(pop)


typedef struct outputs_v
{
    uint16_t controlword;
    int32_t velocity;
    int16_t torque;
    int8_t ModeOp;
    int8_t reserved;
    std::vector <int32_t> position;
} outputs_v;

typedef struct InputsCMD
{
    uint8           slave_index; // slave_index 0~5 for joints 1~6. slave_index 6 for all joints.
    uint8           exe_index;
    CMD_TYPE        cmd;
    outputs_v       profileDatas[7];
} InputsCMD;

typedef struct EngDatas
{
    inputs_t    ActualStatus[7];
} EngDatas;

OSAL_THREAD_FUNC ecatcheck(void *ptr);
int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value);
int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value);
int drive_write32(uint16 slave, uint16 index, uint8 subindex, int32 value);
int drive_setup(uint16 slave);
OSAL_THREAD_FUNC ecatcheck(void *ptr);
bool readM4313Data(uint16_t slave, M4313_TxPDO& data);

class EtherCatThread : public QThread
{
    Q_OBJECT
public:
    volatile int wkc;
    boolean inOP = FALSE;
    uint8 currentgroup = 0;
    int current_line_index = 0;
    int expectedWKC;
    boolean needlf;
    explicit EtherCatThread();
    int setInputsCMD(const InputsCMD &cmd);
    void stopThread();

signals:
    void sendEngDatas(EngDatas engDatas);
    void sendRunOver();
protected:
    void    run() Q_DECL_OVERRIDE;  //线程任务
public:
    mutable QMutex mutex;
    bool stop;
    ServoCommand servo_CMD;
    InputsCMD inputs_CMD;
    InputsCMD temp_CMD;
    EngDatas engDatas;
    int latency_target_fd = -1;
    int32_t latency_target_value = 0;
    struct sched_param schedp;
    char IOmap[4096];
    pthread_t thread1, thread2;
    uint64_t diff, maxt, avg, cycle;
    struct timeval t1, t2;
    int64 toff = 0;
    void add_timespec(struct timespec *ts, int64 addtime);
    void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
    void set_latency_target(void);
    inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2);
    void test_driver(char *ifname, int mode);
    int ether_main(int argc, char *argv[]);
    int slave_execute_command();
};

#endif // ETHERCATTHREAD_H
