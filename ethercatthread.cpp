#include "ethercatthread.h"

EtherCatThread::EtherCatThread()
{
    stop = false;
    servo_CMD.slave_index = 0;
    servo_CMD.cmd = STATE_IDLE;
    servo_CMD.next_cmd = STATE_IDLE;
    servo_CMD.back = NULL;
    servo_CMD.callbackpara = NULL;
    memset(&inputs_CMD, 0x00, sizeof (InputsCMD));
    memset(&temp_CMD, 0x00, sizeof (InputsCMD));
}

void EtherCatThread::stopThread(){
    stop = true;
}

int EtherCatThread::setInputsCMD(const InputsCMD &cmd)
{
    mutex.lock();
    this->inputs_CMD = cmd;
    mutex.unlock();
    return 1;
}

void EtherCatThread::run()
{
    int argc_ether = 2;
    char *argv_ether[] = {"ether_main", "enp172s0"};
    ether_main(argc_ether, argv_ether);
}

int drive_write8(uint16 slave, uint16 index, uint8 subindex, uint8 value)
{
   int wkc;

   wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);

   return wkc;
}

int drive_write16(uint16 slave, uint16 index, uint8 subindex, uint16 value)
{
   int wkc;

   wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);

   return wkc;
}

int drive_write32(uint16 slave, uint16 index, uint8 subindex, int32 value)
{
   int wkc;

   wkc = ec_SDOwrite(slave, index, subindex, FALSE, sizeof(value), &value, EC_TIMEOUTRXM);

   return wkc;
}

// 该函数用于设置PDO映射表
int drive_setup(uint16 slave)
{
   int wkc = 0;
   wkc += drive_write16(slave, 0x1C12, 0, 0);
   printf("1 wkc = %d\n", wkc);
   wkc += drive_write16(slave, 0x1C13, 0, 0);
   printf("2 wkc = %d\n", wkc);
   wkc += drive_write16(slave, 0x1A00, 0, 0);
   printf("3 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1A00, 1, 0x60410010); // Statusword  (ok)
   printf("4 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1A00, 2, 0x60640020); // Position actual value (ok)
   printf("5 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1A00, 3, 0x606C0020); // Velocity actual value (ok)
   printf("6 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1A00, 4, 0x60770010); // Torque actual value  (ok)
   printf("7 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1A00, 5, 0x60610008); // Modes of operation display (ok)
   printf("8 wkc = %d\n", wkc);
   wkc += drive_write8(slave, 0x1A00, 0, 5);
   printf("9 wkc = %d\n", wkc);
   wkc += drive_write8(slave, 0x1600, 0, 0);
   printf("10 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1600, 1, 0x60400010); // Controlword  (ok) wu
   printf("11 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1600, 2, 0x607A0020); // Target position  (ok) you
   printf("12 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1600, 3, 0x60810020); // Target velocity  (ok) wu
   printf("13 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1600, 4, 0x60B20010); // Target torque  (ok) wu
   printf("14 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1600, 5, 0x60600008); // Modes of operation (ok) you
   printf("15 wkc = %d\n", wkc);
   wkc += drive_write32(slave, 0x1600, 6, 0x60830020); // ACC  (ok)  wu
   printf("16 wkc = %d\n", wkc);
   wkc += drive_write8(slave, 0x1600, 0, 6);
   printf("17 wkc = %d\n", wkc);
   wkc += drive_write16(slave, 0x1C12, 1, 0x1600);
   printf("18 wkc = %d\n", wkc);
   wkc += drive_write8(slave, 0x1C12, 0, 1);
   printf("19 wkc = %d\n", wkc);
   wkc += drive_write16(slave, 0x1C13, 1, 0x1A00);
   printf("20 wkc = %d\n", wkc);
   wkc += drive_write8(slave, 0x1C13, 0, 1);
   printf("21 wkc = %d\n", wkc);
   strncpy(ec_slave[slave].name, "Drive", EC_MAXNAME);

   if (wkc != 21)
   {
      printf("Drive %d setup failed\nwkc: %d\n", slave, wkc);
      return -1;
   } else {
       printf("Drive %d setup succeed.\n", slave);
   }
   return 0;
}

bool readM4313Data(uint16_t slave, M4313_TxPDO& data)
{
    if (slave == 0 || slave > ec_slavecount) {
        printf("Invalid slave index: %d\n", slave);
        return false;
    }
    if (ec_slave[slave].inputs == nullptr) {
        printf("Slave input pointer is null\n");
        return false;
    }
    if (ec_slave[slave].Ibytes < sizeof(M4313_TxPDO)) {
        printf("Unexpected input size: %ud, expected at least %ud bytes\n", ec_slave[slave].Ibytes, sizeof(M4313_TxPDO));
        return false;
    }
    std::memcpy(
        &data,
        ec_slave[slave].inputs,
        sizeof(M4313_TxPDO)
    );
    return true;
}

/* add ns to timespec */
void EtherCatThread::add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if (ts->tv_nsec > NSEC_PER_SEC)
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
void EtherCatThread::ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if (delta > (cycletime / 2))
   {
      delta = delta - cycletime;
   }
   if (delta > 0)
   {
      integral++;
   }
   if (delta < 0)
   {
      integral--;
   }
   *offsettime = -(delta / 100) - (integral / 20);
}

inline int64_t EtherCatThread::calcdiff_ns(struct timespec t1, struct timespec t2)
{
   int64_t tdiff;
   tdiff = NSEC_PER_SEC * (int64_t)((int)t1.tv_sec - (int)t2.tv_sec);
   tdiff += ((int)t1.tv_nsec - (int)t2.tv_nsec);
   return tdiff;
}

/* 消除系统时钟偏移函数，取自cyclic_test */
void EtherCatThread::set_latency_target(void)
{
   struct stat s;
   int ret;

   if (stat("/dev/cpu_dma_latency", &s) == 0)
   {
      latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
      if (latency_target_fd == -1)
         return;
      ret = write(latency_target_fd, &latency_target_value, 4);
      if (ret == 0)
      {
         printf("# error setting cpu_dma_latency to %d!: %s\n", latency_target_value, strerror(errno));
         close(latency_target_fd);
         return;
      }
      printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
   }
}

void EtherCatThread::test_driver(char *ifname, int mode)
{
   needlf = FALSE;
   inOP = FALSE;
   int cnt;
   inputs_t  *iptr;
   outputs_t *optr;
   struct sched_param schedp;
   cpu_set_t mask;
   pthread_t thread;
   int chk = 200;

   CPU_ZERO(&mask);
   CPU_SET(2, &mask);
   thread = pthread_self();
   pthread_setaffinity_np(thread, sizeof(mask), &mask);

   memset(&schedp, 0, sizeof(schedp));
   schedp.sched_priority = 99; /* 设置优先级为99，即RT */
   sched_setscheduler(0, SCHED_FIFO, &schedp);

   printf("Starting Redundant test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n", ifname);
      /* find and auto-config slaves */
      if (ec_config_init(FALSE) > 0)
      {
         printf("%d slaves found and configured.\n", ec_slavecount);
         ecx_context.manualstatechange = 1;
         /* wait for all slaves to reach SAFE_OP state */
         int slave_ix;
         /* configure DC options for every DC capable slave found in the list */
         ec_config_map(&IOmap); // 此处调用drive_setup函数，进行PDO映射表设置
         ec_configdc(); // 设置同步时钟，该函数必须在设置pdo映射之后；

         // setup dc for devices
         for (slave_ix = 1; slave_ix <= ec_slavecount; slave_ix++)
         {
            ec_dcsync0(slave_ix, TRUE, EC_CYCLETIME, 0);
         }
         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_slave[0].state = EC_STATE_SAFE_OP;
         ec_writestate(0);
         ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
         for (cnt = 1; cnt <= ec_slavecount; cnt++)
         {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                   cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                   ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
         }
// csp模式没有此处，pp模式有此处代码
//         for(cnt = 1; cnt <= ec_slavecount; cnt++){
//             // note : The following 3 SDO parameters must be written once each time the power is turned on
//             // config profile velocity
//             uint32_t velocity = 65536;
//             ec_SDOwrite(cnt, 0x6081, 0, false, 4, &velocity, EC_TIMEOUTSAFE);
//             // config acc
//             uint32_t acceleration = 100000;
//             ec_SDOwrite(cnt, 0x6083, 0, false, 4, &acceleration, EC_TIMEOUTSAFE);
//             // config jerk : Some versions of the firmware do not allow writing this parameter
//             uint32_t jerk = 10000;
//             ec_SDOwrite(cnt, 0x60A4, 1, false, 4, &jerk, EC_TIMEOUTSAFE);
//         }
         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         /* activate cyclic process data */
         /* wait for all slaves to reach OP state */
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* request OP state for all slaves */
         ec_writestate(0);
         /* wait for all slaves to reach OP state */
         do
         {
             ec_send_processdata();
             ec_receive_processdata(EC_CYCLETIME_US);
             ec_statecheck(0, EC_STATE_OPERATIONAL, EC_CYCLETIME_US);
         } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.\n");
            slave_execute_command();
         }
         else /* ECAT进入OP失败 */
         {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (int i = 1; i <= ec_slavecount; i++)
            {
               if (ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
         /* 断开ECAT通讯 */
         printf("\nRequest safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
         ec_slave[0].state = EC_STATE_PRE_OP;
         ec_writestate(0);
         ec_slave[0].state = EC_STATE_INIT;
         ec_writestate(0);
         ec_readstate();
         if (ec_statecheck(0, EC_STATE_SAFE_OP, 1000) == EC_STATE_INIT)
         {
            printf("ECAT changed into state init\n");
         }
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End driver test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", ifname);
   }
}

int EtherCatThread::slave_execute_command(){
    qRegisterMetaType<inputs_t>("inputs_t");//注册新类型
    qRegisterMetaType<EngDatas>("EngDatas");//注册新类型
    memset(&engDatas, 0x00, sizeof (engDatas));
    inOP = TRUE;
    int slave_index_from_one = 0; // slave_index_from_one 1~6 for joints 1~6. slave_index 7 for all joints.
    inputs_t *iptr;
    outputs_t *optr;
    unsigned int cnt = 1;
    int i = 1;
    while (stop == false)
    {
       osal_usleep(EC_CYCLETIME_US);
       ec_send_processdata();
       wkc = ec_receive_processdata(EC_TIMEOUTRET);  //EC_TIMEOUTRET
       if(cnt % 1 == 0){
           mutex.lock();
           if (inputs_CMD.exe_index == 0xFF)
           {
               temp_CMD = inputs_CMD;
               inputs_CMD.exe_index = 0x00;
           }
           mutex.unlock();
           //开始处理
           if (temp_CMD.exe_index == 0xFF)
           {
               servo_CMD.cmd = temp_CMD.cmd;
               if(servo_CMD.cmd == STATE_1){
                   i = 1;
               }
               if(temp_CMD.cmd == STATE_RUN){
                   current_line_index = 0;
               }
               servo_CMD.slave_index = temp_CMD.slave_index;
               temp_CMD.exe_index = 0x00;
           }
           slave_index_from_one = servo_CMD.slave_index + 1;
           switch (servo_CMD.cmd)
           {
           case STATE_RESET: /* 对驱动器清除故障 */
              iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
              optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
              optr->controlword = 128;
              optr->position = iptr->position;
              servo_CMD.next_cmd = STATE_IDLE;
              break;
           case STATE_1:
              iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
              optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
              optr->controlword = 0x0006;
              optr->ModeOp = CONTROL_MODE;
              printf("STATE_1:Controlword=0x%x, Statusword=%d, position=%d\n", optr->controlword, iptr->statusword, iptr->position);
              if(i<20)
              {
                  servo_CMD.next_cmd = STATE_1;
              }
              else
              {
                  servo_CMD.next_cmd = STATE_2;
              }
              break;
           case STATE_2:
              iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
              optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
              optr->controlword = 0x0086;
              optr->ModeOp = CONTROL_MODE;
              printf("STATE_2:Controlword=0x%x, Statusword=%d, position=%d\n", optr->controlword, iptr->statusword, iptr->position);
              if(i<40)
              {
                  servo_CMD.next_cmd = STATE_2;
              }
              else
              {
                  servo_CMD.next_cmd = STATE_3;
              }
              break;
           case STATE_3:
              iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
              optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
              optr->controlword = 0x0007;
              optr->ModeOp = CONTROL_MODE;
              optr->position = iptr->position;
              printf("STATE_3:Controlword=0x%x, Statusword=%d, position=%d\n", optr->controlword, iptr->statusword, iptr->position);
              if(i<60)
              {
                  servo_CMD.next_cmd = STATE_3;
              }
              else
              {
                  servo_CMD.next_cmd = STATE_4;
              }
              break;
           case STATE_4:
              iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
              optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
              // motor enable
              optr->controlword = 0x000F;
              optr->ModeOp = CONTROL_MODE;
              printf("STATE_4:Controlword=0x%x, Statusword=%d, position=%d\n", optr->controlword, iptr->statusword, iptr->position);
              if(i < 80)
              {
                  servo_CMD.next_cmd = STATE_4;
              }
              else
              {
                  servo_CMD.next_cmd = STATE_5;
              }
              break;
           case STATE_5:
              iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
              optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
              optr->controlword = 0x001F;
              optr->ModeOp = CONTROL_MODE;
              printf("STATE_5:Controlword=0x%x, Statusword=%d, position=%d\n", optr->controlword, iptr->statusword, iptr->position);
              if(i < 100)
              {
                  servo_CMD.next_cmd = STATE_5;
              }
              else
              {
                  servo_CMD.next_cmd = STATE_IDLE;
              }
              break;
           case STATE_RUN:
              if(slave_index_from_one == 8){
                  // for all joints.
                  if(current_line_index < temp_CMD.profileDatas[0].position.size()){
                      for (int i = 0; i < 7; i++)
                      {
                         optr = (outputs_t *)ec_slave[i + 1].outputs;
                         //position. 这个目标位置是绝对位置
                         optr->position = temp_CMD.profileDatas[i].position.at(current_line_index);
                      }
                      current_line_index++;
                      servo_CMD.next_cmd = STATE_RUN;
                  } else {
                      current_line_index = 0;
                      servo_CMD.next_cmd = STATE_IDLE;
                      printf("sendRunOver\n");
                      emit sendRunOver();
                  }
              } else {
                  iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
                  optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
                  //position. 这个目标位置是绝对位置
                  if(current_line_index < temp_CMD.profileDatas[servo_CMD.slave_index].position.size()){
                      optr->position = temp_CMD.profileDatas[servo_CMD.slave_index].position.at(current_line_index);
                      current_line_index++;
                      servo_CMD.next_cmd = STATE_RUN;
                  } else {
                      current_line_index = 0;
                      servo_CMD.next_cmd = STATE_IDLE;
                      printf("sendRunOver\n");
                      emit sendRunOver();
                  }
                  printf("STATE_RUN:Controlword=0x%x, Statusword=%d\n", optr->controlword, iptr->statusword);
              }
              break;
           case STATE_DISABLE:
              iptr = (inputs_t  *)ec_slave[slave_index_from_one].inputs;
              optr = (outputs_t *)ec_slave[slave_index_from_one].outputs;
              optr->ModeOp = 0;
              optr->velocity = 0;
              optr->controlword = 6;
              servo_CMD.next_cmd = STATE_IDLE;
              printf("STATE_DISABLE:Controlword=0x%x, Statusword=%d\n", optr->controlword, iptr->statusword);
              break;
           case STATE_IDLE:
              servo_CMD.next_cmd = STATE_IDLE;
              break;
           default:
              servo_CMD.next_cmd = STATE_IDLE;
              break;
           }
           if (servo_CMD.back)
           {
               servo_CMD.back(servo_CMD.callbackpara);
           }
           servo_CMD.cmd = servo_CMD.next_cmd;
           servo_CMD.next_cmd = STATE_IDLE;
       }
       for (int j = 0; j < 7; j++)
       {
          iptr = (inputs_t  *) ec_slave[j + 1].inputs;
          engDatas.ActualStatus[j].statusword = iptr->statusword;
          engDatas.ActualStatus[j].position  = iptr->position;
          engDatas.ActualStatus[j].velocity  = iptr->velocity;
          engDatas.ActualStatus[j].torque    = iptr->torque;
          engDatas.ActualStatus[j].ModeOp    = iptr->ModeOp;
       }
       M4313_TxPDO ft;
       if (readM4313Data(8, ft) && cnt % 100 == 0) {
           printf("No=%u, %f, %f, %f, %f, %f, %f\n", ft.data_no, ft.fx, ft.fy, ft.fz, ft.mx, ft.my, ft.mz);
       }
       if (cnt % 1 == 0){
           // reduce the Hz
           emit sendEngDatas(engDatas);
       }
       fflush(stdout);
       cnt++;
       i++;
    }
    return 1;
}

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;
    EtherCatThread* etherThread = (EtherCatThread*)ptr;; /* Not used */

    while (1)
    {
        if (etherThread->inOP && ((etherThread->wkc < etherThread->expectedWKC) || ec_group[etherThread->currentgroup].docheckstate))
        {
            if (etherThread->needlf)
            {
                etherThread->needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[etherThread->currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == etherThread->currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[etherThread->currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_CYCLETIME_US))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET * 5);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_CYCLETIME_US))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[etherThread->currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int EtherCatThread::ether_main(int argc, char *argv[])
{
   int mode;
   printf("SOEM (Simple Open EtherCAT Master)\nRedundancy test\n");
   if (argc > 1)
   {
      set_latency_target(); // 消除系统时钟偏移
      /* create thread to handle slave error handling in OP */
      osal_thread_create(&thread1, stack64k * 4, (void*)&ecatcheck, this);
      /* start acyclic part */
      test_driver(argv[1], mode);
   }
   else
   {
      printf("Usage: red_test ifname1 Mode_of_operation\nifname = eth0 for example\n");
   }
   printf("End program\n");
   return (0);
}
