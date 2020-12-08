
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "dynamixel_sdk.h" // Uses Dynamixel SDK library
#include "Kinematics.h"

// Control table address
// Control table address is different in Dynamixel model
#define ADDR_Baudrate 8
#define ADDR_Rtrndelay 9
#define ADDR_Drive_Mode 11
#define ADDR_Oper_Mode 11
#define ADDR_PWM_LIMIT 36
#define ADDR_CUR_LIMIT 38
#define ADDR_VEL_LIMIT 44
#define ADDR_TORQUE_EN 512
#define ADDR_LED_R 513
#define ADDR_LED_G 514
#define ADDR_LED_B 515
#define ADDR_POS_D 528
#define ADDR_POS_I 530
#define ADDR_POS_P 532
#define ADDR_POS_FF1 538
#define ADDR_GOAL_VEL 552
#define ADDR_PROFILE_ACCL 556
#define ADDR_PROFILE_VEL 560
#define ADDR_GOAL_POS 564
#define ADDR_READ_PWM 572
#define ADDR_READ_CUR 574
#define ADDR_READ_VEL 576
#define ADDR_READ_POS 580
#define ADDR_SHUTDOWN 63

// Data Byte Length
#define LEN_GOAL_POS 4
#define LEN_READ_POS 4
#define LEN_READ_VEL 4
#define LEN_READ_CUR 2

#define LEN_IND_READ (LEN_READ_POS + LEN_READ_CUR + LEN_READ_VEL)

// Indirect Address Parameters
#define ADDR_INDADDR_WRITE 168
#define ADDR_INDADDR_READ_POS (ADDR_INDADDR_WRITE + 2 * LEN_GOAL_POS)
#define ADDR_INDADDR_READ_VEL (ADDR_INDADDR_READ_POS + 2 * LEN_READ_POS)
#define ADDR_INDADDR_READ_CUR (ADDR_INDADDR_READ_VEL + 2 * LEN_READ_VEL)

#define ADDR_INDDATA_WRITE 634
#define ADDR_INDDATA_READ_POS (ADDR_INDDATA_WRITE + LEN_GOAL_POS)
#define ADDR_INDDATA_READ_VEL (ADDR_INDDATA_READ_POS + LEN_READ_POS)
#define ADDR_INDDATA_READ_CUR (ADDR_INDDATA_READ_VEL + LEN_READ_VEL)
// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE 300000
#define DEVICENAME "/dev/ttyUSB_DXL" // Check which port is being used on your controller \
                                  // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque
#define pos_mode 3

#define ESC_ASCII_VALUE 0x1b
#define PI 3.1412
#define BILLION 1000000000L
#define MILLION 1000000L

#define ESC_ASCII_VALUE 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

char getKey()
{
  if (kbhit())
  {
    return getch();
  }
  return '\0';
}
int main()
{
  int DXL_ID[6] = {1, 2, 3, 4, 5, 6};
  char input = 0;
  int PROFILE_V = 40;
  int PROFILE_A = 20;
  int DXL_num = sizeof(DXL_ID) / sizeof(DXL_ID[0]);
  long tick = 0;
  long int *motor_angle;
  double x = 0.0, y = 0.0, z = 0.0;
  double x0 = 0.0, y0 = -1810.0, z0 = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  int err[6] = {
      0,
  };

  int DXL_pos[50000][6] = {
      0,
  };
  int DXL_vel[50000][6] = {
      0,
  };
  int DXL_cur[50000][6] = {
      0,
  };
  int t_array[50000] = {
      0,
  };
  int motor_command[50000][6] = {
      0,
  };

  double t_elapsed = 0.0, t_init = 0.0, t_pause = 0.0, t_loop = 0.0;
  uint64_t diff1, diff2;
  struct timespec t0, t_start, t_end1, t_end2;
  double t_loop1 = 0.0, t_loop2 = 0.0;
  long t_sleep = 0;
  double x_amp = 0.0;
  double y_amp = 0.0;
  double z_min = 10.0;
  double z_height = 0.0;
  double step_time = 0.0;
  double tau = 0.0;
  double duration = 0.0;
  double t_lift_1 = 0.0;
  double t_lift_2 = 0.0;
  double period = 0.0;

  uint8_t dxl_error = 0; // Dynamixel error
  uint8_t param_indirect_data_for_write[LEN_GOAL_POS];
  int dxl_comm_result = COMM_TX_FAIL; // Communication result
  bool dxl_addparam_result = false;   // addParam result
  bool dxl_getdata_result = false;    // GetParam result

  // uint8_t param_indirect_data_for_write[LEN_IND_READ];

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_INDDATA_WRITE, LEN_GOAL_POS);

  // Initialize Groupsyncread instance
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_INDDATA_READ_POS, LEN_IND_READ);

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  for (int i = 0; i < DXL_num; i++)
  {
    // Indirect address would not accessible when the torque is already enabled
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_EN, TORQUE_DISABLE, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_Oper_Mode, pos_mode, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_Rtrndelay, 25, &dxl_error);

    // LIMIT setting
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_CUR_LIMIT, 22740, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_VEL_LIMIT, 2900, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_SHUTDOWN, 26, &dxl_error);

    // Torque Enable
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_EN, TORQUE_ENABLE, &dxl_error);
    // Set Profile Vel & Acc
    PROFILE_V = 400; // Slow Down
    PROFILE_A = 800; // Slow Down
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_PROFILE_VEL, PROFILE_V, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_PROFILE_ACCL, PROFILE_A, &dxl_error);

    // PID GAIN
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_POS_P, 8000, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_POS_D, 2000, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_POS_I, 1000, &dxl_error);
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_POS_FF1, 800, &dxl_error);

    for (int j = 0; j < LEN_GOAL_POS; j++)
    {
      if (DXL_ID[i] != 0)
      {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_INDADDR_WRITE + 2 * j, ADDR_GOAL_POS + j, &dxl_error);
      }
    }

    for (int j = 0; j < LEN_READ_POS; j++)
    {
      if (DXL_ID[i] != 0)
      {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_INDADDR_READ_POS + 2 * j, ADDR_READ_POS + j, &dxl_error);
      }
    }

    for (int j = 0; j < LEN_READ_VEL; j++)
    {
      if (DXL_ID[i] != 0)
      {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_INDADDR_READ_VEL + 2 * j, ADDR_READ_VEL + j, &dxl_error);
      }
    }

    for (int j = 0; j < LEN_READ_CUR; j++)
    {
      if (DXL_ID[i] != 0)
      {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID[i], ADDR_INDADDR_READ_CUR + 2 * j, ADDR_READ_CUR + j, &dxl_error);
      }
    }
    // Add parameter storage for the present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i]);
  }
  printf("Press any key to start\n");
  getchar();

  clock_gettime(CLOCK_MONOTONIC, &t0);

  while (1)
  {
    clock_gettime(CLOCK_MONOTONIC, &t_start);

    printf("Tick: %6ld    Loop: %2.3f ms    Elapsed Time: %4.2f s  ||  \n", tick, t_loop2, t_elapsed);

    input = getKey();
    if ((input == ESC_ASCII_VALUE))
    {
      break;
    }

    // Read Present State
    dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

    for (int i = 0; i < DXL_num; i++)
    {
      dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID[i], ADDR_INDDATA_READ_POS, LEN_READ_POS) + groupSyncRead.isAvailable(DXL_ID[i], ADDR_INDDATA_READ_VEL, LEN_READ_VEL) + groupSyncRead.isAvailable(DXL_ID[i], ADDR_INDDATA_READ_CUR, LEN_READ_CUR);
      printf("dxl_getdata_result: %d \n", dxl_getdata_result);
      if (dxl_getdata_result != 3)
      {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", DXL_ID);
        return 0;
      }

      DXL_pos[tick][i] = groupSyncRead.getData(DXL_ID[i], ADDR_INDDATA_READ_POS, LEN_READ_POS);
      DXL_vel[tick][i] = groupSyncRead.getData(DXL_ID[i], ADDR_INDDATA_READ_VEL, LEN_READ_VEL);
      DXL_cur[tick][i] = groupSyncRead.getData(DXL_ID[i], ADDR_INDDATA_READ_CUR, LEN_READ_CUR);
      if (DXL_cur[tick][i] > 32768)
      {
        DXL_cur[tick][i] = DXL_cur[tick][i] - 65536;
      }
    }
    for (int i = 0; i < DXL_num; i++)
    {
      err[i] = (long int)motor_command[tick][i - 1] - (long int)DXL_pos[tick][i];
    }

    printf("Err       DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", err[0], err[1], err[2], err[3], err[4], err[5]);
    printf("POS: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", motor_angle[0], motor_angle[1], motor_angle[2], motor_angle[3], motor_angle[4], motor_angle[5]);
    printf("POS: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", DXL_pos[tick][0], DXL_pos[tick][1], DXL_pos[tick][2], DXL_pos[tick][3], DXL_pos[tick][4], DXL_pos[tick][5]);
    printf("VEL: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", DXL_vel[tick][0], DXL_vel[tick][1], DXL_vel[tick][2], DXL_vel[tick][3], DXL_vel[tick][4], DXL_vel[tick][5]);
    printf("CUR: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", DXL_cur[tick][0], DXL_cur[tick][1], DXL_cur[tick][2], DXL_cur[tick][3], DXL_cur[tick][4], DXL_cur[tick][5]);
    printf("\n");

    t_init = 2.5;
    t_pause = 0.05;

    x_amp = 0.0;
    y_amp = 800.0;
    z_height = 300.0;
    step_time = 2.0;
    tau = 0.5;
    duration = step_time * tau;
    t_lift_1 = (step_time - duration) / 2.0;
    t_lift_2 = t_lift_1 + step_time;

    x0 = 0.0;
    y0 = 0.0;
    z0 = -1600.0;

    if (t_elapsed < t_init)
    {
      x = x0;
      y = y0;
      z = z0;
    }
    else if (t_init < t_elapsed && t_elapsed < t_init + t_pause)
    {
      PROFILE_V = 1000;
      PROFILE_A = 20000;

      for (int i = 0; i < 6; i++)
      {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_PROFILE_VEL, PROFILE_V, &dxl_error);
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID[i], ADDR_PROFILE_ACCL, PROFILE_A, &dxl_error);
      }
      printf("motion Start");
    }
    else
    {
      period = 3.0;
      x = x0 - 600.0 * sin(2.0 * PI / period * tanh((t_elapsed - (t_init + t_pause)) / 10.0) * (t_elapsed - (t_init + t_pause)));
      y = y0 + 0.0 * sin(2.0 * PI / period * tanh((t_elapsed - (t_init + t_pause)) / 10.0) * (t_elapsed - (t_init + t_pause)));
      z = z0 + 0.0 * (1 - cos(2.0 * PI / period * tanh((t_elapsed - (t_init + t_pause)) / 10.0) * (t_elapsed - (t_init + t_pause))));
    }

    if (z > -1100.0)
    {
      z = -1100.0;
    }
    else if (z < -1810.0)
    {
      z = -1810.0;
    }
    printf("x: %4.1f, y: %4.1f, z: %4.1f   ||  ", x, y, z);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    motor_angle = IK(x, y, z, roll, pitch, yaw);

    for (int i = 0; i < DXL_num; i++)
    {
      motor_command[tick][i] = motor_angle[i];
    }

    for (int i = 0; i < 6; i++)
    {
      param_indirect_data_for_write[0] = DXL_LOBYTE(DXL_LOWORD(motor_angle[i]));
      param_indirect_data_for_write[1] = DXL_HIBYTE(DXL_LOWORD(motor_angle[i]));
      param_indirect_data_for_write[2] = DXL_LOBYTE(DXL_HIWORD(motor_angle[i]));
      param_indirect_data_for_write[3] = DXL_HIBYTE(DXL_HIWORD(motor_angle[i]));
      //DXL Write Position/*  */

      // Add values to the Syncwrite storage
      dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_indirect_data_for_write);
    }

    // Syncwrite all
    dxl_comm_result = groupSyncWrite.txPacket();

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    // Time
    clock_gettime(CLOCK_MONOTONIC, &t_end1);
    t_loop = 8000.0;
    diff1 = MILLION * (t_end1.tv_sec - t_start.tv_sec) + (t_end1.tv_nsec - t_start.tv_nsec) / 1000; //usec
    t_loop1 = diff1 * 1e-3;                                                                         // msec
    t_sleep = t_loop - (double)t_loop1 * 1000.0;                                                    // Loop time == 2.50ms

    if (t_sleep > 0 && t_sleep < 50000)
    {
      usleep(t_sleep - 60);
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end2);
    t_elapsed = (double)((t_end2.tv_sec - t0.tv_sec) * 1000000 + (t_end2.tv_nsec - t0.tv_nsec) / 1000) / 1000000;
    diff2 = MILLION * (t_end2.tv_sec - t_start.tv_sec) + (t_end2.tv_nsec - t_start.tv_nsec) / 1000; //usec
    t_loop2 = diff2 * 1e-3;

    if (tick < 50000)
    {
      t_array[tick] = t_elapsed * 1000;
    }

    tick++;
  }

  // Disable Dynamixel Torque
  for (int i = 0; i < DXL_num; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_EN, TORQUE_DISABLE, &dxl_error);
  }
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
