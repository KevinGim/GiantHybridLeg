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
#include "dxl_driver.h"
#include "dynamixel_sdk.h" // Uses Dynamixel SDK library
#include "Kinematics.h"

// PH54-200-S500-R Control Table

#define ADDR_Baudrate 8
#define ADDR_Rtrndelay 9
#define ADDR_Drive_Mode 11
#define ADDR_Oper_Mode 11
#define ADDR_CUR_LIMIT 38
#define ADDR_VEL_LIMIT 44
#define ADDR_TORQUE_EN 512
#define ADDR_LED_R 513
#define ADDR_LED_G 514
#define ADDR_LED_B 515
#define ADDR_POS_D 528
#define ADDR_POS_I 530
#define ADDR_POS_P 532
#define ADDR_GOAL_VEL 552
#define ADDR_PROFILE_ACCL 556
#define ADDR_PROFILE_VEL 560
#define ADDR_GOAL_POS 564
#define ADDR_READ_CUR 574
#define ADDR_READ_VEL 576
#define ADDR_READ_POS 580

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

#define BAUDRATE 3000000
#define DEVICENAME "/dev/ttyUSB0" // Check which port is being used on your controller

#define TORQUE_ENABLE 1	 // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque
#define POS_MODE 3

#define ESC_ASCII_VALUE 0x1b
#define PI 3.1412
#define BILLION 1000000000L
#define MILLION 1000000L

int main()
{
	int DXL_ID[5] = {1, 2, 3, 4, 5};
	char input = 0;
	int PROFILE_V = 40;
	int PROFILE_A = 20;
	int DXL_num = sizeof(DXL_ID) / sizeof(DXL_ID[0]);
	long tick = 0;
	long int *motor_angle;
	float x = 0.0, y = 0.0, z = 0.0;
	float x0 = 0.0, y0 = -1700.0, z0 = 0.0;
	float roll = 0.0, pitch = 0.0, yaw = 0.0;
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
	float t_loop1 = 0.0, t_loop2 = 0.0;
	long t_sleep = 0;

	// Initialize PortHandler Structs
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);
	closePort(port_num);
	// Initialize PacketHandler Structs
	packetHandler();

	// Open port
	if (openPort(port_num))
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
	if (setBaudRate(port_num, BAUDRATE))
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

	// Disable Dynamixel Torque :
	// Indirect address would not accessible when the torque is already enabled
	write1(port_num, DXL_ID, DXL_num, ADDR_TORQUE_EN, TORQUE_DISABLE);
	write1(port_num, DXL_ID, DXL_num, ADDR_Oper_Mode, POS_MODE); // Position Mode
	write1(port_num, DXL_ID, DXL_num, ADDR_Rtrndelay, 2);		 // Return Delay Time = 100us

	// LIMIT
	write2(port_num, DXL_ID, DXL_num, ADDR_CUR_LIMIT, 22000);
	write4(port_num, DXL_ID, DXL_num, ADDR_VEL_LIMIT, 2900);

	ind_addr(port_num, DXL_ID, DXL_num, ADDR_INDADDR_WRITE, ADDR_GOAL_POS, LEN_GOAL_POS);
	ind_addr(port_num, DXL_ID, DXL_num, ADDR_INDADDR_READ_POS, ADDR_READ_POS, LEN_READ_POS);
	ind_addr(port_num, DXL_ID, DXL_num, ADDR_INDADDR_READ_VEL, ADDR_READ_VEL, LEN_READ_VEL);
	ind_addr(port_num, DXL_ID, DXL_num, ADDR_INDADDR_READ_CUR, ADDR_READ_CUR, LEN_READ_CUR);

	write1(port_num, DXL_ID, DXL_num, ADDR_TORQUE_EN, TORQUE_ENABLE);

	// Set Profile Vel & Acc
	PROFILE_V = 200; // Slow Down
	PROFILE_A = 100; // Slow Down
	write4(port_num, DXL_ID, DXL_num, ADDR_PROFILE_VEL, PROFILE_V);
	write4(port_num, DXL_ID, DXL_num, ADDR_PROFILE_ACCL, PROFILE_A);

	// PID GAIN
	write2(port_num, DXL_ID, DXL_num, ADDR_POS_P, 8000);
	write2(port_num, DXL_ID, DXL_num, ADDR_POS_D, 2000);
	write2(port_num, DXL_ID, DXL_num, ADDR_POS_I, 500);

	// Initialize XM430 Groupsyncwrite/read instance
	int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_INDDATA_WRITE, LEN_GOAL_POS);
	int groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_INDDATA_READ_POS, LEN_IND_READ);
	addparam_read(groupread_num, DXL_ID, DXL_num);

	// int dxl_comm_result = COMM_TX_FAIL;
	// uint8_t dxl_getdata_result = False;
	// uint8_t dxl_error = 0;

	clock_gettime(CLOCK_MONOTONIC, &t0);

	// While Loop
	while (1)
	{

		clock_gettime(CLOCK_MONOTONIC, &t_start);

		printf("Tick: %6ld  Loop: %2.3f ms  Elapsed Time: %4.2f s       ", tick, t_loop2, t_elapsed);

		input = getKey();

		if ((input == ESC_ASCII_VALUE))
		{
			break;
		}
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		t_init = 5.0;
		t_pause = 0.05;
		x0 = -100.0;
		y0 = -1500.0;
		z0 = 0.0;
		if (t_elapsed < t_init)
		{
			x = x0;
			y = y0;
			z = z0;
		}
		else if (t_init < t_elapsed && t_elapsed < t_init + t_pause)
		{
			PROFILE_V = 2900; 
			PROFILE_A = 8000; 
			write4(port_num, DXL_ID, DXL_num, ADDR_PROFILE_VEL, PROFILE_V);
			write4(port_num, DXL_ID, DXL_num, ADDR_PROFILE_ACCL, PROFILE_A);
			printf("motion Start");
		}
		else
		{
			x = x0 + 0.0 * sin(2 * PI * 0.4 * (t_elapsed - (t_init + t_pause)));
			y = y0 + 0.0 * (1 - cos(2 * PI * 0.4 * (t_elapsed - (t_init + t_pause))));
			z = z0 + 0.0 * sin(2 * PI * 0.4 * (t_elapsed - (t_init + t_pause)));
		}
		printf("x: %4.2f, y: %4.2f, z: %4.2f   ", x, y, z);

		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////// Motion ////////////////////////////////////////////////////////////////////////////
		motor_angle = IK(x, y, z, roll, pitch, yaw);

		for (int i = 0; i < DXL_num; i++)
		{
			motor_command[tick][i] = motor_angle[i];
		}

		//DXL Write Position/*  */
		addparam_write(groupwrite_num, DXL_ID[0], motor_angle[0], LEN_GOAL_POS);
		addparam_write(groupwrite_num, DXL_ID[1], motor_angle[1], LEN_GOAL_POS);
		addparam_write(groupwrite_num, DXL_ID[2], motor_angle[2], LEN_GOAL_POS);
		addparam_write(groupwrite_num, DXL_ID[3], motor_angle[3], LEN_GOAL_POS);
		addparam_write(groupwrite_num, DXL_ID[4], motor_angle[4], LEN_GOAL_POS);
		// addparam_write(groupwrite_num, DXL_ID[5], motor_angle[5], LEN_GOAL_POS);

		// Syncwrite all
		groupSyncWriteTxPacket(groupwrite_num);
		groupSyncWriteClearParam(groupwrite_num);

		// DXL read
		// Syncread present position
		groupSyncReadTxRxPacket(groupread_num);

		for (int i = 0; i < DXL_num; i++)
		{
			DXL_pos[tick][i] = groupSyncReadGetData(groupread_num, DXL_ID[i], ADDR_INDDATA_READ_POS, LEN_READ_POS);
			DXL_vel[tick][i] = groupSyncReadGetData(groupread_num, DXL_ID[i], ADDR_INDDATA_READ_VEL, LEN_READ_VEL);
			DXL_cur[tick][i] = groupSyncReadGetData(groupread_num, DXL_ID[i], ADDR_INDDATA_READ_CUR, LEN_READ_CUR);
			if (DXL_cur[tick][i] > 32768)
			{
				DXL_cur[tick][i] = DXL_cur[tick][i] - 65536;
			}
		}

		for (int i = 0; i < DXL_num; i++)
		{
			err[i] = (long int)motor_command[tick][i] - (long int)DXL_pos[tick][i];
		}

		printf("err: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", err[0], err[1], err[2], err[3], err[4], err[5]);
		// printf("POS: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", motor_angle[0], motor_angle[1], motor_angle[2], motor_angle[3], motor_angle[4], motor_angle[5]);
		// printf("POS: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", DXL_pos[tick][0], DXL_pos[tick][1], DXL_pos[tick][2], DXL_pos[tick][3], DXL_pos[tick][4], DXL_pos[tick][5]);
		// printf("VEL: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", DXL_vel[tick][0], DXL_vel[tick][1], DXL_vel[tick][2], DXL_vel[tick][3], DXL_vel[tick][4], DXL_vel[tick][5]);
		// printf("CUR: DXL1: %6d   DXL2: %6d   DXL3: %6d   DXL4: %6d   DXL5: %6d   DXL6: %6d     ", DXL_cur[tick][0], DXL_cur[tick][1], DXL_cur[tick][2], DXL_cur[tick][3], DXL_cur[tick][4], DXL_cur[tick][5]);
		printf("\n");

		// Time
		clock_gettime(CLOCK_MONOTONIC, &t_end1);
		t_loop = 4000.0;
		diff1 = MILLION * (t_end1.tv_sec - t_start.tv_sec) + (t_end1.tv_nsec - t_start.tv_nsec) / 1000; //usec
		t_loop1 = diff1 * 1e-3;																			// msec
		t_sleep = t_loop - (float)t_loop1 * 1000.0;														// Loop time == 2.50ms

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
		// end of the while loop
	}

	write1(port_num, DXL_ID, DXL_num, ADDR_TORQUE_EN, TORQUE_DISABLE);

	// Close port
	closePort(port_num);

	FILE *fp = fopen("Data_Log.txt", "w");
	for (int j = 1; j < 50000; j++)
	{
		fprintf(fp, "%d       %d       %d       %d       %d       %d       %d       ", t_array[j],
				motor_command[j - 1][0], motor_command[j - 1][1], motor_command[j - 1][2],
				motor_command[j - 1][3], motor_command[j - 1][4], motor_command[j - 1][5]);
		fprintf(fp, "%d       %d       %d       %d       %d       %d       \n",
				DXL_pos[j][0], DXL_pos[j][1], DXL_pos[j][2],
				DXL_pos[j][3], DXL_pos[j][4], DXL_pos[j][5]);
	}
	fclose(fp);

	return 0;
}
