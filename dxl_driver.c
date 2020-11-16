#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <time.h>
#include "dxl_driver.h"
#include "dynamixel_sdk.h"  


// PH54-200-S500-R Control Table

#define ADDR_Baudrate 8
#define ADDR_Rtrndelay 9
#define ADDR_Drive_Mode 11
#define ADDR_Oper_Mode 11
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
#define ADDR_INDADDR_WRITE 634
#define ADDR_INDADDR_READ_POS (ADDR_INDADDR_WRITE + 2 * LEN_GOAL_POS)
#define ADDR_INDADDR_READ_VEL (ADDR_INDADDR_READ_POS + 2 * LEN_READ_POS)
#define ADDR_INDADDR_READ_CUR (ADDR_INDADDR_READ_VEL + 2 * LEN_READ_VEL)

#define ADDR_INDDATA_WRITE 761
#define ADDR_INDDATA_READ_POS (ADDR_INDDATA_WRITE + LEN_GOAL_POS)
#define ADDR_INDDATA_READ_VEL (ADDR_INDDATA_READ_POS + LEN_READ_POS)
#define ADDR_INDDATA_READ_CUR (ADDR_INDDATA_READ_VEL + LEN_READ_VEL)

// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel


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

char getKey(){
	if(kbhit()) 
	{
		return getch(); 
	}
	return '\0'; 
}


void write1(int port_num, int DXL_ID[], int DXL_num, int ADDR, int Command) {
	for (int i = 0; i < DXL_num; i++) {
		if (DXL_ID[i] != 0) {
			write1ByteTxRx(port_num, 2, DXL_ID[i], ADDR, Command);
		}
	}
}

void write2(int port_num, int DXL_ID[], int DXL_num, int ADDR, int CMD) {
	for (int i = 0; i < DXL_num; i++) {
		if (DXL_ID[i] != 0) {
			write2ByteTxRx(port_num, 2, DXL_ID[i], ADDR, CMD);
		}
	}
}

void write4(int port_num, int DXL_ID[], int DXL_num, int ADDR, int CMD) {
	for (int i = 0; i < DXL_num; i++) {
		if (DXL_ID[i] != 0) {
			write4ByteTxRx(port_num, 2, DXL_ID[i], ADDR, CMD);
		}
	}
}

void ind_addr(int port_num, int DXL_ID[], int DXL_num, int ADDR_IND, int ADDR_DIR, int LEN_DATA) {

	for (int j = 0; j < LEN_DATA; j++) {
		for (int i = 0; i < DXL_num; i++) {
			if (DXL_ID[i] != 0) {
				write2ByteTxRx(port_num, 2, DXL_ID[i], ADDR_IND + 2*j, ADDR_DIR + j);
			}
		}
	}
}

void addparam_read(int groupread_num, int DXL_ID[], int DXL_num){
	for (int i = 0; i < DXL_num; i++) {
		if (DXL_ID[i] != 0) {
			groupSyncReadAddParam(groupread_num, DXL_ID[i]);
			// if (dxl_addparam_result != True)
			// {
			// fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID[i]);
			// // return 0;
			// }
		}
	}
}

void rmparam_read(int groupread_num, int DXL_ID[], int DXL_num){
	for (int i = 0; i < DXL_num; i++) {
		if (DXL_ID[i] != 0) {
			groupSyncReadRemoveParam(groupread_num, DXL_ID[i]);
			// if (dxl_addparam_result != True)
			// {
			// fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", DXL_ID[i]);
			// // return 0;
			// }
		}
	}
}


void addparam_write(int groupwrite_num, int ID, int VALUE, int LEN_DATA){
	if (ID != 0) {
		groupSyncWriteAddParam(groupwrite_num, ID, VALUE, LEN_DATA);
		// if (dxl_addparam_result != True)
		// {
		//   fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed\n", ID);
		// //   return 0;
		// }
	}
}
int* dxl_scan(int port_num){
	static int ID[13];

	for (int j = 0; j <13; j++){
		ID[j] = 0;
	}

	int i = 0;
	for (int id = 1; id < 13; id++) {
		pingGetModelNum(port_num, 2.0, id);
		if ((getLastTxRxResult(port_num, 2.0)) == COMM_SUCCESS)
		{
			ID[i] = id;
			i ++;
		}
		// printf("%d      ", ID[id-1]);
	}

	return ID; 
}

