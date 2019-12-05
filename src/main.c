/*
 * main.c
 *
 *  Created on: 2019年11月30日
 *      Author: E260-Ben
 */
#include <stdio.h>
#include <string.h>
#include "gettime.h"
#include "Kinematics.h"



void inputParameters(Robot* robot);

int main() {
	Robot robot;
	Robot_CAL robot_CAL;
	//输入参数
	inputParameters(&robot);
	//开始计时
	XTime_GetTime(&start);
	//参数预处理
	robotPreCpt(&robot, &robot_CAL);
	//计算正运动学
	forwardKinematic(&robot, &robot_CAL,ALLFrameFlag);
	//计时结束
	XTime_GetTime(&end);
	//显示时间
	u64 time;
	time = (u64) (end - start) * 1000000000 / COUNTS_PER_SECOND;
	printf("CAL TIME is %llu ns.\r\n", time);

	return 0;
}

void inputParameters(Robot* robot) {
	double M[4][4] = { { -1, 0, 0, 0 }, { 0, 1, 0, 6 }, { 0, 0, -1, 2 }, { 0, 0,
			0, 1 } };
	double S[3][6] = { { 0, 0, 1, 4, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, -1,
			-6, 0, -0.1 } };
	double B[3][6] = { { 0, 0, -1, 2, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 1,
			0, 0, 0.1 } };
	double theta[3] = { PI / 2, 3, PI };
	memcpy(robot->robot_M, M, sizeof(M));
	memcpy(robot->robot_S, S, sizeof(S));
	memcpy(robot->robot_B, B, sizeof(B));
	memcpy(robot->robot_joints_coordinate, theta, sizeof(theta));
}
