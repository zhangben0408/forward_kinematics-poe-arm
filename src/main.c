/*
 * main.c
 *
 *  Created on: 2019Äê11ÔÂ30ÈÕ
 *      Author: E260-Ben
 */
#include <stdio.h>
#include <string.h>
#include "FKinematics.h"
#include "gettime.h"



int main() {

	Robot robot;
	Robot_CAL robot_CAL;

	double M[4][4] = { { -1, 0, 0, 0 }, { 0, 1, 0, 6 }, { 0, 0, -1, 2 }, { 0, 0,
			0, 1 } };
	double S[3][6] = { { 0, 0, 1, 4, 0, 0 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, -1,
			-6, 0, -0.1 } };
	double theta[3] = { PI / 2, 3, PI };
	memcpy(robot.robot_M, M, sizeof(M));
	memcpy(robot.robot_S, S, sizeof(S));
	memcpy(robot.robot_joints_coordinate, theta, sizeof(theta));

	XTime_GetTime(&start);
	robotInit(&robot, &robot_CAL);

	forwardKinematic(&robot, &robot_CAL);
	XTime_GetTime(&end);
	u64 time;
	time = (u64) (end - start) * 1000000000 / COUNTS_PER_SECOND;
	printf("CAL TIME is %d ns.\r\n", (u32) time);

	return 0;
}

