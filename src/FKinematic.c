/*
 * FKinematic.c
 *
 *  Created on: 2019Äê11ÔÂ30ÈÕ
 *      Author: E260-Ben
 */
#include "FKinematics.h"

void forwardKinematic(const Robot* robot, Robot_CAL* robot_CAL) {
	forwardKinematicSpace(robot, robot_CAL);
}

void forwardKinematicSpace(const Robot* robot, Robot_CAL* robot_CAL) {
	for (int i = 0; i < DOF; i++)
		matrixExp6(robot_CAL, robot->robot_joints_coordinate, robot->robot_S,
				i);
	for (int i = DOF - 1; i >= 0; i--) {
		matrixMult(robot_CAL->robot_SE3_space[i], robot_CAL->robot_T_space);
	}
}

void forwardKinematicBody(const Robot* robot, Robot_CAL* robot_CAL) {

}

void matrixExp3(Robot_CAL* robot_CAL, int i) {
	for (int j = 0; j < 3; j++)
		for (int k = 0; k < 3; k++) {
			robot_CAL->robot_SE3_space[i][j][k] = I[j][k]
					+ robot_CAL->robot_sin[i]
							* robot_CAL->robot_omegaHatMat_space[i][j][k]
					+ (1 - robot_CAL->robot_cos[i])
							* robot_CAL->robot_omegaHatMat_2_space[i][j][k];
		}
}

void matrixExp6(Robot_CAL* robot_CAL, const Theta* theta, const Screw_axis* S,
		int i) {
	if (robot_CAL->robot_IsZero_space[i] == 1) {
		robot_CAL->robot_SE3_space[i][0][0] = 1;
		robot_CAL->robot_SE3_space[i][1][1] = 1;
		robot_CAL->robot_SE3_space[i][2][2] = 1;
		robot_CAL->robot_SE3_space[i][0][3] = theta[i] * S[i][3];
		robot_CAL->robot_SE3_space[i][1][3] = theta[i] * S[i][4];
		robot_CAL->robot_SE3_space[i][2][3] = theta[i] * S[i][5];

	} else {
		//R
		matrixExp3(robot_CAL, i);
		//*
		double temp[3][3];
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++) {
				temp[j][k] = I[j][k] * theta[i]
						+ (1 - robot_CAL->robot_cos[i])
								* robot_CAL->robot_omegaHatMat_space[i][j][k]
						+ (theta[i] - robot_CAL->robot_sin[i])
								* robot_CAL->robot_omegaHatMat_2_space[i][j][k];
			}
		for (int j = 0; j < 3; j++) {
			robot_CAL->robot_SE3_space[i][j][3] = temp[j][0] * S[i][3]
					+ temp[j][1] * S[i][4] + temp[j][2] * S[i][5];
		}
	}
	//0 0 0 1
	for (int j = 0; j < 3; j++) {
		robot_CAL->robot_SE3_space[i][3][j] = 0;
	}
	robot_CAL->robot_SE3_space[i][3][3] = 1;
}
void matrixMult(SE3matrix A, SE3matrix B) {
	SE3matrix B_temp;
	memcpy(B_temp, B, sizeof(B_temp));
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			B[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				B[i][j] += A[i][k] * B_temp[k][j];
			}
		}
	}
}
