/*
 * RobotConfig.c
 *
 *  Created on: 2019年12月2日
 *      Author: E260-Ben
 */
#include "RobotConfig.h"

void robotInit(const Robot* robot, Robot_CAL* robot_CAL) {
	//计算sin、cos、isZero
	for (int i = 0; i < DOF; i++) {
		robot_CAL->robot_sin[i] = sin(robot->robot_joints_coordinate[i]);
		robot_CAL->robot_cos[i] = cos(robot->robot_joints_coordinate[i]);
		robot_CAL->robot_IsZero_space[i] = isNearZero(robot->robot_S[i]);
	}
	//给T_Space赋初值M
	memcpy(robot_CAL->robot_T_space, robot->robot_M, sizeof(robot->robot_M));
	//计算[w]与[w^2]
	for (int i = 0; i < DOF; i++) {
		calOmegaHatMatrix(robot->robot_S, robot_CAL->robot_omegaHatMat_space, i);
		calOmegaHatMatrix_2(robot->robot_S, robot_CAL->robot_omegaHatMat_2_space, i);
	}
	//SE3清0
	for (int i = 0; i < DOF; i++) {
		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 4; k++)
				robot_CAL->robot_SE3_space[i][j][k] = 0;
	}
}

int isNearZero(const Screw_axis s) {
	double normOfOmega = sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
	if (normOfOmega > NearZero)
		return 0;
	return 1;
}

void calOmegaHatMatrix(const Screw_axis* S, OmegaHatMatrix* omegaHatMatrix, int i) {
	omegaHatMatrix[i][0][0] = 0;
	omegaHatMatrix[i][1][1] = 0;
	omegaHatMatrix[i][2][2] = 0;
	omegaHatMatrix[i][1][0] = S[i][2];
	omegaHatMatrix[i][2][1] = S[i][0];
	omegaHatMatrix[i][0][2] = S[i][1];
	omegaHatMatrix[i][0][1] = -S[i][2];
	omegaHatMatrix[i][2][0] = -S[i][1];
	omegaHatMatrix[i][1][2] = -S[i][0];
}

void calOmegaHatMatrix_2(const Screw_axis* S, OmegaHatMatrix_2* omegaHatMatrix_2,
		int i) {
	omegaHatMatrix_2[i][0][0] = -(S[i][1] * S[i][1] + S[i][2] * S[i][2]);
	omegaHatMatrix_2[i][1][1] = -(S[i][0] * S[i][0] + S[i][2] * S[i][2]);
	omegaHatMatrix_2[i][2][2] = -(S[i][0] * S[i][0] + S[i][1] * S[i][1]);
	omegaHatMatrix_2[i][0][1] = S[i][0] * S[i][1];
	omegaHatMatrix_2[i][0][2] = S[i][0] * S[i][2];
	omegaHatMatrix_2[i][1][2] = S[i][1] * S[i][2];
	omegaHatMatrix_2[i][1][0] = omegaHatMatrix_2[i][0][1];
	omegaHatMatrix_2[i][2][0] = omegaHatMatrix_2[i][0][2];
	omegaHatMatrix_2[i][2][1] = omegaHatMatrix_2[i][1][2];
}
