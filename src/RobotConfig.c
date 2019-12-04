/*
 * RobotConfig.c
 *
 *  Created on: 2019年12月2日
 *      Author: E260-Ben
 */
#include "RobotConfig.h"

void robotPreCpt(const Robot* robot, Robot_CAL* robot_CAL) {
	//计算sin、cos、isZero
	for (int i = 0; i < DOF; i++) {
		robot_CAL->theta_CAL[i].robot_sin = sin(
				robot->robot_joints_coordinate[i]);
		robot_CAL->theta_CAL[i].robot_cos = cos(
				robot->robot_joints_coordinate[i]);
		robot_CAL->forKinSpace.isZero[i] = isNearZero(robot->robot_S[i]);
		robot_CAL->forKinBody.isZero[i] = isNearZero(robot->robot_B[i]);
	}
	//给T赋初值M
	memcpy(robot_CAL->forKinSpace.T, robot->robot_M, sizeof(robot->robot_M));
	memcpy(robot_CAL->forKinBody.T, robot->robot_M, sizeof(robot->robot_M));
	//计算[w]与[w^2]
	for (int i = 0; i < DOF; i++) {
		calOmegaHatMatrix(robot->robot_S, robot_CAL->forKinSpace.omegaHatMat,
				i);
		calOmegaHatMatrix_2(robot->robot_S,
				robot_CAL->forKinSpace.omegaHatMat_2, i);
		calOmegaHatMatrix(robot->robot_B, robot_CAL->forKinBody.omegaHatMat, i);
		calOmegaHatMatrix_2(robot->robot_B, robot_CAL->forKinBody.omegaHatMat_2,
				i);
	}
	//SE3清0
	for (int i = 0; i < DOF; i++) {
		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 4; k++) {
				robot_CAL->forKinSpace.SE3[i][j][k] = 0;
				robot_CAL->forKinBody.SE3[i][j][k] = 0;
			}
	}
}

int isNearZero(const Screw_axis screw_axis) {
	double normOfOmega = sqrt(
			screw_axis[0] * screw_axis[0] + screw_axis[1] * screw_axis[1]
					+ screw_axis[2] * screw_axis[2]);
	if (normOfOmega > NearZero)
		return 0;
	return 1;
}

void calOmegaHatMatrix(const Screw_axis* screw_axis,
		OmegaHatMatrix* omegaHatMatrix, int i) {
	omegaHatMatrix[i][0][0] = 0;
	omegaHatMatrix[i][1][1] = 0;
	omegaHatMatrix[i][2][2] = 0;
	omegaHatMatrix[i][1][0] = screw_axis[i][2];
	omegaHatMatrix[i][2][1] = screw_axis[i][0];
	omegaHatMatrix[i][0][2] = screw_axis[i][1];
	omegaHatMatrix[i][0][1] = -screw_axis[i][2];
	omegaHatMatrix[i][2][0] = -screw_axis[i][1];
	omegaHatMatrix[i][1][2] = -screw_axis[i][0];
}

void calOmegaHatMatrix_2(const Screw_axis* screw_axis,
		OmegaHatMatrix_2* omegaHatMatrix_2, int i) {
	omegaHatMatrix_2[i][0][0] = -(screw_axis[i][1] * screw_axis[i][1]
			+ screw_axis[i][2] * screw_axis[i][2]);
	omegaHatMatrix_2[i][1][1] = -(screw_axis[i][0] * screw_axis[i][0]
			+ screw_axis[i][2] * screw_axis[i][2]);
	omegaHatMatrix_2[i][2][2] = -(screw_axis[i][0] * screw_axis[i][0]
			+ screw_axis[i][1] * screw_axis[i][1]);
	omegaHatMatrix_2[i][0][1] = screw_axis[i][0] * screw_axis[i][1];
	omegaHatMatrix_2[i][0][2] = screw_axis[i][0] * screw_axis[i][2];
	omegaHatMatrix_2[i][1][2] = screw_axis[i][1] * screw_axis[i][2];
	omegaHatMatrix_2[i][1][0] = omegaHatMatrix_2[i][0][1];
	omegaHatMatrix_2[i][2][0] = omegaHatMatrix_2[i][0][2];
	omegaHatMatrix_2[i][2][1] = omegaHatMatrix_2[i][1][2];
}
