/*
 * FKinematic.c
 *
 *  Created on: 2019Äê11ÔÂ30ÈÕ
 *      Author: E260-Ben
 */
#include "Kinematics.h"

void forwardKinematic(const Robot* robot, Robot_CAL* robot_CAL, char flag) {
	if (flag == BaseFrameFlag)
		forwardKinematicSpace(robot, robot_CAL);
	else if (flag == EndEffectorFrameFlag)
		forwardKinematicBody(robot, robot_CAL);
	else if (flag == ALLFrameFlag) {
		forwardKinematicSpace(robot, robot_CAL);
		forwardKinematicBody(robot, robot_CAL);
	}
}

void forwardKinematicSpace(const Robot* robot, Robot_CAL* robot_CAL) {
	for (int i = 0; i < DOF; i++)
		matrixExp6(&(robot_CAL->forKinSpace), robot_CAL->theta_CAL,
				robot->robot_joints_coordinate, robot->robot_S, i);
	for (int i = DOF - 1; i >= 0; i--) {
		matrixMultSpace(robot_CAL->forKinSpace.SE3[i],
				robot_CAL->forKinSpace.T);
	}
}

void forwardKinematicBody(const Robot* robot, Robot_CAL* robot_CAL) {
	for (int i = 0; i < DOF; i++)
		matrixExp6(&(robot_CAL->forKinBody), robot_CAL->theta_CAL,
				robot->robot_joints_coordinate, robot->robot_B, i);
	for (int i = 0; i < DOF; i++) {
		matrixMultBody(robot_CAL->forKinBody.T, robot_CAL->forKinBody.SE3[i]);
	}
}

void matrixExp3(ForKin_CAL* forKin_CAL, Theta_CAL* theta_CAL, int i) {
	for (int j = 0; j < 3; j++)
		for (int k = 0; k < 3; k++) {
			forKin_CAL->SE3[i][j][k] = I[j][k]
					+ theta_CAL[i].robot_sin * forKin_CAL->omegaHatMat[i][j][k]
					+ (1 - theta_CAL[i].robot_cos)
							* forKin_CAL->omegaHatMat_2[i][j][k];
		}
}

void matrixExp6(ForKin_CAL* forKin_CAL, Theta_CAL* theta_CAL,
		const Theta* theta, const Screw_axis* screw_axis, int i) {
	if (forKin_CAL->isZero[i] == 1) {
		forKin_CAL->SE3[i][0][0] = 1;
		forKin_CAL->SE3[i][1][1] = 1;
		forKin_CAL->SE3[i][2][2] = 1;
		forKin_CAL->SE3[i][0][3] = theta[i] * screw_axis[i][3];
		forKin_CAL->SE3[i][1][3] = theta[i] * screw_axis[i][4];
		forKin_CAL->SE3[i][2][3] = theta[i] * screw_axis[i][5];

	} else {
		//R
		matrixExp3(forKin_CAL, theta_CAL, i);
		//*
		double temp[3][3];
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++) {
				temp[j][k] = I[j][k] * theta[i]
						+ (1 - theta_CAL[i].robot_cos)
								* forKin_CAL->omegaHatMat[i][j][k]
						+ (theta[i] - theta_CAL[i].robot_sin)
								* forKin_CAL->omegaHatMat_2[i][j][k];
			}
		for (int j = 0; j < 3; j++) {
			forKin_CAL->SE3[i][j][3] = temp[j][0] * screw_axis[i][3]
					+ temp[j][1] * screw_axis[i][4]
					+ temp[j][2] * screw_axis[i][5];
		}
	}
	//0 0 0 1
	for (int j = 0; j < 3; j++) {
		forKin_CAL->SE3[i][3][j] = 0;
	}
	forKin_CAL->SE3[i][3][3] = 1;
}
void matrixMultSpace(SE3matrix A, SE3matrix B) {
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
void matrixMultBody(SE3matrix A, SE3matrix B) {
	SE3matrix A_temp;
	memcpy(A_temp, A, sizeof(A_temp));
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			A[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				A[i][j] += A_temp[i][k] * B[k][j];
			}
		}
	}
}
void adjoint(const SE3matrix T, AdT adT) {
	double p_so3[3][3] = { { 0, -T[2][3], T[1][3] }, { T[2][3], 0, -T[0][3] }, {
			-T[1][3], T[0][3], 0 } };
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			if ((i < 3) && (j < 3)) {
				adT[i][j] = T[i][j];
			} else if ((i < 3) && (j >= 3)) {
				adT[i][j] = 0;
			} else if ((i >= 3) && (j < 3)) {
				adT[i][j] = 0;
				for(int k=0;k<3;k++)
				{
					adT[i][j]+=p_so3[i-3][k]*T[k][j];
				}
			} else {
				adT[i][j] = T[i - 3][j - 3];
			}
		}
	}
}

