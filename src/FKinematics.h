/*
 * FKinematics.h
 *
 *  Created on: 2019Äê11ÔÂ30ÈÕ
 *      Author: E260-Ben
 */

#ifndef SRC_FKINEMATICS_H_
#define SRC_FKINEMATICS_H_

#include "RobotConfig.h"

static double I[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

void forwardKinematic(const Robot* robot, Robot_CAL* robot_CAL, char flag);
void forwardKinematicSpace(const Robot* robot, Robot_CAL* robot_CAL);
void forwardKinematicBody(const Robot* robot, Robot_CAL* robot_CAL);
void matrixExp6(ForKin_CAL* forKin_CAL, Theta_CAL* theta_CAL,
		const Theta* theta, const Screw_axis* S, int i);
void matrixExp3(ForKin_CAL* forKin_CAL, Theta_CAL* theta_CAL, int i);
void matrixMultSpace(SE3matrix A, SE3matrix B);
void matrixMultBody(SE3matrix A, SE3matrix B);

#endif /* SRC_FKINEMATICS_H_ */
