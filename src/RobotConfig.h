/*
 * RobotConfig.h
 *
 *  Created on: 2019Äê11ÔÂ30ÈÕ
 *      Author: E260-Ben
 */

#ifndef SRC_ROBOTCONFIG_H_
#define SRC_ROBOTCONFIG_H_
#include <math.h>
#include <string.h>

#define DOF				3
#define PI				3.14159265358979323846
#define NearZero		0.000001

#define BaseFrameFlag				0x00
#define EndEffectorFrameFlag	 	0xFF

typedef double M_matrix[4][4];
typedef double T_matrix[4][4];
typedef double Screw_axis[6];
typedef double Theta;
typedef double SE3matrix[4][4];
typedef double Theta_sin;
typedef double Theta_cos;
typedef int OmegaNearZero;
typedef double OmegaHatMatrix[3][3];
typedef double OmegaHatMatrix_2[3][3];


/* The input parameters
 * robot_M					: the configuration of robot's end-effector
 * 							  when it is at its zero position
 * robot_S					: the lists of screw axis S (S has DOF lists)
 * robot_B					: the lists of screw axis B (B has DOF lists)
 * robot_joints_coordinate	: array of robot_joints_coordinate
 * */
typedef struct {
	M_matrix robot_M;
	Screw_axis robot_S[DOF];
	Screw_axis robot_B[DOF];
	Theta robot_joints_coordinate[DOF];
} Robot;

/* sin and cos value of given theta
 * */
typedef struct{
	Theta_sin robot_sin;
	Theta_cos robot_cos;
}Theta_CAL;

/* T			: the configuration of robot's end-effector by given joints coordinate
 * isZero       : judge the omega is near zero or not
 * SE3			: the SE3 matrix of all joints
 * omegaHatMat  : [omega]
 * omegaHatMat_2: [omega]^2
 **/
typedef struct{
	T_matrix T;
	OmegaNearZero isZero[DOF];
	SE3matrix SE3[DOF];
	OmegaHatMatrix omegaHatMat[DOF];
	OmegaHatMatrix_2 omegaHatMat_2[DOF];
}ForKin_CAL;

typedef struct {
	Theta_CAL theta_CAL[DOF];
	ForKin_CAL forKinSpace;
	ForKin_CAL forKinBody;
} Robot_CAL;

void robotInit(const Robot* robot, Robot_CAL* robot_CAL);
int isNearZero(const Screw_axis s);
void calOmegaHatMatrix(const Screw_axis* S, OmegaHatMatrix* omegaHatMatrix,
		int i);
void calOmegaHatMatrix_2(const Screw_axis* S,
		OmegaHatMatrix_2* omegaHatMatrix_2, int i);
#endif /* SRC_ROBOTCONFIG_H_ */
