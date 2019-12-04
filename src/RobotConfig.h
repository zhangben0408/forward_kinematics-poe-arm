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
/*
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

/* robot_T_space			: the configuration of robot's end-effector
 * 				  			  based on the fixed space frame by given joints coordinate
 * robot_T_body				: the configuration of robot's end-effector
 * 				 		      based on the body frame by given joints coordinate
 * robot_sin				: the sin value of all joints coordinates
 * robot_cos				: the cos value of all joints coordinates
 * robot_IsZero_space        		: judge the omega is near zero or not
 * robot_SE3_space				: the SE3 matrix of all joints
 * */
typedef struct {
	//sin & cos
	Theta_sin robot_sin[DOF];
	Theta_cos robot_cos[DOF];
	//in the fixed space frame
	T_matrix robot_T_space;
	OmegaNearZero robot_IsZero_space[DOF];
	SE3matrix robot_SE3_space[DOF];
	OmegaHatMatrix robot_omegaHatMat_space[DOF];
	OmegaHatMatrix_2 robot_omegaHatMat_2_space[DOF];
	//in the body frame
	T_matrix robot_T_body;
} Robot_CAL;

void robotInit(const Robot* robot, Robot_CAL* robot_CAL);
int isNearZero(const Screw_axis s);
void calOmegaHatMatrix(const Screw_axis* S, OmegaHatMatrix* omegaHatMatrix,
		int i);
void calOmegaHatMatrix_2(const Screw_axis* S,
		OmegaHatMatrix_2* omegaHatMatrix_2, int i);
#endif /* SRC_ROBOTCONFIG_H_ */
