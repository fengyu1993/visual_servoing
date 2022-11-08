//
// Created by cyh on 2020/7/14.
// 参考《Modern Robotics Mechanics , Planning , and Control》
// python的函数库写的这个c++函数库
// 所有函数均测试通过，但不排除特殊情况
// 如果发现bug, 请通知我改正...
// 邮箱：3120185244@bit.edu.cn
//

#ifndef MODERNROBOTICS_MODERN_ROBOTICS_LIB_H
#define MODERNROBOTICS_MODERN_ROBOTICS_LIB_H

#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;

bool NearZear(double z);

Vector3d  Normalize(Vector3d  V);

Matrix3d RotInv(Matrix3d R);

Matrix3d VecToso3(Vector3d omg);

Vector3d so3ToVec(Matrix3d  omg);

void AxisAng3(Vector3d expc3, Vector3d& unitV, double& theta);

Matrix3d MatrixExp3(Matrix3d so3mat);

Matrix3d MatrixLog3(Matrix3d R);

void RpToTrans(Matrix3d R, Vector3d p, Matrix4d& T);

void TransToRp(Matrix4d T, Matrix3d& R, Vector3d& p);

Matrix4d TransInv(Matrix4d T);

Matrix4d VecTose3(VectorXd V);

VectorXd se3ToVec(Matrix4d se3);

MatrixXd Adjoint(Matrix4d T);

VectorXd ScrewToAxis(Vector3d q, Vector3d s, double h);

void AxisAng6(VectorXd expc6, VectorXd& S, double& theta);

Matrix4d MatrixExp6(Matrix4d se3mat);

MatrixXd MatrixLog6(Matrix4d T);

MatrixXd SlistToBlist(MatrixXd Slist, Matrix4d M);

MatrixXd BlistToSlist(MatrixXd Blist, Matrix4d M);

Matrix4d FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist);

Matrix4d FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist);

MatrixXd JacobianBody(MatrixXd Blist,VectorXd thetalist);

MatrixXd JacobianSpace(MatrixXd Slist,VectorXd thetalist);

Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd & origin);

bool IKinBody(MatrixXd Blist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist);

bool IKinSpace(MatrixXd Slist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist);

bool IKinSpace_POE(MatrixXd Slist, Matrix4d M, Matrix4d T_eef, VectorXd thetalist0, int method, VectorXd& thetalist);

MatrixXd ad(VectorXd V);

VectorXd InverseDynamics(VectorXd thetalist, VectorXd dthetalist, VectorXd ddthetalist, Vector3d g, VectorXd Ftip, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist);

MatrixXd MassMatrix(VectorXd thetalist, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist);

VectorXd VelQuadraticForces(VectorXd thetalist, VectorXd dthetalist, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist);

VectorXd GravityForces(VectorXd thetalist, Vector3d g, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist);

VectorXd EndEffectorForces(VectorXd thetalist,VectorXd Ftip, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist);

VectorXd ForwardDynamics(VectorXd thetalist, VectorXd dthetalist, VectorXd taulist, Vector3d g, VectorXd Ftip, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist);

void EulerStep(VectorXd thetalist, VectorXd dthetalist, VectorXd ddthetalist, double dt, VectorXd& thetalistNext, VectorXd& dthetalistNext);

MatrixXd InverseDynamicsTrajectory(MatrixXd thetamat, MatrixXd dthetamat, MatrixXd ddthetamat, Vector3d g, MatrixXd Ftipmat, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist);

void ForwardDynamicsTrajectory(VectorXd thetalist, VectorXd dthetalist, MatrixXd taumat, Vector3d g, MatrixXd Ftipmat, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist, double dt, int intRes, MatrixXd& thetamat, MatrixXd& dthetamat);

double CubicTimeScaling(double Tf, double t);

double QuinticTimeScaling(double Tf, double t);

MatrixXd JointTrajectory(VectorXd thetastart,VectorXd thetaend,double Tf,int N,int method);

void ScrewTrajectory(Matrix4d Xstart, Matrix4d Xend, double Tf, int N, int method, Matrix4d X[]);

void CartesianTrajectory(Matrix4d Xstart, Matrix4d Xend, double Tf, int N, int method, Matrix4d X[]);

VectorXd ComputedTorque(VectorXd thetalist, VectorXd dthetalist, VectorXd eint, Vector3d g, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist,
                        VectorXd thetalistd, VectorXd dthetalistd, VectorXd ddthetalistd, double Kp, double Ki, double Kd);

#endif //MODERNROBOTICS_MODERN_ROBOTICS_LIB_H