#include "modern_robotics_lib.h"

/*
    Takes a scalar.
    Checks if the scalar is small enough to be neglected.
    Example Input:
    z = -1e-7
    Output:
    True
    Code:
    cout << NearZear(0.2) << endl;
    cout << NearZear(1e-8) << endl;
*/
bool NearZear(double z){
    return abs(z) < 1e-6;
}

/*
    Takes a vector.
    Scales it to a unit vector.
    Example Input:
    V = [1, 2, 3]
    Output:
    [0.2672612419124244, 0.5345224838248488, 0.8017837257372732]
    Code:
    Vector3d  V(1.0, 2.0, 3.0);
    Vector3d norm_v = Normalize(V);
    cout << norm_v << endl;
*/
Vector3d  Normalize(Vector3d  V){ 
    return V / V.norm();
}

/*
    Takes a 3x3 rotation matrix.
    Returns the inverse (transpose).

    Example Input:
    R = [[0, 0, 1],
         [1, 0, 0],
         [0, 1, 0]]
    Output:
    [[0, 1, 0],
     [0, 0, 1],
     [1, 0, 0]]
     Code:
    Matrix3d R;
    R << 0, 0, 1, 1, 0, 0, 0, 1, 0;
    Matrix3d R_T = RotInv(R);
    cout << R_T << endl;
 */
Matrix3d RotInv(Matrix3d R){
    return R.transpose();
}

/*
    Takes a 3-vector (angular velocity).
    Returns the skew symmetric matrix in so3.
    Example Input:
    omg = [1, 2, 3]
    Output:
    [[ 0, -3,  2],
     [ 3,  0, -1],
     [-2,  1,  0]]
     Code:
    Vector3d omg(1.0, 2.0, 3.0);
    MatrixXd so3 = VecToso3(omg);
    cout << so3 << endl;
*/
Matrix3d VecToso3(Vector3d omg){
    Matrix3d so3 = Matrix3d::Zero(3, 3);
    so3(0,1) = -omg(2);   so3(0,2) = omg(1);
    so3(1,0) = omg(2);    so3(1,2) = -omg(0);
    so3(2,0) = -omg(1);   so3(2,1) = omg(0);
    return so3;
}

/*
    Takes a 3x3 skew-symmetric matrix (an element of so(3)).
    Returns the corresponding vector (angular velocity).
    Example Input:
    so3mat = [[ 0, -3,  2],
              [ 3,  0, -1],
              [-2,  1,  0]]
    Output:
    [1, 2, 3]
    Code
    Vector3d v = so3ToVec(so3);
    cout << v << endl;
 */
Vector3d so3ToVec(Matrix3d  omg){
    Vector3d v(omg(2,1), omg(0,2), omg(1,0));
    return v;
}

/*
    Takes A 3-vector of exponential coordinates for rotation.
    Returns unit rotation axis omghat and the corresponding rotation angle
    theta.
    Example Input:
    expc3 = [1, 2, 3]
    Output:
    ([0.2672612419124244, 0.5345224838248488, 0.8017837257372732],
     3.7416573867739413)
     Code:
    Vector3d expc3(1.0, 2.0, 3.0);
    Vector3d unitV;
    double theta;
    AxisAng3(expc3, unitV, theta);
    cout << unitV << endl;
    cout << theta << endl;
 */
void AxisAng3(Vector3d expc3, Vector3d& unitV, double& theta){
    unitV = Normalize(expc3);
    theta = expc3.norm();
}

/*
    Takes a so(3) representation of exponential coordinates.
    Returns R in SO(3) that is achieved by rotating about omghat by theta from
    an initial orientation R = I.
    Example Input:
    so3mat = [[ 0, -3,  2],
          [ 3,  0, -1],
              [-2,  1,  0]]
    Output:
    [[-0.69492056,  0.71352099,  0.08929286],
     [-0.19200697, -0.30378504,  0.93319235],
     [ 0.69297817,  0.6313497 ,  0.34810748]]
     Code:
    Matrix3d so3mat;
    so3mat << 0.0, -3.0, 2.0, 3.0, 0.0, -1.0, -2.0, 1.0, 0.0;
    cout << MatrixExp3(so3mat) << endl;
    so3mat = Matrix3d::Zero();
    cout << MatrixExp3(so3mat) << endl;
 */
Matrix3d MatrixExp3(Matrix3d so3mat){
    Vector3d omgtheta = so3ToVec(so3mat);
    if (NearZear(omgtheta.norm()))
        return Matrix3d::Identity();
    else{
        double theta = omgtheta.norm();
        Matrix3d omgmat = so3mat / theta;
        return  Matrix3d::Identity() + sin(theta)*omgmat + (1 - cos(theta))*omgmat*omgmat;
    }
}

/*
    Takes R (rotation matrix).
    Returns the corresponding so(3) representation of exponential coordinates.
    Example Input:
    R = [[0, 0, 1],
         [1, 0, 0],
         [0, 1, 0]]
    Output:
    [[          0, -1.20919958,  1.20919958],
     [ 1.20919958,           0, -1.20919958],
     [-1.20919958,  1.20919958,           0]]
     Code:
    MatrixXd R(3,3);
    R << 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    cout << MatrixLog3(R) << endl;
 */
Matrix3d MatrixLog3(Matrix3d R){
    Vector3d omg;
    if (NearZear((R - Matrix3d::Identity()).maxCoeff()))
        return Matrix3d::Zero();
    else if (NearZear(R.trace() + 1)){
        if (! NearZear(1 + R(2,2))){
            Vector3d v(R(0,2), R(1,2), 1 + R(2,2));
            omg = (1.0 / sqrt(2 * (1 + R(2,2)))) * v;
        }
        else if (! NearZear(1 + R(1,1))){
            Vector3d v(R(0,1), 1 + R(1,1), R(2,1));
            omg = (1.0 / sqrt(2 * (1 + R(1,1)))) * v;
        }
        else{
            Vector3d v(1 + R(0,0), R(1,0), R(2,0));
            omg = (1.0 / sqrt(2 * (1 + R(0,0)))) * v;
        }
        return VecToso3(M_PI * omg);
    }
    else{
        double acosinput = (R.trace() - 1) / 2.0;
        if (acosinput > 1)
            acosinput = 1;
        else if (acosinput < -1)
            acosinput = -1;
        double theta = acos(acosinput);
        if(NearZear(theta))
            return Matrix3d::Zero();
        else
            return theta * (1 / (2.0 * sin(theta)))* (R - R.transpose());
    }
}

/*
Takes rotation matrix R and position p.
Returns corresponding homogeneous transformation matrix T in SE(3).
Example Input:
R = [[1, 0,  0],
     [0, 0, -1],
     [0, 1,  0]]
p = [1, 2, 5]
Output:
[[1, 0,  0, 1],
 [0, 0, -1, 2],
 [0, 1,  0, 5],
 [0, 0,  0, 1]]
 Code:
    Matrix3d R;
    R << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;
    Vector3d p(0.0, 0.0, 3.0);
    Matrix4d T;
    RpToTrans(R, p, T);
    cout << T << endl;
 */
void RpToTrans(Matrix3d R, Vector3d p, Matrix4d& T){
    T = Matrix4d::Identity();
    T.block<3,3>(0,0) << R;
    T.block<3,1>(0,3) << p;
}

/*
Takes transformation matrix T in SE(3).
Returns R: The corresponding rotation matrix,
        p: The corresponding position vector.
Example Input:
T = [[1, 0,  0, 0],
     [0, 0, -1, 0],
     [0, 1,  0, 3],
     [0, 0,  0, 1]]
Output:
([[1, 0,  0],
  [0, 0, -1],
  [0, 1,  0]],
[0, 0, 3])
 Code:
    TransToRp(T, R, p);
    cout << R << endl;
    cout << p << endl;
 */
void TransToRp(Matrix4d T, Matrix3d& R, Vector3d& p){
    R << T.block<3,3>(0,0);
    p << T.block<3,1>(0,3);
}

/*
Takes a transformation matrix T.
Returns its inverse.
Uses the structure of transformation matrices to avoid taking a matrix
inverse, for efficiency.
Example Input:
T = [[1, 0,  0, 0],
     [0, 0, -1, 0],
     [0, 1,  0, 3],
     [0, 0,  0, 1]]
Output:
[[1,  0, 0,  0],
 [0,  0, 1, -3],
 [0, -1, 0,  0],
 [0,  0, 0,  1]]
 Code:
    Matrix4d T;
    T << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0;
    Matrix4d Tinv = TransInv(T);
    cout << Tinv << endl;
 */
Matrix4d TransInv(Matrix4d T){
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);
    Matrix3d Rt = R.transpose();
    Matrix4d Tinv = MatrixXd::Identity(4,4);
    Tinv.block<3,3>(0,0) = Rt;
    Tinv.block<3,1>(0,3) = -Rt * p;
    return Tinv;
}

/*
Takes a 6-vector (representing a spatial velocity).
Returns the corresponding 4x4 se(3) matrix.
Example Input:
V = [1, 2, 3, 4, 5, 6]
Output:
[[ 0, -3,  2, 4],
 [ 3,  0, -1, 5],
 [-2,  1,  0, 6],
 [ 0,  0,  0, 0]]
 Code:
    VectorXd V(6);
    V << 1,2,3,4,5,6;
    Matrix4d se3 = VecTose3(V);
    cout << se3 << endl;
 */
Matrix4d VecTose3(VectorXd V){
    Matrix4d se3 = MatrixXd::Zero(4,4);
    Matrix3d so3 = VecToso3(V.block<3,1>(0,0));
    se3.block<3,4>(0,0) << so3, V.block<3,1>(3,0);
    return se3;
}

/*
Takes se3mat a 4x4 se(3) matrix.
Returns the corresponding 6-vector (representing spatial velocity).
Example Input:
se3mat = [[ 0, -3,  2, 4],
          [ 3,  0, -1, 5],
          [-2,  1,  0, 6],
          [ 0,  0,  0, 0]]
Output:
[1, 2, 3, 4, 5, 6]
 Code:
     cout << se3ToVec(se3) << endl;
 */
VectorXd se3ToVec(Matrix4d se3){
    VectorXd V(6);
    V << se3(2,1), se3(0,2),se3(1,0),se3(0,3),se3(1,3),se3(2,3);
    return V;
}

/*
Takes T a transformation matrix SE(3).
Returns the corresponding 6x6 adjoint representation [AdT].
Example Input:
T = [[1, 0,  0, 0],
     [0, 0, -1, 0],
     [0, 1,  0, 3],
     [0, 0,  0, 1]]
Output:
[[1, 0,  0, 0, 0,  0],
 [0, 0, -1, 0, 0,  0],
 [0, 1,  0, 0, 0,  0],
 [0, 0,  3, 1, 0,  0],
 [3, 0,  0, 0, 0, -1],
 [0, 0,  0, 0, 1,  0]]
 Code:
        Matrix4d T;
        T << 1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 3, 0, 0, 0, 1;
        cout << T << endl;
        cout << Adjoint(T) << endl;
 */
MatrixXd Adjoint(Matrix4d T){
    MatrixXd Ad(6,6);
    Ad = MatrixXd::Zero(6,6);
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);
    Ad.block<3,3>(0,0) = R;
    Ad.block<3,3>(3,0) = VecToso3(p) * R;
    Ad.block<3,3>(3,3) = R;
    return Ad;
}

/*
Takes q: A point lying on the screw axis,
      s: A unit vector in the direction of the screw axis,
      h: The pitch of the screw axis.
#Returns the corresponding normalized screw axis.
Example Input:
q = [3, 0, 0]
s = [0, 0, 1]
h = 2
Output:
[0, 0, 1, 0, -3, 2]
 Code:
    Vector3d q(3, 0, 0), s(0, 0, 1);
    double h = 2;
    cout << ScrewToAxis(q, s, h) << endl;
 */
VectorXd ScrewToAxis(Vector3d q, Vector3d s, double h){
    VectorXd screw(6);
    screw.block<3,1>(0,0) << s;
    screw.block<3,1>(3,0) << q.cross(s) + h * s ;
    return  screw;
}

/*
Takes a 6-vector of exponential coordinates for rigid-body motion S*theta.
Returns S: The corresponding normalized screw axis,
        theta: The distance traveled along/about S.
Example Input:
expc6 = [1, 0, 0, 1, 2, 3]
Output:
([1.0, 0.0, 0.0, 1.0, 2.0, 3.0],
1.0)
 Code:
    MatrixXd expc6(6,1), S(6,1); double theta;
    expc6 << 1, 0, 0, 1, 2, 3;
    AxisAng6(expc6, S, theta);
    cout << S << endl;
    cout << theta << endl;
 */
void AxisAng6(VectorXd expc6, VectorXd& S, double& theta){
    theta = expc6.block<3,1>(0,0).norm();
    if (NearZear(theta))
        theta = expc6.block<3,1>(3,0).norm();
    S = expc6 / theta;
}

/*
Takes a se(3) representation of exponential coordinates.
Returns a T matrix SE(3) that is achieved by traveling along/about the
screw axis S for a distance theta from an initial configuration T = I.
Example Input:
se3mat = [[0,                 0,                  0,                 0],
          [0,                 0, -1.570796326794897, 2.356194490192345],
          [0, 1.570796326794897,                  0, 2.356194490192345],
          [0,                 0,                  0,                 0]]
Output:
[[1.0, 0.0,  0.0, 0.0],
 [0.0, 0.0, -1.0, 0.0],
 [0.0, 1.0,  0.0, 3.0],
 [  0,   0,    0,   1]]
 Code:
    Matrix4d se3mat;
    se3mat << 0, 0, 0, 0, 0,  0, -1.570796326794897, 2.356194490192345, 0, 1.570796326794897,0, 2.356194490192345, 0, 0, 0, 0;
    Matrix4d T = MatrixExp6(se3mat);
    cout << se3mat << endl;
    cout << T << endl;
 */
Matrix4d MatrixExp6(Matrix4d se3mat){
    Matrix4d T = MatrixXd::Identity(4,4);
    Matrix3d  so3mat = se3mat.block<3,3>(0,0);
    Vector3d omgtheta = so3ToVec(so3mat);
    if (NearZear(omgtheta.norm()))
        T.block<3,1>(0,3) = se3mat.block<3,1>(0,3);
    else{
        Vector3d omghat; double theta;
        AxisAng3(omgtheta, omghat, theta);
        Matrix3d omgmat = so3mat / theta;
        T.block<3,3>(0,0) << MatrixExp3(so3mat);
        T.block<3,1>(0,3) << (MatrixXd::Identity(3,3) * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * omgmat * omgmat) * se3mat.block<3,1>(0,3) / theta;
    }
    return T;
}

/*
Takes a transformation matrix T in SE(3).
Returns the corresponding se(3) representation of exponential coordinates.
Example Input:
T = [[1,0,0,0], [0,0,-1,0], [0,1,0,3], [0,0,0,1]]
Output:
[[0,                 0,                  0,                 0],
[0,                 0, -1.570796326794897, 2.356194490192345],
[0, 1.570796326794897,                  0, 2.356194490192345],
[0,                 0,                  0,                 0]]
Code:
    se3mat = MatrixLog6(T);
    cout << se3mat << endl;
 */
MatrixXd MatrixLog6(Matrix4d T){
    MatrixXd se3 = MatrixXd::Zero(4,4);
    Matrix3d R; Vector3d p;
    TransToRp(T, R, p);  
    if (NearZear((R - MatrixXd::Identity(3,3)).maxCoeff()))
        se3.block<3,1>(0,3) = T.block<3,1>(0,3);
    else{
        double acosinput = (R.trace() - 1) / 2.0;
        if (acosinput > 1)
            acosinput = 1;
        else if (acosinput < -1)
            acosinput = -1;
        double theta = acos(acosinput);
        Matrix3d omgmat = MatrixLog3(R);
        se3.block<3,3>(0,0) = omgmat;
        if(theta == 0)
            se3.block<3,1>(0,3) = T.block<3,1>(0,3);
        else
            se3.block<3,1>(0,3) = (MatrixXd::Identity(3,3) - omgmat / 2.0 + (1.0 / theta - 1.0 / tan(theta / 2.0) / 2) * omgmat * omgmat / theta) * p;
    }
    return se3;
}

/*
Takes Slist: The joint screw axes in the space frame when the manipulator
             is at the home position
      M: The home configuration (position and orientation) of the
         end-effector,
Returns 
    Blist: The joint screw axes in the end-effector frame when the
             manipulator is at the home position,
Example Input:
Code:
    MatrixXd R_Slist_T(6,6);
    R_Slist_T << 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.4247, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.8091, 0.0000, 0.0000, -1.0000, -0.1110, 0.8091, 0.0000, 0.0000, 1.0000, 0.0000, 0.0296, 0.0000, 0.8091;
    MatrixXd R_Slist = R_Slist_T.transpose();

    Matrix4d R_M(4,4);
    R_M << -1.0000, 0, 0, 0.8091, 0, 0, 1.0000, 0.3790, 0, 1.0000, 0, -0.0296, 0, 0, 0, 1.0000;

    MatrixXd R_Blist = SlistToBlist(R_Slist, R_M);

    std::cout << "R_Slist = "<< std::endl << R_Slist << std::endl;

    std::cout << "R_Blist = "<< std::endl << R_Blist << std::endl;
 */
MatrixXd SlistToBlist(MatrixXd Slist, Matrix4d M)
{
    int N = Slist.cols();
    MatrixXd Blist(6,N);
    MatrixXd Ad_M_inv = Adjoint(TransInv(M));
    for (int i=0; i<N; i++)
    {
        Blist.col(i) = Ad_M_inv * Slist.col(i);
    }
    return Blist;
}

/*
Takes Blist: The joint screw axes in the end-effector frame when the
             manipulator is at the home position,
      M: The home configuration (position and orientation) of the
         end-effector,
Returns 
      Slist: The joint screw axes in the space frame when the manipulator
             is at the home position
Example Input:
Code:
    MatrixXd R_Slist_T(6,6);
    R_Slist_T << 0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.0000, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.4247, 0.0000, 1.0000, 0.0000, -0.0805, 0.0000, 0.8091, 0.0000, 0.0000, -1.0000, -0.1110, 0.8091, 0.0000, 0.0000, 1.0000, 0.0000, 0.0296, 0.0000, 0.8091;
    MatrixXd R_Slist = R_Slist_T.transpose();

    Matrix4d R_M(4,4);
    R_M << -1.0000, 0, 0, 0.8091, 0, 0, 1.0000, 0.3790, 0, 1.0000, 0, -0.0296, 0, 0, 0, 1.0000;

    MatrixXd R_Blist = SlistToBlist(R_Slist, R_M);

    MatrixXd R_Slist_temp = BlistToSlist(R_Blist, R_M);

    std::cout << "R_Slist = "<< std::endl << R_Slist << std::endl;

    std::cout << "R_Blist = "<< std::endl << R_Blist << std::endl;

    std::cout << "R_Slist_temp_error = "<< std::endl << R_Slist - R_Slist_temp << std::endl;
 */
MatrixXd BlistToSlist(MatrixXd Blist, Matrix4d M)
{
    int N = Blist.cols();
    MatrixXd Slist(6,N);
    MatrixXd Ad_M = Adjoint(M);
    for (int i=0; i<N; i++)
    {
        Slist.col(i) = Ad_M * Blist.col(i);
    }
    return Slist;
}



/*
Takes M: The home configuration (position and orientation) of the
         end-effector,
      Blist: The joint screw axes in the end-effector frame when the
             manipulator is at the home position,
      thetalist: A list of joint coordinates.
#Returns T IN SE(3) representing the end-effector frame when the joints are
#at the specified coordinates (i.t.o Body Frame).
Example Input:
import numpy as np
from math import pi
M = [[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]]
Blist = np.array([[0, 0, -1, 2, 0,   0],
                  [0, 0,  0, 0, 1,   0],
                  [0, 0,  1, 0, 0, 0.1]]).T
thetalist = [pi / 2.0, 3, pi]
Output:
[[ -1.14423775e-17   1.00000000e+00   0.00000000e+00  -5.00000000e+00],
 [  1.00000000e+00   1.14423775e-17   0.00000000e+00   4.00000000e+00],
 [              0.               0.              -1.       1.68584073],
 [              0.               0.               0.               1.]]
 Code:
    Matrix4d M;
    M << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1;
    MatrixXd Blist(6,3), temp(3,6);
    temp << 0, 0, -1, 2, 0, 0, 0, 0,  0, 0, 1, 0, 0, 0,  1, 0, 0, 0.1;
    Blist = temp.transpose();
    MatrixXd thetalist(3,1 );
    thetalist << M_PI / 2.0, 3, M_PI;
    Matrix4d T = FKinBody(M, Blist, thetalist);
    cout << T << endl;
 */
Matrix4d FKinBody(Matrix4d M, MatrixXd Blist, VectorXd thetalist){
    Matrix4d T = M;
    for (int i = 0; i < thetalist.size(); i++)
        T *= MatrixExp6(VecTose3(Blist.col(i) * thetalist(i)));
    return T;
}

/*
Takes M: the home configuration (position and orientation) of the
         end-effector,
      Slist: The joint screw axes in the space frame when the manipulator
             is at the home position,
      thetalist: A list of joint coordinates.
Returns T in SE(3) representing the end-effector frame when the joints are
at the specified coordinates (i.t.o Space Frame).
Example Input:
import numpy as np
from math import pi
M = [[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]]
Slist = np.array([[0, 0,  1,  4, 0,    0],
                  [0, 0,  0,  0, 1,    0],
                  [0, 0, -1, -6, 0, -0.1]]).T
thetalist = [pi / 2.0, 3, pi]
Output:
[[ -1.14423775e-17   1.00000000e+00   0.00000000e+00  -5.00000000e+00],
 [  1.00000000e+00   1.14423775e-17   0.00000000e+00   4.00000000e+00],
 [              0.               0.              -1.       1.68584073],
 [              0.               0.               0.               1.]]
Code:
    Matrix4d M;
    M << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1;
    MatrixXd Slist(6,3), temp(3,6);
    temp << 0, 0,  1,  4, 0, 0, 0, 0,  0,  0, 1, 0, 0, 0, -1, -6, 0, -0.1;
    Slist = temp.transpose();
    VectorXd thetalist(3);
    thetalist << M_PI / 2.0, 3, M_PI;
    Matrix4d T = FKinSpace(M, Slist, thetalist);
    cout << T << endl;
 */
Matrix4d FKinSpace(Matrix4d M, MatrixXd Slist, VectorXd thetalist){
    Matrix4d T = M;
    for (int i = thetalist.size()-1; i >= 0 ; i--)
        T = MatrixExp6(VecTose3(Slist.col(i) * thetalist(i))) * T;
    return T;
}

/*
Takes Blist: The joint screw axes in the end-effector frame when the
             manipulator is at the home position,
      thetalist: A list of joint coordinates.
#Returns the corresponding body Jacobian (6xn real numbers).
Example Input:
import numpy as np
Blist = np.array([[0, 0, 1,   0, 0.2, 0.2],
                  [1, 0, 0,   2,   0,   3],
                  [0, 1, 0,   0,   2,   1],
                  [1, 0, 0, 0.2, 0.3, 0.4]]).T
thetalist = [0.2, 1.1, 0.1, 1.2]
Output:
[[-0.04528405  0.99500417  0.          1.        ]
 [ 0.74359313  0.09304865  0.36235775  0.        ]
 [-0.66709716  0.03617541 -0.93203909  0.        ]
 [ 2.32586047  1.66809     0.56410831  0.2       ]
 [-1.44321167  2.94561275  1.43306521  0.3       ]
 [-2.06639565  1.82881722 -1.58868628  0.4       ]]
 Code:
    MatrixXd Blist(6,4), temp(4,6);
    temp << 0, 0, 1, 0, 0.2, 0.2, 1, 0, 0, 2, 0, 3, 0, 1, 0, 0, 2, 1, 1, 0, 0, 0.2, 0.3, 0.4;
    Blist = temp.transpose();
    VectorXd thetalist(4);
    thetalist << 0.2, 1.1, 0.1, 1.2;
    MatrixXd Jb = JacobianBody(Blist,thetalist);
    cout << Jb << endl;
 */
MatrixXd JacobianBody(MatrixXd Blist,VectorXd thetalist){
    int num = thetalist.size();
    MatrixXd Jb = Blist; Matrix4d T = MatrixXd::Identity(4,4);
    for (int i = num - 2; i >= 0; i--){
        T *= MatrixExp6(VecTose3(Blist.col(i+1) * (-thetalist(i+1))));
        Jb.col(i) = Adjoint(T) * Blist.col(i);
    }
    return Jb;
}

/*
Takes Slist: The joint screw axes in the space frame when the manipulator
             is at the home position,
      thetalist: A list of joint coordinates.
Returns the corresponding space Jacobian (6xn real numbers).
Example Input:
import numpy as np
Slist = np.array([[0, 0, 1,   0, 0.2, 0.2],
                  [1, 0, 0,   2,   0,   3],
                  [0, 1, 0,   0,   2,   1],
                  [1, 0, 0, 0.2, 0.3, 0.4]]).T
thetalist = [0.2, 1.1, 0.1, 1.2]
Output:
[[ 0.          0.98006658 -0.09011564  0.95749426]
 [ 0.          0.19866933  0.4445544   0.28487557]
 [ 1.          0.          0.89120736 -0.04528405]
 [ 0.          1.95218638 -2.21635216 -0.51161537]
 [ 0.2         0.43654132 -2.43712573  2.77535713]
 [ 0.2         2.96026613  3.23573065  2.22512443]]
 Code:
    MatrixXd Slist(6,4), temp(4,6);
    temp << 0, 0, 1, 0, 0.2, 0.2, 1, 0, 0, 2, 0, 3, 0, 1, 0, 0, 2, 1, 1, 0, 0, 0.2, 0.3, 0.4;
    Slist = temp.transpose();
    VectorXd thetalist(4);
    thetalist << 0.2, 1.1, 0.1, 1.2;
    MatrixXd Js = JacobianSpace(Slist,thetalist);
    cout << Js << endl;
 */
MatrixXd JacobianSpace(MatrixXd Slist,VectorXd thetalist){
    int num = thetalist.size();
    MatrixXd Js = Slist; Matrix4d T = MatrixXd::Identity(4,4);
    for (int i = 1; i < thetalist.size(); i++){
        T *= MatrixExp6(VecTose3(Slist.col(i-1) * thetalist(i-1)));
        Js.col(i) = Adjoint(T) * Slist.col(i);
    }
    return Js;
}

/*
计算矩阵的伪逆
*/
MatrixXd pseudoInverse(MatrixXd & origin) {
    double er = 0;
    // 进行svd分解
    JacobiSVD<Eigen::MatrixXd> svd_holder(origin, ComputeThinU | ComputeThinV);
    // 构建SVD分解结果
    MatrixXd U = svd_holder.matrixU();
    MatrixXd V = svd_holder.matrixV();
    MatrixXd D = svd_holder.singularValues();

    // 构建S矩阵
    MatrixXd S(V.cols(), U.cols());
    S.setZero();

    for (unsigned int i = 0; i < D.size(); ++i) {
        if (D(i, 0) > er) {
            S(i, i) = 1 / D(i, 0);
        } else {
            S(i, i) = 0;
        }
    }

    return V * S * U.transpose();
}

/*
Takes Blist: The joint screw axes in the end-effector frame when the
             manipulator is at the home position,
      M: The home configuration of the end-effector,
      T: The desired end-effector configuration Tsd,
      thetalist0: An initial guess of joint angles that are close to
                  satisfying Tsd,
      eomg: A small positive tolerance on the end-effector orientation
            error. The returned joint angles must give an end-effector
            orientation error less than eomg,
      ev: A small positive tolerance on the end-effector linear position
          error. The returned joint angles must give an end-effector
          position error less than ev.
Returns thetalist: Joint angles that achieve T within the specified
                   tolerances,
        success: A logical value where TRUE means that the function found
                 a solution and FALSE means that it ran through the set
                 number of maximum iterations without finding a solution
                 within the tolerances eomg and ev.
Uses an iterative Newton-Raphson root-finding method.
The maximum number of iterations before the algorithm is terminated has
been hardcoded in as a variable called maxiterations. It is set to 20 at
the start of the function, but can be changed if needed.
Example Input:
import numpy as np
Blist = np.array([[0, 0, -1, 2, 0,   0],
                  [0, 0,  0, 0, 1,   0],
                  [0, 0,  1, 0, 0, 0.1]]).T
M = [[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]]
T = [[0, 1, 0, -5], [1, 0, 0, 4], [0, 0, -1, 1.6858], [0, 0, 0, 1]]
thetalist0 = [1.5, 2.5, 3]
eomg = 0.01
ev = 0.001
Output:
thetalist:
[1.57073819, 2.999667, 3.14153913]
success:
True
Code:
    MatrixXd Blist(6,3), temp(3,6);
    temp << 0, 0, -1, 2, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0.1;
    Blist = temp.transpose();
    VectorXd thetalist0(3), thetalist(3);
    thetalist0 << 1.5, 2.5, 3;
    Matrix4d M(4,4), T(4,4);
    M << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1;
    T << 0, 1, 0, -5, 1, 0, 0, 4, 0, 0, -1, 1.6858, 0, 0, 0, 1;
    double eomg = 0.01, ev = 0.001;
    bool sus = IKinBody(Blist,M,T, thetalist0, eomg, ev,thetalist);
    cout << sus << endl;
    cout << thetalist << endl;
 */
bool IKinBody(MatrixXd Blist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist){
    thetalist = thetalist0;
    int i = 0, maxiterations = 50;
    MatrixXd Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
    bool err = Vb.block<3,1>(0,0).norm() > eomg ||  Vb.block<3,1>(3,0).norm() > ev;
    while (err && i < maxiterations){
        thetalist += JacobianBody(Blist, thetalist).jacobiSvd(ComputeThinU | ComputeThinV).solve(Vb);
        i++;
        Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
        err = Vb.block<3,1>(0,0).norm() > eomg ||  Vb.block<3,1>(3,0).norm() > ev;
    }
    return (! err);
}

/*
Takes Slist: The joint screw axes in the space frame when the manipulator
             is at the home position,
      M: The home configuration of the end-effector,
      T: The desired end-effector configuration Tsd,
      thetalist0: An initial guess of joint angles that are close to
                  satisfying Tsd,
      eomg: A small positive tolerance on the end-effector orientation
            error. The returned joint angles must give an end-effector
            orientation error less than eomg,
      ev: A small positive tolerance on the end-effector linear position
          error. The returned joint angles must give an end-effector
          position error less than ev.
Returns thetalist: Joint angles that achieve T within the specified
                   tolerances,
        success: A logical value where TRUE means that the function found
                 a solution and FALSE means that it ran through the set
                 number of maximum iterations without finding a solution
                 within the tolerances eomg and ev.
Uses an iterative Newton-Raphson root-finding method.
The maximum number of iterations before the algorithm is terminated has
been hardcoded in as a variable called maxiterations. It is set to 20 at
the start of the function, but can be changed if needed.
Example Input:
import numpy as np
Slist = np.array([[0, 0,  1,  4, 0,    0],
                  [0, 0,  0,  0, 1,    0],
                  [0, 0, -1, -6, 0, -0.1]]).T
M = [[-1, 0, 0, 0], [0, 1, 0, 6], [0, 0, -1, 2], [0, 0, 0, 1]]
T = [[0, 1, 0, -5], [1, 0, 0, 4], [0, 0, -1, 1.6858], [0, 0, 0, 1]]
thetalist0 = [1.5, 2.5, 3]
eomg = 0.01
ev = 0.001
Output:
thetalist:
[1.57073785, 2.99966405, 3.14154125]
success:
True
Code:
    MatrixXd Slist(6,3), temp(3,6);
    temp << 0, 0, 1, 4, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -1, -6, 0, -0.1;
    Slist = temp.transpose();
    VectorXd thetalist0(3), thetalist(3);
    thetalist0 << 1.5, 2.5, 3;
    Matrix4d M(4,4), T(4,4);
    M << -1, 0, 0, 0, 0, 1, 0, 6, 0, 0, -1, 2, 0, 0, 0, 1;
    T << 0, 1, 0, -5, 1, 0, 0, 4, 0, 0, -1, 1.6858, 0, 0, 0, 1;
    double eomg = 0.01, ev = 0.001;
    bool sus = IKinSpace(Slist, M, T, thetalist0, eomg, ev,thetalist);
    cout << sus << endl;
    cout << thetalist << endl;
 */
bool IKinSpace(MatrixXd Slist, Matrix4d M, Matrix4d T, VectorXd thetalist0, double eomg, double ev, VectorXd& thetalist){
    thetalist = thetalist0;
    int i = 0, maxiterations = 50;
    Matrix4d Tsb = FKinSpace(M, Slist, thetalist);
    MatrixXd Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    bool err = Vs.block<3,1>(0,0).norm() > eomg ||  Vs.block<3,1>(3,0).norm() > ev;
    while (err && i < maxiterations){
        thetalist += JacobianSpace(Slist, thetalist).jacobiSvd(ComputeThinU | ComputeThinV).solve(Vs);
        i++;
        Tsb = FKinSpace(M, Slist, thetalist);
        Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
        err = Vs.block<3,1>(0,0).norm() > eomg ||  Vs.block<3,1>(3,0).norm() > ev;
    }
    return (! err);
}

/*
Takes Slist: The joint screw axes in the space frame when the manipulator
             is at the home position,
      M: The home configuration of the end-effector,
      T_eef: The desired end-effector configuration Tsd,
      thetalist0: An initial guess of joint angles that are close to
                  satisfying Tsd
      method : 从可行的逆解中选解方法
                1--与thetalist0距离最小
                2--运动趋势最好
Returns thetalist: Joint angles that achieve T within the specified
                   tolerances,
        success: A logical value where TRUE means that the function found
                 a solution and FALSE means found no solution
用解析的POE逆运动学求解方法求解UR机械臂逆运动学
Code:

*/
bool IKinSpace_POE(MatrixXd Slist, Matrix4d M, Matrix4d T_eef, VectorXd thetalist0, int method, VectorXd& thetalist)
{
    VectorXd theta_flag(8); 
    theta_flag << 1, 1, 1, 1, 1, 1, 1, 1;

    double H1, W1, L1, L2, L1_add_L2, W1_add_W2, H1_minus_H2;
    H1 = -Slist(3,2);   W1 = -Slist(3,4);   L1 = Slist(5,2);    
    L1_add_L2 = M(0,3);     L2 =  L1_add_L2 - L1;
    W1_add_W2 = M(1,3);         H1_minus_H2 = M(2,3);

    /******** 求解 theta_1 ************/
    Vector2d theta_1;
    Vector4d q_5;
    q_5 << L1_add_L2, W1, H1_minus_H2, 1;
    Matrix4d T_a = T_eef * TransInv(M);
    Vector4d q_5_temp = T_a * q_5;
    Vector3d u_p_1, u_p_2, v_p;
    u_p_1 << sqrt( pow(q_5_temp(0),2) + pow(q_5_temp(1),2) - pow(W1,2) ), W1, 0;
    u_p_2 << -u_p_1(0), W1, 0;
    v_p <<  q_5_temp(0), q_5_temp(1), 0;
    theta_1(0) = atan2( Slist.block<3,1>(0,0).dot( u_p_1.cross( v_p ) ),  u_p_1.dot( v_p ) );
    theta_1(1) = atan2( Slist.block<3,1>(0,0).dot( u_p_2.cross( v_p ) ),  u_p_2.dot( v_p ) );

    /******** 求解 theta_5 ************/
    Vector2d theta_5_1, theta_5_2;
    Matrix4d T_b_1 = MatrixExp6( VecTose3( Slist.col(0) * (-theta_1(0)) ) ) * T_a;
    Matrix4d T_b_2 = MatrixExp6( VecTose3( Slist.col(0) * (-theta_1(1)) ) ) * T_a;
    theta_5_1(0) = acos(T_b_1(1,1));    
    theta_5_1(1) = -acos(T_b_1(1,1));
    theta_5_2(0) = acos(T_b_2(1,1));   
    theta_5_2(1) = -acos(T_b_2(1,1));        

    /******** 求解 theta_6 ************/
    Vector2d theta_6_1, theta_6_2;
    theta_6_1(0) = atan2(T_b_1(1,2) / (-sin(theta_5_1(0))), T_b_1(1,0) / (-sin(theta_5_1(0))));
    theta_6_1(1) = atan2(T_b_1(1,2) / (-sin(theta_5_1(1))), T_b_1(1,0) / (-sin(theta_5_1(1))));
    theta_6_2(0) = atan2(T_b_2(1,2) / (-sin(theta_5_2(0))), T_b_2(1,0) / (-sin(theta_5_2(0))));
    theta_6_2(1) = atan2(T_b_2(1,2) / (-sin(theta_5_2(1))), T_b_2(1,0) / (-sin(theta_5_2(1))));

    /******** 求解 theta_234 ************/
    Vector4d theta_234;
    theta_234(0) = atan2(T_b_1(0,2) * cos(theta_6_1(0)) - T_b_1(0,0) * sin(theta_6_1(0)) , T_b_1(2,2) * cos(theta_6_1(0)) - T_b_1(2,0) * sin(theta_6_1(0)));
    theta_234(1) = atan2(T_b_1(0,2) * cos(theta_6_1(1)) - T_b_1(0,0) * sin(theta_6_1(1)) , T_b_1(2,2) * cos(theta_6_1(1)) - T_b_1(2,0) * sin(theta_6_1(1)));
    theta_234(2) = atan2(T_b_2(0,2) * cos(theta_6_2(0)) - T_b_2(0,0) * sin(theta_6_2(0)) , T_b_2(2,2) * cos(theta_6_2(0)) - T_b_2(2,0) * sin(theta_6_2(0)));
    theta_234(3) = atan2(T_b_2(0,2) * cos(theta_6_2(1)) - T_b_2(0,0) * sin(theta_6_2(1)) , T_b_2(2,2) * cos(theta_6_2(1)) - T_b_2(2,0) * sin(theta_6_2(1)));

    /** 求解 theta_3 theta_2 theta_4 **/
    Vector2d theta_2_1, theta_2_2, theta_2_3, theta_2_4;
    Vector2d theta_3_1, theta_3_2, theta_3_3, theta_3_4;
    Vector2d theta_4_1, theta_4_2, theta_4_3, theta_4_4;

    Matrix4d T_c_1 = T_b_1 * MatrixExp6( VecTose3(Slist.col(5) * (-theta_6_1(0))) ) * MatrixExp6( VecTose3(Slist.col(4) * (-theta_5_1(0))) );
    Matrix4d T_c_2 = T_b_1 * MatrixExp6( VecTose3(Slist.col(5) * (-theta_6_1(1))) ) * MatrixExp6( VecTose3(Slist.col(4) * (-theta_5_1(1))) );
    Matrix4d T_c_3 = T_b_2 * MatrixExp6( VecTose3(Slist.col(5) * (-theta_6_2(0))) ) * MatrixExp6( VecTose3(Slist.col(4) * (-theta_5_2(0))) );
    Matrix4d T_c_4 = T_b_2 * MatrixExp6( VecTose3(Slist.col(5) * (-theta_6_2(1))) ) * MatrixExp6( VecTose3(Slist.col(4) * (-theta_5_2(1))) );

    Vector3d O3, O2, q4, u_p, O32, O23;
    Vector4d q_4;
    O3 << L1, W1, H1;           
    O2 << 0, W1, H1;
    q4 << L1_add_L2, W1, H1;    
    q_4 << L1_add_L2, W1, H1, 1;
    u_p = q4 - O3;  
    O32 = O2 - O3;              
    O23 = O3 - O2;

    Vector3d v_p_1, v_p_2, v_p_3, v_p_4;
    Vector4d v_p_1_temp = T_c_1 * q_4;      v_p_1 = v_p_1_temp.block<3,1>(0,0) - O2;
    Vector4d v_p_2_temp = T_c_2 * q_4;      v_p_2 = v_p_2_temp.block<3,1>(0,0) - O2;
    Vector4d v_p_3_temp = T_c_3 * q_4;      v_p_3 = v_p_3_temp.block<3,1>(0,0) - O2;
    Vector4d v_p_4_temp = T_c_4 * q_4;      v_p_4 = v_p_4_temp.block<3,1>(0,0) - O2;
    
    Vector3d w3, w2;
    w3 = Slist.block<3,1>(0,2);
    w2 = Slist.block<3,1>(0,1);
    double theta03 = atan2( w3.dot( u_p.cross(O32) ),  u_p.dot(O32) );
    double theta02_1 = atan2( w2.dot( O23.cross(v_p_1) ),  O23.dot(v_p_1) );
    double theta02_2 = atan2( w2.dot( O23.cross(v_p_2) ),  O23.dot(v_p_2) );
    double theta02_3 = atan2( w2.dot( O23.cross(v_p_3) ),  O23.dot(v_p_3) );
    double theta02_4 = atan2( w2.dot( O23.cross(v_p_4) ),  O23.dot(v_p_4) );

    if (abs(L2 - L1) <= v_p_1.norm() && abs(v_p_1.norm() - L1) <= L2)
    {
        double alpha_1 = acos( ( pow(L2,2) + pow(L1,2) - pow(v_p_1.norm(),2) ) / (2 * L2 * L1) );
        double beta_1 = acos( ( pow(v_p_1.norm(),2) + pow(L1,2) - pow(L2,2) ) / (2 * v_p_1.norm() * L1) );
        theta_3_1(0) = theta03 - alpha_1;
        theta_3_1(1) = theta03 + alpha_1;
        theta_2_1(0) = theta02_1 - beta_1;
        theta_2_1(1) = theta02_1 + beta_1;
        theta_4_1(0) = theta_234(0) - theta_3_1(0) - theta_2_1(0);
        theta_4_1(1) = theta_234(0) - theta_3_1(1) - theta_2_1(1);
    }
    else{
        theta_flag(0) = 0;  
        theta_flag(1) = 0; 
    }

    if (abs(L2 - L1) <= v_p_2.norm() && abs(v_p_2.norm() - L1) <= L2)
    {
        double alpha_2 = acos( ( pow(L2,2) + pow(L1,2) - pow(v_p_2.norm(),2) ) / (2 * L2 * L1) );
        double beta_2 = acos( ( pow(v_p_2.norm(),2) + pow(L1,2) - pow(L2,2) ) / (2 * v_p_2.norm() * L1) );
        theta_3_2(0) = theta03 - alpha_2;
        theta_3_2(1) = theta03 + alpha_2;
        theta_2_2(0) = theta02_2 - beta_2;
        theta_2_2(1) = theta02_2 + beta_2;
        theta_4_2(0) = theta_234(1) - theta_3_2(0) - theta_2_2(0);
        theta_4_2(1) = theta_234(1) - theta_3_2(1) - theta_2_2(1);
    }
    else{
        theta_flag(2) = 0;  
        theta_flag(3) = 0; 
    }        

    if (abs(L2 - L1) <= v_p_3.norm() && abs(v_p_3.norm() - L1) <= L2)
    {
        double alpha_3 = acos( ( pow(L2,2) + pow(L1,2) - pow(v_p_3.norm(),2) ) / (2 * L2 * L1) );
        double beta_3 = acos( ( pow(v_p_3.norm(),2) + pow(L1,2) - pow(L2,2) ) / (2 * v_p_3.norm() * L1) );
        theta_3_3(0) = theta03 - alpha_3;
        theta_3_3(1) = theta03 + alpha_3;
        theta_2_3(0) = theta02_3 - beta_3;
        theta_2_3(1) = theta02_3 + beta_3;
        theta_4_3(0) = theta_234(2) - theta_3_3(0) - theta_2_3(0);
        theta_4_3(1) = theta_234(2) - theta_3_3(1) - theta_2_3(1);
    }
    else{
        theta_flag(4) = 0;  
        theta_flag(5) = 0; 
    }  

    if (abs(L2 - L1) <= v_p_4.norm() && abs(v_p_4.norm() - L1) <= L2)
    {
        double alpha_4 = acos( ( pow(L2,2) + pow(L1,2) - pow(v_p_4.norm(),2) ) / (2 * L2 * L1) );
        double beta_4 = acos( ( pow(v_p_4.norm(),2) + pow(L1,2) - pow(L2,2) ) / (2 * v_p_4.norm() * L1) );
        theta_3_4(0) = theta03 - alpha_4;
        theta_3_4(1) = theta03 + alpha_4;
        theta_2_4(0) = theta02_4 - beta_4;
        theta_2_4(1) = theta02_4 + beta_4;
        theta_4_4(0) = theta_234(3) - theta_3_4(0) - theta_2_4(0);
        theta_4_4(1) = theta_234(3) - theta_3_4(1) - theta_2_4(1);
    }
    else{
        theta_flag(6) = 0;  
        theta_flag(7) = 0; 
    }  

    /************ 逆解整理 ************/
    MatrixXd theta_result_temp(8,6);

    theta_result_temp.block<4,1>(0,0) = MatrixXd::Ones(4,1) *  theta_1(0);
    theta_result_temp.block<4,1>(4,0) = MatrixXd::Ones(4,1) *  theta_1(1);

    theta_result_temp.block<2,1>(0,4) = MatrixXd::Ones(2,1) *  theta_5_1(0);
    theta_result_temp.block<2,1>(2,4) = MatrixXd::Ones(2,1) *  theta_5_1(1);
    theta_result_temp.block<2,1>(4,4) = MatrixXd::Ones(2,1) *  theta_5_2(0);
    theta_result_temp.block<2,1>(6,4) = MatrixXd::Ones(2,1) *  theta_5_2(1);

    theta_result_temp.block<2,1>(0,5) = MatrixXd::Ones(2,1) *  theta_6_1(0);
    theta_result_temp.block<2,1>(2,5) = MatrixXd::Ones(2,1) *  theta_6_1(1);
    theta_result_temp.block<2,1>(4,5) = MatrixXd::Ones(2,1) *  theta_6_2(0);
    theta_result_temp.block<2,1>(6,5) = MatrixXd::Ones(2,1) *  theta_6_2(1);

    theta_result_temp(0,2) = theta_3_1(0);  theta_result_temp(1,2) = theta_3_1(1);
    theta_result_temp(2,2) = theta_3_2(0);  theta_result_temp(3,2) = theta_3_2(1);
    theta_result_temp(4,2) = theta_3_3(0);  theta_result_temp(5,2) = theta_3_3(1);
    theta_result_temp(6,2) = theta_3_4(0);  theta_result_temp(7,2) = theta_3_4(1);

    theta_result_temp(0,1) = theta_2_1(0);  theta_result_temp(1,1) = theta_2_1(1);
    theta_result_temp(2,1) = theta_2_2(0);  theta_result_temp(3,1) = theta_2_2(1);
    theta_result_temp(4,1) = theta_2_3(0);  theta_result_temp(5,1) = theta_2_3(1);
    theta_result_temp(6,1) = theta_2_4(0);  theta_result_temp(7,1) = theta_2_4(1);

    theta_result_temp(0,3) = theta_4_1(0);  theta_result_temp(1,3) = theta_4_1(1);
    theta_result_temp(2,3) = theta_4_2(0);  theta_result_temp(3,3) = theta_4_2(1);
    theta_result_temp(4,3) = theta_4_3(0);  theta_result_temp(5,3) = theta_4_3(1);
    theta_result_temp(6,3) = theta_4_4(0);  theta_result_temp(7,3) = theta_4_4(1);  

    /************ 提取有限逆解 theta_result ************/ 
    int N = theta_flag.sum();

    if (N == 0)
        return false;

    MatrixXd theta_result(N,6); 
    int cnt = 0;
    for (int i = 0; i < 8; i++)
    {
        if(NearZear(theta_flag(i) - 1))
        {
            theta_result.row(cnt) =  theta_result_temp.row(i);
            for (int j = 0; j < 6; j++){
                if(theta_result(cnt, j) > M_PI){
                    theta_result(cnt, j) -= 2*M_PI;
                } 
                else if (theta_result(cnt, j) < -M_PI){
                    theta_result(cnt, j) += 2*M_PI;
                }
            }
            cnt++;
        }           
    }

    /************ 选一组可行解 ************/ 
    for (int i=0; i<6; i++)
    {
        if(thetalist0[i] > M_PI)
            thetalist0[i] -= 2*M_PI;
        else if (thetalist0[i] < -M_PI)
            thetalist0[i] += 2*M_PI;
    }

    switch(method){
        case 1  :{ //计算转动角度最小的一组可行解
            double thetalist_error_min = 100.0;
            for (int i = 0; i < N; i++)
            {
                VectorXd thetalist_error = theta_result.row(i).transpose() - thetalist0;
                double error = thetalist_error.transpose() * thetalist_error;
                if (error < thetalist_error_min)
                {
                    thetalist_error_min = error;
                    thetalist = theta_result.row(i);
                }
            }
            break;
        } 
        case 2  :{ //计算运动趋势最好的可行解
            double det_max = 0;
            for (int i = 0; i < N; i++)
            {
                MatrixXd J = JacobianSpace(Slist, theta_result.row(i).transpose());
                MatrixXd A = J * J.transpose();
                double det = A.determinant();
                if(det > det_max){
                    det_max = det;
                    thetalist = theta_result.row(i);
                }
            }
            break; 
        }
        default : {//计算转动角度最小的一组可行解
            double thetalist_error_min = 100.0;
            for (int i = 0; i < N; i++)
            {
                VectorXd thetalist_error = theta_result.row(i).transpose() - thetalist0;
                double error = thetalist_error.transpose() * thetalist_error;
                if (error < thetalist_error_min)
                {
                    thetalist_error_min = error;
                    thetalist = theta_result.row(i);
                }
            }
            break;
        }
    }

    return true;   
}

/*
Takes 6-vector spatial velocity.
Returns the corresponding 6x6 matrix [adV].
Used to calculate the Lie bracket [V1, V2] = [adV1]V2
Example Input:
V = [1, 2, 3, 4, 5, 6]
Output:
[[0, -3, 2, 0, 0, 0],
 [3, 0, -1, 0, 0, 0],
 [-2, 1, 0, 0, 0, 0],
 [0, -6, 5, 0, -3, 2],
 [6, 0, -4, 3, 0, -1],
 [-5, 4, 0, -2, 1, 0]]
Code:
    VectorXd V(6);
    V << 1, 2, 3, 4, 5, 6;
    cout << ad(V) << endl;
 */
MatrixXd ad(VectorXd V){
    MatrixXd adV = MatrixXd::Zero(6,6);
    Matrix3d omgmat = VecToso3(V.block<3,1>(0,0));
    adV.block<3,3>(0,0) = omgmat;
    adV.block<3,3>(3,0) = VecToso3(V.block<3,1>(3,0));
    adV.block<3,3>(3,3) = omgmat;
    return adV;
}

/*
Takes thetalist: n-vector of joint variables,
      dthetalist: n-vector of joint rates,
      ddthetalist: n-vector of joint accelerations,
      g: Gravity vector g,
      Ftip: Spatial force applied by the end-effector expressed in frame
            {n+1},
      Mlist: List of link frames {i} relative to {i-1} at the home
             position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame.
Returns taulist: The n-vector of required joint forces/torques.
This function uses forward-backward Newton-Euler iterations to solve the
equation:
taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) \
          + g(thetalist) + Jtr(thetalist)Ftip
Example Input (3 Link Robot):
import numpy as np
thetalist = [0.1, 0.1, 0.1]
dthetalist = [0.1, 0.2, 0.3]
ddthetalist = [2, 1.5, 1]
g = [0, 0, -9.8]
Ftip = [1, 1, 1, 1, 1, 1]
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[74.696161552874514, -33.067660158514578, -3.2305731379014242]
Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd ddthetalist(3, 1); ddthetalist << 2, 1.5, 1;
    Vector3d g; g << 0, 0, -9.8;
    VectorXd Ftip(6,1);  Ftip << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    cout << InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist) << endl;
 */
VectorXd InverseDynamics(VectorXd thetalist, VectorXd dthetalist, VectorXd ddthetalist, Vector3d g, VectorXd Ftip, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist){
    int n = thetalist.size();
    Matrix4d Mi = MatrixXd::Identity(4,4);
    MatrixXd Ai = MatrixXd::Zero(6,n);
    MatrixXd AdTi[n+1];
    MatrixXd Vi = MatrixXd::Zero(6, n+1);
    MatrixXd Vdi = MatrixXd::Zero(6, n+1);
    Vdi.col(0) << MatrixXd::Zero(3,1), -g;
    AdTi[n] = Adjoint(TransInv(Mlist[n]));
    VectorXd Fi = Ftip;
    MatrixXd taulist(n,1);
    for (int i = 0; i < n; i++){
        Mi = Mi * Mlist[i];
        Ai.col(i) = Adjoint(TransInv(Mi)) * Slist.col(i);
        AdTi[i] = Adjoint(MatrixExp6(VecTose3(Ai.col(i) * (-thetalist(i)))) * TransInv(Mlist[i]));
        Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
        Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i) + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i);
    }
    for (int i = n-1; i >= 0; i--) {
        Fi = AdTi[i + 1].transpose() * Fi + Glist[i] * Vdi.col(i + 1) - ad(Vi.col(i + 1)).transpose() * (Glist[i] * Vi.col(i + 1));
        MatrixXd temp = Fi.transpose() * Ai.col(i);
        taulist(i,0) = temp(0,0);
    }
    return taulist;
}

/*
Takes thetalist: A list of joint variables,
      Mlist: List of link frames i relative to i-1 at the home position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame.
Returns M: The numerical inertia matrix M(thetalist) of an n-joint serial
           chain at the given configuration thetalist.
This function calls InverseDynamics n times, each time passing a
ddthetalist vector with a single element equal to one and all other inputs
set to zero.
Each call of InverseDynamics generates a single column, and these columns
are assembled to create the inertia matrix.
Example Input (3 Link Robot):
import numpy as np
thetalist = [0.1, 0.1, 0.1]
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[[  2.25433380e+01  -3.07146754e-01  -7.18426391e-03]
 [ -3.07146754e-01   1.96850717e+00   4.32157368e-01]
 [ -7.18426391e-03   4.32157368e-01   1.91630858e-01]]
Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd ddthetalist(3, 1); ddthetalist << 2, 1.5, 1;
    Vector3d g; g << 0, 0, -9.8;
    VectorXd Ftip(6,1);  Ftip << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    cout << MassMatrix(thetalist, Mlist, Glist, Slist) << endl;
 */
MatrixXd MassMatrix(VectorXd thetalist, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist){
    int n = thetalist.size();
    MatrixXd M = MatrixXd::Zero(n,n);
    for (int i = 0; i < n; i++){
        VectorXd ddthetalist = VectorXd::Zero(n);
        ddthetalist(i,0) = 1;
        M.col(i) = InverseDynamics(thetalist, VectorXd::Zero(n), ddthetalist, Vector3d::Zero(), VectorXd::Zero(6), Mlist, Glist, Slist);
    }
    return M;
}

/*
Takes thetalist: A list of joint variables,
      dthetalist: A list of joint rates,
      Mlist: List of link frames i relative to i-1 at the home position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame.
Returns c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
           terms for a given thetalist and dthetalist.
This function calls InverseDynamics with g = 0, Ftip = 0, and
ddthetalist = 0.
Example Input (3 Link Robot):
import numpy as np
thetalist = [0.1, 0.1, 0.1]
dthetalist = [0.1, 0.2, 0.3]
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[0.26453118054501235, -0.055051568289165499, -0.0068913200682489129]
Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd ddthetalist(3, 1); ddthetalist << 2, 1.5, 1;
    Vector3d g; g << 0, 0, -9.8;
    VectorXd Ftip(6,1);  Ftip << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    cout << VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist) << endl;
 */
VectorXd VelQuadraticForces(VectorXd thetalist, VectorXd dthetalist, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist){
    int n = thetalist.size();
    return InverseDynamics(thetalist, dthetalist, VectorXd::Zero(n), Vector3d::Zero(), VectorXd::Zero(6,1), Mlist, Glist, Slist);
}

/*
Takes thetalist: A list of joint variables,
      g: 3-vector for gravitational acceleration,
      Mlist: List of link frames i relative to i-1 at the home position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame.
Returns grav: The joint forces/torques required to overcome gravity at
              thetalist
This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and
ddthetalist = 0.
Example Inputs (3 Link Robot):
import numpy as np
thetalist = [0.1, 0.1, 0.1]
g = [0, 0, -9.8]
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[28.403312618219829, -37.640948171770681, -5.4415891999683605]
Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd ddthetalist(3, 1); ddthetalist << 2, 1.5, 1;
    Vector3d g; g << 0, 0, -9.8;
    VectorXd Ftip(6,1);  Ftip << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    cout << GravityForces(thetalist, g, Mlist, Glist, Slist) << endl;
 */
VectorXd GravityForces(VectorXd thetalist, Vector3d g, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist){
    int n = thetalist.size();
    return InverseDynamics(thetalist, VectorXd::Zero(n,1), VectorXd::Zero(n,1), g, VectorXd::Zero(6,1), Mlist, Glist, Slist);
}

/*
Takes thetalist: A list of joint variables,
      Ftip: Spatial force applied by the end-effector expressed in frame
            {n+1},
      Mlist: List of link frames i relative to i-1 at the home position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame.
Returns JTFtip: The joint forces and torques required only to create the
                end-effector force Ftip.
This function calls InverseDynamics with g = 0, dthetalist = 0, and
ddthetalist = 0.
Example Input (3 Link Robot):
import numpy as np
thetalist = [0.1, 0.1, 0.1]
Ftip = [1, 1, 1, 1, 1, 1]
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[1.4095460782639782, 1.8577149723180628, 1.392409]
Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd ddthetalist(3, 1); ddthetalist << 2, 1.5, 1;
    Vector3d g; g << 0, 0, -9.8;
    VectorXd Ftip(6,1);  Ftip << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    cout << EndEffectorForces(thetalist,Ftip, Mlist, Glist,Slist) << endl;
 */
VectorXd EndEffectorForces(VectorXd thetalist,VectorXd Ftip, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist){
    int n = thetalist.size();
    return InverseDynamics(thetalist, VectorXd::Zero(n,1), MatrixXd::Zero(n,1), Vector3d::Zero(), Ftip, Mlist, Glist, Slist);
}

/*
Takes thetalist: A list of joint variables,
      dthetalist: A list of joint rates,
      taulist: An n-vector of joint forces/torques,
      g: Gravity vector g,
      Ftip: Spatial force applied by the end-effector expressed in frame
            {n+1},
      Mlist: List of link frames i relative to i-1 at the home position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame.
Returns ddthetalist: The resulting joint accelerations.
This function computes ddthetalist by solving:
Mlist(thetalist)ddthetalist = taulist - c(thetalist,dthetalist) \
                              - g(thetalist) - Jtr(thetalist)Ftip
Example Input (3 Link Robot):
import numpy as np
thetalist = [0.1, 0.1, 0.1]
dthetalist = [0.1, 0.2, 0.3]
taulist = [0.5, 0.6, 0.7]
g = [0, 0, -9.8]
Ftip = [1, 1, 1, 1, 1, 1]
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
Output:
[ -0.97392907  25.58466784 -32.91499212]
Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd taulist(3, 1); taulist << 0.5, 0.6, 0.7;
    Vector3d g; g << 0, 0, -9.8;
    VectorXd Ftip(6,1);  Ftip << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    cout << ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist) << endl;

 */
VectorXd ForwardDynamics(VectorXd thetalist, VectorXd dthetalist, VectorXd taulist, Vector3d g, VectorXd Ftip, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist){
    return MassMatrix(thetalist,Mlist,Glist,Slist).inverse() *
                (taulist - VelQuadraticForces(thetalist,dthetalist, Mlist,Glist,Slist)
                         - GravityForces(thetalist,g,Mlist,Glist,Slist)
                         - EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist));
}

/*
Takes thetalist: n-vector of joint variables,
      dthetalist: n-vector of joint rates,
      ddthetalist: n-vector of joint accelerations,
      dt: The timestep delta t.
Returns thetalistNext: Vector of joint variables after dt from first order
                       Euler integration,
        dthetalistNext: Vector of joint rates after dt from first order
                        Euler integration.
Example Inputs (3 Link Robot):
thetalist = [0.1,0.1,0.1]
dthetalist = [0.1,0.2,0.3]
ddthetalist = [2,1.5,1]
dt = 0.1
Output:
thetalistNext:
[ 0.11,  0.12,  0.13]
dthetalistNext:
[ 0.3 ,  0.35,  0.4 ]
 Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd ddthetalist(3, 1); ddthetalist << 2, 1.5, 1;
    double dt = 0.1;
    VectorXd thetalistNext, dthetalistNext;
    EulerStep(thetalist, dthetalist, ddthetalist, dt, thetalistNext, dthetalistNext);
    cout << thetalistNext << endl;
    cout << dthetalistNext << endl;
 */
void EulerStep(VectorXd thetalist, VectorXd dthetalist, VectorXd ddthetalist, double dt, VectorXd& thetalistNext, VectorXd& dthetalistNext){
    thetalistNext = thetalist + dt * dthetalist;
    dthetalistNext = dthetalist + dt * ddthetalist;
}

/*
Takes thetamat: An N x n matrix of robot joint variables,
      dthetamat: An N x n matrix of robot joint velocities,
      ddthetamat: An N x n matrix of robot joint accelerations,
      g: Gravity vector g,
      Ftipmat: An N x 6 matrix of spatial forces applied by the
               end-effector (If there are no tip forces the user should
               input a zero and a zero matrix will be used),
      Mlist: List of link frames i relative to i-1 at the home position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame.
Returns taumat: The N x n matrix of joint forces/torques for the specified
                trajectory, where each of the N rows is the vector of
                joint forces/torques at each time step.
This function uses InverseDynamics to calculate the joint forces/torques
required to move the serial chain along the given trajectory.
#Example Inputs (3 Link Robot):
import numpy as np
from math import pi
from modern_robotics import JointTrajectory
import matplotlib.pyplot as plt
#Create a trajectory to follow using functions from Chapter 9
thetastart =[0, 0, 0]
thetaend = [pi / 2, pi / 2, pi / 2]
Tf = 3
N= 1000
method = 5
traj = JointTrajectory(thetastart,thetaend,Tf,N,method)
thetamat = np.array(traj).copy()
dthetamat = np.zeros((1000,3))
ddthetamat = np.zeros((1000,3))
dt = Tf / (N - 1.0)
for i in range(np.array(traj).shape[0] - 1):
    dthetamat[i + 1,:] = (thetamat[i + 1,:] - thetamat[i,:]) / dt
    ddthetamat[i + 1,:] = (dthetamat[i + 1,:] - dthetamat[i,:]) / dt
Initialise robot descripstion (Example with 3 links)
g = [0, 0, -9.8]
Ftipmat = np.ones((N,6))
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
taumat = InverseDynamicsTrajectory(thetamat,dthetamat,ddthetamat,g, \
                                   Ftipmat,Mlist,Glist,Slist)
#Output using matplotlib to plot the joint forces/torques
Tau1 = taumat[:,0]
Tau2 = taumat[:,1]
Tau3 = taumat[:,2]
Code:
    VectorXd thetastart(3, 1); thetastart << 0, 0, 0;
    VectorXd thetaend(3, 1); thetaend << M_PI / 2, M_PI / 2, M_PI / 2;
    double Tf = 3.0;
    int N = 10;
    int method = 5;
    MatrixXd traj = JointTrajectory(thetastart,thetaend,Tf,N,method);
    MatrixXd thetamat = traj;
    MatrixXd dthetamat = MatrixXd::Zero(N,3);
    MatrixXd ddthetamat = MatrixXd::Zero(N,3);
    double dt = Tf / (N - 1.0);
    for (int i = 0; i < N-1; i++){
        dthetamat.row(i+1) = (thetamat.row(i+1) - thetamat.row(i)) / dt;
        ddthetamat.row(i+1) = (dthetamat.row(i+1) - dthetamat.row(i)) / dt;
    }
//    cout << thetamat << endl;
//    cout << dthetamat << endl;
//    cout << ddthetamat << endl;
    Vector3d g; g << 0, 0, -9.8;
    MatrixXd Ftipmat = MatrixXd::Ones(N, 6);
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    MatrixXd taumat = InverseDynamicsTrajectory(thetamat,dthetamat,ddthetamat,g,Ftipmat,Mlist,Glist,Slist);
    cout << taumat << endl;
 */
MatrixXd InverseDynamicsTrajectory(MatrixXd thetamat, MatrixXd dthetamat, MatrixXd ddthetamat, Vector3d g, MatrixXd Ftipmat, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist) {
    MatrixXd thetamat_0 = thetamat.transpose();
    MatrixXd dthetamat_0 = dthetamat.transpose();
    MatrixXd ddthetamat_0 = ddthetamat.transpose();
    MatrixXd Ftipmat_0 = Ftipmat.transpose();
    MatrixXd taumat = thetamat_0;
    for (int i = 0; i < thetamat_0.cols(); i++) {
        taumat.col(i) = InverseDynamics(thetamat_0.col(i), dthetamat_0.col(i), ddthetamat_0.col(i), g, Ftipmat_0.col(i), Mlist, Glist, Slist);
    }
    return taumat.transpose();
}

/*
#Takes thetalist: n-vector of initial joint variables,
#      dthetalist: n-vector of initial joint rates,
#      taumat: An N x n matrix of joint forces/torques, where each row is
#              the joint effort at any time step,
#      g: Gravity vector g,
#      Ftipmat: An N x 6 matrix of spatial forces applied by the
#               end-effector (If there are no tip forces the user should
#               input a zero and a zero matrix will be used),
#      Mlist: List of link frames {i} relative to {i-1} at the home
#             position,
#      Glist: Spatial inertia matrices Gi of the links,
#      Slist: Screw axes Si of the joints in a space frame,
#      dt: The timestep between consecutive joint forces/torques,
#      intRes: Integration resolution is the number of times integration
#              (Euler) takes places between each time step. Must be an
#              integer value greater than or equal to 1
#Returns thetamat: The N x n matrix of robot joint angles resulting from
#                  the specified joint forces/torques,
#        dthetamat: The N x n matrix of robot joint velocities.
#This function simulates the motion of a serial chain given an open-loop
#history of joint forces/torques.
#It calls a numerical integration procedure that uses ForwardDynamics.
    '''
#Example Inputs (3 Link Robot):
import numpy as np
import matplotlib.pyplot as plt
thetalist = [0.1, 0.1, 0.1]
dthetalist = [0.1, 0.2, 0.3]
taumat = [[3.63, -6.58, -5.57], [3.74, -5.55,  -5.5],
          [4.31, -0.68, -5.19], [5.18,  5.63, -4.31],
          [5.85,  8.17, -2.59], [5.78,  2.79,  -1.7],
          [4.99,  -5.3, -1.19], [4.08, -9.41,  0.07],
          [3.56, -10.1,  0.97], [3.49, -9.41,  1.23]]
#Initialise robot description (Example with 3 links)
g = [0, 0, -9.8]
Ftipmat = np.ones((np.array(taumat).shape[0],6))
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
dt = 0.1
intRes = 8
thetamat,dthetamat \
= ForwardDynamicsTrajectory(thetalist,dthetalist,taumat,g,Ftipmat,Mlist, \
                            Glist,Slist,dt,intRes)
#Output using matplotlib to plot the joint angle/velocities
theta1 = thetamat[:,0]
theta2 = thetamat[:,1]
theta3 = thetamat[:,2]
dtheta1 = dthetamat[:,0]
dtheta2 = dthetamat[:,1]
dtheta3 = dthetamat[:,2]
N = np.array(taumat).shape[0]
Tf = np.array(taumat).shape[0]*dt
timestamp = np.linspace(0,Tf,N)
plt.plot(timestamp, theta1, label = "Theta1")
plt.plot(timestamp, theta2, label = "Theta2")
plt.plot(timestamp, theta3, label = "Theta3")
plt.plot(timestamp, dtheta1, label = "DTheta1")
plt.plot(timestamp, dtheta2, label = "DTheta2")
plt.plot(timestamp, dtheta3, label = "DTheta3")
plt.ylim (-12,10)
plt.legend(loc = 'lower right')
plt.xlabel("Time")
plt.ylabel("Joint Angles/Velocities")
plt.title("Plot of Joint Angles and Joint Velocities")
plt.show()
Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    MatrixXd taumat(10, 3);
    taumat << 3.63, -6.58, -5.57,
              3.74, -5.55, -5.5,
              4.31, -0.68, -5.19,
              5.18, 5.63, -4.31,
              5.85, 8.17, -2.59,
              5.78, 2.79, -1.7,
              4.99, -5.3, -1.19,
              4.08, -9.41, 0.07,
              3.56, -10.1, 0.97,
              3.49, -9.41, 1.23;
    Vector3d g; g << 0, 0, -9.8;
    MatrixXd Ftipmat = MatrixXd::Ones(taumat.rows(), 6);
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    double dt = 0.1;
    int intRes = 8;
    MatrixXd thetamat, dthetamat;
    ForwardDynamicsTrajectory(thetalist,dthetalist,taumat,g,Ftipmat, Mlist,Glist,Slist,dt,intRes, thetamat,dthetamat);
    cout << thetamat << endl;
    cout << dthetamat << endl;
 */
void ForwardDynamicsTrajectory(VectorXd thetalist, VectorXd dthetalist, MatrixXd taumat, Vector3d g, MatrixXd Ftipmat, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist, double dt, int intRes, MatrixXd& thetamat, MatrixXd& dthetamat){
    MatrixXd taumat_0 = taumat.transpose();
    MatrixXd Ftipmat_0 = Ftipmat.transpose();
    MatrixXd thetamat_0 = taumat_0;
    thetamat_0.col(0) = thetalist;
    MatrixXd dthetamat_0 = taumat_0;
    dthetamat_0.col(0) = dthetalist;

    for (int i = 0; i < taumat_0.cols() - 1; i++){
        for (int j = 0; j < intRes; j++){
            VectorXd ddthetalist = ForwardDynamics(thetalist,dthetalist,taumat_0.col(i),g,Ftipmat_0.col(i),Mlist,Glist,Slist);
            EulerStep(thetalist,dthetalist,ddthetalist,dt / intRes, thetalist,dthetalist);
        }
        thetamat_0.col(i+1) = thetalist;
        dthetamat_0.col(i+1) = dthetalist;
    }
    thetamat = thetamat_0.transpose();
    dthetamat = dthetamat_0.transpose();
}

/*
Takes Tf: Total time of the motion in seconds from rest to rest,
      t: The current time t satisfying 0 < t < Tf.
Returns s: The path parameter s(t) corresponding to a third-order
           polynomial motion that begins and ends at zero velocity.
Example Input:
Tf = 2
t = 0.6
Output:
0.216
Code:
    double Tf = 2;
    double t = 0.6;
    cout << CubicTimeScaling(Tf, t) << endl;
 */
double CubicTimeScaling(double Tf, double t){
    return 3 * pow((t / Tf), 2) - 2 * pow((t / Tf), 3);
}

/*
Takes Tf: Total time of the motion in seconds from rest to rest,
      t: The current time t satisfying 0 < t < Tf.
Returns s: The path parameter s(t) corresponding to a fifth-order
           polynomial motion that begins and ends at zero velocity and
           zero acceleration.
Example Input:
Tf = 2
t = 0.6
Output:
0.16308
Code:
    double Tf = 2;
    double t = 0.6;
    cout << QuinticTimeScaling(Tf, t) << endl;
 */
double QuinticTimeScaling(double Tf, double t){
    return 10 * pow((t / Tf),3) - 15 * pow((t / Tf), 4) + 6 * pow((t / Tf), 5);
}

/*
Takes thetastart: The initial joint variables,
      thetaend: The final joint variables,
      Tf: Total time of the motion in seconds from rest to rest,
      N: The number of points N > 1 (Start and stop) in the discrete
         representation of the trajectory,
      method: The time-scaling method, where 3 indicates cubic
              (third-order polynomial) time scaling and 5 indicates
              quintic (fifth-order polynomial) time scaling.
Returns traj: A trajectory as an N x n matrix, where each row is an
              n-vector of joint variables at an instant in time. The first
              row is thetastart and the Nth row is thetaend . The elapsed
              time between each row is Tf/(N - 1).
The returned trajectory is a straight-line motion in joint space.
Example Input:
thetastart = [1, 0, 0, 1, 1, 0.2, 0,1]
thetaend = [1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1]
Tf = 4
N = 6
method = 3
Output:
[[ 1.      0.      0.      1.      1.      0.2     0.      1.    ]
 [ 1.0208  0.052   0.0624  1.0104  1.104   0.3872  0.0936  1.    ]
 [ 1.0704  0.176   0.2112  1.0352  1.352   0.8336  0.3168  1.    ]
 [ 1.1296  0.324   0.3888  1.0648  1.648   1.3664  0.5832  1.    ]
 [ 1.1792  0.448   0.5376  1.0896  1.896   1.8128  0.8064  1.    ]
 [ 1.2     0.5     0.6     1.1     2.      2.      0.9     1.    ]]
 Code:
    VectorXd thetastart(8,1);  thetastart << 1, 0, 0, 1, 1, 0.2, 0, 1;
    VectorXd thetaend(8,1);  thetaend << 1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1;
    double Tf = 4;
    int N = 6;
    int method = 3;
    cout << JointTrajectory(thetastart,thetaend,Tf,N,method) << endl;

 */
MatrixXd JointTrajectory(VectorXd thetastart,VectorXd thetaend,double Tf,int N,int method){
    double timegap = Tf / (N - 1.0), s;
    MatrixXd traj = MatrixXd::Zero(thetastart.size(), N);
    for (int i = 0; i < N; i++){
        if (method == 3)
            s = CubicTimeScaling(Tf, timegap * i);
        else
            s = QuinticTimeScaling(Tf, timegap * i);
        traj.col(i) = s * thetaend + (1 - s) * thetastart;
    }
    return traj.transpose();
}

/*
Takes Xstart: The initial end-effector configuration,
      Xend: The final end-effector configuration,
      Tf: Total time of the motion in seconds from rest to rest,
      N: The number of points N > 1 (Start and stop) in the discrete
         representation of the trajectory,
      method: The time-scaling method, where 3 indicates cubic
              (third-order polynomial) time scaling and 5 indicates
              quintic (fifth-order polynomial) time scaling.
Returns traj: The discretized trajectory as a list of N matrices in SE(3)
              separated in time by Tf/(N-1). The first in the list is
              Xstart and the Nth is Xend .
This function calculates a trajectory corresponding to the screw motion
about a space screw axis.
Example Input:
Xstart = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]]
Xend = [[0, 0, 1, 0.1], [1, 0, 0, 0], [0, 1, 0, 4.1], [0, 0, 0, 1]]
Tf = 5
N = 4
method = 3
Output:
[[[ 1.     0.     0.     1.   ]
  [ 0.     1.     0.     0.   ]
  [ 0.     0.     1.     1.   ]
  [ 0.     0.     0.     1.   ]]
 [[ 0.904 -0.25   0.346  0.441]
  [ 0.346  0.904 -0.25   0.529]
  [-0.25   0.346  0.904  1.601]
  [ 0.     0.     0.     1.   ]]
 [[ 0.346 -0.25   0.904 -0.117]
  [ 0.904  0.346 -0.25   0.473]
  [-0.25   0.904  0.346  3.274]
  [ 0.     0.     0.     1.   ]]
 [[-0.     0.     1.     0.1  ]
  [ 1.    -0.     0.    -0.   ]
  [ 0.     1.    -0.     4.1  ]
  [ 0.     0.     0.     1.   ]]]
Code:
    Matrix4d Xstart;  Xstart << 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1;
    Matrix4d Xend;  Xend << 0, 0, 1, 0.1, 1, 0, 0, 0, 0, 1, 0, 4.1, 0, 0, 0, 1;
    double Tf = 5;
    int N = 4;
    int method = 3;
    Matrix4d X[N];
    ScrewTrajectory(Xstart, Xend, Tf, N, method, X);
    for (int i = 0; i < N; i++)
        cout << X[i] << endl;
 */
void ScrewTrajectory(Matrix4d Xstart, Matrix4d Xend, double Tf, int N, int method, Matrix4d X[]){
    double timegap = Tf / (N - 1.0), s;
    for (int i = 0; i < N; i++){
        if (method == 3)
            s = CubicTimeScaling(Tf, timegap * i);
        else
            s = QuinticTimeScaling(Tf, timegap * i);
        X[i] = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s);
    }
}

/*
Takes Xstart: The initial end-effector configuration,
      Xend: The final end-effector configuration,
      Tf: Total time of the motion in seconds from rest to rest,
      N: The number of points N > 1 (Start and stop) in the discrete
         representation of the trajectory,
      method: The time-scaling method, where 3 indicates cubic
              (third-order polynomial) time scaling and 5 indicates
              quintic (fifth-order polynomial) time scaling.
Returns traj: The discretized trajectory as a list of N matrices in SE(3)
              separated in time by Tf/(N-1). The first in the list is
              Xstart and the Nth is Xend.
This function is similar to ScrewTrajectory, except the origin of the
end-effector frame follows a straight line, decoupled from the rotational
motion.
Example Input:
Xstart = [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 1], [0, 0, 0, 1]]
Xend = [[0, 0, 1, 0.1], [1, 0, 0, 0], [0, 1, 0, 4.1], [0, 0, 0, 1]]
Tf = 5
N = 4
method = 5
Output:
[[[ 1.     0.     0.     1.   ]
  [ 0.     1.     0.     0.   ]
  [ 0.     0.     1.     1.   ]
  [ 0.     0.     0.     1.   ]]
 [[ 0.937 -0.214  0.277  0.811]
  [ 0.277  0.937 -0.214  0.   ]
  [-0.214  0.277  0.937  1.651]
  [ 0.     0.     0.     1.   ]]
 [[ 0.277 -0.214  0.937  0.289]
  [ 0.937  0.277 -0.214  0.   ]
  [-0.214  0.937  0.277  3.449]
  [ 0.     0.     0.     1.   ]]
 [[-0.     0.     1.     0.1  ]
  [ 1.    -0.     0.     0.   ]
  [ 0.     1.    -0.     4.1  ]
  [ 0.     0.     0.     1.   ]]]
Code:
     Matrix4d Xstart;  Xstart << 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1;
    Matrix4d Xend;  Xend << 0, 0, 1, 0.1, 1, 0, 0, 0, 0, 1, 0, 4.1, 0, 0, 0, 1;
    double Tf = 5;
    int N = 4;
    int method = 5;
    Matrix4d X[N];
    CartesianTrajectory(Xstart, Xend, Tf, N, method, X);
    for (int i = 0; i < N; i++)
        cout << X[i] << endl;
 */
void CartesianTrajectory(Matrix4d Xstart, Matrix4d Xend, double Tf, int N, int method, Matrix4d X[]){
    double timegap = Tf / (N - 1.0), s;
    Matrix4d traj[N];
    Matrix3d Rstart, Rend, R;
    Vector3d pstart, pend, p;
    TransToRp(Xstart, Rstart, pstart);
    TransToRp(Xend, Rend, pend);
    for (int i = 0; i < N; i++){
        if (method == 3)
            s = CubicTimeScaling(Tf, timegap * i);
        else
            s = QuinticTimeScaling(Tf, timegap * i);
        R = Rstart * MatrixExp3(MatrixLog3(Rstart.transpose() * Rend) * s);
        p = s * pend + (1 - s) * pstart;
        RpToTrans(R, p, X[i]);
    }
}

/*
Takes thetalist: n-vector of joint variables,
      dthetalist: n-vector of joint rates,
      eint: n-vector of the time-integral of joint errors,
      g: Gravity vector g,
      Mlist: List of link frames {i} relative to {i-1} at the home
             position,
      Glist: Spatial inertia matrices Gi of the links,
      Slist: Screw axes Si of the joints in a space frame,
      thetalistd: n-vector of reference joint variables,
      dthetalistd: n-vector of reference joint velocities,
      ddthetalistd: n-vector of reference joint accelerations,
      Kp: The feedback proportional gain (identical for each joint),
      Ki: The feedback integral gain (identical for each joint),
      Kd: The feedback derivative gain (identical for each joint).
Returns taulist: The vector of joint forces/torques computed by the
                 feedback linearizing controller at the current instant.
Example Input:
import numpy as np
thetalist = [0.1, 0.1, 0.1]
dthetalist = [0.1, 0.2, 0.3]
eint = [0.2, 0.2, 0.2]
g = [0, 0, -9.8]
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0],[0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197],[0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[1, 0, 0, 0], [0, 1, 0, 0],[0, 0, 1, 0.14225], [0, 0, 0, 1]]
G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
Glist = [G1, G2, G3]
Mlist = [M01, M12, M23, M34]
Slist = np.array([[1, 0, 1,      0, 1,     0],
                  [0, 1, 0, -0.089, 0,     0],
                  [0, 1, 0, -0.089, 0, 0.425]]).T
thetalistd = [1.0, 1.0, 1.0]
dthetalistd = [2, 1.2, 2]
ddthetalistd = [0.1, 0.1, 0.1]
Kp = 1.3
Ki = 1.2
Kd = 1.1
Output:
[ 133.00525246  -29.94223324   -3.03276856]
 Code:
    VectorXd thetalist(3, 1); thetalist << 0.1, 0.1, 0.1;
    VectorXd dthetalist(3, 1); dthetalist << 0.1, 0.2, 0.3;
    VectorXd eint(3, 1); eint << 0.2, 0.2, 0.2;
    Vector3d g; g << 0, 0, -9.8;
    Matrix4d M01(4,4);  M01 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.089159, 0, 0, 0, 1;
    Matrix4d M12(4,4);  M12 << 0, 0, 1, 0.28, 0, 1, 0, 0.13585, -1, 0, 0, 0, 0, 0, 0, 1;
    Matrix4d M23(4,4);  M23 << 1, 0, 0, 0, 0, 1, 0, -0.1197, 0, 0, 1, 0.395, 0, 0, 0, 1;
    Matrix4d M34(4,4);  M34 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.14225, 0, 0, 0, 1;
    VectorXd g1(6); g1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1(6,6); G1 = g1.asDiagonal();
    VectorXd g2(6); g2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2(6,6); G2 = g2.asDiagonal();
    VectorXd g3(6); g3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3(6,6); G3 = g3.asDiagonal();
    MatrixXd Glist[3] = {G1, G2, G3};
    MatrixXd Mlist[4] = {M01, M12, M23, M34};
    MatrixXd Slist(6,3), temp(3,6);
    temp << 1, 0, 1, 0, 1, 0, 0, 1, 0, -0.089, 0,0, 0, 1, 0, -0.089, 0, 0.425;
    Slist = temp.transpose();
    MatrixXd thetalistd(3, 1); thetalistd << 1.0, 1.0, 1.0;
    MatrixXd dthetalistd(3, 1); dthetalistd << 2, 1.2, 2;
    MatrixXd ddthetalistd(3, 1); ddthetalistd << 0.1, 0.1, 0.1;
    double Kp = 1.3, Ki = 1.2, Kd = 1.1;
    cout << ComputedTorque(thetalist,dthetalist,eint,g,Mlist,Glist,Slist,thetalistd,dthetalistd,ddthetalistd,Kp,Ki,Kd) << endl;
 */

VectorXd ComputedTorque(VectorXd thetalist, VectorXd dthetalist, VectorXd eint, Vector3d g, MatrixXd Mlist[], MatrixXd Glist[], MatrixXd Slist,
                        VectorXd thetalistd, VectorXd dthetalistd, VectorXd ddthetalistd, double Kp, double Ki, double Kd){
    MatrixXd e = thetalistd - thetalist;
    return MassMatrix(thetalist,Mlist,Glist,Slist) * (Kp * e + Ki * (eint + e) + Kd * (dthetalistd - dthetalist))
            + InverseDynamics(thetalist,dthetalist,ddthetalistd,g,MatrixXd::Zero(6,1),Mlist,Glist,Slist);
}