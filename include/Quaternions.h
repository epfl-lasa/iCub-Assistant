#pragma once
#include <Eigen/Dense>

using namespace Eigen;

//! Converting quaternion to rotation matrix
MatrixXd quat2dc(VectorXd st);

//! Converting rotation matrix to euler angles
Vector3d dc2ang(MatrixXd dc);

//! Converting euler angles to rotation matrix
MatrixXd ang2dc(Vector3d stang);

//! Converting rotation matrix to quaternion
VectorXd dc2quat(MatrixXd dc);

//! Converting quaternion to euler angles
Vector3d quat2ang(VectorXd st);

//! Converting euler angles to quaternion
VectorXd ang2quat(Vector3d stang);

//! loading a double** to a matrix
void matrix3_convert(MatrixXd& in, double out[3][3]);

//! loading a matrix to double**
MatrixXd matrix3_convert(double in[3][3]);

//! Generating a skew symmetric matrix from angular velocities
MatrixXd skew_symmetric(Vector3d in);

//! Getting the vector of angular velocities from a skew symmetric matrix
Vector3d skew_symmetric_inv(MatrixXd &in);

//! Generate a matrix to replace cross product: would be P(a) * q = a x q
MatrixXd make_cross_lh(VectorXd a);

//! Quaternion cross product
VectorXd quat_cross(VectorXd a, VectorXd b);

//! Quaternion triple cross product
VectorXd quat_cross(VectorXd a, VectorXd b, VectorXd c);

//! Normalize a quaternion
VectorXd quat_normalize(VectorXd in);

//! Inverse a quaternion
VectorXd quat_inverse(VectorXd in);

//! Dot product two quaternions
double quat_dot(VectorXd a, VectorXd b);

//! Another implementation of quaternion cross product
VectorXd quat_mul(VectorXd a, VectorXd b);

//! adding two quaternions
VectorXd quat_add(VectorXd a, VectorXd b);

//! subtracting two quaternions
VectorXd quat_sub(VectorXd a, VectorXd b);

//! Finding the cosine between two quaternions
double quat_cos(VectorXd a, VectorXd b);

//! The norm of a quaternion
double quat_norm(VectorXd in);

//! Calculate the angle from a quaternion (assuming axis/angle representation)
double quat_get_theta(VectorXd in);

//! Scale the quaternion (angle) by a scalar
VectorXd quat_scale(VectorXd in, double d);

//! The exponential of a quaternion: e^q
VectorXd quat_exp(VectorXd in);

//! The log of a quaternion: ln(q)
VectorXd quat_log(VectorXd in);

//! The power of a quaternion
VectorXd quat_pow(VectorXd in, double t);

//! Slerp function using quaternion
VectorXd quat_slerp(VectorXd q0, VectorXd q1, double t);

//! Creating a derivative for quaternion
VectorXd quat_deriv(Vector3d in);

//! difference between two quaternions
VectorXd quat_diff(Vector4d a, Vector4d b);

