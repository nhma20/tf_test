/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "geometry.h"
#include <iostream>

/*****************************************************************************/
// Function implementations
/*****************************************************************************/

rotation_matrix_t eulToR(orientation_t eul) {

    Eigen::AngleAxisf rollAngle(eul[0], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(eul[2], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(eul[1], Eigen::Vector3f::UnitX());

    Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3f rotationMatrix = q.normalized().toRotationMatrix();

    return rotationMatrix;

}

vector_t rotateVector(rotation_matrix_t R, vector_t v) {

    vector_t ret_vec = R*v;

    return ret_vec;
}

point_t projectPointOnPlane(point_t point, plane_t plane) {

    line_t l = {
        .p = point,
        .v = plane.normal
    };

    float t = - plane.normal.dot(l.p) / plane.normal.dot(plane.normal);

    point_t proj_point = l.p + (point_t)(t*l.v);

    return proj_point;

}

orientation_t quatToEul(quat_t quat) {

    Eigen::Quaternionf q;
    q.x() = quat[0];
    q.y() = quat[1];
    q.z() = quat[2];
    q.w() = quat[3];

    orientation_t eul  = q.toRotationMatrix().eulerAngles(0, 1, 2);

    return eul;

}

quat_t quatInv(quat_t quat) {

    quat_t ret_quat(quat[0], -quat[1], -quat[2], -quat[3]);

    return ret_quat;

}

quat_t quatMultiply(quat_t quat1, quat_t quat2) {

    quat_t ret_quat(
        quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
        quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
        quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
        quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]
    );

    return ret_quat;

}

rotation_matrix_t quatToMat(quat_t quat) {

    Eigen::Quaternionf q;
    q.x() = quat[0];
    q.y() = quat[1];
    q.z() = quat[2];
    q.w() = quat[3];
    Eigen::Matrix3f test_ER = q.normalized().toRotationMatrix(); //.cast<float>()

    return test_ER;

}

quat_t matToQuat(rotation_matrix_t R) {

    Eigen::Quaternionf q(R);

    quat_t quat(q.x(), q.y(), q.z(), q.w());

    return quat;

}

quat_t eulToQuat(orientation_t eul) {

    Eigen::AngleAxisf rollAngle(eul[0], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf yawAngle(eul[2], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf pitchAngle(eul[1], Eigen::Vector3f::UnitX());

    Eigen::Quaternionf q = rollAngle * yawAngle * pitchAngle;

    quat_t quat;
    quat(0) = q.x();
    quat(1) = q.y();
    quat(2) = q.z();
    quat(3) = q.w();

    return quat;

}

transform_t getTransformMatrix(vector_t vec, quat_t quat) {

    Eigen::Matrix4f Trans; // Transformation Matrix

    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    
    Trans.block<3,3>(0,0) = quatToMat(quat); // Copy rotation matrix into Trans
    
    Trans.block<3,1>(0,3) = vec; // Copy translation matrix into Trans

    return Trans;
}