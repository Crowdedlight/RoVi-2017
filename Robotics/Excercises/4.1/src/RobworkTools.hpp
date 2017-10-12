//
// Created by frederik on 10/10/17.
//

#ifndef INC_4_1_ROBWORKTOOLS_H
#define INC_4_1_ROBWORKTOOLS_H


#include <rw/math.hpp> // Pi, Deg2Rad

class RobworkTools {

    public:

    static rw::math::Transform3D<> forwardKinematics(const std::vector<rw::math::Transform3D<>> &trefs,
                                                     const unsigned int idx, const rw::math::Q &q) {
        if (trefs.size() != q.size()) {
            RW_THROW("The number of local transformations must be equal to the length of the configuration vector");
        }

        rw::math::Transform3D<> baseTi;

        for (unsigned int i = 0; i < idx; ++i) {
            rw::math::Transform3D<> Tz(rw::math::RPY<>(q[i], 0, 0).toRotation3D());
            baseTi = baseTi * trefs[i] * Tz;
        }

        return baseTi;
    };

    static rw::math::RPY<> RotToRPY(rw::math::Rotation3D<> &matrix) {
        double thetaY = asin(-matrix(2, 0));
        double thetaX, thetaZ;

        //if r_31 != 1
        if (matrix(2, 0) != 1) {
            thetaX = atan2(cos(matrix(2, 1)), cos(matrix(2, 2)));
            thetaZ = atan2(cos(matrix(1, 0)), cos(matrix(0, 0)));
        } else {
            throw ("R_31 = 1 error");
        }

        return rw::math::RPY<>(thetaX, thetaY, thetaZ);
    };

    static rw::math::Rotation3D<> matrixRotate(float rotationX, float rotationY, float rotationZ) {
        rw::math::Rotation3D<> rotX = matrixRotate('x', rotationX);
        rw::math::Rotation3D<> rotY = matrixRotate('y', rotationY);
        rw::math::Rotation3D<> rotZ = matrixRotate('z', rotationZ);

        rw::math::Rotation3D<> rotXY = rotX.multiply(rotX, rotY);
        rw::math::Rotation3D<> rotXZ = rotXY.multiply(rotXY, rotZ);

        return rotXZ;
    };

    static rw::math::Rotation3D<> matrixRotate(char axis, float angle) {
        //make rotation matrix around axis with given Vector3D
        if (axis == 'x') {
            rw::math::Rotation3D<> rotMatrix(1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle));
            return rotMatrix;
        } else if (axis == 'y') {
            rw::math::Rotation3D<> rotMatrix(cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle));
            return rotMatrix;
        } else if (axis == 'z') {
            rw::math::Rotation3D<> rotMatrix(cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1);
            return rotMatrix;
        }
    };

    //compute jacobien
    static rw::math::Jacobian getJacobian(const std::vector<rw::math::Transform3D<>> &trefs, const rw::math::Q &q, const rw::math::Transform3D<> &tcpFrame) {
        //trefs include the lastjoint to tcp frame, hence need to do -1
        rw::math::Jacobian jacobian = rw::math::Jacobian::zero(6, trefs.size());

        //calculate pbasetcp
        rw::math::Transform3D<> t_toolbase = forwardKinematics(trefs, trefs.size(), q)*tcpFrame;
        rw::math::Vector3D<> p_toolbase = t_toolbase.P();

        for (int i = 0; i < trefs.size(); i++) {

            //calculate first equation
            rw::math::Transform3D<> tibase = forwardKinematics(trefs, i+1, q);
            rw::math::Vector3D<> zibase = tibase.R().getCol(2);

            //calculate pbasei
            rw::math::Vector3D<> pibase = tibase.P();

            //calculate p_toolbase - pibase
            rw::math::Vector3D<> psum = p_toolbase - pibase;

            //calculate cross product zibase X psum
            rw::math::Vector3D<> coli_pos = rw::math::cross(zibase, psum);

            //Rotation Jacobian
            rw::math::Vector3D<> coli_rot = zibase;

            //add to jacobian
            jacobian.addPosition(coli_pos, 0, i);
            jacobian.addRotation(coli_rot, 0, i);
        }

        return jacobian;
    }

};


#endif //INC_4_1_ROBWORKTOOLS_H
