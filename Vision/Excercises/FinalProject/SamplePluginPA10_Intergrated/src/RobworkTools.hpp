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

        for (unsigned int i = 0; i < trefs.size(); i++) {

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

//    static rw::math::Q<> parabolicBlend(double time, rw::math::Q midpoint, rw::math::VectorND v1, rw::math::VectorND v2, double tau)
//    {
//        rw::math::Q newState = ((v2-v1)/4.0*tau) * pow(time + tau, 2) + v1*(time) + midpoint;
//    }

    static rw::math::Vector2D<> getImagePoints(double x, double y, double z, double f)
    {

        //u
        double u = (f*x)/z;
        //v
        double v = (f*y)/z;

        rw::math::Vector2D<> result = {u,v};
        return result;
    }

    static std::vector<rw::math::Vector2D<>> getImagePoints(std::vector<rw::math::Vector3D<>> points, double z, double f)
    {
        std::vector<rw::math::Vector2D<>> result;
        for (unsigned int i = 0 ; i < points.size() ; i++) {
            //u
            double u = (f*points[i][0])/z;
            //v
            double v = (f*points[i][1])/z;
            rw::math::Vector2D<> temp = {u,v};
            result.push_back(temp);
        }
        return result;
    }

    static rw::math::Jacobian getImageJacobian(double z, double u, double v, double f)
    {
        //calculate Jacobian
        rw::math::Jacobian Jimage(2,6);

        Jimage.elem(0,0) = -(f/z);
        Jimage.elem(0,1) = 0;
        Jimage.elem(0,2) = u/z;
        Jimage.elem(0,3) = (u*v)/f;
        Jimage.elem(0,4) = -(pow(f,2)+pow(u,2))/f;
        Jimage.elem(0,5) = v;

        Jimage.elem(1,0) = 0;
        Jimage.elem(1,1) = -(f/z);
        Jimage.elem(1,2) = v/z;
        Jimage.elem(1,3) = (pow(f,2)+pow(v,2))/f;
        Jimage.elem(1,4) = -(u*v)/f;
        Jimage.elem(1,5) = -u;

        return Jimage;
    }

    static rw::math::Jacobian calcZImage(rw::math::Jacobian jImage, rw::models::Device::Ptr device, rw::math::Q q,
                                         rw::kinematics::State state)
    {
        //set robot right position
        //device->setQ(q, state);

        //baseTool
        rw::math::Rotation3D<> baseRTool = device->baseTend(state).R().inverse();

        //make jacobian
        rw::math::Jacobian S(6,6);

        //set eigen matrix to zero
        S.e().setZero();

        //set eigen matrix block equal to baseTool's
        S.e().block<3,3>(0,0) = S.e().block<3,3>(3,3) = baseRTool.e();

        //get jacobian
        rw::math::Jacobian Jbase = device->baseJend(state);

        return jImage*S*Jbase;
    }

};


#endif //INC_4_1_ROBWORKTOOLS_H
