#include <iostream>
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>

using namespace std;
using namespace rw::math;

Rotation3D<> matrixRotate(char axis, float angle);
Rotation3D<> matrixRotate(float rotationX, float rotationY, float rotationZ);
RPY<> RotToRPY(Rotation3D<> &matrix);

int main() {

    Rotation3D<> matrix = matrixRotate('x', 60);
    cout << matrix << endl << endl;

    Rotation3D<> matrix2 = matrixRotate(60,30,40);
    cout << matrix2 << endl << endl;

    RPY<> matrixRPY = RotToRPY(matrix2);
    cout << matrixRPY << endl << endl;

    return 0;
}

RPY<> RotToRPY(Rotation3D<> &matrix)
{
    double thetaY = asin(-matrix(2,0));
    double thetaX, thetaZ;

    //if r_31 != 1
    if(matrix(2,0) != 1)
    {
        thetaX = atan2(cos(matrix(2,1)), cos(matrix(2,2)));
        thetaZ = atan2(cos(matrix(1,0)), cos(matrix(0,0)));
    } else
    {
        throw("R_31 = 1 error");
    }

    return RPY<>(thetaX, thetaY, thetaZ);
}

Rotation3D<> matrixRotate(float rotationX, float rotationY, float rotationZ)
{
    Rotation3D<> rotX = matrixRotate('x', rotationX);
    Rotation3D<> rotY = matrixRotate('y', rotationY);
    Rotation3D<> rotZ = matrixRotate('z', rotationZ);

    Rotation3D<> rotXY = rotX.multiply(rotX, rotY);
    Rotation3D<> rotXZ = rotXY.multiply(rotXY, rotZ);

    return rotXZ;
}

Rotation3D<> matrixRotate(char axis, float angle)
{
    //make rotation matrix around axis with given Vector3D
    if (axis == 'x')
    {
        Rotation3D<> rotMatrix(1,0,0,0,cos(angle),-sin(angle),0,sin(angle),cos(angle));
        return rotMatrix;
    }else if (axis == 'y')
    {
        Rotation3D<> rotMatrix(cos(angle),0,sin(angle),0,1,0,-sin(angle),0,cos(angle));
        return rotMatrix;
    }else if (axis == 'z') {
        Rotation3D<> rotMatrix(cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1);
        return rotMatrix;
    }
}
