#include <iostream>
#include <vector>
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
Transform3D<> computeTransforms(Q state, vector<Transform3D<>> transformations, int index);

int main() {

    Rotation3D<> matrix = matrixRotate('x', 60);
    cout << matrix << endl << endl;

    Rotation3D<> matrix2 = matrixRotate(60,30,40);
    cout << matrix2 << endl << endl;

    RPY<> matrixRPY = RotToRPY(matrix2);
    cout << matrixRPY << endl << endl;

    //Make transformation vectors from workcell. From base and up
    vector<Transform3D<>> transformations;

    //joint0
    Vector3D<> jt(0,0,0);
    Rotation3D<double> jr = RPY<double>(0, 0, 0).toRotation3D();
    transformations.push_back(Transform3D<double>(jt, jr));

    //joint1
    jt = Vector3D<>(0,0,0.892);
    jr = RPY<double>(90,0,90).toRotation3D();
    transformations.push_back(Transform3D<double>(jt, jr));

    //joint2
    jt = Vector3D<>(0,0.425,0);
    jr = RPY<double>(270,0,0).toRotation3D();
    transformations.push_back(Transform3D<double>(jt, jr));

    //joint3
    jt = Vector3D<>(-0.39243,0,-0.01468);
    jr = RPY<double>(0,0,0).toRotation3D();
    transformations.push_back(Transform3D<double>(jt, jr));

    //joint4
    jt = Vector3D<>(0,0,0.109);
    jr = RPY<double>(90,180,270).toRotation3D();
    transformations.push_back(Transform3D<double>(jt, jr));

    //joint5
    jt = Vector3D<>(0,0,0.093);
    jr = RPY<double>(0,0,0).toRotation3D();
    transformations.push_back(Transform3D<double>(jt, jr));


    //joint6
    jt = Vector3D<>(0,0.082,0);
    jr = RPY<double>(0,270,270).toRotation3D();
    transformations.push_back(Transform3D<double>(jt, jr));

    Q state(6,0.00);

    Transform3D<> result = computeTransforms(state, transformations, 3);
    cout << result << endl << endl;

    return 0;
}

// Ti_base = T1_base * T2_1 * ... * Ti_i-1
// Ti_i-1 = Trefi_i-1 * T_z(q_i)      ==> T_z is always rotation matrix around z. qi is the state vector component from joint i

Transform3D<> computeTransforms(Q state, vector<Transform3D<double>> transformations, int index)
{
    Transform3D<> currentTransform;
    for (int i = 0; i <= index; i++)
    {
        //calculate Ti_i-1
        Transform3D<> Tref = transformations[i];
        //rot matrix around z with Q(i)
        Rotation3D<> rotz = matrixRotate('z', state[i]);

        Transform3D<> Ti;
        Tref.multiply(Tref, Transform3D<double>(rotz), Ti);

        currentTransform.multiply(currentTransform, Ti, currentTransform);
    }

    return currentTransform;
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
