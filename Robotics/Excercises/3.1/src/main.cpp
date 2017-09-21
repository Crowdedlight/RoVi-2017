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

int main() {

    Rotation3D<> matrix = matrixRotate('x', 60);
    cout << matrix << endl << endl;

    return 0;
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
