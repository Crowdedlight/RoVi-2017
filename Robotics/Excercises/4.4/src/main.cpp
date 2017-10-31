#include <iostream>
#include <vector>
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include "RobworkTools.hpp"

using namespace std;
using namespace rw::math;

int main() {

    //Get image points with (x, y, z, f) = (1, 1, 4, 500)
    double x = 1;
    double y = 1;
    double z = 4;
    double f = 500;

    Vector2D<> Image_points = RobworkTools::getImagePoints(x,y,z,f);

    //calculate imageJacobian
    Jacobian Jimage = RobworkTools::getImageJacobian(x,y,z, Image_points[0], Image_points[1], f);


    return 0;
}

