#include <iostream>
#include <string>
#include <vector>
#include <rw/math.hpp> // Pi, Deg2Rad
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/models.hpp>
#include <rw/loaders.hpp>
#include "RobworkTools.hpp"

using namespace std;
using namespace rw::math;
using namespace rw::models;
using namespace rw;

int main() {

    //Get image points with (x, y, z, f) = (1, 1, 4, 500)
    double x = 1;
    double y = 1;
    double z = 4;
    double f = 500;

    Vector2D<> Image_points = RobworkTools::getImagePoints(x,y,z,f);

    //calculate imageJacobian
    Jacobian Jimage = RobworkTools::getImageJacobian(x,y,z, Image_points[0], Image_points[1], f);

    cout << Jimage << endl;

    //load workcell
    const std::string workcell_path = "/home/frederik/Documents/RoVi/Robotics/WorkCells/UR5WorkCellFinal/Scene.wc.xml";
    const std::string device_name = "UR-6-85-5-A";

    WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(workcell_path);
    // Get device and check if it has been loaded correctly
    Device::Ptr device = wc->findDevice(device_name);
    if(device == nullptr) {
        RW_THROW("Device " << device_name << " was not found!");
    }

    // Get an initial configuration, q
    rw::kinematics::State state = wc->getDefaultState();
    rw::math::Q q = Q(6, 1, 1,-1, 1, 2, 1);

    //get image jacobian
    Jacobian ZImage = RobworkTools::calcZImage(Jimage, device, q, state);

    cout << ZImage << endl;

    return 0;
}

