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

    // Create Tref vector
    // Joint 0: <RPY> 0 0 0 </RPY> <Pos> 0 0 3 </Pos>
    rw::math::Vector3D<> V0(0, 0, 3);
    rw::math::RPY<> R0(0, 0, 0);
    rw::math::Transform3D<> T0(V0, R0.toRotation3D());

    // Joint 1: <RPY> 0 0 pi/2 </RPY> <Pos> 0 0 0 </Pos>
    rw::math::Vector3D<> V1(0, 0, 0);
    rw::math::RPY<> R1(0, 0, 90*rw::math::Deg2Rad);
    rw::math::Transform3D<> T1(V1, R1.toRotation3D());

    // Joint 2: <RPY> 0 0 0 </RPY> <Pos> 0 2 0 </Pos>
    rw::math::Vector3D<> V2(0, 2, 0);
    rw::math::RPY<> R2(0, 0, 0);
    rw::math::Transform3D<> T2(V2, R2.toRotation3D());

    // Joint 3: <RPY> 0 0 0 </RPY> <Pos> 2 0 0 </Pos>
    rw::math::Vector3D<> V3(2, 0, 0);
    rw::math::RPY<> R3(0, 0, 0);
    rw::math::Transform3D<> Ttool(V3, R3.toRotation3D());

    vector<Transform3D<>> trefs = {T0, T1, T2};

    //state vector
    Q q(3, 0, -Pi/6.0, Pi/6.0);

    Jacobian jq = RobworkTools::getJacobian(trefs, q, Ttool);

    cout << jq << endl;

    return 0;
}

