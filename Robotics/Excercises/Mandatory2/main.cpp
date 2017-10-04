#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

void PathToLua(QPath& path, string filename);

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}


void PathToLua(QPath& path, string filename)
{
    // start stream to open txt file and append hardcoded part
    ofstream luaout;
    luaout.open ("luaPath.txt");

    //append hardcode
    luaout << "wc = rws.getRobWorkStudio():getWorkCell()\n"
            "state = wc:getDefaultState()\n"
            "device = wc:findDevice(\"KukaKr16\")\n"
            "gripper = wc:findFrame(\"Tool\");\n"
            "bottle = wc:findFrame(\"Bottle\");\n"
            "table = wc:findFrame(\"Table\");\n"
            "\n"
            "function setQ(q)\n"
            "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
            "device:setQ(qq,state)\n"
            "rws.getRobWorkStudio():setState(state)\n"
            "rw.sleep(0.1)\n"
            "end\n"
            "\n"
            "function attach(obj, tool)\n"
            "rw.gripFrame(obj, tool, state)\n"
            "rws.getRobWorkStudio():setState(state)\n"
            "rw.sleep(0.1)\n"
            "end";

    for (int i = 0; i < path.size(); i++)
    {
        luaout << "setQ({";
        for (int j = 0; j < path[i].size(); j++)
        {
            //if first pos don't write out "," else do
            if (j > 0)
                luaout << ",";
            luaout << path[i][j];
        }

        //all parameters in q posted out, end setQ line
        luaout << "})\n";
    }

    //code for pick and place. Need to know when to do that
    //attach(bottle,gripper)
    //attach(bottle,table)

    //file finished, close stream
    luaout.close();
}