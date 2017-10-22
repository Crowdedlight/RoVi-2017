#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <fstream>
#include <string>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rw::kinematics;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10.

void PathToLua(QPath& path, string filename);

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

int main(int argc, char** argv) {
	const string wcFile = "/home/frederik/Documents/RoVi/Robotics/WorkCells/Kr16WallWorkCell/Scene.wc.xml";
	const string deviceName = "KukaKr16";
	cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

	WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
	Device::Ptr device = wc->findDevice(deviceName);
	if (device == NULL) {
		cerr << "Device: " << deviceName << " not found!" << endl;
		return 0;
	}
	State state = wc->getDefaultState();

    //from and to states
    Q from(6,-3.142,-0.827,-3.002,-3.143,0.099,-1.573);
    Q to(6,1.571,0.006,0.030,0.153,0.762,4.49);
    device->setQ(from, state); //set state to start point

    //get gripper and bottle frame
    Frame* tool = wc->findFrame("Tool");
    MovableFrame* bottle = dynamic_cast<MovableFrame *>(wc->findFrame("Bottle"));

    rw::kinematics::Kinematics::gripFrame(bottle, tool, state);

    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

	/** More complex way: allows more detailed definition of parameters and methods */
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
	double extend = 1.3;

	if (!checkCollisions(device, state, detector, from))
		return 0;
	if (!checkCollisions(device, state, detector, to))
		return 0;

    cout << "Planning from " << from << " to " << to << endl;
    ofstream timeout;
    ofstream stepout;
    timeout.open("../timeout.csv");
    stepout.open("../stepout.csv");

    for (double j = 0.05; j < 5; j += 0.05)
    {
        extend = j;
        QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
        cout << "Epsilon = " << j << endl;
        timeout << j << ",";
        stepout << j << ",";
        for (int i = 0; i < 100; i++) {
            //redo seed time of day
            Math::seed();


            QPath path;
            Timer t;
            t.resetAndResume();
            planner->query(from,to,path,MAXTIME);
            t.pause();
            //cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;

            timeout << t.getTime() << ",";
            stepout << path.size() << ",";

            if (t.getTime() >= MAXTIME) {
                cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
            }
        }
        timeout << endl;
        stepout << endl;
    }
    timeout.close();
    stepout.close();
	cout << "Program done." << endl;
    //PathToLua(path, "luaout");
	return 0;
}

void PathToLua(QPath& path, string filename)
{
    // start stream to open txt file and append hardcoded part
    ofstream luaout;
    luaout.open ("../" + filename + ".txt");

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
            "end"
            "\n";

    for (long i = 0; i < path.size(); i++)
    {
        luaout << "setQ({";
        for (long j = 0; j < path[i].size(); j++)
        {
            //if first pos don't write out "," else do
            if (j > 0)
                luaout << ",";
            luaout << path[i][j];
        }

        //all parameters in q posted out, end setQ line
        luaout << "})\n";

        if (i == 0) {
            luaout << "attach(bottle,gripper)" << "\n";
        }
        if(i == path.size()-1) {
            luaout << "attach(bottle,table)" << "\n";
        }
    }

    //file finished, close stream
    luaout.close();
}