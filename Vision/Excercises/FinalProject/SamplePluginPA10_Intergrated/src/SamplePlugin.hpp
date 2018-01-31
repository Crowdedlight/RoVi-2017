#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

// Qt
#include <QTimer>

#include "ui_SamplePlugin.h"
#include "RobworkTools.hpp"
#include "Vision.h"

#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rw/math/LinearAlgebra.hpp>

#include <algorithm>
#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace rw::math;

using namespace rws;

using namespace cv;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();
    void timer();

    void stateChangedListener(const rw::kinematics::State& state);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    Mat getImageFromCamera();
    void setImageFromCamera(cv::Mat img);

    QTimer* _timer;

    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;

    void loadMarkerMotions(std::string s);
    void logAllData(std::vector<Vector2D<>> dudv);
    void resetPositions();

    unsigned int _itterator;
    const int _numOfPoints = 3;
    const std::string projectPath = "/home/frederik/Desktop/Temp";
    bool _includeVisionPart;
    std::string motionFilePath;
    std::string motionSequence[3];
    std::vector<Transform3D<double>> _markerMotions;
    Device::Ptr _device;
    unsigned int _counter;
    MovableFrame* _marker;
    Frame* _cameraFrame;
    double _delta_t;                // Time-step
    const double _f = 823.0;        // Focal lenght
    const double _z = 0.5;          // Distance to camera;
    const int _imageWidth = 1024;
    const int _imageHeight = 768;
    Q _initQ;
    Q _prevQ;
    std::vector<Vector2D<double>> _initUV;
    Transform3D<> _t_pose_init;
    RPY<> _t_rot_init;
    std::ofstream dataLog;             // Log of data for statistics
    std::vector<Vector3D<>> _points;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
