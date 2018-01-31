#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>
#include <string>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

using namespace std::placeholders;

using namespace rw::math;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(projectPath+"/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
  im = imread(projectPath+"/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new GLFrameGrabber(width,height,fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }
    }

    // Set default position of robot and delta T
    _delta_t = 1-0.1;
    _initQ = Q(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
    _itterator = 0;
    _includeVisionPart = true;
    _prevQ = _initQ;

    // Marker points file:
    motionFilePath = "/SamplePluginPA10/motions/MarkerMotion";
    motionSequence[0] = "Slow";
    motionSequence[1] = "Medium";
    motionSequence[2] = "Fast";
    // Load the marker motion into member variable:
    loadMarkerMotions(projectPath + motionFilePath + motionSequence[_itterator] + ".txt");

    // Find a pointer to the device:
    _device = _wc->findDevice("PA10");

    // Finde the frame "Marker":
    _marker = (MovableFrame*) _wc->findFrame("Marker");

    dataLog.open(projectPath+"/headers.txt");
    //Headers:
    std::string del = ", ";
    dataLog << "Joint0" << del << "Joint1" << del << "Joint2" << del
            << "Joint3" << del << "Joint4" << del << "Joint5" << "Joint6" << del
            << "X" << del << "Y" << del << "Z" << del
            << "R" << del << "P" << del << "Y" << del
            << "Error" << std::endl;
    dataLog.close();

    if (_numOfPoints == 1) {
       _points.push_back(Vector3D<>(0,0,0));
    }
    else {
       _points.push_back(Vector3D<>(0.15,0.15,0));
       _points.push_back(Vector3D<>(-0.15,0.15,0));
       _points.push_back(Vector3D<>(0.15,-0.15,0));
    }

}

void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
    dataLog.close();
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
    Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
    return res;
}

void SamplePlugin::btnPressed() {

    QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
        image = ImageLoader::Factory::load(projectPath+"/SamplePluginPA10/markers/Marker1.ppm");
		_textureRender->setImage(*image);
        image = ImageLoader::Factory::load(projectPath+"/SamplePluginPA10/backgrounds/color2.ppm");
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
        resetPositions();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";

        // Calculate init u,v points
        _cameraFrame = _wc->findFrame("Camera");
        if (!_includeVisionPart)
        {
            Transform3D<> transform = inverse(_marker->fTf(_cameraFrame, _state));
            std::vector<Vector3D<>> points;
            for (int i = 0 ; i < _numOfPoints ; i++) {
                points.push_back(transform * _points.at(i));
            }
            for (int i = 0 ; i < _numOfPoints ; i++) {
                _initUV.push_back(RobworkTools::getImagePoints(points.at(i)[0],points.at(i)[1],_z,_f));
            }
        } else
        {
            //get initUV values for the tracked points
            Mat imgTemp = getImageFromCamera();

            //change to BGR for use in vision lib
            cvtColor(imgTemp, imgTemp, COLOR_RGB2BGR);

            Vision vis;
            vector<cv::Point2f> keypoints = vis.getPointsFromMaker1(imgTemp);
            for (unsigned int i = 0 ; i < keypoints.size() ; i++) {
                keypoints[i].x = keypoints[i].x - (_imageWidth/2);
                keypoints[i].y = keypoints[i].y - (_imageHeight/2);
            }

            for (int i = 0 ; i < _numOfPoints ; i++) {
                _initUV.push_back(Vector2D<>(keypoints[i].x, keypoints[i].y));
                cout << keypoints[i] << ", ";
            }
            cout << endl;
        }

        int temp_deltaT = _delta_t * 100;

        // Open datalog file:
        dataLog.open(projectPath+"/dataLog" + motionSequence[_itterator] +"_dT_" + std::to_string(temp_deltaT) + ".csv");

        //resetPositions();

        // Toggle the timer on and off
        if (!_timer->isActive())
            _timer->start(100); // run 10 Hz
        else
            _timer->stop();

	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {
    /**
    // Used for datalogging:
    if (_counter >= _markerMotions.size() && _delta_t <= 0.051 && _itterator < 2) {
        log().info() << "Resetting position, DeltaT, Itterator\n";
        log().info() << "Loading new sequenze..\n";
        _itterator++;
        _delta_t = 1.05;
        loadMarkerMotions(projectPath + motionFilePath + motionSequence[_itterator]);
        resetPositions();
    } else if (_counter >= _markerMotions.size()) {
        log().info() << "Resetting position\n";
        resetPositions();
    }
    if (_counter == 0 && _itterator < 3) {
        log().info() << "Resetting Position, counting down deltaT, making new file..\n";
        _delta_t -= 0.05;
        resetPositions();
        // Open datalog file:
        dataLog.open(projectPath+"/dataLog" + motionSequence[_itterator] +"_dT_" + std::to_string(_delta_t) + ".csv");
        //Headers:
        std::string del = ", ";
        dataLog << "Joint0" << del << "Joint1" << del << "Joint2" << del
                << "Joint3" << del << "Joint4" << del << "Joint5" << del
                << "X" << del << "Y" << del << "Z" << del
                << "R" << del << "P" << del << "Y" << del
                << "du" << del << "dv" << del
                << "du" << del << "dv" << del
                << "du" << del << "dv" << del
                << "Delta_T = " << del << _delta_t << std::endl;
    }
    */
    _state = getRobWorkStudio()->getState();

    Mat img = getImageFromCamera();
    Mat imgTemp = getImageFromCamera();

    //change to BGR for use in vision lib
    cvtColor(imgTemp, imgTemp, COLOR_RGB2BGR);

    if (++_counter < _markerMotions.size()) {
        _marker->setTransform(_markerMotions[_counter], _state);
        getRobWorkStudio()->setState(_state);
    }

    std::vector<Vector2D<>> diffUV;
    std::vector<Vector2D<>> currentUV;

    if (!_includeVisionPart) {
        Transform3D<> transform = inverse(_marker->fTf(_cameraFrame, _state));
        std::vector<Vector3D<>> points;
        for (int i = 0 ; i < _numOfPoints ; i++) {
            points.push_back(transform * _points.at(i));
        }

        for (int i = 0 ; i < _numOfPoints ; i++) {
            currentUV.push_back(RobworkTools::getImagePoints(points.at(i)[0],points.at(i)[1],_z,_f));
        }
    }
    else {
        Vision vis;
        Mat overlay;
        vector<cv::Point2f> keypoints = vis.getPointsFromMaker1(imgTemp, overlay);

        //set current view to overlay
        Mat overlayOut;
        cvtColor(overlay, overlayOut, COLOR_BGR2RGB);
        setImageFromCamera(overlayOut);

        for (unsigned int i = 0 ; i < keypoints.size() ; i++) {
            keypoints[i].x = keypoints[i].x - (_imageWidth/2);
            keypoints[i].y = keypoints[i].y - (_imageHeight/2);
        }

        for (int i = 0 ; i < _numOfPoints ; i++) {
            currentUV.push_back(Vector2D<>(keypoints[i].x, keypoints[i].y));
        }
    }

    // Calculate du, dv:
    for (int i = 0 ; i < _numOfPoints ; i++) {
        Vector2D<> dudv_temp = _initUV.at(i) - currentUV.at(i);
        diffUV.push_back(dudv_temp);
    }

    // Calculate image-Jacobian
    std::vector<Jacobian> Jimage;
    std::vector<Jacobian> Zimage;
    for (int i = 0 ; i < _numOfPoints ; i++) {
        Jimage.push_back(RobworkTools::getImageJacobian(_z, currentUV.at(i)[0], currentUV.at(i)[1], _f));
        Zimage.push_back(RobworkTools::calcZImage(Jimage.at(i), _device, _initQ, _state));
    }

    // Get Eigen:
    std::vector<Eigen::MatrixXd> Zimage_Eigen;
    std::vector<Eigen::MatrixXd> ZImage_Eigen_trans;

    std::vector<Jacobian> Zimage_temp;
    if (_numOfPoints == 1) {
        Zimage_temp.push_back(Zimage[0]);
    }
    else if (_numOfPoints == 3) {
        Jacobian temp(6,7);

        for (int i = 0 , ii = 0 ; i < 3 ; i++, ii+=2) {
            for (int j = 0 ; j < 7 ; j++) {
                temp.elem(ii, j) = Zimage[i].elem(0,j);
                temp.elem(ii+1,j) = Zimage[i].elem(1,j);
            }
        }
        Zimage_temp.push_back(temp);
    }

    Zimage_Eigen.push_back(Zimage_temp.at(0).e());
    ZImage_Eigen_trans.push_back(Zimage_Eigen.at(0).transpose());

    // Now use Moore-Penrosse inverse:
    // Z_MP = inverse(Z^T * Z) * Z^T
    Eigen::MatrixXd ZImage_Moore_Penrose(ZImage_Eigen_trans.at(0) * LinearAlgebra::pseudoInverse(Zimage_Eigen.at(0) * ZImage_Eigen_trans.at(0)));

    std::vector<Jacobian> error_temp;
    for (auto err : diffUV) {
        error_temp.push_back(Jacobian(err.e()));
    }

    std::vector<Jacobian> jack_dudv;
    std::vector<Jacobian> Y;

    if (_numOfPoints == 1) {
        jack_dudv.push_back(error_temp[0]);
        Y.push_back(Jacobian(ZImage_Moore_Penrose * jack_dudv.at(0).e()));
    }
    else if (_numOfPoints == 3) {
        Jacobian Y_temp(6,1);

        for (unsigned int i = 0 , ii = 0 ; i < error_temp.size() ; i++, ii+=2) {
            for (int j = 0 ; j < 1 ; j++) {
                Y_temp.elem(ii, j) = error_temp[i].elem(0,j);
                Y_temp.elem(ii+1,j) = error_temp[i].elem(1,j);
            }
        }

        Y.push_back(Jacobian(ZImage_Moore_Penrose * Y_temp.e()));
    }

    // Now calculate dq:
    std::vector<Q> dq;
    dq.push_back(Q(Y[0].e()));

    // Get the velocity limits of the robot:
    Q velocityLimits = _device->getVelocityLimits();
    Q accelerationLimits = _device->getAccelerationLimits();
    pair<Q,Q> positionLimits = _device->getBounds();
    Q currQ = _device->getQ(_state);

    for (unsigned int i = 0 ; i < dq.size() ; i++) {
        dq.at(i) = dq.at(i)*_delta_t;
    }

    // Generel joint limit handling
    for (unsigned int i = 0 ; i < dq.size() ; i++) {
        for (unsigned int j = 0 ; j < dq[0].size() ; j++) {

            double vMin = -velocityLimits[j]*_delta_t + currQ(j);
            double vMax = velocityLimits[j]*_delta_t + currQ(j);

            double aMin = -0.5*accelerationLimits[j]*_delta_t*(_delta_t+_delta_t) + currQ(j) + 1*(currQ(j)-_prevQ(j));
            double aMax = 0.5*accelerationLimits[j]*_delta_t*(_delta_t+_delta_t) + currQ(j) + 1*(currQ(j)-_prevQ(j));

            double pMin = positionLimits.first[j];
            double pMax = positionLimits.second[j];

            double limitMin = INT_MIN;
            double limitMax = INT_MAX;

            //take most sensitive bound
            vector<double> Qmin = {vMin, aMin, pMin};
            vector<double> Qmax = {vMax, aMax, pMax};

            limitMin = *max_element(Qmin.begin(), Qmin.end());
            limitMax = *min_element(Qmax.begin(), Qmax.end());

            //check for bounds
            if((currQ(j) + dq.at(i)[j]) < limitMin)
                dq.at(i)[j] = limitMin;
            else if ((currQ(j) + dq.at(i)[j]) > limitMax)
                dq.at(i)[j] = limitMax;
        }
    }
    std::vector<Q> temp;
    for (unsigned int i = 0 ; i < dq.size() ; i++) {
        temp.push_back(Q(_device->getQ(_state)));
        temp.at(i) += dq.at(i);
    }

    //log().info() << "dq: " << temp

    if (_counter < _markerMotions.size()) {
        _device->setQ(temp[0], _state);
        _prevQ = temp[0];
        getRobWorkStudio()->setState(_state);
        logAllData(diffUV);
    }
    /** else if (_counter >= _markerMotions.size() && _itterator >= 2) {
        log().info() << "Sequence done.\nDelta_t = " << _delta_t << "\n";
    } */
    else {
        log().info() << "Sequence done.\nDelta_t = " << _delta_t << "\n";
        dataLog.close();
    }
}

void SamplePlugin::stateChangedListener(const State& state) {
  _state = state;
}

Mat SamplePlugin::getImageFromCamera() {

    Mat img;

    if (_framegrabber != NULL) {
        // Get the image as a RW image
        Frame* cameraFrame = _wc->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        img = toOpenCVImage(image);
        Mat imflip;
        cv::flip(img, imflip, 0);
        img = imflip;

    }
    return img;
}

void SamplePlugin::setImageFromCamera(cv::Mat imflip){

    // Show in QLabel
    QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

}

void SamplePlugin::loadMarkerMotions(std::string s){

    _markerMotions.clear();

    // Load the motion of the marker into the member variable
    Vector3D<double> XYZ;
    RPY<double> rpy;

    double x,y,z,R,P,Y;

    std::ifstream input;
    input.open(s);

    if (!input.is_open()) {
        std::cerr << "Error: Could not open file: " << s << ".\n";
    }
    else {
        for (std::string line; getline(input, line);) {
            std::istringstream iss(line);
            // Write to variables:
            iss >> x >> y >> z >> R >> P >> Y;
            // Write to structs:
            XYZ = Vector3D<double>(x,y,z);
            rpy = RPY<double>(R,P,Y);
            // Push_back to member variable:
            //log().info() << "x:" << x << ", y:" << y << ", z:" << z << std::endl;
            _markerMotions.push_back(Transform3D<double>(XYZ,rpy.toRotation3D()));
        }
    }
    input.close();
}

void SamplePlugin::logAllData(std::vector<Vector2D<>> dudv){
    /** Formate: Joint variables, xyz, rpy, dudv */
    std::string del = ",";
    std::string end = "\n";

    Q q_device = _device->getQ(_state);
    Transform3D<> t_pose(_device->baseTframe(_cameraFrame, _state).P());
    RPY<> t_rot(_device->baseTframe(_cameraFrame, _state).R());

    // Joint variables:
    for (unsigned int i = 0 ; i < q_device.size() ; i++) {
        dataLog << q_device[i] << del;
    }
    // Position (xyz)
    for (unsigned int i = 0 ; i < t_pose.P().size() ; i++) {
        dataLog << t_pose.P()[i] << del;
    }
    // Orientation (RPY)
    for (unsigned int i = 0 ; i < t_rot.size() ; i++) {
        dataLog << t_rot[i] << del;
    }

    // dudv:
    double err = 0;
    for (unsigned int i = 0 ; i < dudv.size() ; i++) {
        err += sqrt( dudv[i][0] * dudv[i][0] + dudv[i][1] * dudv[i][1] );
    }
    dataLog << err << std::endl;
}

void SamplePlugin::resetPositions(){
    // Reset counter:
    _counter = 0;

    // Move the marker to the first marker position:
    _marker->setTransform(_markerMotions[0], _state);

    // Move device to start position
    _device->setQ(_initQ, _state);
}
