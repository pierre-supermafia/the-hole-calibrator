#include "ofApp.h"

#define RECONNECT_TIME 400

#define DEPTH_X_RES 640
#define DEPTH_Y_RES 480


//--------------------------------------------------------------
void ofApp::setup(){

	ofLog(OF_LOG_NOTICE) << "MainAPP: looking for RealSense Device...";

	ofSetLogLevel(OF_LOG_VERBOSE);

	realSense = RSDevice::createUniquePtr();

	realSense->checkConnectedDialog();

	//realSense->hardwareReset();

	realSense->setVideoSize(REALSENSE_VIDEO_WIDTH, REALSENSE_VIDEO_HEIGHT);

	ofLog(OF_LOG_NOTICE) << "... RealSense Device found.";

	// we don't want to be running to fast
	//ofSetVerticalSync(true);
	//ofSetFrameRate(30);
    
	/////////////////////////////
	//     DEFINE VIEWPORTS    //
	/////////////////////////////

	float xOffset = VIEWGRID_WIDTH; //ofGetWidth() / 3;
	float yOffset = VIEWPORT_HEIGHT / N_CAMERAS;

	viewMain.x = xOffset;
	viewMain.y = 0;
	viewMain.width = ofGetWidth() - xOffset - MENU_WIDTH / 2; //xOffset * 2;
	viewMain.height = VIEWPORT_HEIGHT;

	for (int i = 0; i < N_CAMERAS; i++) {

		viewGrid[i].x = 0;
		viewGrid[i].y = yOffset * i;
		viewGrid[i].width = xOffset;
		viewGrid[i].height = yOffset;
	}

    iMainCamera = 0;

	previewCam.setUpAxis(glm::vec3(0, 0, 1));
	previewCam.setTranslationSensitivity(2., 2., 2.);
	previewCam.setNearClip(0.001f);

	bool invisibleMenu = false;
	
	/////////////////////////////
	//   REALSENSE GUI   SETUP //
	/////////////////////////////

	ofLog(OF_LOG_NOTICE) << "MainAPP: loading postprocessing GUI";

	post = gui.addPanel();
	post->setName("PostProcessing");
	post->add(realSense->param_usePostProcessing);
	post->add(realSense->param_filterDecimation);
	post->add(realSense->param_filterDecimation_mag);
	post->add(realSense->param_filterDisparities);
	post->add(realSense->param_filterSpatial);
	post->add(realSense->param_filterSpatial_smoothAlpha);
	post->add(realSense->param_filterSpatial_smoothDelta);
	post->add(realSense->param_filterSpatial_mag);
	post->add(realSense->param_filterTemporal);
	post->add(realSense->param_filterTemporal_smoothAlpha);
	post->add(realSense->param_filterTemporal_smoothDelta);
	post->add(realSense->param_filterTemporal_persistency);

	post->loadFromFile("postprocessing.xml");


    /////////////////////////////
    //CALIBRATION GUI   SETUP //
    ////////////////////////////
	ofLog(OF_LOG_NOTICE) << "MainAPP: loading calibration settings";

    setupCalib = gui.addPanel();
    
    setupCalib->setName("Calibration Panel");

    setupCalib->add(calibPoint_X.set("calibrationPoint_X", ofVec2f(REALSENSE_VIDEO_WIDTH / 2, REALSENSE_VIDEO_HEIGHT / 2), ofVec2f(0, 0), ofVec2f(REALSENSE_VIDEO_WIDTH, REALSENSE_VIDEO_HEIGHT)));
    setupCalib->add(calibPoint_Y.set("calibrationPoint_Y", ofVec2f(REALSENSE_VIDEO_WIDTH / 2, REALSENSE_VIDEO_HEIGHT / 2), ofVec2f(0, 0), ofVec2f(REALSENSE_VIDEO_WIDTH, REALSENSE_VIDEO_HEIGHT)));
    setupCalib->add(calibPoint_O.set("calibrationPoint_Z", ofVec2f(REALSENSE_VIDEO_WIDTH / 2, REALSENSE_VIDEO_HEIGHT / 2), ofVec2f(0, 0), ofVec2f(REALSENSE_VIDEO_WIDTH, REALSENSE_VIDEO_HEIGHT)));
 
	setupCalib->setVisible(invisibleMenu);

    setupCalib->loadFromFile("settings.xml");

	////////////////////////////
	//   GUI   Transfromation //
	////////////////////////////
	ofLog(OF_LOG_NOTICE) << "MainAPP: loading transformation matrix";

	guitransform = gui.addPanel();

	guitransform->setName("Transformation");

	transformationGuiGroup.setName("Matrix");
	transformationGuiGroup.add(transformation.set("Transform", ofMatrix4x4()));

	guitransform->addGroup(transformationGuiGroup);

	guitransform->setVisible(invisibleMenu);

	guitransform->loadFromFile("transformation.xml");

	updateMatrix();

	////////////////////////////
	//   GUI   Networking     //
	////////////////////////////

	ofLog(OF_LOG_NOTICE) << "MainAPP: loading networking parameters";

	networking = gui.addPanel();
	networking->setName("Broadcasting");
	networking->add<ofxGuiIntInputField>(serverId.set("ServerID", 0, 0, 10));

	networking->loadFromFile("broadcast.xml");


	/////////////////////////////
	//   GUI   DEVICE PARAMS   //
	/////////////////////////////

	ofLog(OF_LOG_NOTICE) << "MainAPP: loading Device Operation GUI";

	device = gui.addPanel();

    ////////////////////////
    //    RealSense       // 
    ////////////////////////

	ofLog(OF_LOG_NOTICE) << "MainAPP: starting attached Device...";

	// firing up the device, creating the GUI and loading the device parameters
	if (realSense->capture()) {
		createGUIDeviceParams();
	}

	ofLog(OF_LOG_NOTICE) << "...starting attached Device done.";

    /////////////////
	// creating preview point cloud is bogging the system down, so switched off at startup
	bPreviewPointCloud = false;
 
    setupViewports();

    createHelp();
    
    capMesh.reSize(4);

	if (ofIsGLProgrammableRenderer()) {
		ofLog(OF_LOG_NOTICE) << "ofIsGLProgrammableRenderer() = " << ofToString(ofIsGLProgrammableRenderer());
	}
}

void ofApp::createGUIDeviceParams() {

	device->clear();
	device->setName("RealSense Device");
	std::string serial = realSense->getSerialNumber(-1);
	device->add<ofxGuiLabel>(serial);

	intrinsicGuiGroup.clear();
	intrinsicGuiGroup.setName("Settings");
	intrinsicGuiGroup.add(realSense->param_deviceLaser);
	intrinsicGuiGroup.add(realSense->param_deviceLaser_mag);
	intrinsicGuiGroup.add(realSense->param_deviceAutoExposure);
	intrinsicGuiGroup.add(realSense->param_deviceExposure_mag);
	intrinsicGuiGroup.add(realSense->param_deviceGain_mag);
	intrinsicGuiGroup.add(realSense->param_deviceFrameQueSize_mag);
	intrinsicGuiGroup.add(realSense->param_deviceAsicTemparature);
	intrinsicGuiGroup.add(realSense->param_deviceProjectorTemparature);

	device->addGroup(intrinsicGuiGroup);
}

//--------------------------------------------------------------
void ofApp::setupViewports(){
	//call here whenever we resize the window
 
	networking->setWidth(MENU_WIDTH / 4);
	device->setWidth(MENU_WIDTH / 4);
	post->setWidth(MENU_WIDTH / 4);

	networking->setPosition(ofGetWidth() - MENU_WIDTH / 2, 20);
	device->setPosition(ofGetWidth() - MENU_WIDTH / 4, 20);
	post->setPosition(ofGetWidth() - MENU_WIDTH / 4, 400);
}

void ofApp::measurementCycleRaw(){
    if(cycleCounter < N_MEASURMENT_CYCLES){
        planePoint1Meas[cycleCounter] = calcPlanePoint(calibPoint_X, 0, 1);
        planePoint2Meas[cycleCounter] = calcPlanePoint(calibPoint_Y, 0, 1);
        planePoint3Meas[cycleCounter] = calcPlanePoint(calibPoint_O, 0, 1);
		if (planePoint1Meas[cycleCounter].z > 0 &&
			planePoint2Meas[cycleCounter].z > 0 &&
			planePoint3Meas[cycleCounter].z > 0) {
			cycleCounter++;
		}
    } else {
        planePoint_X = ofVec3f();
        planePoint_Y = ofVec3f();
        planePoint_O = ofVec3f();
        for(int y = 0; y < N_MEASURMENT_CYCLES; y++){
            planePoint_X += planePoint1Meas[y];
            planePoint_Y += planePoint2Meas[y];
            planePoint_O += planePoint3Meas[y];
        }
        planePoint_X /= N_MEASURMENT_CYCLES;
        planePoint_Y /= N_MEASURMENT_CYCLES;
        planePoint_O /= N_MEASURMENT_CYCLES;
        bUpdateMeasurment = false;
        bUpdateMeasurmentFine = true;
        cycleCounter = 0;
    }
}

void ofApp::measurementCycleFine(){
    if(cycleCounter < N_MEASURMENT_CYCLES){
        ofVec3f p1meas = calcPlanePoint(calibPoint_X, 0, 1);
        ofVec3f p2meas = calcPlanePoint(calibPoint_Y, 0, 1);
        ofVec3f p3meas = calcPlanePoint(calibPoint_O, 0, 1);
        if(planePoint_X.z / 1.05 < p1meas.z &&
           p1meas.z < planePoint_X.z * 1.05 &&
           planePoint_Y.z / 1.05 < p2meas.z &&
           p2meas.z < planePoint_Y.z * 1.05 &&
           planePoint_O.z / 1.05 < p3meas.z &&
           p3meas.z < planePoint_O.z * 1.05){
            planePoint1Meas[cycleCounter] = p1meas;
            planePoint2Meas[cycleCounter] = p2meas;
            planePoint3Meas[cycleCounter] = p3meas;
            cycleCounter++;
        }
    } else {
        planePoint_X = ofVec3f();
        planePoint_Y = ofVec3f();
        planePoint_O = ofVec3f();
        for(int y = 0; y < N_MEASURMENT_CYCLES; y++){
            planePoint_X += planePoint1Meas[y];
            planePoint_Y += planePoint2Meas[y];
            planePoint_O += planePoint3Meas[y];
        }
        planePoint_X /= N_MEASURMENT_CYCLES;
        planePoint_Y /= N_MEASURMENT_CYCLES;
        planePoint_O /= N_MEASURMENT_CYCLES;
        bUpdateMeasurmentFine = false;
        cycleCounter = 0;
        updateCalc();
    }
}

//--------------------------------------------------------------
void ofApp::updateCalc(){

	// This algorithm calculates the transformation matrix to 
	// transform from the camera centered coordinate system to the
	// calibration points defined coordinate system, where
	//   point o represents the coordinate center
	//   point x represents the x - axis from the coordinate center
	//   point y represents the y - axis from the coordinate center

	// translation vector to new coordinate system
	glm::vec3 translate = glm::vec3(planePoint_O);

	glm::vec3 newXAxis = glm::normalize(glm::vec3(planePoint_X - planePoint_O));
	glm::vec3 newYAxis = glm::normalize(glm::vec3(planePoint_Y - planePoint_O));
	glm::vec3 newZAxis = glm::cross(newXAxis, newYAxis);

	// Uncomment the next line in case there could be some conditions
	// where it is preferable to have an orthogonal basis
	// newYAxis = glm::cross(newZAxis, newXAxis);

	// the following solution was inspired by this post: https://stackoverflow.com/questions/34391968/how-to-find-the-rotation-matrix-between-two-coordinate-systems
	// however: it uses a 4x4 matrix and puts translation data as follows:
	//{ x.x x.y x.z 0 y.x y.y y.z 0 z.x z.y z.z 0 t.x t.y t.z 1 }

	float mat[16] = {
		newXAxis.x,
		newXAxis.y,
		newXAxis.z,
		0,
		newYAxis.x,
		newYAxis.y,
		newYAxis.z,
		0,
		newZAxis.x,
		newZAxis.y,
		newZAxis.z,
		0,
		translate.x,
		translate.y,
		translate.z,
		1
	};

	// and what we need at the end is the inverse of this:
	glm::mat4 transform = glm::inverse(glm::make_mat4x4(mat));

    geometry.clear();
    geometry.setMode(OF_PRIMITIVE_LINES);

	geometry.addColor(ofColor::red);
	geometry.addVertex(translate);
	geometry.addColor(ofColor::red);
	geometry.addVertex(translate + newXAxis);

	geometry.addColor(ofColor::green);
    geometry.addVertex(translate);
    geometry.addColor(ofColor::green);
    geometry.addVertex(translate + newYAxis); 
    
    geometry.addColor(ofColor::blue);
    geometry.addVertex(translate);
    geometry.addColor(ofColor::blue);
    geometry.addVertex(translate + newZAxis);

    calcdata = string("distance to new coordinate center point: " + ofToString(glm::length(translate)) + "\n");
	calcdata += "position point X: " + ofToString(planePoint_X) + "\n";
	calcdata += "position point Y: " + ofToString(planePoint_Y) + "\n";
	calcdata += "position point Z: " + ofToString(planePoint_O) + "\n";
	calcdata += "distance to X: " + ofToString(planePoint_X.length()) + "\n";
	calcdata += "distance to Y: " + ofToString(planePoint_Y.length()) + "\n";
    calcdata += "distance to Z: " + ofToString(planePoint_O.length()) + "\n";
	calcdata += "distance X to Z: " + ofToString(ofVec3f(planePoint_X - planePoint_O).length()) + "\n";
	calcdata += "distance Y to Z: " + ofToString(ofVec3f(planePoint_Y - planePoint_O).length()) + "\n";
    
    bUpdateCalc = false;
    
	ofLog(OF_LOG_NOTICE) << "updating... ";

	transformation.set(ofMatrix4x4(transform));

	updateMatrix();
}

//--------------------------------------------------------------
void ofApp::updateMatrix(){

	sphere_X.setPosition(planePoint_X);
	sphere_Y.setPosition(planePoint_Y);
	sphere_Z.setPosition(planePoint_O);

	sphere_X.setRadius(0.05);
	sphere_Y.setRadius(0.05);
	sphere_Z.setRadius(0.05);

	deviceTransform = transformation.get(); 
}

//--------------------------------------------------------------
glm::vec3 ofApp::calcPlanePoint(ofParameter<ofVec2f> & cpoint, int size, int step){
	glm::vec3 ppoint;

	int width = 2 * realSense->getDepthWidth();
    int height = 2 * realSense->getDepthHeight();
   
    int counter = 0;
    
    int minX = ((cpoint.get().x - size) >= 0)?(cpoint.get().x - size): 0;
    int minY = ((cpoint.get().y - size) >= 0)?(cpoint.get().y - size): 0;
    int maxX = ((cpoint.get().x + size) < width)?(cpoint.get().x + size): width - 1;
    int maxY = ((cpoint.get().y + size) < height)?(cpoint.get().y + size): height - 1;

	glm::vec3 coord;
    for(int y = minY; y < maxY + 1; y += step) {
        for(int x = minX; x < maxX + 1; x += step) {
 			coord = realSense->getSpacePointFromInfraLeftFrameCoord(glm::vec2(x, y));
            if(coord.z != 0) {
				ppoint += coord;
                counter++;
            }
        }
    }
    ppoint /= counter;
  
    return ppoint;
}


//--------------------------------------------------------------
void ofApp::update(){
	
	ofBackground(100, 100, 100);
    	
	// there is a new frame and we are connected
	if(realSense->update(ofxRealSenseTwo::PointCloud::INFRALEFT)) {

        if(bUpdateMeasurment){
            measurementCycleRaw();
        }
        if(bUpdateMeasurmentFine){
            measurementCycleFine();
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){

	ofSetColor(255, 255, 255);

	//Draw viewport previews
	realSense->drawDepthStream(viewGrid[0]);
	realSense->drawInfraLeftStream(viewGrid[1]);
	
	switch (iMainCamera) {
		case 0:
			realSense->drawDepthStream(viewMain);
			drawCalibrationPoints();
			break;
		case 1:
			realSense->drawInfraLeftStream(viewMain);
			drawCalibrationPoints();
			break;
		case 2:
			previewCam.begin(viewMain);
			mainGrid.drawPlane(5., 5, false);
			drawPreview();
			previewCam.end();
			break;
		default:
			break;
	}
	
	//Draw opengl viewport previews (ofImages dont like opengl calls before they are drawn
	if(iMainCamera != 2){ // make sure the camera is drawn only once (so the interaction with the mouse works)
		previewCam.begin(viewGrid[2]);
		mainGrid.drawPlane(5., 5, false);
		drawPreview();
		previewCam.end();
	}

	glDisable(GL_DEPTH_TEST);
	ofPushStyle();
	// Highlight background of selected camera
	ofSetColor(255, 0, 255, 255);
	ofNoFill();
	ofSetLineWidth(3);
	ofDrawRectangle(viewGrid[iMainCamera]);


	// draw instructions
	ofSetColor(255, 255, 255);
    
    if(bShowHelp) {
		ofDrawBitmapString(help, 20 ,VIEWPORT_HEIGHT + 20);
        if(bShowCalcData){
            ofDrawBitmapString(calcdata, 20 ,VIEWPORT_HEIGHT + 20);
        }
    }

    ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), ofGetWidth() - 200, 10);

    ofPopStyle();
}

void ofApp::drawPreview() {
	glPointSize(4);
	glEnable(GL_DEPTH_TEST);

	ofPushMatrix();

    //This moves the crossingpoint of the kinect center line and the plane to the center of the stage
    //ofTranslate(-planeCenterPoint.x, -planeCenterPoint.y, 0);
	ofMultMatrix(deviceTransform);
	if (bPreviewPointCloud) {
		realSense->draw();
	}

	ofFill();
	ofSetColor(255, 0, 0);
	sphere_X.draw();
	sphere_Y.draw();
	sphere_Z.draw();

	geometry.draw();
    
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

void ofApp::drawCalibrationPoints(){
    glDisable(GL_DEPTH_TEST);
    ofPushStyle();
    ofSetColor(255, 0, 0);
    ofNoFill();
    ofDrawBitmapString("o", calibPoint_O.get().x/REALSENSE_DEPTH_WIDTH*viewMain.width + VIEWGRID_WIDTH + 5, calibPoint_O.get().y -5);
    ofDrawBitmapString("x", calibPoint_X.get().x/REALSENSE_DEPTH_WIDTH*viewMain.width + VIEWGRID_WIDTH + 5, calibPoint_X.get().y -5);
    ofDrawBitmapString("y", calibPoint_Y.get().x/REALSENSE_DEPTH_WIDTH*viewMain.width + VIEWGRID_WIDTH + 5, calibPoint_Y.get().y -5);
    ofDrawCircle(calibPoint_X.get().x/REALSENSE_DEPTH_WIDTH*viewMain.width + VIEWGRID_WIDTH, calibPoint_X.get().y, 2);
    ofDrawCircle(calibPoint_Y.get().x/REALSENSE_DEPTH_WIDTH*viewMain.width + VIEWGRID_WIDTH, calibPoint_Y.get().y, 2);
    ofDrawCircle(calibPoint_O.get().x/REALSENSE_DEPTH_WIDTH*viewMain.width + VIEWGRID_WIDTH, calibPoint_O.get().y, 2);
    ofPopStyle();
    glEnable(GL_DEPTH_TEST);
}

//--------------------------------------------------------------
void ofApp::exit() {
    ofLog(OF_LOG_NOTICE) << "exiting application...";

	realSense->stop();
}

void ofApp::createHelp(){
	stringstream helpStream;
	helpStream << "press o, x, z and then mouse-click -> to change the calibration points\n";
	helpStream << "press k -> to update the transformation\n";
	helpStream << "press r -> view calculation results\n";
    helpStream << "press s -> to save current settings.\n";
	helpStream << "press l -> to load last saved settings\n";
	helpStream << "\n";
	helpStream << "press 1 - 3 -> to change the viewport\n";
	helpStream << "press p -> to show pointcloud\n";
    helpStream << "press h -> to show help \n";
	help = helpStream.str();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	bUpdateSetMesurmentPoint = -1;
	switch (key) {
		case ' ':
			break;
			
		case 'p':
			bPreviewPointCloud = !bPreviewPointCloud;
            break;
            
        case 'r':
            bShowCalcData = !bShowCalcData;
            break;
            
        case 'k':
            bUpdateMeasurment = true;
            break;
 
        case 's':
            setupCalib->saveToFile("settings.xml");
			post->saveToFile("postprocessing.xml");
			device->saveToFile(realSense->getSerialNumber(-1) + ".xml");
			guitransform->saveToFile("transformation.xml");
			networking->saveToFile("broadcast.xml");
			break;

        case 'l':
            setupCalib->loadFromFile("settings.xml");
			post->loadFromFile("postprocessing.xml");
			device->loadFromFile(realSense->getSerialNumber(-1) + ".xml");
			guitransform->loadFromFile("transformation.xml");
			networking->loadFromFile("broadcast.xml");
			break;
           
		case 'h':
			bShowHelp = !bShowHelp;
            if (bShowHelp) {
                createHelp();
            }
			break;
						
		case 'x':
			bUpdateSetMesurmentPoint = key;
			break;

		case 'y':
			bUpdateSetMesurmentPoint = key;
			break;

		case 'o':
			bUpdateSetMesurmentPoint = key;
			break;
            
		case '1':
            iMainCamera = 0;
			break;
			
		case '2':
            iMainCamera = 1;
			break;
			
		case '3':
            iMainCamera = 2;
			break;				            
	}

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){


}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
	stroke.push_back(ofPoint(x,y));
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    if(iMainCamera == 0 || iMainCamera == 1 && bUpdateSetMesurmentPoint != -1) {
        if(bUpdateSetMesurmentPoint == 'x'){
            int posX = (x - VIEWGRID_WIDTH) / viewMain.width * REALSENSE_DEPTH_WIDTH;
            int posY = y;
            if(0 <= posX && posX < REALSENSE_DEPTH_WIDTH &&
               0 <= posY && posY < REALSENSE_DEPTH_HEIGHT)
                calibPoint_X.set(glm::vec2(posX, posY));
        }else if(bUpdateSetMesurmentPoint == 'y'){
            int posX = (x - VIEWGRID_WIDTH) / viewMain.width * REALSENSE_DEPTH_WIDTH;
            int posY = y;
            if(0 <= posX && posX < REALSENSE_DEPTH_WIDTH &&
               0 <= posY && posY < REALSENSE_DEPTH_HEIGHT)
                calibPoint_Y.set(glm::vec2(posX, posY));
        }else if(bUpdateSetMesurmentPoint == 'o'){
            int posX = (x - VIEWGRID_WIDTH) / viewMain.width * REALSENSE_DEPTH_WIDTH;
            int posY = y;
            if(0 <= posX && posX < REALSENSE_DEPTH_WIDTH &&
               0 <= posY && posY < REALSENSE_DEPTH_HEIGHT)
                calibPoint_O.set(glm::vec2(posX, posY));
        }
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
	setupViewports();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}


