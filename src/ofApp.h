#pragma once

#include "ofMain.h"
#include "ofxGuiExtended.h"
#include "Planef.h"
#include "Linef.h"
#include "Grid.h"
#include "Frustum.h"
#include "CaptureMeshArray.h"

#include "ofxRealSenseTwo.h"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API 

#include <ofMatrix4x4.h>

#define N_CAMERAS 3

#define VIEWGRID_WIDTH  132
#define MENU_WIDTH      1000
#define VIEWPORT_HEIGHT 480

#define REALSENSE_DEPTH_WIDTH   848
#define REALSENSE_DEPTH_HEIGHT  480

#define REALSENSE_VIDEO_WIDTH   848
#define REALSENSE_VIDEO_HEIGHT  480

#define N_MEASURMENT_CYCLES 10

using namespace std;
using namespace ofxRealSenseTwo;

//helpfull links during development:
// https://github.com/openframeworks/openFrameworks/issues/3817

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
        
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);		

        vector <string> storeText;
		
        //ofxUDPManager udpConnection;

		ofTrueTypeFont  mono;
		ofTrueTypeFont  monosm;
		vector<ofPoint> stroke;
    

    bool bShowVisuals = false;
    
    //////////////////
    //OPENGL CAMERAS//
    //////////////////

    //viewports
    void setupViewports();
    
    ofRectangle viewMain;
    ofRectangle viewGrid[N_CAMERAS];

    //camera pointers
    ofCamera * cameras[N_CAMERAS];
    int iMainCamera;

    ofEasyCam cam;
    
    grid mainGrid;
    
    shared_ptr<ofBaseGLRenderer> opengl;
    shared_ptr<ofCairoRenderer> cairo;
    ofTexture render;

    ofEasyCam previewCam;
    
    /////////////
    //RealSense//
    /////////////
        
	RSDevicePtr realSense;

    ofMatrix4x4 unprojection;

    bool dispRaw;

    bool bPreviewPointCloud;
    
    ofVboMesh previewmesh;//, capturemesh;
    
    CaptureMeshArray capMesh;

    void drawPreview();

	void createGUIDeviceParams();

    ///////////////
    //CALCULATION//
    ///////////////
    void updateCalc();
    void updateMatrix();
    void measurementCycleRaw();
    void measurementCycleFine();

    void drawCalibrationPoints();
	glm::vec3 calcPlanePoint(ofParameter<ofVec2f> & cpoint, int _size, int _step);
    
    bool bUpdateCalc = false;
    bool bUpdateMeasurment = false;
	bool bUpdateMeasurmentFine = false;
	bool bUpdateImageMask = false;
	char  bUpdateSetMesurmentPoint = -1;
    
    int cycleCounter = 0;
   
    ofVec3f planePoint1Meas[N_MEASURMENT_CYCLES];
    ofVec3f planePoint2Meas[N_MEASURMENT_CYCLES];
    ofVec3f planePoint3Meas[N_MEASURMENT_CYCLES];
    
    ofVec3f planePoint_X;
    ofVec3f planePoint_Y;
    ofVec3f planePoint_O;

    ofVec3f planeCenterPoint;

    ofSpherePrimitive sphere_X;
    ofSpherePrimitive sphere_Y;
    ofSpherePrimitive sphere_Z;

    ofVboMesh geometry;
        
    ofMatrix4x4 deviceTransform;

    string calcdata;
    
    bool bShowCalcData;

    //////////////
    //PROPERTIES//
    //////////////
    ofxGui gui;
    
    ofxGuiPanel* setupCalib;
	ofxGuiPanel* device;
	ofxGuiPanel* post;
	ofxGuiPanel* guitransform;
	ofxGuiPanel* operating;
    ofxGuiPanel* networking;

    ofParameter<ofVec2f> calibPoint_X;
    ofParameter<ofVec2f> calibPoint_Y;
    ofParameter<ofVec2f> calibPoint_O;
 
	ofParameterGroup transformationGuiGroup;

    ofParameter<ofMatrix4x4> transformation;
    
    ofParameterGroup intrinsicGuiGroup;

    ofParameter<float> depthCorrectionBase;
    ofParameter<float> depthCorrectionDivisor;
    ofParameter<float> pixelSizeCorrector;

    ofParameter<int> blobGrain;

    ofParameter<int> serverId;

    //////////
    // HELP //
    //////////

    string help;

    bool bShowHelp = true;

    void createHelp();

};

