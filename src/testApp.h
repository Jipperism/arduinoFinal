#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxXmlSettings.h"
#include "ofxUI.h"
#include "ofxOsc.h"
#include "ofxMidi.h"

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

		void updateKinect();
		void setGui();
		void guiEvent(ofxUIEventArgs &e);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

        ofxKinect kinect;
		ofSerial serial;

        ofxCvContourFinder contourFinder;
        ofxCvGrayscaleImage grayImage;
        ofxCvGrayscaleImage grayThreshNear;
        ofxCvGrayscaleImage grayThreshFar;
        int lowTreshold, highTreshold, nBlobs;
        float minBlobSize, maxBlobSize;

        int servoDelay;
        float byteOutput, maxDistance;
        float kinectDistance, kinectOutput;

        float border1, border2;

        ofxUISuperCanvas *gui;

};
