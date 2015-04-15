#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxXmlSettings.h"
#include "ofxUI.h"
#include "ofxOsc.h"
#include "ofxMidi.h"

//Uncomment de volgende regel als je met 2 kinects werkt
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
		void exit();

        void setupKinect();
		void updateKinect();
		void setupGui();
		void guiEvent(ofxUIEventArgs &e);
		void setupMidi();
        void sendMidi(int byteOutput);
        unsigned char determine_sendByte();
        unsigned char testByte;

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
        ofxCvContourFinder contourFinder;
        ofxCvGrayscaleImage grayImage, grayThreshNear, grayThreshFar;
        ofPoint kinectWindowPos;

		#ifdef USE_TWO_KINECTS
        ofxKinect kinect2;
        ofxCvContourFinder contourFinder2;
        ofxCvGrayscaleImage grayImage2, grayThreshNear2, grayThreshFar2;
        ofPoint kinect2WindowPos;
        #endif

        ofSoundPlayer track1, track2, track3;

        ofSerial serial;

        int lowTreshold, highTreshold, nBlobs;
        float minBlobSize, maxBlobSize;

        float byteOutput, maxDistance;
        float kinectDistance, kinectOutput, kinect2Distance;

        float border1, border2, downSpeed;

        ofxUISuperCanvas *gui;

        bool bDrawContours;

        int midiChannel;
        ofxMidiOut midiOut;

};
