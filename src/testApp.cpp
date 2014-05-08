#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    ofSetVerticalSync(true);
    ofSetFrameRate(60);

    kinect.setRegistration(true);
    kinect.init();
    kinect.open();
    kinect.setCameraTiltAngle(0);

    lowTreshold = 230;
	highTreshold = 70;
	nBlobs = 1;
	minBlobSize = ofGetWindowHeight()*ofGetWindowWidth()/20;
	maxBlobSize = ofGetWindowHeight()*ofGetWindowWidth()/2;

    grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

    serial.enumerateDevices();
    //serial.setup("COM5", 9600);
    servoDelay = 15;

    kinectOutput = 0;
    kinectDistance = 0;
    nBlobs = 1;

}

//--------------------------------------------------------------
void testApp::update(){

    updateKinect();

    if(ofGetFrameNum() % 4 == 0){
        if(servoDelay < 10){
            servoDelay = 10;
        }
        else if (servoDelay > 20){
            servoDelay = 20;
        }
        ofClamp(servoDelay, 10, 20);
        //serial.writeByte(servoDelay);
        //cout << servoDelay << endl;
    }

    if(kinectOutput > kinectDistance){
        kinectOutput = kinectDistance;
        cout << kinectOutput << endl;
    }
}

//--------------------------------------------------------------
void testApp::updateKinect(){

    kinect.update();

    grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);


    grayThreshNear = grayImage;
    grayThreshFar = grayImage;
    grayThreshNear.threshold(lowTreshold, true);
    grayThreshFar.threshold(highTreshold);
    cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
    grayImage.flagImageChanged();

    contourFinder.findContours(grayImage, minBlobSize, maxBlobSize, nBlobs, true);

    if(contourFinder.nBlobs != 0){
        for (int i=0; i < nBlobs; i++){
            ofVec3f tempPos = contourFinder.blobs[i].centroid;
            ofPoint tempKinectPos = kinect.getWorldCoordinateAt(tempPos.x, tempPos.y);
            if (tempKinectPos.z != 0){

                if (tempKinectPos.z > kinectDistance){
                kinectDistance = tempKinectPos.z;
                cout << kinectDistance << endl;
                }
            }
        }
    }

}

//--------------------------------------------------------------
void testApp::draw(){
    grayImage.draw(0,0,ofGetWindowWidth(),ofGetWindowHeight());
    contourFinder.draw(0,0,ofGetWindowWidth(), ofGetWindowHeight());

}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

    if (key == OF_KEY_UP){
        servoDelay += 1;
    }

    if (key == OF_KEY_DOWN){
        servoDelay -= 1;
    }

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){

}

//--------------------------------------------------------------
void testApp::exit() {

    kinect.setCameraTiltAngle(0);
    kinect.close();

}

//--------------------------------------------------------------
