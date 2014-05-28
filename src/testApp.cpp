#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    ofSetVerticalSync(true);
    ofSetFrameRate(60);

    kinect.setRegistration(true);
    kinect.init();
    kinect.open();
    kinect2.init();
	kinect2.open();
    kinect.setCameraTiltAngle(0);
    kinect2.setCameraTiltAngle(0);

    lowTreshold = 300;
	highTreshold = 0;
	nBlobs = 1;
	minBlobSize = ofGetWindowHeight()*ofGetWindowWidth()/20;
	maxBlobSize = ofGetWindowHeight()*ofGetWindowWidth()/2;

    grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

    grayImage2.allocate(kinect2.width, kinect2.height);
	grayThreshNear2.allocate(kinect2.width, kinect2.height);
	grayThreshFar2.allocate(kinect2.width, kinect2.height);

    serial.enumerateDevices();
    serial.setup("COM5", 9600);
    servoDelay = 15;

    kinectOutput = 3500;
    kinectDistance = 0;
    nBlobs = 1;
    byteOutput = 0;
    maxDistance = 3500;

    border1 = 2000;
    border2 = 1000;
    kinectOutput = maxDistance;

    downSpeed = 5;

    setGui();

    kinectWindowPos.x = 0;
    kinectWindowPos.y = 0;
    kinect2WindowPos.x = 640;
    kinect2WindowPos.y = 0;




}

//--------------------------------------------------------------
void testApp::setGui(){

    gui = new ofxUISuperCanvas("KINECT & BORDERS");
	gui->loadSettings("settings.xml");
	gui->setTheme(OFX_UI_THEME_RUSTICORANGE);
	gui->addSpacer();

	gui->addTextArea("Info", "Info", 2);
	gui->addFPS();
	gui->addMinimalSlider("Distance", 0, maxDistance, &kinectDistance);
	gui->addMinimalSlider("Output", 0, maxDistance, &kinectOutput);
	gui->addMinimalSlider("ByteOutput", 0, 300, &byteOutput);

	gui->addSpacer();
	gui->addTextArea("Border settings", "Border settings", 2);
	gui->addRangeSlider("Borders", 0, maxDistance, &border2, &border1);
	gui->addSlider("Afnamesnelheid", 1, 9, &downSpeed);
	gui->addLabelToggle("Swap windows", false);

	//gui->addSlider("Border 1", 0, maxDistance, &border1);
	//gui->addSlider("Border 2", 0, maxDistance, &border2);

    gui->autoSizeToFitWidgets();
    ofAddListener(gui->newGUIEvent, this, &testApp::guiEvent);

}

//--------------------------------------------------------------
void testApp::update(){

    updateKinect();

    if(ofGetFrameNum() % 4 == 0){
        if(kinectOutput > border2){
            byteOutput = ofMap(kinectOutput, maxDistance, border2, 300, 0, true);
            serial.writeByte(byteOutput);
        }
        else if(kinectOutput <= border2 && kinectOutput > border1){
            byteOutput = ofMap(kinectOutput, border2, border1, 200, 100, true);
            serial.writeByte(byteOutput);
        }
        else if(kinectOutput <= border1 && kinectOutput > 0){
            byteOutput = ofMap(kinectOutput, border1, 0, 100, 0, true);
            serial.writeByte(byteOutput);
        }
    }

    if(kinectDistance < kinectOutput && kinectDistance != 0){
        kinectOutput = kinectDistance;
    }
    kinectOutput *= (1 + 0.001*downSpeed);
}

//--------------------------------------------------------------
void testApp::updateKinect(){

    kinect.update();
    kinect2.update();

    grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

    grayThreshNear = grayImage;
    grayThreshFar = grayImage;
    grayThreshNear.threshold(lowTreshold, true);
    grayThreshFar.threshold(highTreshold);
    cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
    grayImage.flagImageChanged();

    grayImage2.setFromPixels(kinect2.getDepthPixels(), kinect2.width, kinect2.height);

    grayThreshNear2 = grayImage2;
    grayThreshFar2 = grayImage2;
    grayThreshNear2.threshold(lowTreshold, true);
    grayThreshFar2.threshold(highTreshold);
    cvAnd(grayThreshNear2.getCvImage(), grayThreshFar2.getCvImage(), grayImage2.getCvImage(), NULL);
    grayImage2.flagImageChanged();

    contourFinder.findContours(grayImage, minBlobSize, maxBlobSize, nBlobs, true);
    contourFinder2.findContours(grayImage2, minBlobSize, maxBlobSize, nBlobs, true);

    if(contourFinder.nBlobs != 0){
        //cout << "Nailed it" << endl;
        for (int i=0; i < nBlobs; i++){
            ofVec3f tempPos = contourFinder.blobs[i].centroid;
            ofPoint tempKinectPos = kinect.getWorldCoordinateAt(tempPos.x, tempPos.y);
            if (tempKinectPos.z != 0){
                kinectDistance = tempKinectPos.z;
            }
        }
    }

        if(contourFinder2.nBlobs != 0){
        for (int i=0; i < nBlobs; i++){
            ofVec3f tempPos = contourFinder2.blobs[i].centroid;
            ofPoint tempKinectPos = kinect2.getWorldCoordinateAt(tempPos.x, tempPos.y);
            if (tempKinectPos.z != 0){
                kinect2Distance = tempKinectPos.z;
            }
        }
    }

    if (kinect2Distance < kinectDistance){
        kinectDistance = kinect2Distance;
    }
}

//--------------------------------------------------------------
void testApp::draw(){

    ofSetColor(255,255,255);
    kinect.draw(kinectWindowPos.x, kinectWindowPos.y ,640,480);
    kinect2.draw(kinect2WindowPos.x, kinect2WindowPos.y,640,480);

    ofSetColor(0,0,0);
    //grayImage.draw(0,0,ofGetWindowWidth(),ofGetWindowHeight());
    contourFinder.draw(kinectWindowPos.x, kinectWindowPos.y, 640, 480);
    contourFinder2.draw(kinect2WindowPos.x, kinect2WindowPos.y ,640, 480);

    ofSetColor(255,0,0);
    if(contourFinder.nBlobs != 0){
        ofPoint centroidPos = contourFinder.blobs[0].centroid;
        centroidPos += kinectWindowPos;
        ofCircle(centroidPos, 20);
    }

    if(contourFinder2.nBlobs != 0){
        ofPoint centroidPos = contourFinder2.blobs[0].centroid;
        centroidPos += kinect2WindowPos;
        ofCircle(centroidPos, 20);
    }
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

    if (key == OF_KEY_UP){
        servoDelay += 1;
    }

    if (key == OF_KEY_DOWN){
        servoDelay -= 1;
    }
    if (key == ' '){
        kinectDistance = 0;
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
    kinect2.setCameraTiltAngle(0);
    kinect2.close();

}

//--------------------------------------------------------------
void testApp::guiEvent(ofxUIEventArgs &e) {

    string name = e.widget->getName();

    if (name == "Swap windows"){
        ofxUILabelToggle *toggle = (ofxUILabelToggle *) e.widget;
        //swapWindows = ;
        if (toggle->getValue()){
            ofPoint kinectTempWindowPos = kinectWindowPos;
            kinectWindowPos = kinect2WindowPos;
            kinect2WindowPos = kinectTempWindowPos;
            swapWindows = true;
        }

        if (!toggle->getValue()){
            ofPoint kinectTempWindowPos = kinectWindowPos;
            kinectWindowPos = kinect2WindowPos;
            kinect2WindowPos = kinectTempWindowPos;
            swapWindows = false;
        }
    }
}
