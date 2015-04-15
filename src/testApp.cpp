#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){

    // ofSetLogLevel(OF_LOG_VERBOSE);
    testByte = 0;

    setupKinect();

    ofSetVerticalSync(true);
    ofSetFrameRate(60);

    // Kinect interface default settings
    lowTreshold = 300;
	highTreshold = 0;
	nBlobs = 1;
	minBlobSize = ofGetWindowHeight()*ofGetWindowWidth()/20;
	maxBlobSize = ofGetWindowHeight()*ofGetWindowWidth()/2;

    kinectOutput = 3500;
    kinectDistance = 0;
    byteOutput = 0;
    maxDistance = 3500;

    border1 = 2000;
    border2 = 1000;
    kinectOutput = maxDistance;

    bDrawContours = true;

    // Serial default settings
    serial.enumerateDevices();
    serial.setup("COM3", 9600);
    downSpeed = 5;

    setupGui();
    setupMidi();

}

//--------------------------------------------------------------
void testApp::setupMidi() {

    midiOut.listPorts();
	midiOut.openPort(1);
	midiChannel = 1;

}

//--------------------------------------------------------------
void testApp::setupKinect(){

    // Initialise kinect connection and set default angle
    kinect.setRegistration(true);
    kinect.init();
    kinect.open();
    angle = 0;
    kinect.setCameraTiltAngle(0);

    // Allocate grayscale images
    grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
    kinectWindowPos.x = 0;
    kinectWindowPos.y = 0;

    #ifdef USE_TWO_KINECTS
    kinect2.setRegistration(true);
    kinect2.init();
	kinect2.open();
    kinect2.setCameraTiltAngle(0);
    grayImage2.allocate(kinect2.width, kinect2.height);
	grayThreshNear2.allocate(kinect2.width, kinect2.height);
	grayThreshFar2.allocate(kinect2.width, kinect2.height);
    kinect2WindowPos.x = 640;
    kinect2WindowPos.y = 0;
    #endif


}

//--------------------------------------------------------------
void testApp::setupGui(){

    gui = new ofxUISuperCanvas("KINECT & BORDERS");
	gui->loadSettings("settings.xml");
	gui->setTheme(OFX_UI_THEME_GRAYRED);
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
	gui->addLabelToggle("Draw contours", &bDrawContours);

	gui->addSpacer();
	gui->addIntSlider("Low treshold", 0, 300, &lowTreshold);
	gui->addIntSlider("High treshold", 0, 300, &highTreshold);
    gui->addSlider("MinBlobSize", 0, kinect.width*kinect.height, &minBlobSize);
    gui->addSlider("MaxBlobSize", 0, kinect.width*kinect.height, &maxBlobSize);
    gui->addIntSlider("Tilt Angle", 0, 15, &angle);

	#ifdef USE_TWO_KINECTS
	gui->addLabelToggle("Swap windows", false);
	#endif

    gui->autoSizeToFitWidgets();
    ofAddListener(gui->newGUIEvent, this, &testApp::guiEvent);

}

//--------------------------------------------------------------
unsigned char testApp::determine_sendByte(){

    // Map byteOutput to a value between 0 and 255, according to the
    // stage it is in
    if(kinectOutput > border2){
            byteOutput =
            ofMap(kinectOutput, maxDistance, border2, 0, 85, true);
    } else if (kinectOutput <= border2 && kinectOutput > border1){
            byteOutput =
            ofMap(kinectOutput, border2, border1, 85, 170, true);
    } else if(kinectOutput <= border1 && kinectOutput > 0){
            byteOutput =
            ofMap(kinectOutput, border1, 170, 255, true);
    }

    // Convert the float value of byteOutput to an unsigned char
    int tempByte = (int)byteOutput;
    unsigned char sendByte = (char)tempByte;
    return sendByte;

}

//--------------------------------------------------------------
void testApp::update(){


    // Run loop every 4 frames because of serial reading speed arduino
    if(ofGetFrameNum() % 4 == 0){
        updateKinect();
        unsigned char sendByte = determine_sendByte();
        serial.writeByte(sendByte);
        sendMidi(byteOutput);
        cout << (int)sendByte << endl;
    }

    kinectOutput *= (1 + 0.001*downSpeed);

}

//--------------------------------------------------------------
void testApp::sendMidi(int byteOutput) {
    // cout << ofMap(byteOutput, 0, 300, 0, 127) << endl;
    midiOut.sendControlChange(midiChannel, 1, ofMap(byteOutput, 0, 300, 127, 0));
    if(byteOutput >= 200) {
        midiOut.sendControlChange(midiChannel, 2, ofMap(byteOutput, 200, 300, 0, 127));
        if(byteOutput >= 100){
            midiOut.sendControlChange(midiChannel, 3, ofMap(byteOutput, 100, 300, 0, 127));
        }
    }

    midiOut.sendControlChange(midiChannel, 1, ofMap(byteOutput, 0, 300, 0, 127));
}

//--------------------------------------------------------------
void testApp::updateKinect(){

    kinect.update();

    // Substract grayscales for blobtracking
    grayImage.setFromPixels(kinect.getDepthPixels(),
                            kinect.width, kinect.height);
    grayThreshNear = grayImage;
    grayThreshFar = grayImage;
    grayThreshNear.threshold(lowTreshold, true);
    grayThreshFar.threshold(highTreshold);
    cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(),
          grayImage.getCvImage(), NULL);
    grayImage.flagImageChanged();

    contourFinder.findContours(grayImage, minBlobSize, maxBlobSize,
                               nBlobs, true);

    if(contourFinder.nBlobs != 0){
        for (int i=0; i < nBlobs; i++){
            ofVec3f tempPos = contourFinder.blobs[i].centroid;
            ofPoint tempKinectPos =
            kinect.getWorldCoordinateAt(tempPos.x, tempPos.y);
            if (tempKinectPos.z != 0){
                kinectDistance = tempKinectPos.z;
            }
        }
    }


    #ifdef USE_TWO_KINECTS
    kinect2.update();

    grayImage2.setFromPixels(kinect2.getDepthPixels(),
                             kinect2.width, kinect2.height);

    grayThreshNear2 = grayImage2;
    grayThreshFar2 = grayImage2;
    grayThreshNear2.threshold(lowTreshold, true);
    grayThreshFar2.threshold(highTreshold);
    cvAnd(grayThreshNear2.getCvImage(), grayThreshFar2.getCvImage(),
          grayImage2.getCvImage(), NULL);
    grayImage2.flagImageChanged();

    contourFinder2.findContours(grayImage2, minBlobSize,
                                maxBlobSize, nBlobs, true);

    if(contourFinder2.nBlobs != 0){
        for (int i=0; i < nBlobs; i++){
            ofVec3f tempPos = contourFinder2.blobs[i].centroid;
            ofPoint tempKinectPos =
            kinect2.getWorldCoordinateAt(tempPos.x, tempPos.y);
            if (tempKinectPos.z != 0){
                kinect2Distance = tempKinectPos.z;
            }
        }
    }

    if (kinect2Distance < kinectDistance){
        kinectDistance = kinect2Distance;
    }
    #endif

    // Only set the new kinectOutput to the new distance if it is smaller
    // then the previous value (the blob has come closer).
    if(kinectDistance < kinectOutput && kinectDistance != 0){
        kinectOutput = kinectDistance;
    }

}

//--------------------------------------------------------------
void testApp::draw(){

    // Draw visual output of the kinects
    ofSetColor(255,255,255);
    kinect.draw(kinectWindowPos.x, kinectWindowPos.y, 640, 480);

    // Draw centroids
    ofSetColor(255,0,0);
    if(contourFinder.nBlobs != 0){
        ofPoint centroidPos = contourFinder.blobs[0].centroid;
        centroidPos += kinectWindowPos;
        ofCircle(centroidPos, 20);
    }

    // Draw contours
    ofSetColor(0,0,0);
    if (bDrawContours){
        contourFinder.draw(kinectWindowPos.x, kinectWindowPos.y, 640, 480);
    }

    ofSetColor(255,255,255);
    grayImage.draw(0, 0, 640, 480);

    #ifdef USE_TWO_KINECTS
    ofSetColor(255,255,255);
    kinect2.draw(kinect2WindowPos.x, kinect2WindowPos.y, 640, 480);

    ofSetColor(255,0,0);
    if(contourFinder2.nBlobs != 0){
        ofPoint centroidPos = contourFinder2.blobs[0].centroid;
        centroidPos += kinect2WindowPos;
        ofCircle(centroidPos, 20);
    }

    ofSetColor(0,0,0);
    if (bDrawContours){
        contourFinder2.draw(kinect2WindowPos.x,
                            kinect2WindowPos.y, 640, 480);
    }
    #endif
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

    if (key == ' '){
        kinectDistance = 0;
    }
    if (key == OF_KEY_UP){
        testByte += 1;
    }
    if (key == OF_KEY_DOWN){
        testByte -= 1;
    }
    cout << (int)testByte << endl;
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
    midiOut.closePort();

    #ifdef USE_TWO_KINECTS
    kinect2.setCameraTiltAngle(0);
    kinect2.close();
    #endif

}

//--------------------------------------------------------------
void testApp::guiEvent(ofxUIEventArgs &e) {

    string name = e.widget->getName();

    #ifdef USE_TWO_KINECTS
    if (name == "Swap windows"){
        ofxUILabelToggle *toggle = (ofxUILabelToggle *) e.widget;
        if (toggle->getValue()){
            ofPoint kinectTempWindowPos = kinectWindowPos;
            kinectWindowPos = kinect2WindowPos;
            kinect2WindowPos = kinectTempWindowPos;
        }

        if (!toggle->getValue()){
            ofPoint kinectTempWindowPos = kinectWindowPos;
            kinectWindowPos = kinect2WindowPos;
            kinect2WindowPos = kinectTempWindowPos;
        }
    }
    #endif

    kinect.setCameraTiltAngle(angle);
}
