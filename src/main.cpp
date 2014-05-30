#include "ofMain.h"
#include "testApp.h"

//========================================================================
int main( ){

    int tempWidth = 640;
    #ifdef USE_TWO_KINECTS
    tempWidth *= 2;
    #endif

	ofSetupOpenGL(tempWidth, 480, OF_WINDOW);			// <-------- setup the GL context

	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new testApp());

}
