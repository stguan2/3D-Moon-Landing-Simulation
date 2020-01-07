#pragma once

#include "ofMain.h"
#include  "ofxAssimpModelLoader.h"
#include "ParticleSystem.h"
#include "ParticleEmitter.h"
#include "box.h"
#include "ray.h"
#include "Octree.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void drawAxis(ofVec3f);
		void initLightingAndMaterials();
		void savePicture();
		void toggleWireframeMode();
		void togglePointsDisplay();
		void toggleSelectTerrain();
		void setCameraTarget();
		bool  doPointSelection();
		void drawBox(const Box &box);
		Box meshBounds(const ofMesh &);
		void collisionDetect();
		void landing();
		void playEngineSound();

		bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);

		ofEasyCam cam;
		ofCamera *theCam;
		ofCamera topCam, trackingCam, downCam, moveTrackCamera, frontCam;
		ofxAssimpModelLoader mars, rover, lander;
		ofSoundPlayer engineSound;
		ofLight light;
		ofLight keyLight, landerLight;
		Box boundingBox;
		Box landerBox;
		vector<Box> level1, level2, level3;
		Octree octree;
		ofVec3f contactPt1, contactPt2, contactPt3, contactPt4;

		float score;
		float highScore;
		//Emitters
		//

		ParticleEmitter playerEmitter;
		ParticleEmitter ringEmitter;

		//Forces
		//

		TurbulenceForce* turbForce;
		GravityForce* gravityForce;
		ThrustForce* thrustForce;
		ImpulseForce* impulseForce;
		ThrustForce* engineThrustForce;
	
		bool toggleLeafOnlyDisplay;
		bool bAltKeyDown;
		bool bCtrlKeyDown;
		bool bWireframe;
		bool bDisplayPoints;
		bool bPointSelected;

		bool bRoverLoaded;
		bool bTerrainSelected;
		bool bLanderLoaded;
		bool bCollision;
		bool gameOver;
		bool startEngine;
	
		ofVec3f selectedPoint;
		ofVec3f intersectPoint;

		const float selectionRange = 4.0;
};
