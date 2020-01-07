
//--------------------------------------------------------------
//
//  Kevin M. Smith
//
//  Mars HiRise Project - startup scene
// 
//  This is an openFrameworks 3D scene that includes an EasyCam
//  and example 3D geometry which I have reconstructed from Mars
//  HiRis photographs taken the Mars Reconnaisance Orbiter
//
//  You will use this source file (and include file) as a starting point
//  to implement assignment 5  (Parts I and II)
//
//  Please do not modify any of the keymappings.  I would like 
//  the input interface to be the same for each student's 
//  work.  Please also add your name/date below.

//  Please document/comment all of your work !
//  Have Fun !!
//
//  Student Name:   Steven Guan
//  Date: 5/16/19


#include "ofApp.h"
#include "Util.h"



//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup(){

	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bRoverLoaded = false;
	bLanderLoaded = false;
	bTerrainSelected = true;
	bCollision = false;
	startEngine = false;
	gameOver = false;
//	ofSetWindowShape(1024, 768);
	cam.setDistance(10);
	cam.setNearClip(.1);
	cam.setFov(65.5);   // approx equivalent to 28mm in 35mm format
	ofSetVerticalSync(true);
	cam.disableMouseInput();
	ofEnableSmoothing();
	ofEnableDepthTest();

	if (lander.loadModel("geo/lander.obj")) {
		lander.setScaleNormalization(false);
		lander.setRotation(0, 180, 0, 0, 1);
		lander.setPosition(-10, 10, -10);

		bLanderLoaded = true;
	}
	else {
		cout << "Error: Can't load model" << "geo/lander.obj" << endl;
		ofExit(0);
	}

	engineSound.loadSound("sounds/wind.mp3");

	ofVec3f min = lander.getSceneMin() + lander.getPosition();
	ofVec3f max = lander.getSceneMax() + lander.getPosition();
	landerBox = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));

	// scoring
	score = 0;
	highScore = 0;

	// setup rudimentary lighting 
	//
	initLightingAndMaterials();

	mars.loadModel("geo/moon-houdini.obj");
	mars.setScaleNormalization(false);
	mars.setRotation(0, 180, 0, 0, 1);

	float createTime = ofGetElapsedTimeMillis();
	octree.create(mars.getMesh(0), 8);
	cout << "Time to create octree in milliseconds: " << ofGetElapsedTimeMillis() - createTime << endl;

	boundingBox = octree.meshBounds(mars.getMesh(0));

	// Camera Setup

	topCam.setPosition(glm::vec3(lander.getPosition().x, lander.getPosition().y + 75, lander.getPosition().z));
	topCam.setNearClip(.1);
	topCam.setFov(40);
	topCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));

	trackingCam.setPosition(glm::vec3(0, 60, 150));
	trackingCam.setNearClip(.1);
	trackingCam.setFov(50);
	trackingCam.lookAt(lander.getPosition());

	downCam.setPosition(lander.getPosition());
	downCam.setNearClip(.1);
	downCam.setFov(65);
	downCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));

	//looking at z
	//note that all other cameras look at the spacecraft's front
	//therefore looking through the front makes the left and right keys inverted
	//easy fix by changing these two things here and in update 
	//(but that will change camera to look through the back and not the front):
	//lander.getPosition().z + landerBox.width() / 2 -> lander.getPosition().z - landerBox.width() / 2
	//front + ofVec3f(0, 0, 50) -> front - ofVec3f(0, 0, 50)
	//
	ofVec3f front = ofVec3f(lander.getPosition().x, lander.getPosition().y + landerBox.height() / 2, lander.getPosition().z + landerBox.width() / 2);
	frontCam.setPosition(front);
	frontCam.setNearClip(.1);
	frontCam.setFov(65);
	frontCam.lookAt(front + ofVec3f(0, 0, 50));

	moveTrackCamera.setPosition(glm::vec3(20, 20, 30) + lander.getPosition());
	moveTrackCamera.setNearClip(.1);
	moveTrackCamera.setFov(65);
	moveTrackCamera.lookAt(lander.getPosition());

	theCam = &cam;


	//Particle emitter and forces setup
	//

	float gravity = 1;
	turbForce = new TurbulenceForce(ofVec3f(-.15, -.25, -.15), ofVec3f(.15, .25, .15));
	gravityForce = new GravityForce(ofVec3f(0, -gravity, 0));
	thrustForce = new ThrustForce();
	impulseForce = new ImpulseForce();
	engineThrustForce = new ThrustForce(ofVec3f(0,-20,0));

	playerEmitter.sys->addForce(turbForce);
	playerEmitter.sys->addForce(impulseForce);
	playerEmitter.sys->addForce(gravityForce);
	playerEmitter.sys->addForce(thrustForce);
	playerEmitter.setPosition(lander.getPosition());
	playerEmitter.setLifespan(-1);
	playerEmitter.setVelocity(ofVec3f(0, 0, 0));
	playerEmitter.setGroupSize(1);
	playerEmitter.setEmitterType(RadialEmitter);
	playerEmitter.particleRadius = 2;

	//spawn one particle to control lander
	playerEmitter.spawn(ofGetElapsedTimef());

	ringEmitter.sys->addForce(engineThrustForce);
	ringEmitter.setEmitterType(DiscEmitter);
	ringEmitter.setPosition(lander.getPosition());
	ringEmitter.setLifespan(2);
	ringEmitter.setGroupSize(10);
	ringEmitter.setParticleRadius(.02);
	ringEmitter.radius = .5;
	ringEmitter.setVelocity(ofVec3f(0, 0, 0));
	ringEmitter.start();
}

//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {
	ofVec3f min = lander.getSceneMin() + lander.getPosition();
	ofVec3f max = lander.getSceneMax() + lander.getPosition();
	landerBox = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
	landerLight.setPosition(lander.getPosition());

	if (startEngine) {
		ringEmitter.setRate(10);
		playEngineSound();
	}
	else {
		ringEmitter.setRate(0);
		engineSound.stop();
	}

	playerEmitter.sys->update();
	ringEmitter.update();
	ringEmitter.setPosition(lander.getPosition() + ofVec3f(0, .5, 0));

	// Controlling lander with the created particle
	if (playerEmitter.sys->particles.size() > 0) {
		lander.setPosition(playerEmitter.sys->particles[0].position.x, playerEmitter.sys->particles[0].position.y, playerEmitter.sys->particles[0].position.z);
	}

	collisionDetect();
	landing();

	//Camera Update

	topCam.setPosition(glm::vec3(lander.getPosition().x, lander.getPosition().y + 75, lander.getPosition().z));
	topCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));

	trackingCam.lookAt(lander.getPosition());

	moveTrackCamera.setPosition(glm::vec3(20, 20, 30) + lander.getPosition());
	moveTrackCamera.lookAt(lander.getPosition());

	ofVec3f front = ofVec3f(lander.getPosition().x, lander.getPosition().y + landerBox.height() / 2, lander.getPosition().z + landerBox.width() / 2);
	frontCam.setPosition(front);
	frontCam.lookAt(front + ofVec3f(0, 0, 50));

	downCam.setPosition(lander.getPosition());
	downCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));

	// Remove turbulence at low altitutdes
	if (lander.getPosition().y < 1.5) {
		turbForce->set(ofVec3f(0, 0, 0), ofVec3f(0, 0, 0));
	}
	else {
		turbForce->set(ofVec3f(-.15, -.25, -.15), ofVec3f(.15, .25, .15));
	}
}

//--------------------------------------------------------------
void ofApp::draw() {

	//	ofBackgroundGradient(ofColor(20), ofColor(0));   // pick your own backgroujnd
	ofBackground(ofColor::black);
	//	cout << ofGetFrameRate() << endl;

	theCam->begin();

	ringEmitter.draw();

	ofPushMatrix();

	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		//mars.drawWireframe();
		//if (bLanderLoaded) {
		//	lander.drawWireframe();
		//}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
		mars.drawFaces();

		if (bLanderLoaded) {
			lander.drawFaces();
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}


	if (bDisplayPoints) {                // display points as an option    
		glPointSize(3);
		ofSetColor(ofColor::green);
		mars.drawVertices();
	}

	// highlight selected point (draw sphere around selected point)
	//
	if (bPointSelected) {
		ofSetColor(ofColor::blue);
		ofDrawSphere(selectedPoint, .1);
	}

	ofSetColor(ofColor::red);
	ofDrawSphere(0, 0, 0, .2);

	ofPopMatrix();
	
	theCam->end();

	//Framerate
	string str;
	str += "Frame Rate: " + std::to_string(ofGetFrameRate());
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str, ofGetWindowWidth() - 205, 15);

	//Altitude
	string str2;
	str2 += "Altitude (AGL): " + std::to_string(lander.getPosition().y);
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str2, ofGetWindowWidth() - 205, 30);

	//High score for current session
	string str4;
	str4 += "High Score: " + std::to_string(highScore);
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str4, ofGetWindowWidth() - 205, 45);

	//Current score
	string str5;
	str5 += "Current Score: " + std::to_string(score);
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str5, ofGetWindowWidth() - 205, 60);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box ofApp::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}

//draw a box from a "Box" class
//for testing purposes
//
void ofApp::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));
	

	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}


void ofApp::keyPressed(int key) {

	switch (key) {
	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'D':
	case 'd':
		toggleLeafOnlyDisplay = !toggleLeafOnlyDisplay;
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		break;
	case 'r':
		cam.reset();
		if (gameOver) {
			playerEmitter.sys->particles[0].position = (ofVec3f(-10, 10, -10));
			gameOver = false;
			bCollision = false;
			playerEmitter.sys->particles[0].velocity = ofVec3f(0, 0, 0);
			playerEmitter.sys->reset();
		}
		break;
	case 's':
		savePicture();
		break;
	case 't':
		setCameraTarget();
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
		break;
	case 'w':
		toggleWireframeMode();
		break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;

	//Camera F-Keys
	case OF_KEY_F1:
		theCam = &moveTrackCamera;
		break;
	case OF_KEY_F2:
		theCam = &topCam;
		break;
	case OF_KEY_F3:
		theCam = &trackingCam;
		break;
	case OF_KEY_F4:
		theCam = &downCam;
		break;
	case OF_KEY_F5:
		theCam = &frontCam;
		break;
	case OF_KEY_F6:
		theCam = &cam;
	case OF_KEY_UP:
		if (bCtrlKeyDown) {
			thrustForce->set(ofVec3f(0, 0, -5));
		}
		else {
			thrustForce->set(ofVec3f(0, 5, 0));
		}
		startEngine = true;
		break;
	case OF_KEY_DOWN:
		if (bCtrlKeyDown) {
			thrustForce->set(ofVec3f(0, 0, 5));
		}
		else {
			thrustForce->set(ofVec3f(0, -5, 0));
		}
		startEngine = true;
		break;
	case OF_KEY_LEFT:
		thrustForce->set(ofVec3f(-5, 0, 0));
		startEngine = true;
		break;
	case OF_KEY_RIGHT:
		thrustForce->set(ofVec3f(5, 0, 0));
		startEngine = true;
		break;
	default:
		break;
	}
}

// Checks when lander touches the ground
// Uses four different points on lander
void ofApp::collisionDetect() {
	float restitution = 0.8;
	Vector3 c = landerBox.center();

	// Create contact points for each leg
	contactPt1 = ofVec3f(c.x() + landerBox.width() / 2, c.y() - landerBox.height() / 2, c.z());
	contactPt2 = ofVec3f(c.x() - landerBox.width() / 2, c.y() - landerBox.height() / 2, c.z());
	contactPt3 = ofVec3f(c.x(), c.y() - landerBox.height() / 2, c.z() + landerBox.width() / 2);
	contactPt4 = ofVec3f(c.x(), c.y() - landerBox.height() / 2, c.z() - landerBox.width() / 2);
	// + landerBox.position unnecessary since landerBox gets updated through update function

	ofVec3f vel = playerEmitter.sys->particles[0].velocity;

	//Only check when lander going down
	if (vel.y > 0) return;

	TreeNode node;
	//Check contactPt1 
	octree.ptIntersect(contactPt1, octree.root, node);
	if (node.points.size()>0) {
		bCollision = true;

		Vector3 vec3 = node.box.center();

		//impulse force
		//
		ofVec3f norm = ofVec3f(0, 1, 0);
		ofVec3f f = (restitution + 1.0) * ((-vel.dot(norm)*norm));
		impulseForce->apply(ofGetFrameRate() * f);
	}

	//Check contactPt2
	octree.ptIntersect(contactPt2, octree.root, node);
	if (node.points.size() > 0) {
		bCollision = true;

		Vector3 vec3 = node.box.center();

		//impulse force
		//
		ofVec3f norm = ofVec3f(0, 1, 0);
		ofVec3f f = (restitution + 1.0) * ((-vel.dot(norm)*norm));
		impulseForce->apply(ofGetFrameRate() * f);
	}

	//Check contactPt3
	octree.ptIntersect(contactPt3, octree.root, node);
	if (node.points.size() > 0) {
		bCollision = true;

		Vector3 vec3 = node.box.center();

		//impulse force
		//
		ofVec3f norm = ofVec3f(0, 1, 0);
		ofVec3f f = (restitution + 1.0) * ((-vel.dot(norm)*norm));
		impulseForce->apply(ofGetFrameRate() * f);
	}

	//Check contactPt4
	octree.ptIntersect(contactPt4, octree.root, node);
	if (node.points.size() > 0) {
		bCollision = true;

		Vector3 vec3 = node.box.center();

		//impulse force
		//
		ofVec3f norm = ofVec3f(0, 1, 0);
		ofVec3f f = (restitution + 1.0) * ((-vel.dot(norm)*norm));
		impulseForce->apply(ofGetFrameRate() * f);
	}
}

// Once collision is detected, determines a point value depending on how far/close you landed on the specified location (center)
// Also changes gameOver from false to true forcing user to reset game to get another score (with the r key)
// Also updates the high score for the game session
void ofApp::landing()
{
	ofVec3f vel = ofVec3f(0, 0, 0);
	if (playerEmitter.sys->particles.size() > 0) {
		vel = playerEmitter.sys->particles[0].velocity;
	}
	if (!gameOver) {
		if (bCollision) {
			cout << playerEmitter.sys->particles[0].velocity << endl;
			//update score
			//if lander lands too fast: score = 0
			//else score = 100 - (x + y + z position) or 0 (whichever is greater)
			if (playerEmitter.sys->particles[0].velocity.y > -5) {
				float x = abs(lander.getPosition().x) + abs(lander.getPosition().y + abs(lander.getPosition().z));
				if (100 - x > 0) {
					score = 100 - x;
				}
				else {
					score = 0;
				}
				if (score > highScore) {
					highScore = score;
				}
			}
			//update high score if score > high score
			else {
				score = 0;
			}

			bCollision = false;
			gameOver = true;
		}
	}
}

void ofApp::playEngineSound()
{
	if (engineSound.isPlaying()) return;
	else engineSound.play();
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}

void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {

	switch (key) {
	
	case OF_KEY_ALT:
		cam.disableMouseInput();
		bAltKeyDown = false;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = false;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_UP:
	case OF_KEY_DOWN:
	case OF_KEY_LEFT:
	case OF_KEY_RIGHT:
		thrustForce->set(ofVec3f(0, 0, 0));
		startEngine = false;
		//ringEmitter boolean audio = false
		break;
	default:
		break;

	}
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
	glm::vec3 sw = theCam->screenToWorld(glm::vec3(x, y, 0));	//rayPoint
	glm::vec3 d = sw - theCam->getPosition();					//rayDir
	glm::vec3 dn = glm::normalize(d);							//rayDir.normalize()
	glm::vec3 origin = theCam->getPosition();				
	Ray ray = 
		Ray(Vector3(origin.x, origin.y, origin.z), Vector3(dn.x, dn.y, dn.z));
//	bool hit = boundingBox.intersect(ray, 0, 10000);			//Check intersection
	TreeNode rtn;
	float pointSelectionTime = ofGetElapsedTimeMillis();
	octree.intersect(ray, octree.root, rtn);
	if (rtn.points.size() > 0) {
		Vector3 vec3 = rtn.box.center();
		selectedPoint = ofVec3f(vec3.x(), vec3.y(), vec3.z());
		//bPointSelected = true;
		//cout << "Time to select point in milliseconds: " << ofGetElapsedTimeMillis() - pointSelectionTime << endl;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {


}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}


//
//  ScreenSpace Selection Method: 
//  This is not the octree method, but will give you an idea of comparison
//  of speed between octree and screenspace.
//
//  Select Target Point on Terrain by comparing distance of mouse to 
//  vertice points projected onto screenspace.
//  if a point is selected, return true, else return false;
//
bool ofApp::doPointSelection() {

	ofMesh mesh = mars.getMesh(0);
	int n = mesh.getNumVertices();
	float nearestDistance = 0;
	int nearestIndex = 0;

	bPointSelected = false;

	ofVec2f mouse(mouseX, mouseY);
	vector<ofVec3f> selection;

	// We check through the mesh vertices to see which ones
	// are "close" to the mouse point in screen space.  If we find 
	// points that are close, we store them in a vector (dynamic array)
	//
	for (int i = 0; i < n; i++) {
		ofVec3f vert = mesh.getVertex(i);
		ofVec3f posScreen = cam.worldToScreen(vert);
		float distance = posScreen.distance(mouse);
		if (distance < selectionRange) {
			selection.push_back(vert);
			bPointSelected = true;
		}
	}

	//  if we found selected points, we need to determine which
	//  one is closest to the eye (camera). That one is our selected target.
	//
	if (bPointSelected) {
		float distance = 0;
		for (int i = 0; i < selection.size(); i++) {
			ofVec3f point =  cam.worldToCamera(selection[i]);

			// In camera space, the camera is at (0,0,0), so distance from 
			// the camera is simply the length of the point vector
			//
			float curDist = point.length(); 

			if (i == 0 || curDist < distance) {
				distance = curDist;
				selectedPoint = selection[i];
			}
		}
	}
	return bPointSelected;
}

// Set the camera to use the selected point as it's new target
//  
void ofApp::setCameraTarget() {

}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL
//
void ofApp::initLightingAndMaterials() {
	keyLight.setup();
	keyLight.enable();
	keyLight.setAreaLight(10, 10);
	keyLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	keyLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	keyLight.setSpecularColor(ofFloatColor(1, 1, 1));

	keyLight.rotate(45, ofVec3f(0, 1, 0));
	keyLight.rotate(-45, ofVec3f(1, 0, 0));
	keyLight.setPosition(30, 30, 30);

	landerLight.setup();
	landerLight.enable();
	landerLight.setSpotlight();
	landerLight.setScale(2);
	landerLight.setSpotlightCutOff(10);
	landerLight.setAttenuation(5, .001, .001);
	landerLight.setAmbientColor(ofFloatColor(0.1, 0.1, 0.1));
	landerLight.setDiffuseColor(ofFloatColor(1, 1, 1));
	landerLight.setSpecularColor(ofFloatColor(1, 1, 1));
	landerLight.rotate(-90, ofVec3f(1, 0, 0));
	landerLight.setPosition(lander.getPosition());


	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
} 

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {

	ofVec3f point;
	mouseIntersectPlane(ofVec3f(0, 0, 0), cam.getZAxis(), point);

	if (rover.loadModel(dragInfo.files[0])) {
		rover.setScaleNormalization(false);
		rover.setScale(.005, .005, .005);
		rover.setPosition(point.x, point.y, point.z);
		bRoverLoaded = true;
	}
	else cout << "Error: Can't load model" << dragInfo.files[0] << endl;
}

bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
	glm::vec3 mouse(mouseX, mouseY, 0);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}