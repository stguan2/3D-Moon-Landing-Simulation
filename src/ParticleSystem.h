#pragma once
//  Kevin M. Smith - CS 134 SJSU

#include "ofMain.h"
#include "Particle.h"


//  Pure Virtual Function Class - must be subclassed to create new forces.
//
class ParticleForce {
protected:
public:
	bool applyOnce = false;
	bool applied = false;
	virtual void updateForce(Particle *) = 0;
};

class ParticleSystem {
	ofColor color = NULL;
public:
	void add(const Particle &);
	void addForce(ParticleForce *);
	void remove(int);
	void update();
	void setLifespan(float);
	void reset();
	int removeNear(const ofVec3f & point, float dist);
	void draw();
	void setParticleColor(ofColor color);
	vector<Particle> particles;
	vector<ParticleForce *> forces;
};



// Some convenient built-in forces
//
class GravityForce: public ParticleForce {
	ofVec3f gravity;
public:
	GravityForce(const ofVec3f & gravity);
	GravityForce() {}
	void set(const ofVec3f & g) { gravity = g; }
	void updateForce(Particle *);
};

class TurbulenceForce : public ParticleForce {
	ofVec3f tmin, tmax;
public:
	TurbulenceForce(const ofVec3f & min, const ofVec3f &max);
	TurbulenceForce() {}
	void set(const ofVec3f & mn, const ofVec3f & mx) { tmin = mn; tmax = mx; }
	void updateForce(Particle *);
};

class ThrustForce : public ParticleForce {
	ofVec3f dir = ofVec3f(0, 0, 0);
public:
	ThrustForce(const ofVec3f & d);
	ThrustForce() {}
	void set(const ofVec3f &d) { dir = d; }
	void updateForce(Particle *);
};

//Impulse Force copied from Collision Detection Part 1 video
//
class ImpulseForce : public ParticleForce {
public:
	ImpulseForce() {
		applyOnce = true;
		applied = true;
		force = ofVec3f(0, 0, 0);
	}
	void apply(const ofVec3f f) {
		applied = false;
		force = f;
	}
	void updateForce(Particle *particle) {
		particle->forces += force;
	}

	ofVec3f force;	
};
