#pragma once

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include "OgreApplicationContext.h"
#include "OgreCameraMan.h"

/* Bullet3 Physics */
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

using namespace Ogre;
using namespace OgreBites;

#include "Player.h"

//Based on the Ogre3d examples
class Game : public ApplicationContext, public InputListener
{
private:
    //Ogre Scene Manager.
    SceneManager* scnMgr;

    //Collision configuration.
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

    //The default collision dispatcher
    
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
 
    btDiscreteDynamicsWorld* dynamicsWorld;

    btAlignedObjectArray<btCollisionShape*> collisionShapes;

    Player *player;

    bool wDown;

    bool aDown;

    bool sDown;

    bool dDown;

public:
//construct game
	Game();

//deconstruct game
	virtual ~Game();

 //carries out setup for everything
  void setup();

  
  //Sets up the camera
	void setupCamera();

	void setupBoxMesh();

  void setupBoxMesh2();

  void setupFallingBox();

  void setupPlayer();

  void setupFloor();

  //Creates, lights and adds them to the scene.  All based on the sample code, needs moving out into a level class.
	void setupLights();


	bool keyPressed(const KeyboardEvent& evt);

	bool keyReleased(const KeyboardEvent& evt);

	bool mouseMoved(const MouseMotionEvent& evt);

	bool frameStarted (const FrameEvent &evt);

	bool frameEnded(const FrameEvent &evt);

  //Sets up the bullet environment
  
  void bulletInit();
};
