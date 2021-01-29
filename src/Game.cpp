/*-------------------------------------------------------------------------
Significant portions of this project are based on the Ogre Tutorials
- https://ogrecave.github.io/ogre/api/1.10/tutorials.html
Copyright (c) 2000-2013 Torus Knot Software Ltd

Manual generation of meshes from here:
- http://wiki.ogre3d.org/Generating+A+Mesh

*/

#include <exception>
#include <iostream>

#include "Game.h"


Game::Game() : ApplicationContext("OgreTutorialApp")
{
	dynamicsWorld = NULL;

	wDown = false;
	aDown = false;
	sDown = false;
	dDown = false;
}


Game::~Game()
{
	//cleanup in the reverse order of creation/initialization

	///-----cleanup_start----
	//remove the rigidbodies from the dynamics world and delete them

	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);

		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}

		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
}


void Game::setup()
{
	// do not forget to call the base first
	ApplicationContext::setup();

	addInputListener(this);

	// get a pointer to the already created root
	Root* root = getRoot();
	scnMgr = root->createSceneManager();

	// register our scene with the RTSS
	RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
	shadergen->addSceneManager(scnMgr);

	bulletInit();

	setupCamera();

	setupFloor();

	setupLights();

	setupBoxMesh();

	setupBoxMesh2();

	setupFallingBox();

	setupPlayer();
}



void Game::setupCamera()
{
	//create cam
	Camera* cam = scnMgr->createCamera("myCam");

	//setup camera
	cam->setNearClipDistance(5);

	//position to scene
	SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
	camNode->setPosition(0, 400, 1000);
	camNode->lookAt(Vector3(0, 0, 0), Node::TransformSpace::TS_WORLD);
	camNode->attachObject(cam);

	//viewport for the camera
	Viewport* vp = getRenderWindow()->addViewport(cam);
	vp->setBackgroundColour(ColourValue(1, 18, 174));

	//link camera and view port
	cam->setAspectRatio(Real(vp->getActualWidth()) / Real(vp->getActualHeight()));
}

void Game::bulletInit()
{
	//collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	collisionConfiguration = new btDefaultCollisionConfiguration();

	//use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	//btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	overlappingPairCache = new btDbvtBroadphase();

	//the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -20, 0));
}
//player setup
void Game::setupPlayer()
{
	SceneNode* sceneRoot = scnMgr->getRootSceneNode();
	float mass = 1.0f;

	//axis
	Vector3 axis(1.0, 1.0, 0.0);
	axis.normalise();

	//angle
	Radian rads(Degree(0.0));

	player = new Player();
	player->createMesh(scnMgr);
	player->attachToNode(sceneRoot);

	player->setRotation(axis, rads);
	player->setPosition(0.0f, 0.0f, 0.0f);
	//collision
	player->createRigidBody(mass);
	player->addToCollisionShapes(collisionShapes);
	player->addToDynamicsWorld(dynamicsWorld);
}
//wall
void Game::setupBoxMesh()
{
	Entity* box = scnMgr->createEntity("cube.mesh");
	box->setCastShadows(true);

	SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
	thisSceneNode->attachObject(box);



	// Axis
	Vector3 axis(1.0, 1.0, 0.0);
	axis.normalise();

	//angle
	Radian rads(Degree(0.0));

	//quat from axis angle
	Quaternion quat(rads, axis);

	// thisSceneNode->setOrientation(quat);
	thisSceneNode->setScale(1.0, 7.5, 4.0);

	//get bounding box
	thisSceneNode->_updateBounds();
	const AxisAlignedBox& b = thisSceneNode->_getWorldAABB();

	//scenenode rotation
	thisSceneNode->setOrientation(quat);

	thisSceneNode->setPosition(400, 0, 0);

	Vector3 meshBoundingBox(b.getSize());

	if (meshBoundingBox == Vector3::ZERO)
	{
		std::cout << "bounding volume size is zero." << std::endl;
	}

	//create a dynamic rigidbody

	btCollisionShape* colShape = new btBoxShape(btVector3(meshBoundingBox.x / 2.0f, meshBoundingBox.y / 2.0f, meshBoundingBox.z / 2.0f));
	std::cout << "Mesh box col shape [" << (float)meshBoundingBox.x << " " << meshBoundingBox.y << " " << meshBoundingBox.z << "]" << std::endl;
	collisionShapes.push_back(colShape);

	//create dynamic objects
	btTransform startTransform;
	startTransform.setIdentity();

	Vector3 pos = thisSceneNode->_getDerivedPosition();
	startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));

	Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
	startTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

	btScalar mass(0.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
	{
		colShape->calculateLocalInertia(mass, localInertia);
	}
	//debug
	std::cout << "Local inertia [" << (float)localInertia.x() << " " << localInertia.y() << " " << localInertia.z() << "]" << std::endl;

	//motionstate
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	//ogre link
	body->setUserPointer((void*)thisSceneNode);

	dynamicsWorld->addRigidBody(body);
}
//wall2
void Game::setupBoxMesh2()//all code is same as previous wall/boxmesh
{
	Entity* box = scnMgr->createEntity("cube.mesh");
	box->setCastShadows(true);

	SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
	thisSceneNode->attachObject(box);

	//axis
	Vector3 axis(1.0, 1.0, 0.0);
	axis.normalise();

	//angle
	Radian rads(Degree(0.0));

	Quaternion quat(rads, axis);

	thisSceneNode->setScale(1.0, 7.5, 4.0);

	//get bounding box here.
	thisSceneNode->_updateBounds();
	const AxisAlignedBox& b = thisSceneNode->_getWorldAABB();
   
	thisSceneNode->setOrientation(quat);

	thisSceneNode->setPosition(-400, 0, 0);

	Vector3 meshBoundingBox(b.getSize());

	if (meshBoundingBox == Vector3::ZERO)
	{
		std::cout << "bounding voluem size is zero." << std::endl;
	}

	//create a dynamic rigidbody

	btCollisionShape* colShape = new btBoxShape(btVector3(meshBoundingBox.x / 2.0f, meshBoundingBox.y / 2.0f, meshBoundingBox.z / 2.0f));
	std::cout << "Mesh box col shape [" << (float)meshBoundingBox.x << " " << meshBoundingBox.y << " " << meshBoundingBox.z << "]" << std::endl;
	collisionShapes.push_back(colShape);

	// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
	startTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

	Vector3 pos = thisSceneNode->_getDerivedPosition();
	startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));



	btScalar mass(0.f);

	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
	{
		colShape->calculateLocalInertia(mass, localInertia);
	}

	std::cout << "Local inertia [" << (float)localInertia.x() << " " << localInertia.y() << " " << localInertia.z() << "]" << std::endl;

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	body->setUserPointer((void*)thisSceneNode);

	dynamicsWorld->addRigidBody(body);
}

//falling box (same code as previous box meshes however float values are changed to make the object act different to the walls)
void Game::setupFallingBox() {
	Entity* box = scnMgr->createEntity("cube.mesh");
	box->setCastShadows(true);

	SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
	thisSceneNode->attachObject(box);

	Vector3 axis(1.0, 1.0, 0.0);
	axis.normalise();

	Radian rads(Degree(90.0));

	Quaternion quat(rads, axis);

	thisSceneNode->setScale(1.0, 1.0, 1.0);

	//get bounding box here.
	thisSceneNode->_updateBounds();
	const AxisAlignedBox& b = thisSceneNode->_getWorldAABB();
  
	thisSceneNode->setOrientation(quat);

	thisSceneNode->setPosition(-150, 300, 0);

	Vector3 meshBoundingBox(b.getSize());

	if (meshBoundingBox == Vector3::ZERO)
	{
		std::cout << "bounding voluem size is zero." << std::endl;
	}

	//create a dynamic rigidbody

	btCollisionShape* colShape = new btBoxShape(btVector3(meshBoundingBox.x / 2.0f, meshBoundingBox.y / 2.0f, meshBoundingBox.z / 2.0f));
	std::cout << "Mesh box col shape [" << (float)meshBoundingBox.x << " " << meshBoundingBox.y << " " << meshBoundingBox.z << "]" << std::endl;
	collisionShapes.push_back(colShape);

	// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	Vector3 pos = thisSceneNode->_getDerivedPosition();
	startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));

	Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
	startTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

	btScalar mass(1.f);

	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
	{
		colShape->calculateLocalInertia(mass, localInertia);
	}

	std::cout << "Local inertia [" << (float)localInertia.x() << " " << localInertia.y() << " " << localInertia.z() << "]" << std::endl;

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	body->setUserPointer((void*)thisSceneNode);

	dynamicsWorld->addRigidBody(body);
}

void Game::setupFloor()
{
	//create plane
	Plane plane(Vector3::UNIT_Y, 0);

	//define plane mesh
	MeshManager::getSingleton().createPlane(
		"ground", RGN_DEFAULT,
		plane,
		4000, 4000, 20, 20,
		true,
		1, 5, 5,
		Vector3::UNIT_Z);

	//create entity for ground
	Entity* groundEntity = scnMgr->createEntity("ground");

	//setup ground entity
	//shadows off
	groundEntity->setCastShadows(false);

	groundEntity->setMaterialName("Examples/Rockwall");

	//create a scene node to add the mesh
	SceneNode* thisSceneNode = scnMgr->getRootSceneNode()->createChildSceneNode();
	thisSceneNode->attachObject(groundEntity);

	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(1500.), btScalar(50.), btScalar(1500.)));

	collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();

	Vector3 pos = thisSceneNode->_getDerivedPosition();

	//dimensions are 1/2 heights
	groundTransform.setOrigin(btVector3(pos.x, pos.y - 50.0, pos.z));

	Quaternion quat2 = thisSceneNode->_getDerivedOrientation();
	groundTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));


	btScalar mass(0.);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	//add the body to the dynamics world
	dynamicsWorld->addRigidBody(body);
}

bool Game::frameStarted(const Ogre::FrameEvent& evt)
{
	//call base class
	ApplicationContext::frameStarted(evt);
	if (this->dynamicsWorld != NULL)
	{
		// Apply new forces
		if (wDown)
			player->forward();

		if (aDown)
		{
			player->turnLeft();
		}

		if (sDown)
		{
			player->backwards();
		}

		if (dDown)
		{
			player->turnRight();
		}

		dynamicsWorld->stepSimulation((float)evt.timeSinceLastFrame, 10);

		//update positions of all objects
		for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			btTransform trans;

			if (body && body->getMotionState())
			{
				body->getMotionState()->getWorldTransform(trans);

				void* userPointer = body->getUserPointer();

				//player should update self
				if (userPointer == player)
				{
					//
				}
				else //keeps other objects working
				{
					if (userPointer)
					{
						btQuaternion orientation = trans.getRotation();
						Ogre::SceneNode* sceneNode = static_cast<Ogre::SceneNode*>(userPointer);
						sceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
						sceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
					}
				}
			}
			else
			{
				trans = obj->getWorldTransform();
			}
		}

		//update player here
		player->update();
	}
	return true;
}

bool Game::frameEnded(const Ogre::FrameEvent& evt)
{
	if (this->dynamicsWorld != NULL)
	{
		dynamicsWorld->stepSimulation((float)evt.timeSinceLastFrame, 10);
	}
	return true;
}

void Game::setupLights()
{
	//setup abient light
	scnMgr->setAmbientLight(ColourValue(1, 1, 1));
	scnMgr->setShadowTechnique(ShadowTechnique::SHADOWTYPE_STENCIL_MODULATIVE);

	//add a spotlight
	Light* spotLight = scnMgr->createLight("SpotLight");

	//configure
	spotLight->setDiffuseColour(0, 0, 1.0);
	spotLight->setSpecularColour(0, 0, 1.0);
	spotLight->setType(Light::LT_SPOTLIGHT);
	spotLight->setSpotlightRange(Degree(35), Degree(50));

}
//key inputs
bool Game::keyPressed(const KeyboardEvent& evt)
{
	if (evt.keysym.sym == SDLK_ESCAPE)
	{
		getRoot()->queueEndRendering();
	}

	if (evt.keysym.sym == 'w')
	{
		wDown = true;//debug for all keys
		std::cout << "W key pressed" << std::endl;
	}

	if (evt.keysym.sym == 'a')
	{
		aDown = true;
		std::cout << "A key pressed" << std::endl;
	}

	if (evt.keysym.sym == 's')
	{
		sDown = true;
		std::cout << "S key pressed" << std::endl;
	}

	if (evt.keysym.sym == 'd')
	{
		dDown = true;
		std::cout << "D key pressed" << std::endl;
	}

	return true;
}

bool Game::keyReleased(const KeyboardEvent& evt)
{

	if (evt.keysym.sym == 'w')
	{
		wDown = false;
	}

	if (evt.keysym.sym == 'a')
	{
		aDown = false;
	}

	if (evt.keysym.sym == 's')
	{
		sDown = false;
	}

	if (evt.keysym.sym == 'd')
	{
		dDown = false;
	}

	return true;
}


bool Game::mouseMoved(const MouseMotionEvent& evt)
{
	std::cout << "Got Mouse" << std::endl;
	return true;
}
