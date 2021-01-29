#ifndef PLAYER_H_
#define PLAYER_H_

/* Ogre3d Graphics*/
#include "Ogre.h"

/* Bullet3 Physics */
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

using namespace Ogre;

class Player
{
private:
  SceneNode* boxSceneNode;     
  Entity* box;                 
  Vector3 meshBoundingBox;     

  btCollisionShape* colShape;  
  btRigidBody* body;          
  btDiscreteDynamicsWorld* dynamicsWorld;  

  float forwardForce; 
  float turningForce; 
  btScalar linearDamping; 
  btScalar angularDamping; 

public:
  Player();
  ~Player();

  //creates the mesh
  void createMesh(SceneManager* scnMgr);
  //Creates a new child of the given parent node
  void attachToNode(SceneNode* parent);


  //sets the scale
  void setScale(float x, float y, float z);
  
  //sets orientation
  void setRotation(Vector3 axis, Radian angle);

  //sets position
  void setPosition(float x, float y, float z);

  void boundingBoxFromOgre();

  void createRigidBody(float mass);
 
  void addToCollisionShapes(btAlignedObjectArray<btCollisionShape*> &collisionShapes);
  
  void addToDynamicsWorld(btDiscreteDynamicsWorld* dynamicsWorld);

  void setMass(float mass);

  void update();

  //moves player
  void forward();

  void turnRight();

  void turnLeft();

  void backwards();


};

#endif
