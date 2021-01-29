#include "Player.h"
#include <iostream>
Player::Player()
{
  boxSceneNode = nullptr;
  box = nullptr;
  Vector3 meshBoundingBox(0.0f,0.0f,0.0f);

  colShape = nullptr;
  dynamicsWorld = nullptr;

  //hardcoded in player

  forwardForce = 100.0f;
  turningForce = 20.0f;
  linearDamping = 0.2f;
  angularDamping = 0.8f;
}

Player::~Player()
{

}

void Player::createMesh(SceneManager* scnMgr)
{
  box = scnMgr->createEntity("ninja.mesh");
}

void Player::attachToNode(SceneNode* parent)
{
  boxSceneNode = parent->createChildSceneNode();
  boxSceneNode->attachObject(box);
  boxSceneNode->setScale(1.0f,1.0f,1.0f);
  boundingBoxFromOgre();
}

void Player::setScale(float x, float y, float z)
{
    boxSceneNode->setScale(x,y,z);
}


void Player::setRotation(Vector3 axis, Radian rads)
{
  //quat from axis angle
  Quaternion quat(rads, axis);
  boxSceneNode->setOrientation(quat);
}

void Player::setPosition(float x, float y, float z)
{
  boxSceneNode->setPosition(x,y,z);
}

void Player::boundingBoxFromOgre()
{
  //get bounding box here
  boxSceneNode->_updateBounds();
  const AxisAlignedBox& b = boxSceneNode->_getWorldAABB();
  Vector3 temp(b.getSize());
  meshBoundingBox = temp;
}

void Player::createRigidBody(float bodyMass)
{
  colShape = new btBoxShape(btVector3(meshBoundingBox.x/2.0f, meshBoundingBox.y/20.0f, meshBoundingBox.z/2.0f));

  // Create Dynamic Objects
  btTransform startTransform;
  startTransform.setIdentity();

  Quaternion quat2 = boxSceneNode->_getDerivedOrientation();
  startTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

  Vector3 pos = boxSceneNode->_getDerivedPosition();
  startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));

  btScalar mass(bodyMass);

  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  bool isDynamic = (mass != 0.f);

  btVector3 localInertia(0, 0, 0);
  if (isDynamic)
  {
      colShape->calculateLocalInertia(mass, localInertia);
  }

  btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
  body = new btRigidBody(rbInfo);

  body->setDamping(linearDamping,angularDamping);

  //set user pointer to this object
  body->setUserPointer((void*)this);
}

void Player::addToCollisionShapes(btAlignedObjectArray<btCollisionShape*> &collisionShapes)
{
  collisionShapes.push_back(colShape);
}

void Player::addToDynamicsWorld(btDiscreteDynamicsWorld* dynamicsWorld)
{
  this->dynamicsWorld = dynamicsWorld;
  dynamicsWorld->addRigidBody(body);
}

void Player::update()
{
  btTransform trans;

  if (body && body->getMotionState())
  {
    body->getMotionState()->getWorldTransform(trans);
    btQuaternion orientation = trans.getRotation();

    boxSceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
    boxSceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
  }
}

void Player::forward()
{
    btVector3 fwd(0.0f,0.0f,-forwardForce);
    btVector3 push;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //get the orientation of the rigid body in world space
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //rotate the local force into the global space.
        push = quatRotate(orientation, fwd);

        //activate the body
        body->activate();

        //apply a force to the center of the body
        body->applyCentralForce(push);
    }
}

void Player::turnLeft()
{
    btVector3 left(turningForce, 0.0f, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (body && body->getMotionState())
    {
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        btVector3 front(trans.getOrigin());

        front += btVector3(0.0f, 0.0f, meshBoundingBox.z / 2);

        turn = quatRotate(orientation, left);

        if (body->getLinearVelocity().length() > 0.0f)
            body->applyForce(turn, front);
    }

}
    
void Player::turnRight()
{
    //apply a turning force to the front of the body.
    btVector3 right(turningForce, 0.0f, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //get orientation of the body
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //get position of the body
        btVector3 front(trans.getOrigin());

        //use original bounding mesh to get the front center
        front += btVector3(0.0f, 0.0f, meshBoundingBox.z / 2);

        //orientated the local force into world space
        turn = quatRotate(orientation, right);

        if (body->getLinearVelocity().length() > 0.0f)
            body->applyForce(turn, front);
    }
}
void Player::backwards()
{
    //create vector in local coordinates
    btVector3 fwd(0.0f, 0.0f, forwardForce);
    btVector3 push;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //get orientation of the rigid body in world space
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //rotate local force into global space.
        push = quatRotate(orientation, fwd);

        //activate the body
        body->activate();

        //apply a force to the center of the body
        body->applyCentralForce(push);
    }
}
