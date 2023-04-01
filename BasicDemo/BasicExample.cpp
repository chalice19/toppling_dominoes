/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BasicExample.h"
#include "Dominos.h"

// #include "btBulletDynamicsCommon.h"

// #include "LinearMath/btVector3.h"
// #include "LinearMath/btAlignedObjectArray.h"

// #include "../bullet3/CommonInterfaces/CommonRigidBodyBase.h"

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52; 
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(5.), btScalar(1.), btScalar(5.)));
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -1, 0));

	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btScalar w(.15f);
		btScalar h(.3f);
		btScalar d(.03f);
		Dominos domino(w, h, d);

		btBoxShape* colShape = createBoxShape(domino.dimensions());

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		// domino.put_turn_left(this, colShape, {0.f, 0.f, 0.0f}, {.0f, .0f, -1.0f});

		domino.put_straight(this, 5, colShape, {0.f, 0.f, 1.0f}, {.0f, .0f, .0f}, {.0f, .0f, .0f});
		domino.put_turn_right(this, colShape, domino.get_last_rot(), domino.get_last_pos());

		// btVector3 v1, v2;
		// domino.create_fork({1.f, 0.f, 0.0f}, domino.get_last_pos(), v1, v2);

		// domino.put_straight(this, 5, colShape, {1.f, 0.f, 1.0f}, {SIMD_PI / 4, .0f, .0f}, v1);
		// domino.put_straight(this, 5, colShape, {-1.f, 0.f, 1.0f}, {-SIMD_PI / 4, .0f, .0f}, v2);
		
		// domino.put_turn_left(this, colShape, btQuaternion(2.5 * SIMD_PI / 4.f, .0f, .0f), v1);
		// domino.put_turn_left(this, colShape, btQuaternion(SIMD_PI / 4, .0f, .0f), v2);

		//// Create Dynamic Objects
		// btTransform startTransform;
		// startTransform.setIdentity();

		// btScalar mass(1.f);
		// btScalar dominos_step(0.2f);

		// for (int j = 0; j < 5; j++)
		// {
		// 	startTransform.setOrigin(btVector3(
		// 		btScalar(0),
		// 		btScalar(h),
		// 		dominos_step * j));

		// 	createRigidBody(mass, startTransform, colShape);
		// }

		// btScalar angle_step(SIMD_PI / 14.);
		// btScalar fork_shift();

		// for (int i = 0; i < 7; i++)
		// {
		// 	startTransform.setOrigin(btVector3(
		// 		btScalar(1. - sin(angle_step * i) + w * 0.65),
		// 		btScalar(h),
		// 		btScalar(cos(angle_step * i)) + dominos_step * 4));

		// 	btQuaternion quat({0.f, 1.f, 0.f}, SIMD_PI / 2 - angle_step * i);
		// 	startTransform.setRotation(quat);

		// 	createRigidBody(mass, startTransform, colShape);
		// }

		// for (int i = 0; i < 7; i++)
		// {
		// 	startTransform.setOrigin(btVector3(
		// 		btScalar(-1. + sin(angle_step * i) - w * 0.65),
		// 		btScalar(h),
		// 		btScalar(cos(angle_step * i)) + dominos_step * 4));

		// 	btQuaternion quat({0.f, 1.f, 0.f}, SIMD_PI / 2 + angle_step * i);
		// 	startTransform.setRotation(quat);

		// 	createRigidBody(mass, startTransform, colShape);
		// }
	}
	{
		btBoxShape* colShape = createBoxShape(btVector3(.07, .1, .007));

		btCollisionShape* sphereShape = new btSphereShape(btScalar(0.04));
		m_collisionShapes.push_back(sphereShape);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(0, 0.6, -0.75));

		btScalar mass(0.075f);

		btVector3 localInertia(0, 0, 0);
		sphereShape->calculateLocalInertia(mass, localInertia);

		btRigidBody *sphere = createRigidBody(mass, transform, sphereShape);
		sphere->setLinearVelocity({0.f, 0.f, 4.f});
	}
	{
		// create a hummer
		// create a new compound shape for making a hummer from `cube`s
		btCompoundShape* compoundShape = new btCompoundShape();
		btTransform transform;

		btBoxShape* cube1 = createBoxShape(btVector3(.008, 0.25, .008));
		m_collisionShapes.push_back(cube1);

		transform.setIdentity();
		transform.setOrigin(btVector3(0, .23, 0));
		compoundShape->addChildShape(transform, cube1);


		btBoxShape* cube2 = createBoxShape(btVector3(.04, .04, 0.08));
		m_collisionShapes.push_back(cube2);

		transform.setIdentity();
		transform.setOrigin(btVector3(0, 0, 0));
		compoundShape->addChildShape(transform, cube2);

		btScalar masses[2] = {0.5, 0.5};
		btTransform principal;
		btVector3 inertia;
		compoundShape->calculatePrincipalAxisTransform(masses, principal, inertia);

		// new compund shape to store
		btCompoundShape* compound2 = new btCompoundShape();
		m_collisionShapes.push_back(compound2);

		// recompute the shift to make sure the compound shape is re-aligned
		for (int i = 0; i < compoundShape->getNumChildShapes(); i++)
			compound2->addChildShape(compoundShape->getChildTransform(i) * principal.inverse(),
									 compoundShape->getChildShape(i));

		delete compoundShape;

		transform.setIdentity();
		transform.setOrigin(btVector3(0., 0.5, -2.));
		createRigidBody(1.0, transform, compound2);
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)
