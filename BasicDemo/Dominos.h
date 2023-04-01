#ifndef TEST_EXAMPLE_H
#define TEST_EXAMPLE_H

#include <iostream>


#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../bullet3/examples/CommonInterfaces/CommonRigidBodyBase.h"

class Dominos
{
public:
    Dominos(btScalar w_in, btScalar h_in, btScalar d_in)
        : mass(1.0f), w(w_in), h(h_in), d(d_in){}
    ~Dominos() {}

    btVector3 dimensions() {
        return {w, h, d};
    }

    void put_straight(CommonRigidBodyBase* base, int n, btCollisionShape *col_shape, btVector3 axis, btQuaternion quat, btVector3 initial) {
        btTransform transform;
		transform.setIdentity();

		btScalar dominos_step(h / 3. * 2.);
        axis[1] = .0f;
        axis.normalize();

		for (int i = 0; i < n; i++)
		{
            btVector3 new_pos = axis * i * dominos_step;
            new_pos[1] = h;
            new_pos += initial;
            last_pos = new_pos;
			transform.setOrigin(btVector3(new_pos));
            transform.setRotation(btQuaternion(quat));
			base->createRigidBody(mass, transform, col_shape);
		}

        last_rot = quat;
    }

    void put_turn_left(CommonRigidBodyBase* base, btCollisionShape *col_shape, btQuaternion quat, btVector3 initial) {
        btTransform transform;
		transform.setIdentity();

		btScalar dominos_step(h / 3. * 2.);
        btScalar angle_step(SIMD_PI / 14.);

		for (int i = 0; i < 7; i++)
		{
			btQuaternion q({0.f, 1.f, 0.f}, SIMD_PI / 2 - angle_step * i);
			transform.setRotation(quat * q);

            btScalar angle = angle_step * i - quat.getAngle();
            btVector3 new_pos(cos(quat.getAngle()) - sin(angle), .0f, cos(angle) - sin(quat.getAngle()));
			transform.setOrigin(btVector3(new_pos + initial));
            
			base->createRigidBody(mass, transform, col_shape);
		}
    }

    void put_turn_right(CommonRigidBodyBase* base, btCollisionShape *col_shape, btQuaternion quat, btVector3 initial) {
        btTransform transform;
		transform.setIdentity();

		btScalar dominos_step(h / 3. * 2.);
        btScalar angle_step(SIMD_PI / 14.);

		for (int i = 0; i < 7; i++)
		{
			btQuaternion q({0.f, 1.f, 0.f}, SIMD_PI / 2 + angle_step * i);
			transform.setRotation(quat * q);

            btScalar angle = angle_step * i - quat.getAngle();
            btVector3 new_pos(-cos(quat.getAngle()) + sin(angle), .0f, cos(angle) - sin(quat.getAngle()));
			transform.setOrigin(btVector3(new_pos + initial));
            
			base->createRigidBody(mass, transform, col_shape);
		}
    }

    void create_fork(btVector3 axis, btVector3 initial, btVector3& branch_1, btVector3& branch_2) {
		btScalar dominos_step(h / 3.);
        btScalar angle(w / dominos_step);

        axis[1] - .0f;
        axis.normalize();
        btVector3 step = axis * dominos_step / 2.;

        branch_1 = step.rotate({.0f, 1.f, .0f}, angle) + initial;
        branch_2 = step.rotate({.0f, 1.f, .0f}, -angle) + initial;
    }

    btVector3 get_last_pos() {
        return last_pos;
    }

    btQuaternion get_last_rot() {
        return last_rot;
    }

private:
    btScalar mass;
    btScalar w, h, d;
    btVector3 last_pos;
    btQuaternion last_rot;
};


#endif  //BASIC_DEMO_PHYSICS_SETUP_H
