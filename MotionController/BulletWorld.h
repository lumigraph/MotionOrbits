#pragma once

#include <vector>

#include "mathclass/position.h"
#include "mathclass/vector.h"
#include "mathclass/quater.h"

class btBroadphaseInterface;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;

class BulletObject;
class BulletCompound;

class BulletWorld
{
public:
	BulletWorld();
	virtual ~BulletWorld();

	void initialize();
	void finalize();

	void update();
	void render();

	BulletObject* addBox( math::position& p, math::quater& q, math::vector& size, double mass );
	BulletObject* addSphere( math::position& p, double radius, double mass );
	BulletObject* addCylinder( math::position& p, math::quater& q, double radius, double height, double mass );
	BulletObject* addCylinder( math::position& p0, math::position& p1, double radius, double mass );

	BulletCompound* addBasketStand( math::position& p, double angle, double radius, double height, double mass );
	BulletCompound* addPunchTarget( math::position& p, double radius, double height, double mass );

	bool removeObject( BulletObject* obj );
	bool removeCompound( BulletCompound* comp );

	void removeAll();

	inline btDiscreteDynamicsWorld* getDynamicsWorld()	{ return dynamics_world; }

	inline std::vector< BulletCompound* >* getBulletCompounds()	{ return &bullet_compounds; }
	inline std::vector< BulletObject* >* getBulletObjects()		{ return &bullet_objects; }

protected:
	void tickSimulation();
	void detectCollisions();

	btBroadphaseInterface* broadphase_interface;
	btDefaultCollisionConfiguration* collision_configuration;
	btCollisionDispatcher* collision_dispatcher;
	btSequentialImpulseConstraintSolver* constraint_solver;
	btDiscreteDynamicsWorld* dynamics_world;

	//
	std::vector< BulletObject* > bullet_objects;
	std::vector< BulletCompound* > bullet_compounds;
};
