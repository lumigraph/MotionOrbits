#pragma once

#include <vector>

#include "mathclass/position.h"
#include "mathclass/vector.h"
#include "mathclass/quater.h"

class BulletWorld;
class BulletObject;

class btTypedConstraint;

class BulletCompound
{
public:
	BulletCompound( BulletWorld* world );
	virtual ~BulletCompound();

	bool createBasketStand( math::position& p, double angle, double radius, double height, double mass );
	bool createPunchTarget( math::position& p, double radius, double height, double mass );
//	void createFloorMat( math::position& p, math::quater& q, double radius, double height );

	void destroy();

	void addBulletObject( BulletObject* o );
	inline std::vector< BulletObject* >* getBulletObjects() { return &bullet_objects; }

	inline void collide()		{ collision_count ++; }
	inline void uncollide()		{ collision_count --; }
	inline bool isCollided()	{ return ( collision_count > 0 ); }

protected:
	BulletWorld* bullet_world;
	std::vector< BulletObject* > bullet_objects;

	std::vector< btTypedConstraint* > typed_constraints;

	math::position location;
	math::quater rotation;

	int collision_count;
};
