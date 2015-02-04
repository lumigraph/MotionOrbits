#pragma once

#include "mathclass/position.h"
#include "mathclass/vector.h"
#include "mathclass/quater.h"
#include "mathclass/transq.h"

#include <vector>

class BulletWorld;
class BulletCompound;

class btRigidBody;
class btCollisionShape;
class btDefaultMotionState;

class BulletObject
{
public:
	enum
	{
		UNDEFINED = -1,
		BOX,
		SPHERE,
		CYLINDER,
	};

	BulletObject( BulletWorld* world );
	virtual ~BulletObject();

	void createBox( math::position& p, math::quater& q, math::vector& size, double mass );
	void createSphere( math::position& p, double radius, double mass );
	void createCylinder( math::position& p, math::quater& q, double radius, double height, double mass );
	void createCylinder( math::position& p0, math::position& p1, double radius, double mass );

	void update();
	void render();

	void destroy();

	void addCollision( BulletObject* coll_obj );
	void clearCollisions();

	//
	inline math::position getLocation()		{ return location; }
	inline math::quater getRotation()		{ return rotation; }
	inline math::vector getSize()			{ return size; }

	inline void setColor( double r, double g, double b )	{ color = math::vector(r,g,b); }
	inline math::vector getColor()							{ return color; }

	inline void hide()	{ is_hidden = true; }
	inline void show()	{ is_hidden = false; }
	
	inline bool isShown()	{ return !is_hidden; }
	inline bool isHidden()	{ return is_hidden; }

	inline bool isCollided()									{ return !collided_objects.empty(); }
	inline std::vector< BulletObject* >* getCollidedObjects()	{ return &collided_objects; }

	inline void setBulletCompound( BulletCompound* c )		{ bullet_compound = c; }
	inline BulletCompound* getBulletCompound()				{ return bullet_compound; }

	inline btCollisionShape* getCollisionShape()			{ return collision_shape; }
	inline btDefaultMotionState* getMotionState()			{ return motion_state; }
	inline btRigidBody* getRigidBody()						{ return rigid_body; }

	/*
	inline void setCollisionShape( btCollisionShape* s )	{ collision_shape = s; }
	inline btCollisionShape* getCollisionShape()			{ return collision_shape; }

	inline void setMotionState( btDefaultMotionState* s )	{ motion_state = s; }
	inline btDefaultMotionState* getMotionState()			{ return motion_state; }

	inline void setRigidBody( btRigidBody* b )				{ rigid_body = b; }
	inline btRigidBody* getRigidBody()						{ return rigid_body; }
	*/

protected:
	void create( math::position& p, math::quater& q, double mass );

	BulletWorld* bullet_world;
	BulletCompound* bullet_compound;

	int shape_type;

	math::position location;
	math::quater rotation;
	math::vector size;
	
	math::vector color;
	
	bool is_hidden;

	std::vector< BulletObject* > collided_objects;

	//
	btRigidBody* rigid_body;
	btCollisionShape* collision_shape;
	btDefaultMotionState* motion_state;
};
