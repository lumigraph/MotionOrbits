#include "BulletObject.h"

#include "BulletWorld.h"
#include "BulletCompound.h"

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

#include "DrawingTool.h"

extern DrawingTool drawing_tool;


BulletObject::BulletObject( BulletWorld* world )
{
	bullet_world = world;
	bullet_compound = 0;

	shape_type = UNDEFINED;

	location = math::position(0,0,0);
	rotation = math::quater(1,0,0,0);
	size = math::vector(0,0,0);

	color = math::vector(0.5,0.5,0.5);
	is_hidden = false;

	rigid_body = 0;
	collision_shape = 0;
	motion_state = 0;
}

BulletObject::~BulletObject()
{
	destroy();
}

void BulletObject::destroy()
{
	if( rigid_body )
	{
		btDiscreteDynamicsWorld* dynamics_world = bullet_world->getDynamicsWorld();
		dynamics_world->removeRigidBody( this->rigid_body );
	}

	delete rigid_body;
	delete collision_shape;
	delete motion_state;

	clearCollisions();

	rigid_body = 0;
	collision_shape = 0;
	motion_state = 0;

	location = math::position(0,0,0);
	rotation = math::quater(1,0,0,0);
	size = math::vector(0,0,0);

	shape_type = UNDEFINED;
}

void BulletObject::createBox( math::position& p, math::quater& q, math::vector& size, double mass )
{
	destroy();

	this->shape_type = BOX;
	this->size = size;

	btVector3 bt_half_size( size.x()/2, size.y()/2, size.z()/2 );
	this->collision_shape = new btBoxShape( bt_half_size );	

	create( p, q, mass );
}

void BulletObject::createSphere( math::position& p, double radius, double mass )
{
	destroy();

	this->shape_type = SPHERE;
	this->size = math::vector( radius, 0, 0 );
	this->collision_shape = new btSphereShape( radius );	

	create( p, math::quater(1,0,0,0), mass );
}

void BulletObject::createCylinder( math::position& p, math::quater& q, double radius, double height, double mass )
{
	destroy();

	this->shape_type = CYLINDER;
	this->size = math::vector( radius, height, radius );

	btVector3 bt_half_size( size.x(), size.y()/2, size.z() );
	this->collision_shape = new btCylinderShape( bt_half_size );

	create( p, q, mass );
}

void BulletObject::createCylinder( math::position& p0, math::position& p1, double radius, double mass )
{
	if( p0 == p1 )
	{
		return;
	}

	math::position p = math::interpolate( 0.5, p0, p1 );

	math::vector dir( p1-p0 );
	float length = dir.length();
	dir /= length;

	math::quater q = math::quater( math::vector(0,1,0), dir );
	/*
	if( dir.x()==0 && dir.y()==0 )
	{
		if( dir.z() > 0 )	q = math::quater(1,0,0,0);
		else				q = math::quater(0,0,1,0);
	}
	else
	{
		q = math::quater( math::vector(0,1,0), dir );
	}
	*/
	createCylinder( p, q, radius, length, mass );
}

void BulletObject::create( math::position& p, math::quater& q, double mass )
{
	this->location = p;
	this->rotation = q;

	btVector3 bt_p( p.x(), p.y(), p.z() );
	btQuaternion bt_q( q.x(), q.y(), q.z(), q.w() );
	this->motion_state = new btDefaultMotionState( btTransform( bt_q, bt_p ) );

	btScalar local_mass( mass );
	btVector3 local_intertia;
	this->collision_shape->calculateLocalInertia( mass, local_intertia ); 

	btRigidBody::btRigidBodyConstructionInfo construction_info( local_mass, motion_state, collision_shape, local_intertia );   
	construction_info.m_restitution = 0.75;
	construction_info.m_friction = 0.1;

	this->rigid_body = new btRigidBody( construction_info );
	this->rigid_body->setUserPointer( this );

	btDiscreteDynamicsWorld* dynamics_world = bullet_world->getDynamicsWorld();
	dynamics_world->addRigidBody( this->rigid_body );
}

void BulletObject::update()
{
	if( rigid_body )
	{
		btTransform T = rigid_body->getWorldTransform();

		btVector3 trans = T.getOrigin();
		btQuaternion rot = T.getRotation();

		this->location = math::position( trans.x(), trans.y(), trans.z() );
		this->rotation = math::quater( rot.w(), rot.x(), rot.y(), rot.z() );
	}
}

void BulletObject::render()
{
	if( !rigid_body || is_hidden )
	{
		return;
	}

	btTransform T = rigid_body->getWorldTransform();
	
	float mat[16];
	T.getOpenGLMatrix( mat );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glMultMatrixf( mat );

	drawing_tool.setColor( color.x(), color.y(), color.z(), 1 );

	switch( shape_type ) {
	case BOX:
		{
			drawing_tool.drawBox( math::position(0,0,0), this->size ); 
		}
		break;

	case SPHERE:
		{
			drawing_tool.drawSphere( math::position(0,0,0), this->size.x() );
		}
		break;

	case CYLINDER:
		{
			drawing_tool.drawCylinder( math::position(0,-size.y()/2,0), math::position(0,size.y()/2,0), size.x() );
		}
		break;
	}

	glPopMatrix();
}

void BulletObject::addCollision( BulletObject* coll_obj )
{
	collided_objects.push_back( coll_obj );
	
	if( bullet_compound )
	{
		bullet_compound->collide();
	}
}

void BulletObject::clearCollisions()
{
	int num_collisions = (int)collided_objects.size();
	collided_objects.clear();

	if( bullet_compound )
	{
		for( int i=0; i < num_collisions; i++ )
		{
			bullet_compound->uncollide();
		}
	}
}
