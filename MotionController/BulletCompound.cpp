#include "BulletCompound.h"

#include "BulletWorld.h"
#include "BulletObject.h"

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"


BulletCompound::BulletCompound( BulletWorld* world )
{
	this->bullet_world = world;

	this->location = math::position(0,0,0);
	this->rotation = math::quater(1,0,0,0);

	this->collision_count = 0;
}

BulletCompound::~BulletCompound()
{
	destroy();
}

void BulletCompound::destroy()
{
	btDynamicsWorld* dynamics_world = bullet_world->getDynamicsWorld();
	if( dynamics_world )
	{
		std::vector< btTypedConstraint* >::iterator itor_c = typed_constraints.begin();
		while( itor_c != typed_constraints.end() )
		{
			btTypedConstraint* constraint = ( *itor_c ++ );
			dynamics_world->removeConstraint( constraint );
		}
	}

	std::vector< BulletObject* >::iterator itor_o = bullet_objects.begin();
	while( itor_o != bullet_objects.end() )
	{
		BulletObject* obj = ( *itor_o ++ );
		bullet_world->removeObject( obj );
	}
	bullet_objects.clear();
}

void BulletCompound::addBulletObject( BulletObject* o )
{
	bullet_objects.push_back( o );
	o->setBulletCompound( this );
}

bool BulletCompound::createBasketStand( math::position& p, double angle, double radius, double height, double mass )
{
	math::quater q = math::rotate_transq( angle, math::vector(0,1,0) ).rotation;

	double base_size = radius * 4;
	double base_height = 5;

	double board_size = base_size;
	double board_thickness = base_height;

	double stand_radius = 2.5;
	double stand_height = height - board_size;
	
	BulletObject* base = bullet_world->addBox( p+math::vector(0,base_height/2,0), q, math::vector(base_size, base_height, base_size), mass * 0.8 );
	BulletObject* stand = bullet_world->addCylinder( p+math::vector(0,base_height,0), p+math::vector(0,base_height+stand_height,0), stand_radius, mass * 0.1 );
	BulletObject* board = bullet_world->addBox( p+math::vector(0,base_height+stand_height+board_size/2,0), q, math::vector(board_size, board_size, board_thickness), mass * 0.1 );

	base->setColor( 1, 1, 1 );
	stand->setColor( 0.5, 0.5, 0.5 );
	board->setColor( 0, 1, 0 );

	addBulletObject( base );
	addBulletObject( stand );
	addBulletObject( board );

	//
	// omit constraints (assume mass == 0)

	return true;
}

bool BulletCompound::createPunchTarget( math::position& p, double radius, double height, double mass )
{
	double base_height = 5;
	double stand_radius = 2.5;

	if( height < base_height + radius * 2 || stand_radius > radius * 0.5 )
	{
		return false;
	}

	BulletObject* base = bullet_world->addCylinder( p, p+math::vector(0,base_height,0), radius, mass * 0.5 );
	BulletObject* stand = bullet_world->addCylinder( p+math::vector(0,base_height,0), p+math::vector(0,height-radius*2,0), stand_radius, mass * 0.25 );
	BulletObject* target = bullet_world->addSphere( p+math::vector(0,height-radius,0), radius, mass * 0.25 );

	addBulletObject( base );
	addBulletObject( stand );
	addBulletObject( target );

	//
	btRigidBody* base_body = base->getRigidBody();
	btRigidBody* stand_body = stand->getRigidBody();
	btRigidBody* target_body = target->getRigidBody();

	btDynamicsWorld* dynamics_world = bullet_world->getDynamicsWorld();
	if( dynamics_world )
	{
		btTypedConstraint* constraint1 = new btFixedConstraint( *base_body, *stand_body, btTransform(btQuaternion(0,0,0,1), btVector3(0,base->getSize().y()/2,0)), btTransform(btQuaternion(0,0,0,1), btVector3(0,-stand->getSize().y()/2,0)) );
		btTypedConstraint* constraint2 = new btFixedConstraint( *stand_body, *target_body, btTransform(btQuaternion(0,0,0,1), btVector3(0,stand->getSize().y()/2,0)), btTransform(btQuaternion(0,0,0,1), btVector3(0,-target->getSize().x(),0)) );

		dynamics_world->addConstraint( constraint1, true );
		dynamics_world->addConstraint( constraint2, true );
	}

	return true;
}

