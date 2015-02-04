#include "BulletWorld.h"

#include "BulletObject.h"
#include "BulletCompound.h"

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

BulletWorld::BulletWorld()
{
	broadphase_interface = 0;
	collision_configuration = 0;
	collision_dispatcher = 0;
	constraint_solver = 0;
	dynamics_world = 0;
}

BulletWorld::~BulletWorld()
{
	finalize();
}

void BulletWorld::initialize()
{
	broadphase_interface = new btDbvtBroadphase();
	collision_configuration = new btDefaultCollisionConfiguration();
	collision_dispatcher = new btCollisionDispatcher( collision_configuration );
	constraint_solver = new btSequentialImpulseConstraintSolver();
	
	dynamics_world = new btDiscreteDynamicsWorld( collision_dispatcher, broadphase_interface, constraint_solver, collision_configuration );
	dynamics_world->setGravity( btVector3(0, -10, 0) );
}

void BulletWorld::finalize()
{
	removeAll();

	delete dynamics_world;
	delete constraint_solver;
	delete collision_dispatcher;
	delete collision_configuration;
	delete broadphase_interface;

	dynamics_world = 0;
	constraint_solver = 0;
	collision_dispatcher = 0;
	collision_configuration = 0;
	broadphase_interface = 0;
}

BulletObject* BulletWorld::addBox( math::position& p, math::quater& q, math::vector& size, double mass )
{
	if( !dynamics_world )
	{
		return 0;
	}

	BulletObject* obj = new BulletObject( this );
	obj->createBox( p, q, size, mass );

	bullet_objects.push_back( obj );
	return obj;
}

BulletObject* BulletWorld::addSphere( math::position& p, double radius, double mass )
{
	if( !dynamics_world )
	{
		return 0;
	}

	BulletObject* obj = new BulletObject( this );
	obj->createSphere( p, radius, mass );

	bullet_objects.push_back( obj );
	return obj;
}

BulletObject* BulletWorld::addCylinder( math::position& p, math::quater& q, double radius, double height, double mass )
{
	if( !dynamics_world )
	{
		return 0;
	}

	BulletObject* obj = new BulletObject( this );
	obj->createCylinder( p, q, radius, height, mass );

	bullet_objects.push_back( obj );
	return obj;
}

BulletObject* BulletWorld::addCylinder( math::position& p0, math::position& p1, double radius, double mass )
{
	if( !dynamics_world )
	{
		return 0;
	}

	BulletObject* obj = new BulletObject( this );
	obj->createCylinder( p0, p1, radius, mass );

	bullet_objects.push_back( obj );
	return obj;
}

BulletCompound* BulletWorld::addBasketStand( math::position& p, double angle, double radius, double height, double mass )
{
	if( !dynamics_world )
	{
		return 0;
	}

	BulletCompound* comp = new BulletCompound( this );
	bool is_created = comp->createBasketStand( p, angle, radius, height, mass );

	if( is_created )
	{
		bullet_compounds.push_back( comp );
		return comp;
	}
	else
	{
		delete comp;
		return 0;
	}
}

BulletCompound* BulletWorld::addPunchTarget( math::position& p, double radius, double height, double mass )
{
	if( !dynamics_world )
	{
		return 0;
	}

	BulletCompound* comp = new BulletCompound( this );
	bool is_created = comp->createPunchTarget( p, radius, height, mass );

	if( is_created )
	{
		bullet_compounds.push_back( comp );
		return comp;
	}
	else
	{
		delete comp;
		return 0;
	}
}

bool BulletWorld::removeCompound( BulletCompound* comp )
{
	std::vector< BulletCompound* >::iterator itor_c = bullet_compounds.begin();
	while( itor_c != bullet_compounds.end() )
	{
		BulletCompound* c = ( *itor_c );
		if( c == comp )
		{
			bullet_compounds.erase( itor_c );
			delete c;
			return true;
		}
		itor_c ++;
	}
	return false;
}

bool BulletWorld::removeObject( BulletObject* obj )
{
	std::vector< BulletObject* >::iterator itor_o = bullet_objects.begin();
	while( itor_o != bullet_objects.end() )
	{
		BulletObject* o = ( *itor_o );
		if( o == obj )
		{
			bullet_objects.erase( itor_o );
			delete o;

			return true;
		}
		itor_o ++;
	}
	return false;
}

void BulletWorld::removeAll()
{
	std::vector< BulletCompound* >::iterator itor_c = bullet_compounds.begin();
	while( itor_c != bullet_compounds.end() )
	{
		BulletCompound* comp = ( *itor_c ++ );
		delete comp;
	}
	bullet_compounds.clear();

	std::vector< BulletObject* >::iterator itor_o = bullet_objects.begin();
	while( itor_o != bullet_objects.end() )
	{
		BulletObject* obj = ( *itor_o ++ );
		delete obj;
	}
	bullet_objects.clear();
}

void BulletWorld::update()
{
	tickSimulation();
	detectCollisions();
}

void BulletWorld::render()
{
	std::vector< BulletObject* >::iterator itor_o = bullet_objects.begin();
	while( itor_o != bullet_objects.end() )
	{
		BulletObject* obj = ( *itor_o ++ );
		obj->render();
	}
}

void BulletWorld::tickSimulation()
{
	if( dynamics_world )
	{
		dynamics_world->stepSimulation( 1/60.0, 10 );
	}

	std::vector< BulletObject* >::iterator itor_o = bullet_objects.begin();
	while( itor_o != bullet_objects.end() )
	{
		BulletObject* obj = ( *itor_o ++ );
		obj->update();
	}
}

void BulletWorld::detectCollisions()
{
	if( !collision_dispatcher )
	{
		return;
	}

	std::vector< BulletObject* >::iterator itor_o = bullet_objects.begin();
	while( itor_o != bullet_objects.end() )
	{
		BulletObject* obj = ( *itor_o ++ );
		obj->clearCollisions();
	}

	int num_manifolds = this->collision_dispatcher->getNumManifolds();
    for( int i=0; i < num_manifolds; i++ )
    {
        btPersistentManifold* contactManifold =  collision_dispatcher->getManifoldByIndexInternal(i);
        btCollisionObject* obA = const_cast<btCollisionObject*>( contactManifold->getBody0() );
        btCollisionObject* obB = const_cast<btCollisionObject*>( contactManifold->getBody1() );

		BulletObject* obj0 = static_cast<BulletObject*>( obA->getUserPointer() );
		BulletObject* obj1 = static_cast<BulletObject*>( obB->getUserPointer() );

		obj0->addCollision( obj1 );
		obj1->addCollision( obj0 );

		/*
        int numContacts = contactManifold->getNumContacts();
        for (int j=0;j<numContacts;j++)
        {
            btManifoldPoint& pt = contactManifold->getContactPoint(j);
            if (pt.getDistance()<0.f)
            {
                const btVector3& ptA = pt.getPositionWorldOnA();
                const btVector3& ptB = pt.getPositionWorldOnB();
                const btVector3& normalOnB = pt.m_normalWorldOnB;
            }
        }
		*/
    }
}
