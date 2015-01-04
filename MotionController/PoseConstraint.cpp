#include "PoseConstraint.h"

// JOINT CONSTRAINT

JointConstraint::JointConstraint()
{
	type = Type::NONE;
	transform = math::identity_transq;
	doc = 0;
}

JointConstraint::JointConstraint( const math::vector& position )
: transform( math::quater(1,0,0,0), position )
{
	type = Type::POSITION;
	doc = 3;
}

JointConstraint::JointConstraint( const math::quater& orientation )
: transform( orientation, math::vector(0,0,0) )
{
	type = Type::ORIENTATION;
	doc = 3;
}

JointConstraint::JointConstraint( const math::transq& xform )
: transform( xform )
{
	type = Type::TRANSQ;
	doc = 6;
}

JointConstraint::~JointConstraint()
{
}

// POSTURE CONSTRAINT

PoseConstraint::PoseConstraint()
{
	doc = 0;
}

PoseConstraint::~PoseConstraint()
{
	removeAllConstraints();
}

bool PoseConstraint::isConstrained( unsigned int joint )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool PoseConstraint::addConstraint( unsigned int joint, const math::vector& position )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		return false;
	}
	else
	{
		JointConstraint* pos_constraint = new JointConstraint( position );
		joint_constraints[ joint ] = pos_constraint;

		doc += pos_constraint->getDOC();

		return true;
	}
}

bool PoseConstraint::addConstraint( unsigned int joint, const math::quater& orientation )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		return false;
	}
	else
	{
		JointConstraint* orient_constraint = new JointConstraint( orientation );
		joint_constraints[ joint ] = orient_constraint;

		doc += orient_constraint->getDOC();

		return true;
	}
}

bool PoseConstraint::addConstraint( unsigned int joint, const math::transq& transform )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		return false;
	}
	else
	{
		JointConstraint* transq_constraint = new JointConstraint( transform );
		joint_constraints[ joint ] = transq_constraint;

		doc += transq_constraint->getDOC();

		return true;
	}
}

bool PoseConstraint::getConstraint( unsigned int joint, JointConstraint** joint_constraint )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		*joint_constraint = itor->second;

		return true;
	}
	else
	{
		return false;
	}
}

bool PoseConstraint::getConstraint( unsigned int joint, math::transq* transform )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		JointConstraint* joint_constraint = itor->second;
		*transform = joint_constraint->getTransform();

		return true;
	}
	else
	{
		return false;
	}
}

JointConstraint* PoseConstraint::getConstraint( unsigned int joint )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		return itor->second;
	}
	else
	{
		return 0;
	}
}

bool PoseConstraint::removeConstraint( unsigned int joint )
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.find( joint );
	if( itor != joint_constraints.end() )
	{
		JointConstraint* joint_constraint = itor->second;
		delete joint_constraint;

		joint_constraints.erase( itor );

		return true;
	}
	else
	{
		return false;
	}

}

unsigned int PoseConstraint::getNumConstraints()
{
	return (unsigned int)joint_constraints.size();
}

void PoseConstraint::removeAllConstraints()
{
	std::map< unsigned int, JointConstraint* >::iterator itor = joint_constraints.begin();
	while( itor != joint_constraints.end() )
	{
		JointConstraint* joint_constraint = itor->second;
		delete joint_constraint;

		itor ++;
	}
	joint_constraints.clear();
}

