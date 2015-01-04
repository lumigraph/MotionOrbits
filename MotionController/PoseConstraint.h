#pragma once

#include <map>
#include "mathclass/transq.h"

class MotionData;

class JointConstraint
{
public:
	enum Type
	{
		NONE = -1,
		POSITION,
		ORIENTATION,
		TRANSQ,
	};

	JointConstraint();
	~JointConstraint();

	JointConstraint( const math::vector& position );
	JointConstraint( const math::quater& orientation );
	JointConstraint( const math::transq& transform );

	inline Type getType() { return type; }
	inline unsigned int getDOC() { return doc; }

	inline const math::transq& getTransform()	{ return transform; }
	inline const math::vector& getPosition()	{ return transform.translation; }
	inline const math::quater& getOrientation()	{ return transform.rotation; }

	inline void setTransform( const math::transq& t )	{ transform = t; }
	inline void setPosition( const math::vector& v )	{ transform.translation = v; }
	inline void setOrientation( const math::quater& q )	{ transform.rotation = q; }

private:
	Type			type;
	math::transq	transform;
	unsigned int	doc;
};

class PoseConstraint
{
public:
	PoseConstraint();
	~PoseConstraint();

	bool isConstrained( unsigned int joint );

	bool addConstraint( unsigned int joint, const math::vector& position );
	bool addConstraint( unsigned int joint, const math::quater& orientation );
	bool addConstraint( unsigned int joint, const math::transq& transform );

	bool getConstraint( unsigned int joint, JointConstraint** joint_constraint );
	bool getConstraint( unsigned int joint, math::transq* transform );
	
	bool removeConstraint( unsigned int joint );

	unsigned int getNumConstraints();
	JointConstraint* getConstraint( unsigned int joint );

	void removeAllConstraints();

	inline std::map< unsigned int, JointConstraint* >* getConstraints()	{ return &joint_constraints; }
	inline unsigned int getDOC() { return doc; }

private:
	std::map< unsigned int, JointConstraint* > joint_constraints;
	
	unsigned int doc;
};
