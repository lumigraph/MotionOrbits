#pragma once

#include "mathclass/position.h"
#include "mathclass/quater.h"
#include "mathclass/transq.h"
#include "mathclass/vectorN.h"

#include <vector>

class Skeleton;

class PoseData
{
public:
	PoseData();
	virtual ~PoseData();

	void initialize( unsigned int num_joints );
	void finalize();

	bool copy( PoseData* pose );
	bool blend( PoseData* pose1, PoseData* pose2, double ratio );
	bool blend( PoseData* pose1, math::transq& T1, PoseData* pose2, math::transq& T2, double ratio );

	bool blend( std::vector< std::pair<PoseData*, math::transq> >* poses, std::vector<double>* ratios );
	math::vector blendVects( std::vector< math::vector >* vects, std::vector<double>* ratios );
	math::quater blendQuats( std::vector< math::quater >* quats, std::vector<double>* ratios ); 

	bool addDisplacement( Skeleton* s, const math::vectorN& d );
	void transform( const math::transq& t );

	void setJointTransform( unsigned int i, const math::transq& t );
	math::transq getJointTransform( unsigned int i );

	math::transq getGlobalTransform( Skeleton* skeleton, unsigned int joint_index, const math::transq& body_transform = math::identity_transq );
	math::position getPosition( Skeleton* skeleton, unsigned int i );
	math::quater getOrientation( Skeleton* skeleton, unsigned int i );

	inline unsigned int getNumJoints()	{ return num_joints; }

	static math::transq calcDisplacement( PoseData* p1, PoseData* p2 );

protected:
	unsigned int	num_joints;
	math::transq*	joint_transforms;
};

