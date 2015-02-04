#include "PoseData.h"
#include "Skeleton.h"
#include "Joint.h"
#include "Human.h"

#include "mathclass/QmGeodesic.h"

#include <cassert>

PoseData::PoseData()
{
	num_joints = 0;
	joint_transforms = 0;
}

PoseData::~PoseData()
{
	finalize();
}

void PoseData::initialize( unsigned int num_joints )
{
	if( num_joints > 0 )
	{
		finalize();

		this->num_joints = num_joints;
		joint_transforms = new math::transq[ num_joints ];
		
		unsigned int i;
		for( i=0; i < num_joints; i++ )
		{
			setJointTransform( i, math::identity_transq );
		}
	}
	else
	{
		assert( false );
	}
}

void PoseData::finalize()
{
	delete[] joint_transforms;
	joint_transforms = 0;
	num_joints = 0;
}

void PoseData::setJointTransform( unsigned int i, const math::transq& t )
{
	if( i < num_joints )
	{
		joint_transforms[ i ] = t;
	}
	else
	{
		assert( false );
	}
}

math::transq PoseData::getJointTransform( unsigned int i )
{
	if( i < num_joints )
	{
		return joint_transforms[ i ];
	}
	else
	{
		assert( false );
		return math::identity_transq;
	}
}

void PoseData::transform( const math::transq& t )
{
	setJointTransform( 0, t * getJointTransform(0) );
}

bool PoseData::copy( PoseData* pose )
{
	if( this->num_joints != pose->num_joints )
	{
		return false;
	}

	unsigned int i;
	for( i=0; i < num_joints; i++ )
	{
		this->setJointTransform( i, pose->getJointTransform(i) );
	}
	return true;
}

bool PoseData::blend( PoseData* pose1, PoseData* pose2, double ratio )
{
	if( this->num_joints != pose1->num_joints ||
		this->num_joints != pose2->num_joints )
	{
		return false;
	}

	// 1. alignment
	math::transq disp = PoseData::calcDisplacement( pose1, pose2 );

	// 2. warping
	math::transq src_t = pose1->getJointTransform( 0 );
	math::transq tgt_t = disp * pose2->getJointTransform( 0 );

	if( src_t.rotation % tgt_t.rotation < 0 )		tgt_t.rotation = -tgt_t.rotation;

	math::transq warp_t = math::interpolate( ratio, src_t, tgt_t );
	this->setJointTransform( 0, warp_t );

	for( unsigned int j=1; j < num_joints; j++ )
	{
		math::transq src_t = pose1->getJointTransform( j );
		math::transq tgt_t = pose2->getJointTransform( j );

		if( src_t.rotation % tgt_t.rotation < 0 )		tgt_t.rotation = -tgt_t.rotation;
	
		math::transq warp_t = math::interpolate( ratio, src_t, tgt_t );

		this->setJointTransform( j, warp_t );
	}
	return true;
}

bool PoseData::blend( PoseData* pose1, math::transq& T1, PoseData* pose2, math::transq& T2, double ratio )
{
	if( this->num_joints != pose1->num_joints ||
		this->num_joints != pose2->num_joints )
	{
		return false;
	}

	math::transq src_t = T1 * pose1->getJointTransform( 0 );
	math::transq tgt_t = T2 * pose2->getJointTransform( 0 );

	if( src_t.rotation % tgt_t.rotation < 0 )		tgt_t.rotation = -tgt_t.rotation;

	math::transq warp_t = math::interpolate( ratio, src_t, tgt_t );
	this->setJointTransform( 0, warp_t );

	for( unsigned int j=1; j < num_joints; j++ )
	{
		math::transq src_t = pose1->getJointTransform( j );
		math::transq tgt_t = pose2->getJointTransform( j );

		if( src_t.rotation % tgt_t.rotation < 0 )		tgt_t.rotation = -tgt_t.rotation;
	
		math::transq warp_t = math::interpolate( ratio, src_t, tgt_t );

		this->setJointTransform( j, warp_t );
	}
	return true;
}

bool PoseData::blend( std::vector< std::pair<PoseData*, math::transq> >* poses, std::vector< double >* ratios )
{
	if( poses->size() != ratios->size() )
	{
		return false;
	}

	int num_poses = poses->size();

	if( num_poses == 0 )
	{
		return false;
	}
	else if( num_poses == 1 )
	{
		PoseData* pose = ( *poses )[0].first;
		math::transq T = ( *poses )[0].second;

		this->copy( pose );
		this->setJointTransform( 0, T * this->getJointTransform(0) );

		return true;
	}
	else	// num_poses >= 2
	{
		PoseData* pose1 = ( *poses )[ num_poses-2 ].first;
		PoseData* pose2 = ( *poses )[ num_poses-1 ].first;

		math::transq T1 = ( *poses )[ num_poses-2 ].second;
		math::transq T2 = ( *poses )[ num_poses-1 ].second;

		double ratio1 = ( *ratios )[ num_poses-2 ];
		double ratio2 = ( *ratios )[ num_poses-1 ];
		double ratio_sum = ratio1 + ratio2;

		double t1 = ratio2 / ratio_sum;
		double t2 = ratio1 / ratio_sum;

		PoseData* blended_pose = new PoseData;
		blended_pose->initialize( pose1->getNumJoints() );
		blended_pose->blend( pose1, T1, pose2, T2, t1 );

		poses->pop_back();
		poses->pop_back();

		ratios->pop_back();
		ratios->pop_back();

		//
		poses->push_back( std::make_pair(blended_pose, math::identity_transq ) );
		ratios->push_back( ratio_sum );

		this->blend( poses, ratios );

		poses->pop_back();
		ratios->pop_back();
		//

		poses->push_back( std::make_pair(pose1, T1) );
		poses->push_back( std::make_pair(pose2, T2) );

		ratios->push_back( ratio1 );
		ratios->push_back( ratio2 );

		delete blended_pose;

		return true;
	}

	/*
	std::vector< math::vector > vects;
	std::vector< math::quater > quats;

	for( int n=0; n < num_poses; n++ )
	{
		PoseData* pose = ( *poses )[ n ].first;
		math::transq T = ( *poses )[ n ].second;

		math::transq rootT = T * pose->getJointTransform( 0 );
		vects.push_back( rootT.translation );
		quats.push_back( rootT.rotation );
	}
	math::vector root_v = blendVects( &vects, ratios );
	math::quater root_q = blendQuats( &quats, ratios );
	this->setJointTransform( 0, math::transq(root_q, root_v) );

	for( int j=1; j < num_joints; j++ )
	{
		quats.clear();

		for( int n=0; n < num_poses; n++ )
		{
			PoseData* pose = ( *poses )[ n ].first;
			math::transq jointT = pose->getJointTransform( j );
			quats.push_back( jointT.rotation );
		}
		math::vector joint_v = math::vector(0,0,0);
		math::quater joint_q = blendQuats( &quats, ratios );
		this->setJointTransform( j, math::transq(joint_q, joint_v) );
	}

	return true;
	*/
}

math::vector PoseData::blendVects( std::vector< math::vector >* vects, std::vector< double >* ratios )
{
	if( vects->size() != ratios->size() )
	{
		return math::vector(0,0,0);
	}

	math::vector weighted_sum(0,0,0);
	double sum_of_weights = 0;

	int num_vects = (int)vects->size();
	for( int i=0; i < num_vects; i++ )
	{
		weighted_sum += ( *vects )[i] * ( *ratios )[i];
		sum_of_weights += ( *ratios )[i];
	}
	return weighted_sum / sum_of_weights;
}

math::quater PoseData::blendQuats( std::vector< math::quater >* quats, std::vector< double >* ratios )
{
	if( quats->size() != ratios->size() )
	{
		return math::quater(1,0,0,0);
	}

	int num_quats = (int)quats->size();

	math::quater blend_q = ( *quats )[0];
	math::vector blend_v;
	double e;

	do
	{
		blend_v = math::vector(0,0,0);
		for( int i=0; i < num_quats; i++ )
		{
			blend_v += ( *ratios )[i] * math::ln( blend_q.inverse() * ( *quats )[i] );
		}
		blend_q = blend_q * math::exp( blend_v );

		e = blend_v.length();
	}
	while( e > 1.0e-4 );

	return blend_q;
}

bool PoseData::addDisplacement( Skeleton* skeleton, const math::vectorN& d )
{
	unsigned int num_joints = skeleton->getNumJoints();
	if( this->num_joints != num_joints )
	{
		return false;
	}

	unsigned int total_dof = skeleton->calcDOF();
	if( total_dof != d.size() )
	{
		return false;
	}

	int i, j = 0;
	math::vector p, v;

	for( i=0; i < NUM_JOINTS_IN_HUMAN; i++ )
	{
		Joint* joint = skeleton->getHumanJoint( i );
		if( joint )
		{
			unsigned int id = joint->getIndex();
			unsigned int dof = joint->getDOF();

			switch( dof ) {
			case 6:
				{
					p = math::vector( d[j  ], d[j+1], d[j+2] );
					v = math::vector( d[j+3], d[j+4], d[j+5] );					
					math::vector t = joint_transforms[ id ].translation;
					math::quater r = joint_transforms[ id ].rotation;
					joint_transforms[ id ].translation = t + p;
					joint_transforms[ id ].rotation = r * math::exp(v);
				}
				break;

			case 3:
				{
					v = math::vector( d[j], d[j+1], d[j+2] );				
					math::quater r = joint_transforms[ id ].rotation;
					joint_transforms[ id ].rotation = r * math::exp(v);
				}
				break;

			case 1:
				{
					v = math::vector( d[j], 0, 0 );				
					math::quater r = joint_transforms[ id ].rotation;
					joint_transforms[ id ].rotation = r * math::exp(v);
				}
				break;
			}
			j += dof;
		}
	}
	return true;
}

math::transq PoseData::getGlobalTransform( Skeleton* skeleton, unsigned int joint_index, const math::transq& body_transform )
{
	math::transq joint_transform = math::identity_transq;

	Joint* joint = skeleton->getJointByIndex( joint_index );
	while( joint )
	{
		math::vector joint_offset = joint->getOffset();
		math::transq local_translation = math::translate_transq( joint_offset );
		math::transq local_rotation = joint_transforms[ joint_index ];
		math::transq local_transform = local_translation * local_rotation;
		
		joint_transform = local_transform * joint_transform;	// child * parent

		joint = joint->getParent();
		if( joint )
		{
			joint_index = joint->getIndex();
		}
	}
	math::transq global_transform = body_transform * joint_transform;
	return global_transform;
}

math::position PoseData::getPosition( Skeleton* skeleton, unsigned int i )
{
	math::position p(0,0,0);
	math::transq t = getGlobalTransform( skeleton, i );
	p *= t;

	return p;
}

math::quater PoseData::getOrientation( Skeleton* skeleton, unsigned int i )
{
	math::transq t = getGlobalTransform( skeleton, i );
	return t.rotation;
}

math::transq PoseData::calcDisplacement( PoseData* p1, PoseData* p2 )
{
	math::transq t1 = p1->getJointTransform( 0 );
	math::transq t2 = p2->getJointTransform( 0 );

	math::vector v1 = t1.translation;
	math::vector v2 = t2.translation;

	math::quater q1 = t1.rotation;
	math::quater q2 = t2.rotation;

	QmGeodesic g1( math::quater(1,0,0,0), math::y_axis );
	QmGeodesic g2( math::quater(1,0,0,0), math::y_axis );

	math::transq T1( g1.nearest( q1 ), math::vector( v1.x(), 0, v1.z() ) );
	math::transq T2( g2.nearest( q2 ), math::vector( v2.x(), 0, v2.z() ) );

	return math::transq( T1 * T2.inverse() );
}
