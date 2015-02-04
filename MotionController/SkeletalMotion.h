#pragma once

#include <string>

#include "mathclass/position.h"
#include "mathclass/vector.h"
#include "mathclass/quater.h"
#include "mathclass/transq.h"

#include "PoseDistance.h"

class Joint;
class Skeleton;

class PoseData;
class MotionData;

class SkeletalMotion
{
public:
	SkeletalMotion();
	virtual ~SkeletalMotion();

	bool importFromBVH( const std::string& path );
	bool exportIntoBVH( const std::string& path );

	void initialize( Skeleton* skeleton, unsigned int num_frames );
	void copyFrom( SkeletalMotion* source_motion );
	void copyFrom( SkeletalMotion* source_motion, unsigned int source_f1, unsigned int source_fN, const math::transq& source_T, unsigned int f );
	void join( SkeletalMotion* motionA, SkeletalMotion* motionB  );
	void concat( SkeletalMotion* source_motion );
	void downsample( unsigned int interval );
	void cleanup();

	unsigned int getNumJoints();
	unsigned int getNumFrames();
	
	Joint* getJointByIndex( unsigned int joint_index );
	Joint* getJointByName( const std::string& joint_name );
	Joint* getHumanJoint( int human_id );
	
	PoseData* getPoseData( unsigned int frame_index );
	void setPoseData( unsigned int frame_index, PoseData* pose_data );

	math::transq getGlobalTransform( unsigned int frame_index, unsigned int joint_index, const math::transq& body_transform = math::identity_transq );
	math::transq getLocalTransform( unsigned int frame_index, unsigned int joint_index );

	math::position	getPosition( unsigned int f, unsigned int i );
	math::quater	getOrientation( unsigned int f, unsigned int i );
	math::vector	getLinearVelocity( unsigned int f, unsigned int i );
	math::vector	getAngularVelocity( unsigned int f, unsigned int i );
	math::vector	getLinearAcceleration( unsigned int f, unsigned int i );
	math::vector	getAngularAcceleration( unsigned int f, unsigned int i );

	void placeFromTrans( unsigned int f, math::transq T, double* x, double* z, double* angle );
	void transFromPlace( unsigned int f, math::transq* T, double x, double z, double angle );

	inline Skeleton*	getSkeleton()	{ return skeleton; }
	inline MotionData*	getMotionData()	{ return motion_data; }

	static double calcDistance( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2 );
	static double calcDistanceDTW( SkeletalMotion* m1, unsigned int s1, unsigned int e1, SkeletalMotion* m2, unsigned int s2, unsigned int e2 );
	static double calcDistanceUniform( SkeletalMotion* m1, unsigned int s1, unsigned int e1, SkeletalMotion* m2, unsigned int s2, unsigned int e2 );
	static double calcDistanceEW( SkeletalMotion* m1, unsigned int s1, unsigned int e1, SkeletalMotion* m2, unsigned int s2, unsigned int e2, double end_weight );
	
	static SkeletalMotion* blend( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2, unsigned int length, double weight );
	static SkeletalMotion* stitch( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2, unsigned int length );
	static SkeletalMotion* stitch( SkeletalMotion* m1, unsigned int f1, math::transq& T1, SkeletalMotion* m2, unsigned int f2, math::transq& T2, unsigned int length );

	static math::transq calcDisplacement( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2 );

	void alignOrientation();

	void precomputeDistance();
	bool loadDistance( const std::string& path );
	bool saveDistance( const std::string& path );

	inline float getDistance( unsigned int f1, unsigned int f2 )	
	{ 
		if( is_distance_precomputed )	
			return pose_distance->get(f1, f2); 
		else
			return SkeletalMotion::calcDistance( this, f1, this, f2 );
	}
	inline bool isDistancePrecomputed()	
	{ 
		return is_distance_precomputed; 
	}

protected:
	Skeleton*		skeleton;
	MotionData*		motion_data;
	PoseDistance*	pose_distance;
	bool			is_distance_precomputed;
};
