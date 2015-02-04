#include "SkeletalMotion.h"
#include "Skeleton.h"
#include "MotionData.h"
#include "PoseData.h"
#include "Joint.h"

#include <iostream>
#include <fstream>
#include <stack>
#include <cassert>

#include "mathclass/transq.h"
#include "mathclass/vector.h"
#include "mathclass/QmGeodesic.h"


SkeletalMotion::SkeletalMotion()
{
	skeleton = 0;
	motion_data = 0;
	pose_distance = 0;
	is_distance_precomputed = false;
}

SkeletalMotion::~SkeletalMotion()
{
	cleanup();
}

void SkeletalMotion::cleanup()
{
	delete skeleton;
	delete motion_data;
	delete pose_distance;

	skeleton = 0;
	motion_data = 0;
	pose_distance = 0;
}

void SkeletalMotion::initialize( Skeleton* source_skeleton, unsigned int num_frames )
{
	if( !source_skeleton )
	{
		return;
	}

	this->cleanup();
	this->skeleton = new Skeleton;
	this->motion_data = new MotionData;

	this->skeleton->copyFrom( source_skeleton );
	this->motion_data->initialize( num_frames, skeleton->getNumJoints() );
}

void SkeletalMotion::copyFrom( SkeletalMotion* source_motion )
{
	Skeleton* source_skeleton = source_motion->getSkeleton();
	MotionData* source_data = source_motion->getMotionData();

	if( !source_skeleton || !source_data )
	{
		return;
	}

	this->cleanup();
	this->skeleton = new Skeleton;
	this->motion_data = new MotionData;
	
	this->skeleton->copyFrom( source_skeleton );
	this->motion_data->copyFrom( source_data );
}

void SkeletalMotion::copyFrom( SkeletalMotion* source_motion, unsigned int source_f1, unsigned int source_fN, const math::transq& source_T, unsigned int f )
{
	Skeleton* source_skeleton = source_motion->getSkeleton();
	MotionData* source_data = source_motion->getMotionData();
	unsigned int source_num_frames = source_motion->getNumFrames();
	unsigned int length = source_fN - source_f1 + 1;

	if( !source_skeleton || !source_data )	return;
	if( this->skeleton->getNumJoints() != source_skeleton->getNumJoints() )	return;
	if( source_f1 >= source_num_frames || source_fN >= source_num_frames )	return;
	if( f + length - 1 >= this->getNumFrames() )	return;

	unsigned int i;
	for( i=0; i < length; i++ )
	{
		unsigned int source_f = source_f1 + i;
		unsigned int this_f = f + i;

		PoseData* source_pose = source_motion->getPoseData( source_f );
		PoseData* this_pose = this->getPoseData( this_f );

		this_pose->copy( source_pose );
		this_pose->transform( source_T );
	}
}

// [ADD LEE START]
void SkeletalMotion::join( SkeletalMotion* motionA, SkeletalMotion* motionB ) 
{
	Skeleton* motionA_skeleton = motionA->getSkeleton();
	MotionData* motionA_data = motionA->getMotionData();

	Skeleton* motionB_skeleton = motionB->getSkeleton();
	MotionData* motionB_data = motionB->getMotionData();

	unsigned int num_frames = motionA_data->getNumFrames() + motionB_data->getNumFrames();

	if( !motionA_skeleton || !motionA_data || !motionB_skeleton || !motionB_data )	return;
	if( motionA_skeleton->getNumJoints() != motionB_skeleton->getNumJoints() )	return;

	this->cleanup();
	this->skeleton = new Skeleton;
	this->motion_data = new MotionData;

	this->skeleton->copyFrom( motionA_skeleton );
	this->motion_data->initialize( num_frames, skeleton->getNumJoints() );

	unsigned int i;
	for( i=0; i < num_frames; i++ )
	{
		PoseData* source_pose;
		if ( i < motionA_data->getNumFrames()) {
			source_pose = motionA->getPoseData( i );
		} else {
			source_pose = motionB->getPoseData( i - motionA_data->getNumFrames());
		}

		this->setPoseData(i, source_pose);
	}
}
// [ADD LEE END]

void SkeletalMotion::concat( SkeletalMotion* source_motion )
{
	if( this->getNumJoints() != source_motion->getNumJoints() )	return;

	MotionData* this_data = this->motion_data;
	MotionData* source_data = source_motion->getMotionData();

	unsigned int num_source_frames = source_data->getNumFrames(), f;
	for( f=0; f < num_source_frames; f++ )
	{
		PoseData* source_pose = source_data->getPoseData( f );
		this_data->addPoseData( source_pose );
	}
}

bool SkeletalMotion::importFromBVH( const std::string &path )
{
	std::ifstream in( path.c_str(), std::ios::in );
	if( !in.is_open() )
	{
		return false;
	}

	cleanup();

	skeleton = new Skeleton;
	motion_data = new MotionData;

	//
	char buffer[1024];
	std::string bufstr;
	
	std::stack< Joint* > stack_joints;
	Joint* current_joint = 0;
	unsigned int num_end_effectors = 0;

	//
	enum {
		XRot = 0,
		YRot,
		ZRot,
		XTrans,
		YTrans,
		ZTrans,
	};
	std::map< Joint*, std::vector< unsigned int >* > joint_channels;

	while( in ) 
	{
		in >> buffer; 
		bufstr = buffer;
	
		if( bufstr == "ROOT" ) {
			in >> buffer;
			bufstr = buffer;

			current_joint = skeleton->createJoint( 0, bufstr );
			continue;
		}
		if( bufstr == "OFFSET" ) {
			double x, y, z;
			in >> x >> y >> z;

			if( current_joint == skeleton->getRootJoint() )	// is this hack necessary ???
			{
				x = y = z = 0;
			}
			current_joint->setOffset( math::vector(x, y, z) );
			continue;
		}
		if( bufstr == "CHANNELS" ) {
			int n;
			char buf2[1024];
			std::string bufstr2;
			in >> n;

			std::vector< unsigned int >* channels = new std::vector< unsigned int >;

			for(int i=0; i < n; ++i) {
				in >> buf2; bufstr2 = buf2;
				if( bufstr2 == "Xrotation" ) channels->push_back( XRot );
				if( bufstr2 == "Yrotation" ) channels->push_back( YRot );
				if( bufstr2 == "Zrotation" ) channels->push_back( ZRot );
	
				if( bufstr2 == "Xposition" ) channels->push_back( XTrans );
				if( bufstr2 == "Yposition" ) channels->push_back( YTrans );
				if( bufstr2 == "Zposition" ) channels->push_back( ZTrans );
			}
			joint_channels[ current_joint ] = channels;

			continue;
		}
		if( bufstr == "JOINT" ) {
			in >> buffer;
			bufstr = buffer;

			current_joint = skeleton->createJoint( current_joint, bufstr );
			continue;
		}
		if(bufstr == "End") {
			in >> buffer;

			char end_name[ 128 ];
			sprintf_s( end_name, "EndEffector%d", num_end_effectors++ );
			std::string name( end_name );

			current_joint = skeleton->createJoint( current_joint, name );
		}
	
		if(bufstr == "{") {
			stack_joints.push( current_joint );
		}
		if(bufstr == "}") {
			stack_joints.pop();
			if( !stack_joints.empty() )
			{
				current_joint = stack_joints.top();
			}
			else
			{
				current_joint = 0;
			}
		}
		
		if(bufstr == "MOTION") {
			unsigned int num_frames;
			double frame_interval;
			in >> buffer;
			in >> num_frames;
			in >> buffer;
			in >> buffer;
			in >> frame_interval;
	
			unsigned int num_joints = skeleton->getNumJoints();
			motion_data->initialize( num_frames, num_joints );

			unsigned int i, f;
			for( f=0; f < num_frames; f++ )
			{
				if( in.fail() )
				{
					std::cout << "ERROR in streaming input\n";
				}

				for( i=0; i < num_joints; i++ )
				{
					math::transq transform = math::identity_transq;

					Joint* joint = skeleton->getJointByIndex( i );

					std::map< Joint*, std::vector< unsigned int >* >::iterator itor_j = joint_channels.find( joint );
					if( itor_j != joint_channels.end() )
					{
						math::transq t = math::identity_transq;

						std::vector< unsigned int >* channels = itor_j->second;
						std::vector< unsigned int >::iterator itor_c = channels->begin();
						while( itor_c != channels->end() )
						{
							unsigned int channel = ( *itor_c ++ );
							double value;
							in >> value;

//							std::cout << value << " ";

							double radian = value * M_PI / 180.0f;

							switch( channel ) {
							case XRot :		t = math::rotate_transq( radian, math::vector(1,0,0) );		break;
							case YRot :		t = math::rotate_transq( radian, math::vector(0,1,0) );		break;
							case ZRot :		t = math::rotate_transq( radian, math::vector(0,0,1) );		break;
							case XTrans :	t = math::translate_transq( math::vector(value, 0, 0) );	break;
							case YTrans :	t = math::translate_transq( math::vector(0, value, 0) );	break;
							case ZTrans :	t = math::translate_transq( math::vector(0, 0, value) );	break;
							}
							transform = transform * t;
						}
					}
					motion_data->setJointTransform( f, i, transform );
				}
//				std::cout << "\n";
			}
		}
	}

	std::map< Joint* , std::vector< unsigned int >* >::iterator itor_j = joint_channels.begin();
	while( itor_j != joint_channels.end() )
	{
		std::vector< unsigned int >* channels = itor_j->second;
		itor_j ++;
		delete channels;
	}

	return true;
}

bool SkeletalMotion::exportIntoBVH( const std::string &path )
{
	if( !skeleton || !motion_data )
	{
		return false;
	}

	std::ofstream out( path.c_str(), std::ios::out );
	if( !out.is_open() )
	{
		return false;
	}

	unsigned int num_frames = motion_data->getNumFrames();
	unsigned int num_joints = motion_data->getNumJoints();
	
	unsigned int f1 = 0, fN = num_frames-1;
	
	if( f1 >= num_frames || fN >= num_frames || f1 > fN )
	{
		return false;
	}

	// SKELETON
	Joint* root = skeleton->getRootJoint();
	std::string root_name = root->getName();
	math::vector root_offset = root->getOffset();

	out << "HIERARCHY\n";
	
	std::vector< Joint* > joint_list;
	std::stack< Joint* > stack_joints;
	std::stack< unsigned int > stack_counts;
	unsigned int depth = 0;

	stack_joints.push( root );
	stack_counts.push( 1 );

	while( !stack_joints.empty() )
	{
		Joint* joint = stack_joints.top();
		stack_joints.pop();

		unsigned int count = stack_counts.top();
		stack_counts.pop();
		stack_counts.push( --count );

		std::string name = joint->getName();
		math::vector offset = joint->getOffset();

		if( joint == root )
		{
			out << "ROOT " << name << "\n";
			out << "{\n";
			out << "OFFSET " << offset.x() << " " << offset.y() << " " << offset.z() << "\n";
			out << "CHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation Xrotation\n";
			joint_list.push_back( joint );
		}
		else
		{
			std::vector< Joint* >* child_joints = joint->getChildren();
			if( !child_joints->empty() )
			{
				out << "JOINT " << name << "\n";
				out << "{\n";
				out << "OFFSET " << offset.x() << " " << offset.y() << " " << offset.z() << "\n";
				out << "CHANNELS 3 Zrotation Yrotation Xrotation\n";
				joint_list.push_back( joint );
			}
			else
			{
				out << "End Site\n";
				out << "{\n";
				out << "OFFSET " << offset.x() << " " << offset.y() << " " << offset.z() << "\n";
				out << "}\n";
			}
		}

		std::vector< Joint* >* child_joints = joint->getChildren();
		if( child_joints->empty() )
		{
			while( stack_counts.size() > 1 && stack_counts.top() == 0 )
			{
				stack_counts.pop();
				out << "}\n";
			}
		}
		else
		{
			unsigned int num_children = (unsigned int)child_joints->size();
			stack_counts.push( num_children );

			unsigned int c;
			for( c=0; c < num_children; c++ )
			{
				Joint* child_joint = ( *child_joints )[ num_children-c-1 ];
				stack_joints.push( child_joint );
			}
		}
	}

	//
	unsigned int num_frames_exported = fN - f1 + 1;
	unsigned int f, j;

	out << "MOTION\n";
	out << "Frames: " << num_frames_exported << "\n";
	out << "Frame Time: 0.033333\n";

	for( f=f1; f <= fN; f++ )
	{
		PoseData* pose = motion_data->getPoseData( f );

		std::vector< Joint* >::iterator itor_j = joint_list.begin();
		while( itor_j != joint_list.end() )
		{
			Joint* joint = ( *itor_j ++ );
			math::transq transform = pose->getJointTransform( joint->getIndex() );
			if( joint == skeleton->getRootJoint() )
			{
				math::vector T = transform.translation;
				out << T.x() << " " << T.y() << " " << T.z() << " ";
			}
			math::quater R = transform.rotation;
			math::vector E = math::Quater2EulerAngle( R ) * 180.0 / M_PI;
			out << E.z() << " " << E.y() << " " << E.x() << " ";
		}
		out << "\n";
	}
	return true;
}

unsigned int SkeletalMotion::getNumJoints()
{
	if( skeleton )
	{
		return skeleton->getNumJoints();
	}
	else
	{
		return 0;
	}
}

unsigned int SkeletalMotion::getNumFrames()
{
	if( motion_data )
	{
		return motion_data->getNumFrames();
	}
	else
	{
		return 0;
	}
}

Joint* SkeletalMotion::getJointByIndex( unsigned int joint_index )
{
	if( skeleton )
	{
		return skeleton->getJointByIndex( joint_index );
	}
	else
	{
		return 0;
	}
}

Joint* SkeletalMotion::getJointByName( const std::string& joint_name )
{
	if( skeleton )
	{
		return skeleton->getJointByName( joint_name );
	}
	else
	{
		return 0;
	}
}

Joint* SkeletalMotion::getHumanJoint( int human_joint_id )
{
	if( skeleton )
	{
		return skeleton->getHumanJoint( human_joint_id );
	}
	else
	{
		return 0;
	}
}

PoseData* SkeletalMotion::getPoseData( unsigned int frame_index )
{
	if( motion_data )
	{
		return motion_data->getPoseData( frame_index );
	}
	else
	{
		return 0;
	}
}

void SkeletalMotion::setPoseData( unsigned int frame_index, PoseData* pose_data )
{
	PoseData* original_pose = getPoseData( frame_index );
	original_pose->copy( pose_data );
}

math::transq SkeletalMotion::getGlobalTransform( unsigned int frame_index, unsigned int joint_index, const math::transq& body_transform /*=math::identity_transq*/)
{
	math::transq joint_transform = math::identity_transq;

	Joint* joint = skeleton->getJointByIndex( joint_index );
	while( joint )
	{
		math::vector joint_offset = joint->getOffset();
		math::transq local_translation = math::translate_transq( joint_offset );
		math::transq local_rotation = motion_data->getJointTransform( frame_index, joint_index );
		math::transq local_transform = local_translation * local_rotation;
		
		joint_transform = local_transform * joint_transform;	// parent * child

		joint = joint->getParent();
		if( joint )
		{
			joint_index = joint->getIndex();
		}
	}
	math::transq global_transform = body_transform * joint_transform;
	return global_transform;
}

math::transq SkeletalMotion::getLocalTransform( unsigned int frame_index, unsigned int joint_index )
{
	math::transq joint_transform = math::identity_transq;

	Joint* joint = skeleton->getJointByIndex( joint_index );
	while( joint && joint->getParent() )
	{
		math::vector joint_offset = joint->getOffset();
		math::transq local_translation = math::translate_transq( joint_offset );
		math::transq local_rotation = motion_data->getJointTransform( frame_index, joint_index );
		math::transq local_transform = local_translation * local_rotation;
		
		joint_transform = local_transform * joint_transform;	// parent * child

		joint = joint->getParent();
		if( joint )
		{
			joint_index = joint->getIndex();
		}
	}
	return joint_transform;
}

void SkeletalMotion::placeFromTrans( unsigned int f, math::transq t, double* x, double* z, double* angle )
{
	Joint* root = skeleton->getRootJoint();
	math::transq T = getGlobalTransform( f, root->getIndex(), t );

	*x = T.translation.x();
	*z = T.translation.z();

	//
	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );
	T.rotation = g.nearest( T.rotation );

	math::vector front(0,0,1);	// ?
	front = math::rotate( T.rotation, front );

	*angle = atan2( front.x(), front.z() );
}

void SkeletalMotion::transFromPlace( unsigned int f, math::transq* t, double x, double z, double angle )
{
	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );

	math::transq trans = math::translate_transq( x, 0, z );
	math::transq rot = math::rotate_transq( angle, math::vector(0,1,0) );
	math::transq desired_transq = trans * rot;

	Joint* root = skeleton->getRootJoint();
	math::transq current_transq = getGlobalTransform( f, root->getIndex(), math::identity_transq );
	current_transq.rotation = g.nearest( current_transq.rotation );

	*t = desired_transq * current_transq.inverse();
	t->rotation = g.nearest( t->rotation );
	t->translation.set_y( 0 );
}

math::position SkeletalMotion::getPosition( unsigned int f, unsigned int i )
{
	math::position p(0,0,0);
	math::transq t = getGlobalTransform( f, i );
	p *= t;

	return p;
}

math::quater SkeletalMotion::getOrientation( unsigned int f, unsigned int i )
{
	math::transq t = getGlobalTransform( f, i );
	return t.rotation;
}

math::vector SkeletalMotion::getLinearVelocity( unsigned int f, unsigned int i )
{
	math::position p1, p2;

	if( f == 0 )
	{
		p1 = getPosition( f+1, i );
		p2 = getPosition( f, i );

		return p1 - p2;
	}

	if( f == this->getNumFrames()-1 )
	{
		p1 = getPosition( f, i );
		p2 = getPosition( f-1, i );

		return p1 - p2;
	}

	p1 = getPosition( f+1, i );
	p2 = getPosition( f-1, i );

	return (p1 - p2)/2.0;
}

math::vector SkeletalMotion::getAngularVelocity( unsigned int f, unsigned int i )
{
	math::quater q1, q2;

	if( f == 0 )
	{
		q1 = getOrientation( f+1, i );
		q2 = getOrientation( f, i );

		if( q1%q2 < 0 ) q2 = -q2;

		return math::ln( q2.inverse() * q1 );
	}

	if( f == this->getNumFrames()-1 )
	{
		q1 = getOrientation( f, i );
		q2 = getOrientation( f-1, i );

		if( q1%q2 < 0 ) q2 = -q2;

		return math::ln( q2.inverse() * q1 );
	}

	q1 = getOrientation( f+1, i );
	q2 = getOrientation( f-1, i );

	if( q1%q2 < 0 ) q2 = -q2;

	return math::ln( q2.inverse() * q1 );
}

math::vector SkeletalMotion::getLinearAcceleration( unsigned int f, unsigned int i )
{
	math::vector v1, v2;

	if( f == 0 )
	{
		v1 = getLinearVelocity( f+1, i );
		v2 = getLinearVelocity( f, i );

		return v1 - v2;
	}

	if( f == this->getNumFrames()-1 )
	{
		v1 = getLinearVelocity( f, i );
		v2 = getLinearVelocity( f-1, i );

		return v1 - v2;
	}

	v1 = getLinearVelocity( f+1, i );
	v2 = getLinearVelocity( f-1, i );

	return (v1 - v2)/2.0;
}

math::vector SkeletalMotion::getAngularAcceleration( unsigned int f, unsigned int i )
{
	math::vector v1, v2;

	if( f == 0 )
	{
		v1 = getAngularVelocity( f+1, i );
		v2 = getAngularVelocity( f, i );

		return v1 - v2;
	}

	if( f == this->getNumFrames()-1 )
	{
		v1 = getAngularVelocity( f, i );
		v2 = getAngularVelocity( f-1, i );

		return v1 - v2;
	}

	v1 = getAngularVelocity( f+1, i );
	v2 = getAngularVelocity( f-1, i );

	return (v1 - v2)/2.0;
}

void SkeletalMotion::alignOrientation()
{
	math::quater q1;
	math::quater q2;

	for( unsigned int j=0; j < skeleton->getNumJoints(); j++ )
	{
		for( unsigned int f=1; f < motion_data->getNumFrames(); f++ )
		{
			math::transq t1 = motion_data->getJointTransform( f-1, j );
			math::transq t2 = motion_data->getJointTransform( f, j );

			math::quater q1 = t1.rotation;
			math::quater q2 = t2.rotation;

			if( q1%q2 < 0 )
			{
				t2.rotation = -q2;

				motion_data->setJointTransform( f, j, t2 );
			}
		}
	}
}

void SkeletalMotion::downsample( unsigned int interval )
{
	if( !motion_data )	return;

	unsigned int num_frames = motion_data->getNumFrames();
	unsigned int num_joints = motion_data->getNumJoints();
	unsigned int num_sampled_frames = num_frames / interval + ( num_frames % interval ? 1 : 0 );
	
	if( num_sampled_frames == 0 )	return;

	MotionData* sampled_motion = new MotionData;
	sampled_motion->initialize( num_sampled_frames, num_joints );

	unsigned int i;
	for( i=0; i < num_sampled_frames; i ++ )
	{
		unsigned int f = i * interval;
		PoseData* pose = motion_data->getPoseData( f );
		sampled_motion->setPoseData( i, pose );
	}

	delete motion_data;
	motion_data = sampled_motion;
}

void SkeletalMotion::precomputeDistance()
{
	if( !motion_data )	return;

	delete pose_distance;
	pose_distance = new PoseDistance;
	
	unsigned int n = motion_data->getNumFrames();
	pose_distance->initialize( n );

	std::cout << "(*) start precomputing pose distances\n";
	std::cout << "- # of frames: " << n << "\n";

	unsigned int i, j;
	for( j=0; j < n-1; j++ )
	{
		for( i=j+1; i < n; i++ )
		{
			double dist = SkeletalMotion::calcDistance( this, i, this, j );
			pose_distance->set( i, j, (float)dist );

			// DEBUG
			std::cout << "- computed the distance of (" << i << ", " << j << ") pair as " << dist << "\n";
		}
	}

	is_distance_precomputed = true;
}

bool SkeletalMotion::loadDistance( const std::string& path )
{
	if( !motion_data )
	{
		return false;
	}

	if( !pose_distance )
	{
		pose_distance = new PoseDistance;
	}

	bool is_loaded = pose_distance->load( path );

	if( !is_loaded )
	{
		delete pose_distance;
		pose_distance = 0;
		is_distance_precomputed = false;
		return false;
	}
	else
	{
		if( motion_data->getNumFrames() != pose_distance->getNumFrames() )
		{
			delete pose_distance;
			pose_distance = 0;
			is_distance_precomputed = false;
			return false;
		}
		else
		{
			is_distance_precomputed = true;
			return true;
		}
	}
}

bool SkeletalMotion::saveDistance( const std::string& path )
{
	if( !pose_distance )
	{
		return false;
	}
	else
	{
		bool is_saved = pose_distance->save( path );
		return is_saved;
	}
}


// STATIC FUNCTIONS

double SkeletalMotion::calcDistance( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2 )
{
	unsigned int num_frames1 = m1->getNumFrames();
	unsigned int num_frames2 = m2->getNumFrames();

	if( f1 >= num_frames1 || f2 >= num_frames2 )
	{
		assert( false );
		return 0;
	}

	if( m1 == m2 && m1->isDistancePrecomputed() )
	{
		 return m1->getDistance( f1, f2 );
	}

	Skeleton* s1 = m1->getSkeleton();
	Skeleton* s2 = m2->getSkeleton();
	MotionData* d1 = m1->getMotionData();
	MotionData* d2 = m2->getMotionData();

	double dist = 0;

	std::vector< Joint* >* joint_list = s1->getJointList();
	std::vector< Joint* >::iterator itor_j = joint_list->begin();
	while( itor_j != joint_list->end() )
	{
		Joint* j1 = ( *itor_j ++ );
		std::string name = j1->getName();

		Joint* j2 = m2->getJointByName( name );
		if( !j2 )
		{
			return DBL_MAX;
		}

		math::transq t1 = d1->getJointTransform( f1, j1->getIndex() );
		math::transq t2 = d2->getJointTransform( f2, j2->getIndex() );

		math::quater q1 = t1.rotation;
		math::quater q2 = t2.rotation;

		dist += math::distance( q1, q2 );
	}
	
	return dist;
}

double SkeletalMotion::calcDistanceUniform( SkeletalMotion* m1, unsigned int s1, unsigned int e1, SkeletalMotion* m2, unsigned int s2, unsigned int e2 )
{
	math::transq disp = SkeletalMotion::calcDisplacement( m1, s1, m2, s2 );

	unsigned int n1 = e1 - s1 + 1;
	unsigned int n2 = e2 - s2 + 1;

	SkeletalMotion* long_m = ( n1 > n2 ? m1 : m2 );
	unsigned int long_s = ( n1 > n2 ? s1 : s2 );
	unsigned int long_e = ( n1 > n2 ? e1 : e2 );

	SkeletalMotion* short_m = ( n1 > n2 ? m2 : m1 );
	unsigned int short_s = ( n1 > n2 ? s2 : s1 );
	unsigned int short_e = ( n1 > n2 ? e2 : e1 );

	unsigned int long_n = long_e - long_s + 1;
	unsigned int short_n = short_e - short_s + 1;
	double ratio = (double)short_n / (double)long_n;

	double dist = 0;
	unsigned int i;

	for( i=0; i < long_n; i++ )
	{
		unsigned int long_f = long_s + i;
		unsigned int short_f = short_s + (unsigned int)( (double)i * ratio );
		if( short_f > short_e )	short_f = short_e;

//		math::transq long_t = long_m->getGlobalTransform( long_f, 0, math::identity_transq );
//		math::transq short_t = short_m->getGlobalTransform( short_f, 0, disp );

//		dist += math::len( long_t.translation - short_t.translation );
		dist += calcDistance( long_m, long_f, short_m, short_f );
	}

	dist /= (double)long_n;

	return dist;
}

double SkeletalMotion::calcDistanceDTW( SkeletalMotion* m1, unsigned int s1, unsigned int e1, SkeletalMotion* m2, unsigned int s2, unsigned int e2 )
{
	unsigned int n1 = e1 - s1 + 1;
	unsigned int n2 = e2 - s2 + 1;

	double* distance_array = new double[ (n1+1)*(n2+1) ];
	char* direction_array = new char[ (n1+1)*(n2+1) ];

#define DTW(i,j)	distance_array[ (j)*(n1+1) + (i) ]
#define DIR(i,j)	direction_array[ (j)*(n1+1) + (i) ]
#define MIN3(i,j,k)	( (i) > (j) ? ( (j) > (k) ? (k) : (j) ) : ( (i) > (k) ? (k) : (i) ) )

	unsigned int i, j, f1, f2;

	for( i=0; i <= n1; i++ )
	{
		DTW(i,0) = DBL_MAX;
	}

	for( j=0; j <= n2; j++ )
	{
		DTW(0,j) = DBL_MAX;
	}
	DTW(0,0) = 0;

	for( i=1; i <= n1; i++ )
	{
		for( j=1; j <= n2; j++ )
		{
			f1 = s1 + i - 1;
			f2 = s2 + j - 1;

			double dist = SkeletalMotion::calcDistance( m1, f1, m2, f2 );
			if( dist == DBL_MAX )
			{
				delete[] distance_array;
				return DBL_MAX;
			}

			double min = MIN3( DTW(i-1,j), DTW(i,j-1), DTW(i-1,j-1) );
			
			DTW(i,j) = dist + min;
			if( min == DTW(i-1,j) )	DIR(i,j) = 'i';
			if( min == DTW(i,j-1) ) DIR(i,j) = 'j';
			if( min == DTW(i-1,j-1) ) DIR(i,j) = 'k';
		}
	}

	i = n1;
	j = n2;

	unsigned int n = 1;
	while( !( i == 1 && j == 1 ) )
	{
		if( DIR(i,j) == 'i' )		i--;
		else if( DIR(i,j) == 'j' )	j--;
		else	{ i--; j--; }
		n ++;
	}

	double dist = DTW(n1, n2) / (double)n;

	delete[] distance_array;
	delete[] direction_array;

	return dist;

#undef MIN3
#undef DTW
}

double SkeletalMotion::calcDistanceEW( SkeletalMotion* m1, unsigned int s1, unsigned int e1, SkeletalMotion* m2, unsigned int s2, unsigned int e2, double end_weight )
{
	double dist_DTW = calcDistanceDTW( m1, s1, e1, m2, s2, e2 );
	double dist_front = calcDistance( m1, s1, m2, s2 );
	double dist_back = calcDistance( m1, e1, m2, e2 );
	double dist_end = ( dist_front + dist_back ) / 2;

	double dist = dist_end * end_weight + dist_DTW * ( 1-end_weight );
	return dist;
}

math::transq SkeletalMotion::calcDisplacement( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2 )
{
	Joint* r1 = m1->getSkeleton()->getRootJoint();
	Joint* r2 = m2->getSkeleton()->getRootJoint();

	math::quater o1 = m1->getOrientation( f1, r1->getIndex() );
	math::position p1 = m1->getPosition( f1, r1->getIndex() );

	math::quater o2 = m2->getOrientation( f2, r2->getIndex() );
	math::position p2 = m2->getPosition( f2, r2->getIndex() );

	QmGeodesic g1( math::quater(1,0,0,0), math::y_axis );
	QmGeodesic g2( math::quater(1,0,0,0), math::y_axis );

	math::transq t1( g1.nearest( o1 ), math::vector( p1[0], 0, p1[2] ) );
	math::transq t2( g2.nearest( o2 ), math::vector( p2[0], 0, p2[2] ) );

	return math::transq( t1 * t2.inverse() );
}

SkeletalMotion* SkeletalMotion::blend( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2, unsigned int length, double weight )
{
	Skeleton* skeleton = m1->getSkeleton();
	if( skeleton != m2->getSkeleton() )		return 0;

	unsigned int L1 = m1->getNumFrames();
	unsigned int L2 = m2->getNumFrames();

	if( f1 > (L1-length) )	return 0;
	if( f2 > (L2-length) )	return 0;

	SkeletalMotion* blend_motion = new SkeletalMotion;
	blend_motion->initialize( skeleton, length );

	unsigned int f;
	for( f=0; f < length; f++ )
	{
		PoseData blend_pose;
		blend_pose.initialize( skeleton->getNumJoints() );
		blend_pose.blend( 
			m1->getPoseData( f1 + f ),
			m2->getPoseData( f2 + f ),
			weight
		);				
		blend_motion->setPoseData( f, &blend_pose );
	}

	blend_motion->alignOrientation();
	return blend_motion;
}

SkeletalMotion* SkeletalMotion::stitch( SkeletalMotion* m1, unsigned int f1, SkeletalMotion* m2, unsigned int f2, unsigned int length )
{
	Skeleton* skeleton = m1->getSkeleton();
	if( skeleton != m2->getSkeleton() )		return 0;

	unsigned int L1 = m1->getNumFrames();
	unsigned int l1 = length;

	unsigned int L2 = m2->getNumFrames();
	unsigned int l2 = length;

	if( f1 < (l1-1) )		return 0;
	if( f2 < (l1) )			return 0;
	
	if( f1 > (L1-l2-1) )	return 0;
	if( f2 > (L2-l2) )		return 0;

	SkeletalMotion* transit_motion = new SkeletalMotion;
	transit_motion->initialize( skeleton, l1 + l2 );

	PoseData blend_pose;
	blend_pose.initialize( skeleton->getNumJoints() );

	unsigned int f;
	for( f=0; f < l1; f++ )
	{
		blend_pose.blend( 
			m1->getPoseData( f1 - (l1-f-1) ),
			m2->getPoseData( f2 - (l1-f) ),
			0.5 * (double)f / (double)(l1-1)
		);				
		transit_motion->setPoseData( f, &blend_pose );
	}
	for( f=0; f < l2; f++ )
	{
		blend_pose.blend(
			m1->getPoseData( f1 + (f+1) ),
			m2->getPoseData( f2 + f ),
			0.5 + 0.5 * (double)f / (double)(l2-1)
		);
		transit_motion->setPoseData( l1+f, &blend_pose );
	}

	transit_motion->alignOrientation();
	return transit_motion;
}

SkeletalMotion* SkeletalMotion::stitch( SkeletalMotion* m1, unsigned int f1, math::transq& T1, SkeletalMotion* m2, unsigned int f2, math::transq& T2, unsigned int length )
{
	Skeleton* skeleton = m1->getSkeleton();
	if( skeleton != m2->getSkeleton() )		return 0;

	unsigned int L1 = m1->getNumFrames();
	unsigned int l1 = length;

	unsigned int L2 = m2->getNumFrames();
	unsigned int l2 = length;

	if( f1 < (l1-1) )		return 0;
	if( f2 < (l1) )			return 0;
	
	if( f1 > (L1-l2-1) )	return 0;
	if( f2 > (L2-l2) )		return 0;

	SkeletalMotion* transit_motion = new SkeletalMotion;
	transit_motion->initialize( skeleton, l1 + l2 );

	PoseData blend_pose;
	blend_pose.initialize( skeleton->getNumJoints() );

	unsigned int f;
	for( f=0; f < l1; f++ )
	{
		blend_pose.blend( 
			m1->getPoseData( f1 - (l1-f-1) ), T1,
			m2->getPoseData( f2 - (l1-f) ), T2,
			0.5 * (double)f / (double)(l1-1)
		);				
		transit_motion->setPoseData( f, &blend_pose );
	}
	for( f=0; f < l2; f++ )
	{
		blend_pose.blend(
			m1->getPoseData( f1 + (f+1) ), T1,
			m2->getPoseData( f2 + f ), T2,
			0.5 + 0.5 * (double)f / (double)(l2-1)
		);
		transit_motion->setPoseData( l1+f, &blend_pose );
	}

	transit_motion->alignOrientation();
	return transit_motion;
}
