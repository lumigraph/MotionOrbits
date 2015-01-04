#include "Character.h"
#include "SkeletalMotion.h"
#include "Skeleton.h"
#include "Joint.h"
#include "MotionData.h"
#include "PoseData.h"
#include "MotionEdit.h"
#include "MotionGraph.h"

#include "mathclass/QmGeodesic.h"

#include <cassert>
#include <deque>

#define DEFAULT_BLEND_LENGTH	5

Character::Character()
{
	skeletal_motion = 0;
	motion_graph = 0;
	
	x = 0;
	z = 0;
	angle = 0;

	frame = 0;
	f1 = 0;
	fN = 0;

	transform = math::identity_transq;

	blend_length = DEFAULT_BLEND_LENGTH;
	is_in_blend = false;
	blend_frame = 0;
	blend_motion = 0;
}

Character::~Character()
{
	clearPath();
}

Character* Character::clone()
{
	Character* character = new Character;

	character->skeletal_motion = this->skeletal_motion;
	character->motion_graph = this->motion_graph;
	
	character->x = this->x;
	character->z = this->z;
	character->angle = this->angle;
	character->transform = this->transform;

	character->frame = this->frame;
	character->f1 = this->f1;
	character->fN = this->fN;

	character->is_in_blend = this->is_in_blend;
	character->blend_length = this->blend_length;
	character->blend_frame = this->blend_frame;
	if( this->blend_motion )
	{
		character->blend_motion = new SkeletalMotion;
		character->blend_motion->copyFrom( this->blend_motion );
	}
	else
	{
		character->blend_motion = 0;
	}

	std::deque< MotionGraph::Node* >::iterator itor_n = this->node_path.begin();
	while( itor_n != this->node_path.end() )
	{
		MotionGraph::Node* node = ( *itor_n ++ );
		character->node_path.push_back( node );
	}

	std::deque< unsigned int >::iterator itor_s = this->segment_path.begin();
	while( itor_s != this->segment_path.end() )
	{
		unsigned int segment = ( *itor_s ++ );
		character->segment_path.push_back( segment );
	}

	return character;
}

math::transq Character::getGlobalTransform( unsigned int joint_index )
{
	if( is_in_blend )
	{
		return blend_motion->getGlobalTransform( blend_frame, joint_index, transform );
	}
	else
	{
		return skeletal_motion->getGlobalTransform( frame, joint_index, transform );
	}
}

PoseData* Character::getCurrentPose()
{
	if( is_in_blend )
	{
		return blend_motion->getPoseData( blend_frame );
	}
	else
	{
		return skeletal_motion->getPoseData( frame );
	}
}

void Character::embody( SkeletalMotion* m, MotionGraph* g )
{
	skeletal_motion = m;
	motion_graph = g;
	
	x = 0;
	z = 0;
	angle = 0;

	frame = 0;
	f1 = 0;
	fN = 0;

	transform = math::identity_transq;

	clearPath();
}

void Character::clearPath()
{
	node_path.clear();
	segment_path.clear();

	delete blend_motion;
	blend_motion = 0;
	blend_frame = 0;
	is_in_blend = false;
}

unsigned int Character::extendPath( MotionGraph::Node* to_node )
{
	if( !skeletal_motion || !motion_graph )
	{
		return 0;
	}

	bool hasNode = motion_graph->hasNode( to_node );
	if( !hasNode )
	{
		return 0;
	}

	unsigned int path_len = getNodePathLength();
	unsigned int k = 0;

	if( path_len == 0 )
	{
		node_path.push_back( to_node );
		k = 1;
	}
	else
	{
		MotionGraph::Node* from_node = getPathNode( path_len-1 );

		std::vector< MotionGraph::Edge* >* edges = motion_graph->findEdges( from_node, to_node );
		if( edges )
		{
			node_path.push_back( to_node );
			k = 1;
		}
		else
		{
			std::deque< MotionGraph::Node* > path;
			bool is_path_found = motion_graph->findPath( from_node, to_node, &path );
			if( !is_path_found )
			{
				return false;
			}

			std::deque< MotionGraph::Node* >::iterator itor_n = path.begin();
			itor_n ++;	// skip the 'from' node that is already in the 'node path'

			while( itor_n != path.end() )
			{
				MotionGraph::Node* node = ( *itor_n ++ );
				node_path.push_back( node );
				k ++;
			}
		}
	}
	connectSegments();

	return k;
}

unsigned int Character::extendPathRandomly( unsigned int k )
{
	if( !skeletal_motion || !motion_graph )
	{
		return 0;
	}

	unsigned int i;
	for( i=0; i < k; i++ )
	{
		unsigned int path_len = getNodePathLength();

		if( path_len == 0 )
		{
			unsigned int num_nodes = motion_graph->getNumNodes();
			if( num_nodes == 0 )
			{
				break;
			}

			unsigned int node_index = rand() % num_nodes;
			MotionGraph::Node* to_node = motion_graph->getNode( node_index );
			node_path.push_back( to_node );
		}
		else
		{
			MotionGraph::Node* from_node = getPathNode( path_len-1 );

			std::vector< MotionGraph::Edge* >* edges = from_node->getNextEdges();
			unsigned int num_edges = (unsigned int)edges->size();
			if( num_edges == 0 )
			{
				break;
			}

			unsigned int edge_index = rand() % num_edges;
			MotionGraph::Node* to_node = ( *edges )[ edge_index ]->getToNode();
			node_path.push_back( to_node );
		}
	}
	connectSegments();

	return i;
}

unsigned int Character::shortenPath( unsigned int k )
{
	if( !skeletal_motion || !motion_graph )
	{
		return 0;
	}

	unsigned int path_len = getNodePathLength();
	if( path_len < 2 )
	{
		return 0;
	}

	if( k > path_len-1 )
	{
		k = path_len-1;
	}

	unsigned int i;
	for( i=0; i < k; i++ )
	{
		node_path.pop_back();
		segment_path.pop_back();
	}

	return k;
}

void Character::place( double x, double z, double angle )
{
	if( !skeletal_motion || !motion_graph )
	{
		return;
	}
	unsigned int node_path_len = getNodePathLength();
	unsigned int segment_path_len = getSegmentPathLength();

	if( node_path_len == 0 || segment_path_len == 0 )
	{
		return;
	}

	transFromPlace( skeletal_motion, frame, &transform, x, z, angle );

	this->x = x;
	this->z = z;
	this->angle = angle;
}

bool Character::update()
{
	if( !skeletal_motion || !motion_graph )
	{
		return false;
	}

	unsigned int path_len = getNodePathLength();
	if( path_len == 0 )
	{
		return false;
	}

	if( frame < fN )
	{
		proceedToNextFrame();
	}
	else
	{
		proceedToNextNode();
	}
	updateBlendFrame();
	updatePlacement();

	return true;
}

void Character::proceedToNextFrame()
{
	frame ++;
}

void Character::proceedToNextNode()
{
	if( !skeletal_motion || !motion_graph )
	{
		return;
	}

	unsigned int node_path_len = getNodePathLength();
	unsigned int segment_path_len = getSegmentPathLength();

	if( node_path_len < 2 || segment_path_len < 2 )
	{
		return;
	}

	placeFromTrans( skeletal_motion, frame, transform, &x, &z, &angle );

	node_path.pop_front();
	segment_path.pop_front();

	MotionGraph::Node* node = node_path[ 0 ];
	unsigned int segment_index = segment_path[ 0 ];	
	std::pair<unsigned int, unsigned int> segment = node->getSegment( segment_index );

	f1 = segment.first;
	fN = segment.second;
	frame = f1;

	transFromPlace( skeletal_motion, frame, &transform, x, z, angle );
}

void Character::whereToGo( double* to_x, double* to_z, double* to_angle, MotionGraph::Node* to_node )
{
	unsigned int path_len, i;
	
	if( to_node )
	{
		extendPath( to_node );
	}

	path_len = getNodePathLength();
	placeFromTrans( skeletal_motion, fN, transform, to_x, to_z, to_angle );

	unsigned int to_f1, to_fN;
	math::transq to_transform;

	for( i=0; i < path_len; i++ )
	{
		MotionGraph::Node* node = node_path[ i ];
		unsigned int segment_index = segment_path[ i ];
		std::pair<unsigned int, unsigned int> segment = node->getSegment( segment_index );
		
		to_f1 = segment.first;
		to_fN = segment.second;

		transFromPlace( skeletal_motion, to_f1, &to_transform, *to_x, *to_z, *to_angle );
		placeFromTrans( skeletal_motion, to_fN, to_transform, to_x, to_z, to_angle );
	}

	if( to_node )
	{
		shortenPath( 1 );
	}
}

// ASSUMPTION: ( fN-f1+1 ) > blend_length * 2
void Character::updateBlendFrame()
{
	unsigned int num_frames_before = frame - f1 + 1;
	unsigned int num_frames_after = fN - frame + 1;

	if( blend_motion )
	{
		if( num_frames_before <= blend_length )
		{
			is_in_blend = true;
			blend_frame = blend_length + num_frames_before - 1;
		}
		else if( num_frames_after <= blend_length )
		{
			is_in_blend = true;
			blend_frame = blend_length - num_frames_after;
		}
		else if( num_frames_before == blend_length+1 )
		{
			is_in_blend = false;
			delete blend_motion;
			blend_motion = 0;
		}
		else
		{
			assert( false );
		}
	}
	else
	{
		is_in_blend = false;

		if( num_frames_after == blend_length+1 )
		{
			blendMotions();
		}
	}
}

void Character::updatePlacement()
{
	if( !skeletal_motion || !motion_graph )
	{
		return;
	}
	unsigned int node_path_len = getNodePathLength();
	unsigned int segment_path_len = getSegmentPathLength();

	if( node_path_len == 0 || segment_path_len == 0 )
	{
		return;
	}
	
	if( !is_in_blend )
	{
		placeFromTrans( skeletal_motion, frame, transform, &x, &z, &angle );
	}
	else
	{
		placeFromTrans( blend_motion, blend_frame, transform, &x, &z, &angle );
	}
}

void Character::placeFromTrans( SkeletalMotion* m, unsigned int f, math::transq t, double* x, double* z, double* angle )
{
	Skeleton* skeleton = m->getSkeleton();
	Joint* root = skeleton->getRootJoint();
	math::transq T = m->getGlobalTransform( f, root->getIndex(), t );

	*x = T.translation.x();
	*z = T.translation.z();

	//
	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );
	T.rotation = g.nearest( T.rotation );

	math::vector front(0,0,1);	// ?
	front = math::rotate( T.rotation, front );

	*angle = atan2( front.x(), front.z() );
}

void Character::transFromPlace( SkeletalMotion* m, unsigned int f, math::transq* t, double x, double z, double angle )
{
	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );

	math::transq trans = math::translate_transq( x, 0, z );
	math::transq rot = math::rotate_transq( angle, math::vector(0,1,0) );
	math::transq desired_transq = trans * rot;

	Skeleton* skeleton = m->getSkeleton();
	Joint* root = skeleton->getRootJoint();

	math::transq current_transq = m->getGlobalTransform( f, root->getIndex(), math::identity_transq );
	current_transq.rotation = g.nearest( current_transq.rotation );

	*t = desired_transq * current_transq.inverse();
	t->rotation = g.nearest( t->rotation );
	t->translation.set_y( 0 );
}

void Character::connectSegments()
{
	unsigned int node_path_len = getNodePathLength();
	unsigned int segment_path_len = getSegmentPathLength();

	if( node_path_len > segment_path_len )
	{
		unsigned int num_segments_to_add = node_path_len - segment_path_len, i;
		for( i=0; i < num_segments_to_add; i++ )
		{
			if( segment_path_len+i == 0 )
			{
				MotionGraph::Node* node = node_path[ segment_path_len+i ];
				std::vector< std::pair<unsigned int, unsigned int> >* segments = node->getSegments();
				unsigned int num_segments = (unsigned int)segments->size();

				unsigned int index = (unsigned int)( (double)rand() / (double)RAND_MAX * num_segments );
				if( index == num_segments )	index --;

				segment_path.push_back( index );
			}
			else
			{
				MotionGraph::Node* curr_node = node_path[ segment_path_len+i-1 ];
				unsigned int curr_segment_index = segment_path[ segment_path_len+i-1 ];
				std::pair<unsigned int, unsigned int> curr_segment = curr_node->getSegment( curr_segment_index );

				MotionGraph::Node* next_node = node_path[ segment_path_len+i ];
				std::vector< std::pair<unsigned int, unsigned int> >* next_segments = next_node->getSegments();	
				unsigned int num_next_segments = (unsigned int)next_segments->size(), j;
			
				double min_dist = DBL_MAX;
				unsigned int min_index = 0;

				for( j=0; j < num_next_segments; j++ )
				{
					std::pair<unsigned int, unsigned int> next_segment = next_node->getSegment( j );

					double dist = SkeletalMotion::calcDistance( 
						skeletal_motion, curr_segment.second,
						skeletal_motion, next_segment.first
						);

					if( dist < min_dist )
					{
						min_dist = dist;
						min_index = j;
					}
				}
		
				segment_path.push_back( min_index );
			}
		}
	}
}

void Character::blendMotions()
{
	unsigned int path_len = getNodePathLength();
	if( path_len > 1 )
	{
		MotionGraph::Node* next_node = node_path[ 1 ];
		unsigned int next_segment_index = segment_path[ 1 ];
		std::pair<unsigned int, unsigned int> next_segment = next_node->getSegment( next_segment_index );

		unsigned int curr_f1 = f1;
		unsigned int curr_fN = fN;
		unsigned int curr_L = curr_fN - curr_f1 + 1;
		math::transq curr_T = transform;

		unsigned int next_f1 = next_segment.first;
		unsigned int next_fN = next_segment.second;
		unsigned int next_L = next_fN - next_f1 + 1;
		math::transq next_T = math::identity_transq;

		double int_x = 0, int_z = 0, int_angle = 0;
		placeFromTrans( skeletal_motion, curr_fN, curr_T, &int_x, &int_z, &int_angle );
		transFromPlace( skeletal_motion, next_f1, &next_T, int_x, int_z, int_angle );
	
		if( curr_L > blend_length*2 && next_L > blend_length*2 )
		{
			delete blend_motion;
			blend_motion = 0;

			Skeleton* skeleton = skeletal_motion->getSkeleton();
			blend_motion = new SkeletalMotion;
			blend_motion->initialize( skeleton, blend_length*2 );

			unsigned int f;
			for( f=0; f < blend_length; f++ )
			{
				PoseData blend_pose;
				blend_pose.initialize( skeleton->getNumJoints() );
				if( next_f1 < blend_length )
				{
					blend_pose.copy( 
						skeletal_motion->getPoseData( curr_fN - (blend_length-f-1) )
					);
				}
				else
				{
					blend_pose.blend( 
						skeletal_motion->getPoseData( curr_fN - (blend_length-f-1) ),
						skeletal_motion->getPoseData( next_f1 - (blend_length-f) ),
						0.5 * (double)f / (double)(blend_length-1)
					);				
				}
				blend_motion->setPoseData( f, &blend_pose );
			}
			for( f=0; f < blend_length; f++ )
			{
				PoseData blend_pose;
				blend_pose.initialize( skeleton->getNumJoints() );
				if( curr_fN + blend_length >= skeletal_motion->getNumFrames() )
				{
					blend_pose.copy(
						skeletal_motion->getPoseData( next_f1 + f )
					);
				}
				else
				{
					blend_pose.blend(
						skeletal_motion->getPoseData( next_f1 + f ),
						skeletal_motion->getPoseData( curr_fN + f + 1 ),
						0.5 - 0.5 * (double)f / (double)(blend_length-1)
					);
				}				
				blend_motion->setPoseData( blend_length+f, &blend_pose );
			}

			blend_motion->alignOrientation();
		}
	}
}
