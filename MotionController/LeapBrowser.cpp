#include "LeapBrowser.h"

#include "LeapListener.h"

#include "SkeletalMotion.h"
#include "MotionData.h"
#include "Skeleton.h"
#include "Joint.h"
#include "Human.h"

#include "PoseData.h"
#include "PoseConstraint.h"
#include "PoseIK.h"

#include "Character.h"
#include "SearchTree.h"
#include "SpacePartition.h"

#include "mathclass/QmGeodesic.h"

#include <GL/glut.h>
#include "DrawingTool.h"

extern DrawingTool drawing_tool;

using namespace Leap;


static const int DEFAULT_NUM_NEIGHBORS_TO_TRACK = 10;
static const int DEFAULT_NUM_POSES_TO_BLEND = 10;

static const int DEFAULT_NUM_EXTENDS_PER_UPDATE = 1;
static const int DEFAULT_NUM_TRACKS_PER_UPDATE = 1;

static const int DEFAULT_TARGET_HUMAN_JOINT = Human::RIGHT_PALM;

static const double DEFAULT_LOOKAT_HEIGHT = 50.0;
static const double DEFAULT_LEAP_HEIGHT = -50.0;

static const int DEFAULT_LOOKAT_ANIMATION_DURATION = 30;


LeapBrowser::LeapBrowser()
{
//	leap_listener = new LeapListener;
//	leap_controller = new Controller;
//	leap_controller.addListener( *leap_listener );

	leap_transform = math::identity_transq;
	cursor_location = math::position(0,0,0);

	motion_data = 0;
	motion_graph = 0;
	search_tree = 0;
	character = 0;

	target_human_joint = Human::UNDEFINED;
	target_joint_index = (unsigned int)-1;

	current_lookat = math::position(0,0,0);
	original_lookat = math::position(0,0,0);
	desired_lookat = math::position(0,0,0);

	is_tracking_pose = false;
	is_animating_character = false;
	is_modifying_lookat = false;
	is_cursor_located = false;

	tracked_node = 0;
	tracked_frame = 0;
	
	blended_pose = 0;
	edited_pose = 0;

	sequenced_motion = 0;
	played_back_motion = 0;

	//
	num_neighbors_to_track = DEFAULT_NUM_NEIGHBORS_TO_TRACK;
	num_poses_to_blend = DEFAULT_NUM_POSES_TO_BLEND;

	num_extends_per_update = DEFAULT_NUM_EXTENDS_PER_UPDATE;
	num_tracks_per_update = DEFAULT_NUM_TRACKS_PER_UPDATE;

	//
	lookat_animation_duration = DEFAULT_LOOKAT_ANIMATION_DURATION;
	lookat_animation_frame = 0;
}

LeapBrowser::~LeapBrowser()
{
	finalize();

	//
//	leap_controller.removeListener( *leap_listener );
//	delete leap_controller;
//	delete leap_listener;
}

void LeapBrowser::initialize( const std::string& path_motion, const std::string& path_graph, void (*setup_skeleton)( Skeleton* ), float x, float z, float angle, int node_index/*=0*/ )
{
	motion_data = new SkeletalMotion;
	motion_data->importFromBVH( path_motion );

	motion_graph = new MotionGraph;
	motion_graph->load( path_graph );

	setup_skeleton( motion_data->getSkeleton() );

	target_human_joint = DEFAULT_TARGET_HUMAN_JOINT;
	target_joint_index = motion_data->getHumanJoint( target_human_joint )->getIndex();

	//
	character = new Character;
	character->embody( motion_data, motion_graph );
	
	MotionGraph::Node* start_node = motion_graph->getNode( node_index );
	if( start_node )
	{
		character->extendPath( start_node );
	}
	else
	{
		character->extendPathRandomly( 1 );
	}
	character->place( x, z, angle );

	//
	search_tree = new SearchTree;
	search_tree->initialize( motion_data, motion_graph, character->getPathNode(0), x, z, angle, 0, target_human_joint );

	//
	current_lookat = math::position( x, DEFAULT_LOOKAT_HEIGHT, z );
	original_lookat = math::position( 0, 0, 0 );
	desired_lookat = math::position( 0, 0, 0 );

	leap_transform = math::transq( math::quater(1,0,0,0), math::vector(x, DEFAULT_LEAP_HEIGHT, z) );
	cursor_location = math::position( 0, 0, 0 );

	//
	is_tracking_pose = true;
	is_animating_character = false;
	is_modifying_lookat = false;
	is_cursor_located = false;
}

void LeapBrowser::finalize()
{
	delete sequenced_motion;
	delete played_back_motion;

	sequenced_motion = 0;
	played_back_motion = 0;

	//
	delete blended_pose;
	delete edited_pose;

	blended_pose = 0;
	edited_pose = 0;

	tracked_node = 0;
	tracked_frame = 0;

	neighbor_nodes.clear();
	neighbor_poses.clear();

	motion_sequence.clear();

	//
	delete search_tree;
	delete character;
	delete motion_graph;
	delete motion_data;

	search_tree = 0;
	character = 0;
	motion_graph = 0;
	motion_data = 0;

	//
	is_tracking_pose = false;
	is_animating_character = false;
	is_modifying_lookat = false;
	is_cursor_located = false;
}

void LeapBrowser::update()
{
	static unsigned int count = 0;

	updateCharacter();
	updateCursorLocation();
	updateLookAt();

	if( is_tracking_pose && is_cursor_located )
	{
		if( count % num_extends_per_update == 0 )
		{
			updateSearchTree();
		}
		if( count % num_tracks_per_update == 0 )
		{
			updateTrackedPose();
			blendNeighborPoses();
			editBlendedPose();
		}
	}
	count ++;
}

void LeapBrowser::startAnimation()
{
	is_animating_character = true;
	is_tracking_pose = false;

	//
	delete played_back_motion;
	played_back_motion = 0;

	played_back_motion = new SkeletalMotion;
	played_back_motion->initialize( motion_data->getSkeleton(), 0 );
}

void LeapBrowser::endAnimation()
{
	is_animating_character = false;
	is_tracking_pose = true;
}

void LeapBrowser::moveToNext()
{
	if( tracked_node )
	{
		std::deque< MotionGraph::Node* > graph_path;

		SearchTree::Node* node = tracked_node;
		while( node )
		{
			MotionGraph::Node* graph_node = node->getGraphNode();
			graph_path.push_front( graph_node );
			node = node->getParent();
		}

		character->clearPath();
		std::deque< MotionGraph::Node* >::iterator itor_n = graph_path.begin();
		while( itor_n != graph_path.end() )
		{
			MotionGraph::Node* graph_node = ( *itor_n ++ );
			character->extendPath( graph_node );

			//
			motion_sequence.push_back( graph_node );
		}

		startAnimation();
	}
}

void LeapBrowser::replayFromFirst()
{
	if( !motion_sequence.empty() )
	{
		character->clearPath();

		std::vector< MotionGraph::Node* >::iterator itor_n = motion_sequence.begin();
		while( itor_n != motion_sequence.end() )
		{
			MotionGraph::Node* node = ( *itor_n ++ );
			character->extendPath( node );
		}
		character->place( 0, 0, 0 );		// initial x, z, angle?

		modifyLookAt( math::position(0, current_lookat.y(), 0) );

		//
		startAnimation();
	}
}

void LeapBrowser::sequenceMotion()
{
	if( !motion_sequence.empty() )
	{
		delete sequenced_motion;
		sequenced_motion = 0;

		Character temp_char;
		temp_char.embody( motion_data, motion_graph );

		std::vector< MotionGraph::Node* >::iterator itor_n = motion_sequence.begin();
		while( itor_n != motion_sequence.end() )
		{
			MotionGraph::Node* node = ( *itor_n ++ );
			temp_char.extendPath( node );
		}
		temp_char.place( 0, 0, 0 );			// initial x, z, angle?

		//
		sequenced_motion = new SkeletalMotion;
		sequenced_motion->initialize( motion_data->getSkeleton(), 0 );

		bool is_updated = false;
		do
		{
			PoseData* pose = temp_char.getCurrentPose();
			math::transq T = temp_char.getTransform();

			sequenced_motion->getMotionData()->addPoseData( pose );
			PoseData* new_pose = sequenced_motion->getPoseData( sequenced_motion->getNumFrames()-1 );
			new_pose->setJointTransform( 0, T * new_pose->getJointTransform(0) );

			is_updated = temp_char.update();
		}
		while( is_updated );
	}
}

void LeapBrowser::exportSequencedMotion( const std::string& path_sequenced )
{
	if( sequenced_motion )
	{
		sequenced_motion->exportIntoBVH( path_sequenced );
	}
}

//
void LeapBrowser::updateCharacter()
{
	if( !is_animating_character )
	{
		return;
	}

	bool is_updated = character->update();
	if( !is_updated )
	{
		double x = character->getX();
		double z = character->getZ();
		double angle = character->getAngle();

		//
		MotionGraph::Node* prev_node = character->getPathNode(0);
				
		std::vector< MotionGraph::Edge* >* next_edges = prev_node->getNextEdges();
		int num_next_edges = (int)next_edges->size();
		int edge_index = rand() % num_next_edges;
				
		MotionGraph::Edge* next_edge = ( *next_edges )[ edge_index ];
		MotionGraph::Node* next_node = next_edge->getToNode();

		//
		character->clearPath();
		character->extendPath( next_node );
		character->place( x, z, angle );

		resetSearchTree();

		//
		modifyLookAt( math::position(x, current_lookat.y(), z) );

		//
		endAnimation();
	}
	else
	{
		PoseData* pose = character->getCurrentPose();
		math::transq T = character->getTransform();

		played_back_motion->getMotionData()->addPoseData( pose );
		PoseData* new_pose = played_back_motion->getPoseData( played_back_motion->getNumFrames()-1 );
		new_pose->setJointTransform( 0, T * new_pose->getJointTransform(0) );
	}
}

void LeapBrowser::updateCursorLocation()
{
	bool is_index_found = false;

	Frame frame = leap_controller.frame();	
	HandList hands = frame.hands();
	for( int h=0; h < hands.count(); h++ )
	{
		Hand hand = hands[h];
		FingerList fingers = hand.fingers();
		for( int f=0; f < fingers.count(); f++ )
		{
			Finger finger = fingers[f];
			if( finger.type() == Finger::Type::TYPE_INDEX )
			{
				is_index_found = true;

				Vector position = finger.tipPosition();
				cursor_location = math::position(position.x, position.y, position.z) * leap_transform;

				break;
			}
		}
		if( is_index_found )
		{
			break;
		}
	}

	if( !is_index_found )
	{
		cursor_location = math::position(0,0,0);
		is_cursor_located = false;
	}
	else
	{
		is_cursor_located = true;
	}
}

inline static bool sort_neighbor_poses( std::pair< std::pair<SearchTree::Node*, int>, double >& lhs, std::pair< std::pair<SearchTree::Node*, int>, double>& rhs )
{
	return ( lhs.second < rhs.second );
}

void LeapBrowser::updateSearchTree()
{
	if( !search_tree ) 
	{
		return;
	}

	search_tree->extend( cursor_location );
}

void LeapBrowser::updateTrackedPose()
{
	neighbor_nodes.clear();
	neighbor_poses.clear();

	//
	search_tree->kNN( cursor_location.x(), cursor_location.z(), num_neighbors_to_track, &neighbor_nodes );

	math::transq old_T;
	int old_f = 0;

	if( tracked_node )
	{
		old_T = tracked_node->getTransform();
		old_f = tracked_frame;
	}

	//
	double min_dist = +DBL_MAX;
	SearchTree::Node* min_node = 0;
	int min_frame = 0;

	int num_neighbors_found = (int)neighbor_nodes.size();
	for( int k=0; k < num_neighbors_found; k++ )
	{
		SearchTree::Node* node = neighbor_nodes[ k ];
		math::transq T = node->getTransform();

		int f1 = node->getGraphNode()->getSegment(0).first;
		int fN = node->getGraphNode()->getSegment(0).second;
			
		for( int f=f1; f <= fN; f++ )
		{
			math::position P = search_tree->calcTrackPoint( node, f );
			double d = math::vector( cursor_location-P ).length();

			double dist = +DBL_MAX;

			if( tracked_node )
			{
				double effort = calcEffortForMovement( old_T, old_f, T, f );
				double alpha = 1/d;
				dist = d + alpha * effort;
			}
			else
			{
				dist = d;
			}

			if( dist < min_dist )
			{
				min_dist = dist;
				min_node = node;
				min_frame = f;
			}
			//
			neighbor_poses.push_back( std::make_pair( std::make_pair(node, f), dist ) );
		}
	}
	tracked_node = min_node;
	tracked_frame = min_frame;

	//
//	std::sort( neighbor_poses.begin(), neighbor_poses.end(), sort_neighbor_poses );
}

void LeapBrowser::blendNeighborPoses()
{
	if( neighbor_poses.empty() )
	{
		return;
	}

	delete blended_pose;
	blended_pose = 0;
	
	blended_pose = new PoseData;
	blended_pose->initialize( motion_data->getNumJoints() );
	
	if( neighbor_poses.size() >= num_poses_to_blend )
	{
		std::vector< std::pair<PoseData*, math::transq> > poses;
		std::vector< double > ratios;

		for( int i=0; i < num_poses_to_blend; i++ )
		{
			SearchTree::Node* node = neighbor_poses[i].first.first;
			int f = neighbor_poses[i].first.second;
			double dist = neighbor_poses[i].second;

			PoseData* pose = motion_data->getPoseData( f );
			math::transq T = node->getTransform();
			double ratio = 1.0 / dist;

			poses.push_back( std::make_pair(pose, T) );
			ratios.push_back( ratio );
		}
		blended_pose->blend( &poses, &ratios );
	}
	else
	{
		SearchTree::Node* nearest_node = neighbor_poses[0].first.first;
		int nearest_f = neighbor_poses[0].first.second;

		blended_pose->copy( motion_data->getPoseData( nearest_f ) );
		blended_pose->setJointTransform( 0, nearest_node->getTransform() * blended_pose->getJointTransform(0) );
	}
}

void LeapBrowser::editBlendedPose()
{
	if( !blended_pose )
	{
		return;
	}

	delete edited_pose;
	edited_pose = 0;

	edited_pose = new PoseData;
	edited_pose->initialize( motion_data->getNumJoints() );
	edited_pose->copy( blended_pose );

	unsigned int left_hand = motion_data->getSkeleton()->getHumanJoint( Human::LEFT_PALM )->getIndex();
	unsigned int right_hand = motion_data->getSkeleton()->getHumanJoint( Human::RIGHT_PALM )->getIndex();
	unsigned int left_foot = motion_data->getSkeleton()->getHumanJoint( Human::LEFT_FOOT )->getIndex();
	unsigned int right_foot = motion_data->getSkeleton()->getHumanJoint( Human::RIGHT_FOOT )->getIndex();

	math::transq lh_transq = edited_pose->getGlobalTransform( motion_data->getSkeleton(), left_hand );
	math::transq rh_transq = edited_pose->getGlobalTransform( motion_data->getSkeleton(), right_hand );
	math::transq lf_transq = edited_pose->getGlobalTransform( motion_data->getSkeleton(), left_foot );
	math::transq rf_transq = edited_pose->getGlobalTransform( motion_data->getSkeleton(), right_foot );

	PoseConstraint pose_constraint;
	pose_constraint.addConstraint( Human::LEFT_PALM, lh_transq );
	pose_constraint.addConstraint( Human::RIGHT_PALM, rh_transq );
	pose_constraint.addConstraint( Human::LEFT_FOOT, lf_transq );
	pose_constraint.addConstraint( Human::RIGHT_FOOT, rf_transq );

	//
	pose_constraint.removeConstraint( target_human_joint );
	pose_constraint.addConstraint( target_human_joint, math::vector(cursor_location.x(), cursor_location.y(), cursor_location.z()) );

	//
	PoseIK pose_ik;
	pose_ik.initialize();
	pose_ik.ik_body( motion_data->getSkeleton(), blended_pose, edited_pose, &pose_constraint, 0.0 );
}

double LeapBrowser::calcEffortForMovement( math::transq& T1, int f1, math::transq& T2, int f2 )
{
	math::transq root_T1 = motion_data->getGlobalTransform( f1, 0, T1 );
	math::transq root_T2 = motion_data->getGlobalTransform( f2, 0, T2 );

	math::vector root_trans1 = root_T1.translation;
	math::vector root_trans2 = root_T2.translation;
	double root_trans_dist = math::vector( root_trans1 - root_trans2 ).length();

	math::quater root_rot1 = root_T1.rotation;
	math::quater root_rot2 = root_T2.rotation;

	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );
	math::quater root_rotY1 = g.nearest( root_rot1 );
	math::quater root_rotY2 = g.nearest( root_rot2 );
	double root_rot_dist = math::distance( root_rotY1, root_rotY2 );

	//
	math::transq root_t1( root_rotY1, root_trans1 );
	math::transq root_t2( root_rotY2, root_trans2 );
	math::transq disp( root_t1 * root_t2.inverse() );

	double sum_joint_dists = 0.0;
	int num_joints = motion_data->getNumJoints();
	for( int j=1; j < num_joints; j++ )
	{
		math::vector joint_trans1 = motion_data->getGlobalTransform( f1, j ).translation;
		math::vector joint_trans2 = motion_data->getGlobalTransform( f2, j, disp ).translation;
		double joint_trans_dist = math::vector( joint_trans1 - joint_trans2 ).length();
		sum_joint_dists += joint_trans_dist;
	}
	double avg_joint_dists = sum_joint_dists / ( num_joints-1 );

	double root_trans_weight = 1.0;
	double root_rot_weight = 0.1;
	double joint_trans_weight = 1.0;

	double dist = root_trans_dist * root_trans_weight + root_rot_dist * root_rot_weight + avg_joint_dists * joint_trans_weight;
	return dist;
}

void LeapBrowser::resetSearchTree()
{
	MotionGraph::Node* node = character->getPathNode( 0 );
	double x = character->getX();
	double z = character->getZ();
	double angle = character->getAngle();

	search_tree->initialize( motion_data, motion_graph, node, x, z, angle, 0, target_human_joint );

	//
	tracked_node = 0;
	tracked_frame = 0;

	neighbor_nodes.clear();
	neighbor_poses.clear();

	delete blended_pose;
	blended_pose = 0;

	delete edited_pose;
	edited_pose = 0;
}

void LeapBrowser::modifyLookAt( math::position& new_lookat )
{
	original_lookat = current_lookat;
	desired_lookat = new_lookat;

	lookat_animation_frame = 0;
	is_modifying_lookat = true;
}

void LeapBrowser::updateLookAt()
{
	if( !is_modifying_lookat )
	{
		return;
	}

	lookat_animation_frame ++;

	double t = (double)lookat_animation_frame / (double)lookat_animation_duration;
	current_lookat = math::interpolate( t, original_lookat, desired_lookat );

	if( lookat_animation_frame == lookat_animation_duration )
	{
		original_lookat = math::position(0,0,0);
		desired_lookat = math::position(0,0,0);

		lookat_animation_frame = 0;
		is_modifying_lookat = false;

		//
		leap_transform.translation.set_x( current_lookat.x() );
		leap_transform.translation.set_y( DEFAULT_LEAP_HEIGHT );
		leap_transform.translation.set_z( current_lookat.z() );
	}
}


//
void LeapBrowser::drawHands()
{
	Frame frame = leap_controller.frame();

	math::vector trans = leap_transform.translation;
	math::quater rot_quat = leap_transform.rotation;
	math::vector rot_vect = math::ln( rot_quat );
	
	double angle = rot_vect.length() * 2.0;
	math::vector axis = ( angle != 0 ? rot_vect / angle : rot_vect );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glTranslatef( trans.x(), trans.y(), trans.z() );
	if( angle != 0 )
	{
		glRotatef( angle * 180.0 / M_PI, axis.x(), axis.y(), axis.z() );
	}

	HandList hands = frame.hands();
	for( HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl )
	{
		const Hand hand = *hl;

		float joint_radius = 5.0f;
		float bone_radius = 4.5f;

		Vector prev_palm_start(0,0,0);
		Vector prev_palm_end(0,0,0);

		int f = 0;
		const FingerList fingers = hand.fingers();
		for( FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl, ++f )
		{
			const Finger finger = *fl;

			Vector curr_palm_start(0,0,0);
			Vector curr_palm_end(0,0,0);

			for( int b=0; b < 4; b++ )
			{
				Bone::Type bone_type = static_cast<Bone::Type>( b );
				Bone bone = finger.bone( bone_type );

				Vector start = bone.prevJoint();
				Vector end = bone.nextJoint();

				math::position p0( start.x, start.y, start.z );
				math::position p1( end.x, end.y, end.z );

				if( is_tracking_pose == true && finger.type() == Finger::Type::TYPE_INDEX && b==3 )
				{
					drawing_tool.setColor( 1, 0, 0, 1 );
				}
				else
				{
					drawing_tool.setColor( 0.5, 0.7, 0.5, 1 );
				}
				drawing_tool.drawSphere( p1, joint_radius );
				
				drawing_tool.setColor( 0.5, 0.7, 0.5, 1 );
				drawing_tool.drawSphere( p0, joint_radius );
				drawing_tool.drawCylinder( p0, p1, bone_radius );

				//
				if( b == 0 && fl != fingers.begin() || b == 1 && fl == fingers.begin() )
				{
					curr_palm_start = start;
					curr_palm_end = end;
				}
			}

			if( f > 1 )	//fl != fingers.begin() )
			{
				drawing_tool.setColor( 0.5, 0.7, 0.5, 1 );
				drawing_tool.applyColor();

				glBegin( GL_QUADS );
				glVertex3f( prev_palm_start.x, prev_palm_start.y, prev_palm_start.z );
				glVertex3f( prev_palm_end.x, prev_palm_end.y, prev_palm_end.z );
				glVertex3f( curr_palm_end.x, curr_palm_end.y, curr_palm_end.z );
				glVertex3f( curr_palm_start.x, curr_palm_start.y, curr_palm_start.z );
				glEnd();
			}

			prev_palm_start = curr_palm_start;
			prev_palm_end = curr_palm_end;
		}
	}

	glPopMatrix();
}

void LeapBrowser::drawCharacter()
{
	if( !character )
	{
		return;
	}

	float thickness = 5.0f;

	PoseData* pose = character->getCurrentPose();
	math::transq T = character->getTransform();

	drawing_tool.setColor( 1, 0.4, 0, 1 );
	drawing_tool.drawPose( motion_data->getSkeleton(), pose, thickness, T );

	//
	int joint_index = motion_data->getHumanJoint( target_human_joint )->getIndex();
	math::vector joint_t = pose->getGlobalTransform( motion_data->getSkeleton(), joint_index, T ).translation;
	math::position joint_p( joint_t.x(), joint_t.y(), joint_t.z() );

	/*
	drawing_tool.setColor( 1, 0, 0, 1 );
	drawing_tool.drawSphere( joint_p, thickness * 1.1 );
	*/
}

void LeapBrowser::drawSearchNodeTopDown( SearchTree::Node* node )
{
	float x1 = node->getStartX();
	float z1 = node->getStartZ();
	
	float xN = node->getEndX();
	float zN = node->getEndZ();

	math::position p1( x1, 1, z1 );
	math::position pN( xN, 1, zN );
	math::position p = node->getPosition();

	glPointSize( 3.0 );
	glLineWidth( 1.5 );

	glDisable( GL_LIGHTING );
	
	glBegin( GL_POINTS );

	glColor3f( 0, 0, 0 );
	glVertex3f( x1, 1, z1 );
	glVertex3f( xN, 1, zN );

//	glColor3f( 1, 0, 0 );
//	glVertex3f( p.x(), p.y(), p.z() );

	glEnd();

	glColor3f( 0, 0, 1 );
	glBegin( GL_LINES );
	glVertex3f( x1, 1, z1 );
	glVertex3f( xN, 1, zN );
	glEnd();

	if( node->isLeaf() )
	{
		glPointSize( 10.0 );
		glColor3f( 1, 1, 1 );

		glBegin( GL_POINTS );
		glVertex3f( xN, 1, zN );
		glEnd();
	}
	else
	{
		std::vector< SearchTree::Node* >* children = node->getChildren();
		std::vector< SearchTree::Node* >::iterator itor_c = children->begin();
		while( itor_c != children->end() )
		{
			SearchTree::Node* child = ( *itor_c ++ );
			drawSearchNodeTopDown( child );
		}
	}
	glEnable( GL_LIGHTING );
}

void LeapBrowser::drawSearchNodeBottomUp( SearchTree::Node* node )
{
	float x1 = node->getStartX();
	float z1 = node->getStartZ();
	
	float xN = node->getEndX();
	float zN = node->getEndZ();

	math::position p1( x1, 1, z1 );
	math::position pN( xN, 1, zN );
	math::position p = node->getPosition();

	glPointSize( 3.0 );
	glLineWidth( 1.5 );

	glDisable( GL_LIGHTING );
	
	glBegin( GL_POINTS );

	glColor3f( 0, 0, 0 );
	glVertex3f( x1, 1, z1 );
	glVertex3f( xN, 1, zN );

//	glColor3f( 1, 0, 0 );
//	glVertex3f( p.x(), p.y(), p.z() );

	glEnd();

	glColor3f( 0, 0, 1 );
	glBegin( GL_LINES );
	glVertex3f( x1, 1, z1 );
	glVertex3f( xN, 1, zN );
	glEnd();

	if( node->getParent() )
	{
		drawSearchNodeBottomUp( node->getParent() );
	}
	glEnable( GL_LIGHTING );
}

void LeapBrowser::drawSearchTree()
{
	SearchTree::Node* root = search_tree->getRootNode();
	if( root )
	{
		drawSearchNodeTopDown( root );
	}
}

void LeapBrowser::drawSpatialPartition()
{
	SpacePartition* space_partition = search_tree->getSpacePartition();
	if( !space_partition )
	{
		return;
	}

	glDisable( GL_LIGHTING );

	std::vector< std::vector< SpaceBin* >* >* grid = space_partition->getGrid();
	std::vector< std::vector< SpaceBin* >* >::iterator itor_bl = grid->begin();
	while( itor_bl != grid->end() )
	{
		std::vector< SpaceBin* >* bins = ( *itor_bl ++ );
		std::vector< SpaceBin* >::iterator itor_b = bins->begin();
		while( itor_b != bins->end() )
		{
			SpaceBin* bin = ( *itor_b ++ );
			int num_nodes = bin->getNumNodes();
			if( num_nodes > 0 )
			{
				double x_min = bin->getMinX();
				double z_min = bin->getMinZ();
				double x_max = bin->getMaxX();
				double z_max = bin->getMaxZ();

				double X_MIN, X_MAX, Z_MIN, Z_MAX;
				search_tree->calcGlobalCoords( x_min, z_min, &X_MIN, &Z_MIN );
				search_tree->calcGlobalCoords( x_max, z_max, &X_MAX, &Z_MAX );

				//
				float color = (float)num_nodes / 10;
				if( color > 1.0f )	color = 1.0f;
				glColor3f( color, color, color );

				glBegin( GL_QUADS );
				glVertex3f( X_MIN, 1, Z_MIN );
				glVertex3f( X_MIN, 1, Z_MAX );
				glVertex3f( X_MAX, 1, Z_MAX );
				glVertex3f( X_MAX, 1, Z_MIN );
				glEnd();
			}
		}
	}

	glEnable( GL_LIGHTING );
}

void LeapBrowser::drawTrackedNeighbors()
{
	if( neighbor_nodes.empty() )
	{
		return;
	}

	drawing_tool.setColor( 0, 1, 0.4, 1 );
	std::vector< SearchTree::Node* >::iterator itor_n = neighbor_nodes.begin();
	while( itor_n != neighbor_nodes.end() )
	{
		SearchTree::Node* node = ( *itor_n ++ );

		math::transq T = node->getTransform();
		int f = node->getEndFrame();

		drawing_tool.drawPose( motion_data, f, 5, T );	
	}
}

void LeapBrowser::drawTreePath()
{
	if( tracked_node )
	{
		drawSearchNodeBottomUp( tracked_node );
	}
}

void LeapBrowser::drawTrackedPose()
{
	if( !tracked_node )
	{
		return;
	}

	math::transq T = tracked_node->getTransform();
	drawing_tool.setColor( 0, 0.4, 1, 1 );
	drawing_tool.drawPose( motion_data, tracked_frame, 5, T );

	if( is_cursor_located )
	{
		math::vector joint_trans = motion_data->getGlobalTransform( tracked_frame, target_joint_index, T ).translation;
		math::position joint_pos( joint_trans.x(), joint_trans.y(), joint_trans.z() );

		drawing_tool.setColor( 0, 0, 0, 1 );
		drawing_tool.drawCylinder( joint_pos, cursor_location, 3 );
	}
}

void LeapBrowser::drawBlendedPose()
{
	if( !blended_pose )
	{
		return;
	}

	drawing_tool.setColor( 0, 1, 0.4, 1 );
	drawing_tool.drawPose( motion_data->getSkeleton(), blended_pose, 5 );

	if( is_cursor_located )
	{
		math::vector joint_trans = blended_pose->getGlobalTransform( motion_data->getSkeleton(), target_joint_index ).translation;
		math::position joint_pos( joint_trans.x(), joint_trans.y(), joint_trans.z() );

		drawing_tool.setColor( 0, 0, 0, 1 );
		drawing_tool.drawCylinder( joint_pos, cursor_location, 3 );
	}
}

void LeapBrowser::drawEditedPose()
{
	if( !edited_pose )
	{
		return;
	}

	drawing_tool.setColor( 1, 0, 0.4, 1 );
	drawing_tool.drawPose( motion_data->getSkeleton(), edited_pose, 5 );

	if( is_cursor_located )
	{
		math::vector joint_trans = edited_pose->getGlobalTransform( motion_data->getSkeleton(), target_joint_index ).translation;
		math::position joint_pos( joint_trans.x(), joint_trans.y(), joint_trans.z() );

		drawing_tool.setColor( 0, 0, 0, 1 );
		drawing_tool.drawCylinder( joint_pos, cursor_location, 3 );
	}
}

