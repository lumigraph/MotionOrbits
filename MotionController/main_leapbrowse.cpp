#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdarg.h>
#include <windows.h>

#include <vector>
#include <algorithm>

#include <GL/glew.h>
#include <GL/glut.h>

#include "Leap.h"
#include "LeapListener.h"

#include "DrawingTool.h"
#include "trackball.h"

#include "SkeletalMotion.h"
#include "MotionData.h"
#include "PoseData.h"

#include "MotionEdit.h"
#include "MotionGraph.h"

#include "Skeleton.h"
#include "Joint.h"

#include "Human.h"
#include "PoseConstraint.h"
#include "PoseData.h"
#include "PoseIK.h"

#include "Character.h"
#include "SearchTree.h"
#include "SpacePartition.h"

#include "mathclass/QmGeodesic.h"


//#define BASKETBALL
#define BOXING
//#define BBOY

#ifdef BOXING
#define PATH_BVH	"../data/boxing/boxing_shadow_m_edit.bvh"
#define PATH_GRAPH	"../data/boxing/graph.txt"
#endif

#ifdef BASKETBALL
#define PATH_BVH	"../data/basketball/shooting_refined.bvh"
#define PATH_GRAPH	"../data/basketball/graph.txt"
#endif

#ifdef BBOY
#define PATH_BVH	"../data/b-boy/B_boy.bvh"
#define PATH_GRAPH	"../data/b-boy/graph.txt"
#endif

#define PATH_EDITED	"../data/edited_motion.bvh"

//
static void initialize();
static void finalize();

// glut callback functions
static void reshape( int w, int h );
static void display();
static void idle();
static void finalize();
static void timer( int timer_id );

static void keyboard( unsigned char key, int x, int y );
static void special( int key, int x, int y );
static void mouse( int button, int state, int x, int y );
static void motion( int x, int y );

//
static unsigned int win_width = 800;
static unsigned int win_height = 800;
static float win_aspect_ratio = ( (float)win_width / (float)win_height );
static float fovy = 60.0f;
static float z_near = 1.0f;
static float z_far = 10000.0f;
static float view_distance = 500.0f;

static math::position current_lookat(0,50,0);
static math::position original_lookat(0,0,0);
static math::position desired_lookat(0,0,0);

static bool is_lookat_animated = false;
static float lookat_animation_frame = 0;
static float lookat_animation_duration = 30;

//
static bool is_lbutton_down = false;
static bool is_rbutton_down = false;
static int track_x = 0, track_y = 0;

static math::position goal_point(0,0,0);
static bool is_tracking = false;
static int controlled_human_joint = Human::RIGHT_PALM;

static SearchTree::Node* tracked_node = 0;
static int tracked_frame = 0;
static std::vector< SearchTree::Node* > tracked_neighbors;

static PoseData* blended_pose = 0;
static PoseData* edited_pose = 0;

static std::vector< MotionGraph::Node* > motion_sequence;
static int sequence_index = 0;
static int frame_index = 0;

//
static unsigned int time_interval = 1;
static bool is_playing = false;
static bool is_simplified = false;

//
static QTrackBall		track_ball;

static SkeletalMotion	motion_data;
static MotionGraph		motion_graph;
static SearchTree		search_tree;
static Character		character;

extern DrawingTool		drawing_tool;

//
using namespace Leap;

static LeapListener		listener;
static Controller		controller;

//
enum {
	NEAREST = 0,
	BLENDED,
	DEFORMED
};
static int view_mode = NEAREST;

static bool is_neighbors_displayed = false;
static bool is_search_tree_displayed = false;
static bool is_spatial_partition_displayed = false;
static bool is_tree_path_displayed = false;

//
extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupCMUSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );


void startLeapBrowser( int* argcp, char** argv )
{ 			
	atexit( finalize );

	glutInit( argcp, argv );
	glutInitWindowSize( win_width, win_height );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL );
    glutCreateWindow( "Leap Browser" );

	glewInit();

	//
    GLenum status = glCheckFramebufferStatusEXT( GL_FRAMEBUFFER_EXT );
 
    switch( status ) {
        case GL_FRAMEBUFFER_COMPLETE_EXT:
            std::cout<<"GL_FRAMEBUFFER_COMPLETE_EXT!: SUCCESS\n";
            break;
 
        case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
            std::cout<<"GL_FRAMEBUFFER_UNSUPPORTED_EXT!: ERROR\n";
            exit(0);
            break;

        default:
            exit(0);
    }

	if (glewIsSupported("GL_VERSION_2_0"))
		std::cout << "Ready for OpenGL 2.0\n";
	else {
		std::cout << "OpenGL 2.0 not supported\n";
		exit(0);
	}

	initialize();

	// call-back initialization
	glutReshapeFunc( reshape );
	glutDisplayFunc( display );
	glutKeyboardFunc( keyboard ); 
	glutSpecialFunc( special );
	glutMouseFunc( mouse );
	glutMotionFunc( motion );
	glutIdleFunc( idle );
	glutTimerFunc( time_interval, timer, 1 );
	
	glutMainLoop();
} 

inline static bool sort_cycles( MotionGraph* left_cycle, MotionGraph* right_cycle )
{
	return (left_cycle->getNumNodes() < right_cycle->getNumNodes() );
}

void initialize()
{
	motion_data.importFromBVH( PATH_BVH );
	motion_graph.load( PATH_GRAPH );
	
#ifdef BOXING
	setupBoxingSkeleton( motion_data.getSkeleton() );
#endif

#ifdef BBOY
	setupBboySkeleton( motion_data.getSkeleton() );
#endif

#ifdef BASKETBALL
	setupBasketballSkeleton( motion_data.getSkeleton() );
#endif

	//
	character.embody( &motion_data, &motion_graph );
	character.extendPathRandomly( 1 );
	character.place( 0, 0, 0 );

	//
	search_tree.initialize( &motion_data, &motion_graph, character.getPathNode(0), 0, 0, 0, 0, controlled_human_joint );
	is_tracking = true;

	//
	float q[4] = { 0, 0, 0, 1 };
	track_ball.SetQuat( q );

	//
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_NORMALIZE );
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

	//
	controller.addListener( listener );
}

void finalize()
{
	controller.removeListener( listener );
}

static void modifyLookAt( math::position& new_lookat )
{
	original_lookat = current_lookat;
	desired_lookat = new_lookat;

	lookat_animation_frame = 0;
	is_lookat_animated = true;
}

static void animateLookAt()
{
	if( is_lookat_animated )
	{
		lookat_animation_frame ++;

		double t = (double)lookat_animation_frame / (double)lookat_animation_duration;
		current_lookat = math::interpolate( t, original_lookat, desired_lookat );

		if( lookat_animation_frame == lookat_animation_duration )
		{
			original_lookat = math::position(0,0,0);
			desired_lookat = math::position(0,0,0);

			lookat_animation_frame = 0;
			is_lookat_animated = false;
		}
	}
}

void reshape( int w, int h )
{
	win_width = w;
	win_height = h;
	win_aspect_ratio = (float)w / (float)h;

	track_ball.SetWindowInfo( win_width, win_height, 0, 0 );
	drawing_tool.setWindowSize( win_width, win_height );

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective( fovy, win_aspect_ratio, z_near, z_far );

	glViewport( 0, 0, win_width, win_height );
}

static void setupLight()
{
	glEnable( GL_LIGHTING );
	glEnable( GL_LIGHT0 );

	GLfloat position[] = { 0.0f, 0.0f, 1.0f, 0.0f };		// directional source
	GLfloat diffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat specular[] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat ambient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat shininess = 100.0f;

	glLightfv( GL_LIGHT0, GL_POSITION, position );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuse );
	glLightfv( GL_LIGHT0, GL_SPECULAR, specular );
	glLightfv( GL_LIGHT0, GL_AMBIENT, ambient );
	glLightf( GL_LIGHT0, GL_SHININESS, shininess );
}

static void drawFloor()
{
	drawing_tool.setColor( 0.8, 0.8, 0.8, 0.4 );
	drawing_tool.drawBox( math::position(0,0,0), math::vector(500,1,500) );
}

static void drawCharacter()
{
	float thickness = 5.0f;

	PoseData* pose = character.getCurrentPose();
	math::transq T = character.getTransform();

	drawing_tool.setColor( 1, 0.4, 0, 1 );
	drawing_tool.drawPose( motion_data.getSkeleton(), pose, thickness, T );

	//
	int joint_index = motion_data.getHumanJoint( controlled_human_joint )->getIndex();
	math::vector joint_t = pose->getGlobalTransform( motion_data.getSkeleton(), joint_index, T ).translation;
	math::position joint_p( joint_t.x(), joint_t.y(), joint_t.z() );

	drawing_tool.setColor( 1, 0, 0, 1 );
	drawing_tool.drawSphere( joint_p, thickness * 1.1 );
}

static void drawHands()
{
	Frame frame = controller.frame();

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glTranslatef( current_lookat.x(), -50, current_lookat.z() );

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

				if( is_tracking == true && finger.type() == Finger::Type::TYPE_INDEX && b==3 )
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

static void drawSearchNode( SearchTree::Node* node )
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

	glColor3f( 1, 0, 0 );
	glVertex3f( p.x(), p.y(), p.z() );

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
			drawSearchNode( child );
		}
	}
	glEnable( GL_LIGHTING );
}

static void drawSearchNodeInverse( SearchTree::Node* node )
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

	glColor3f( 1, 0, 0 );
	glVertex3f( p.x(), p.y(), p.z() );

	glEnd();

	glColor3f( 0, 0, 1 );
	glBegin( GL_LINES );
	glVertex3f( x1, 1, z1 );
	glVertex3f( xN, 1, zN );
	glEnd();

	if( node->getParent() )
	{
		drawSearchNodeInverse( node->getParent() );
	}
	glEnable( GL_LIGHTING );
}

static void drawSearchTree()
{
	SearchTree::Node* root = search_tree.getRootNode();
	if( root )
	{
		drawSearchNode( root );
	}
}

static void drawSpacePartition()
{
	SpacePartition* space_partition = search_tree.getSpacePartition();
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
				search_tree.calcGlobalCoords( x_min, z_min, &X_MIN, &Z_MIN );
				search_tree.calcGlobalCoords( x_max, z_max, &X_MAX, &Z_MAX );

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

static void drawTrackedPose()
{
	if( !tracked_node )
	{
		return;
	}

	int joint_index = motion_data.getHumanJoint( controlled_human_joint )->getIndex();
	math::vector joint_trans(0,0,0);

	switch( view_mode ) {
	case NEAREST:
		{
			math::transq T = tracked_node->getTransform();
			drawing_tool.setColor( 0, 0.4, 1, 1 );
			drawing_tool.drawPose( &motion_data, tracked_frame, 5, T );

			joint_trans = motion_data.getGlobalTransform( tracked_frame, joint_index, T ).translation;
		}
		break;
		
	case BLENDED:
		{
			if( blended_pose )
			{
				drawing_tool.setColor( 0, 1, 0.4, 1 );
				drawing_tool.drawPose( motion_data.getSkeleton(), blended_pose, 5 );

				joint_trans = blended_pose->getGlobalTransform( motion_data.getSkeleton(), joint_index ).translation;
			}
		}
		break;

	case DEFORMED:
		{
			if( edited_pose )
			{
				drawing_tool.setColor( 1, 0, 0.4, 1 );
				drawing_tool.drawPose( motion_data.getSkeleton(), edited_pose, 5 );

				joint_trans = edited_pose->getGlobalTransform( motion_data.getSkeleton(), joint_index ).translation;
			}
		}
		break;
	}

	if( joint_trans != math::vector(0,0,0) && goal_point != math::position(0,0,0) && is_tracking == true )
	{
		math::position joint_pos( joint_trans.x(), joint_trans.y(), joint_trans.z() );

		drawing_tool.setColor( 0, 0, 0, 1 );
		drawing_tool.drawCylinder( joint_pos, goal_point, 3 );
	}

	if( is_neighbors_displayed )
	{
		drawing_tool.setColor( 0, 1, 0.4, 1 );
		std::vector< SearchTree::Node* >::iterator itor_n = tracked_neighbors.begin();
		while( itor_n != tracked_neighbors.end() )
		{
			SearchTree::Node* node = ( *itor_n ++ );

			math::transq T = node->getTransform();
			int f = node->getEndFrame();

			drawing_tool.drawPose( &motion_data, f, 5, T );	
		}
	}

	if( is_tree_path_displayed )
	{
		drawSearchNodeInverse( tracked_node );
	}
}

void display()
{
	glClearDepth( 1 );
	glClearColor( 1, 1, 1, 1 );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	setupLight();

	glTranslatef( 0, 0, -view_distance );

	float rot_mat[16];
	track_ball.BuildRotationMatrix( (float *)rot_mat );
	glMultMatrixf( (float *)rot_mat );

	animateLookAt();
	glTranslatef( -current_lookat.x(), -current_lookat.y(), -current_lookat.z() );

	//
	glDisable( GL_DEPTH_TEST );
	glColorMask( GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE );

	glEnable( GL_STENCIL_TEST );
	glStencilOp( GL_REPLACE, GL_REPLACE, GL_REPLACE );
	glStencilFunc( GL_ALWAYS, 1, 0xffffffff );

	drawFloor();

	glEnable( GL_DEPTH_TEST );
	glColorMask( GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE );

	glStencilOp( GL_KEEP, GL_KEEP, GL_KEEP );
	glStencilFunc( GL_EQUAL, 1, 0xffffffff );

	glPushMatrix();
	glScalef( 1, -1, 1 );

	drawCharacter();
	drawHands();
	drawTrackedPose();

	glPopMatrix();

	glDisable( GL_STENCIL_TEST );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	
	drawFloor();

	glDisable( GL_BLEND );

	drawCharacter();
	drawHands();
	drawTrackedPose();

	if( is_spatial_partition_displayed )
	{
		drawSpacePartition();
	}
	if( is_search_tree_displayed )
	{
		drawSearchTree();
	}
	
	//
	glutSwapBuffers();
}

inline static bool sort_search_nodes( SearchTree::Node* lhs, SearchTree::Node* rhs )
{
	return ( lhs->getDistanceToGoal() < rhs->getDistanceToGoal() );
}

inline static bool sort_tracked_poses( std::pair< std::pair<SearchTree::Node*, int>, double >& lhs, std::pair< std::pair<SearchTree::Node*, int>, double>& rhs )
{
	return ( lhs.second < rhs.second );
}

static double calcEffortForMovement( math::transq& T1, int f1, math::transq& T2, int f2 )
{
	/*
	std::vector< std::pair<unsigned int, double> > joints;
	joints.push_back( std::make_pair( Human::PELVIS, 4.0 ) );
	joints.push_back( std::make_pair( Human::LEFT_PALM, 1.0 ) );
	joints.push_back( std::make_pair( Human::RIGHT_PALM, 1.0 ) );
	joints.push_back( std::make_pair( Human::LEFT_FOOT, 2.0 ) );
	joints.push_back( std::make_pair( Human::RIGHT_FOOT, 2.0 ) );

	double sum_of_dist = 0.0;
	double sum_of_weights = 0.0;

	int num_joints = (int)joints.size();
	for( int j=0; j < num_joints; j++ )
	{
		unsigned int human_joint = joints[j].first;
		double weight = joints[j].second;

		unsigned int joint_index = motion_data.getHumanJoint( human_joint )->getIndex();
		math::vector v1 = motion_data.getGlobalTransform( f1, joint_index, T1 ).translation;
		math::vector v2 = motion_data.getGlobalTransform( f2, joint_index, T2 ).translation;

		double dist = math::vector( v1-v2 ).length();
		sum_of_dist += ( dist * weight );
		sum_of_weights += ( weight );
	}

	return ( sum_of_dist / sum_of_weights );
	*/

	math::transq root_T1 = motion_data.getGlobalTransform( f1, 0, T1 );
	math::transq root_T2 = motion_data.getGlobalTransform( f2, 0, T2 );

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
	int num_joints = motion_data.getNumJoints();
	for( int j=1; j < num_joints; j++ )
	{
		math::vector joint_trans1 = motion_data.getGlobalTransform( f1, j ).translation;
		math::vector joint_trans2 = motion_data.getGlobalTransform( f2, j, disp ).translation;
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

static bool trackIndexFinger()
{
	bool is_index_found = false;

	Frame frame = controller.frame();	
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
				goal_point = math::position(position.x, position.y, position.z);
				goal_point += math::vector(current_lookat.x(), -50, current_lookat.z());

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
		goal_point = math::position(0,0,0);
	}

	/*
	PoseData* pose = character.getCurrentPose();
	math::transq T = character.getTransform();

	int joint_index = motion_data.getHumanJoint( controlled_human_joint )->getIndex();
	math::vector joint_t = pose->getGlobalTransform( motion_data.getSkeleton(), joint_index, T ).translation;
	goal_point = math::position( joint_t.x(), joint_t.y(), joint_t.z() );
	*/

	return is_index_found;
}

static void resetSearchTree()
{
	MotionGraph::Node* node = character.getPathNode( 0 );
	double x = character.getX();
	double z = character.getZ();
	double angle = character.getAngle();

	search_tree.initialize( &motion_data, &motion_graph, node, x, z, angle, 0, controlled_human_joint );

	//
	tracked_node = 0;
	tracked_frame = 0;
	tracked_neighbors.clear();

	delete blended_pose;
	blended_pose = 0;

	delete edited_pose;
	edited_pose = 0;
}

static void updateTrackedPose()
{
	int K = 10;

	tracked_neighbors.clear();
	search_tree.kNN( goal_point.x(), goal_point.z(), K, &tracked_neighbors );

	math::transq old_T;
	int old_f = 0;

	if( tracked_node )
	{
		old_T = tracked_node->getTransform();
		old_f = tracked_frame;
	}

	std::vector< std::pair< std::pair<SearchTree::Node*, int>, double > > tracked_poses;

	double min_dist = +DBL_MAX;
	SearchTree::Node* min_node = 0;
	int min_frame = 0;

	int num_neighbors_found = (int)tracked_neighbors.size();
	for( int k=0; k < num_neighbors_found; k++ )
	{
		SearchTree::Node* node = tracked_neighbors[ k ];
		math::transq T = node->getTransform();

		int f1 = node->getGraphNode()->getSegment(0).first;
		int fN = node->getGraphNode()->getSegment(0).second;
			
		for( int f=f1; f <= fN; f++ )
		{
			math::position P = search_tree.calcTrackPoint( node, f );
			double d = math::vector( goal_point-P ).length();

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
			tracked_poses.push_back( std::make_pair( std::make_pair(node, f), dist ) );
		}
	}
	tracked_node = min_node;
	tracked_frame = min_frame;

	//
	std::sort( tracked_poses.begin(), tracked_poses.end(), sort_tracked_poses );

	//
	if( tracked_node )
	{
		if( blended_pose )
		{
			delete blended_pose;
		}
		blended_pose = new PoseData;
		blended_pose->initialize( motion_data.getNumJoints() );
	
		int NUM_BLENDS = 10;

		if( tracked_poses.size() >= NUM_BLENDS )
		{
			std::vector< std::pair<PoseData*, math::transq> > poses;
			std::vector< double > ratios;

			for( int i=0; i < NUM_BLENDS; i++ )
			{
				SearchTree::Node* node = tracked_poses[i].first.first;
				int f = tracked_poses[i].first.second;
				double dist = tracked_poses[i].second;

				PoseData* pose = motion_data.getPoseData( f );
				math::transq T = node->getTransform();
				double ratio = 1.0 / dist;

				poses.push_back( std::make_pair(pose, T) );
				ratios.push_back( ratio );
			}
			blended_pose->blend( &poses, &ratios );
		}
		else
		{
			blended_pose->copy( motion_data.getPoseData( min_frame ) );
			blended_pose->setJointTransform( 0, min_node->getTransform() * blended_pose->getJointTransform(0) );
		}

		//
		if( edited_pose )
		{
			delete edited_pose;
		}
		edited_pose = new PoseData;
		edited_pose->initialize( motion_data.getNumJoints() );
		edited_pose->copy( blended_pose );

		unsigned int left_hand = motion_data.getSkeleton()->getHumanJoint( Human::LEFT_PALM )->getIndex();
		unsigned int right_hand = motion_data.getSkeleton()->getHumanJoint( Human::RIGHT_PALM )->getIndex();
		unsigned int left_foot = motion_data.getSkeleton()->getHumanJoint( Human::LEFT_FOOT )->getIndex();
		unsigned int right_foot = motion_data.getSkeleton()->getHumanJoint( Human::RIGHT_FOOT )->getIndex();

		math::transq lh_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), left_hand );
		math::transq rh_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), right_hand );
		math::transq lf_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), left_foot );
		math::transq rf_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), right_foot );

		PoseConstraint pose_constraint;
		pose_constraint.addConstraint( Human::LEFT_PALM, lh_transq );
		pose_constraint.addConstraint( Human::RIGHT_PALM, rh_transq );
		pose_constraint.addConstraint( Human::LEFT_FOOT, lf_transq );
		pose_constraint.addConstraint( Human::RIGHT_FOOT, rf_transq );

		//
		pose_constraint.removeConstraint( controlled_human_joint );
		pose_constraint.addConstraint( controlled_human_joint, math::vector(goal_point.x(), goal_point.y(), goal_point.z()) );

		//
		PoseIK pose_ik;
		pose_ik.initialize();
		pose_ik.ik_body( motion_data.getSkeleton(), blended_pose, edited_pose, &pose_constraint, 0.0 );
	}
}

void idle()
{
	static unsigned int count = 0;

	if( is_playing )
	{
		if( count % 1 == 0 )
		{
			bool is_updated = character.update();
			if( !is_updated )
			{
				double x = character.getX();
				double z = character.getZ();
				double angle = character.getAngle();

				//
				MotionGraph::Node* prev_node = character.getPathNode(0);
				
				std::vector< MotionGraph::Edge* >* next_edges = prev_node->getNextEdges();
				int num_next_edges = (int)next_edges->size();
				int edge_index = rand() % num_next_edges;
				
				MotionGraph::Edge* next_edge = ( *next_edges )[ edge_index ];
				MotionGraph::Node* next_node = next_edge->getToNode();

				//
				character.clearPath();
				character.extendPath( next_node );
				character.place( x, z, angle );

				resetSearchTree();

				//
				modifyLookAt( math::position(x, current_lookat.y(), z) );

				//
				is_playing = false;
				is_tracking = true;
			}
		}
	}
	
	if( is_tracking )
	{
		bool is_index_found = trackIndexFinger();
		if( is_index_found )
		{
			if( count % 1 == 0 )
			{
				search_tree.extend( goal_point );
			}
			if( count % 3 == 0 )
			{
//				search_tree.shrink( goal_point );
			}
			if( count % 1 == 0 )
			{
				updateTrackedPose();
			}
		}
	}

	glutPostRedisplay();

	count ++;
}

void timer( int timer_id )
{
	glutTimerFunc( time_interval, timer, timer_id );
}

void keyboard( unsigned char key, int x, int y )
{
	switch( key ) {
	case 9:	// tab
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

				character.clearPath();
				std::deque< MotionGraph::Node* >::iterator itor_n = graph_path.begin();
				while( itor_n != graph_path.end() )
				{
					MotionGraph::Node* graph_node = ( *itor_n ++ );
					character.extendPath( graph_node );

					//
					motion_sequence.push_back( graph_node );
				}
				//character.place( 0, 0, 0 );

				is_playing = true;
				is_tracking = false;
			}
		}
		break;
	case 13:	// enter
		{
			if( !motion_sequence.empty() )
			{
				Character temp_char;
				temp_char.embody( &motion_data, &motion_graph );
				character.embody( &motion_data, &motion_graph );

				std::vector< MotionGraph::Node* >::iterator itor_n = motion_sequence.begin();
				while( itor_n != motion_sequence.end() )
				{
					MotionGraph::Node* node = ( *itor_n ++ );
					temp_char.extendPath( node );
					character.extendPath( node );
				}
				temp_char.place( 0, 0, 0 );
				character.place( 0, 0, 0 );

				//
				SkeletalMotion temp_motion;
				temp_motion.initialize( motion_data.getSkeleton(), 0 );

				bool is_updated = false;
				do
				{
					PoseData* pose = temp_char.getCurrentPose();
					math::transq T = temp_char.getTransform();

					temp_motion.getMotionData()->addPoseData( pose );
					PoseData* new_pose = temp_motion.getPoseData( temp_motion.getNumFrames()-1 );
					new_pose->setJointTransform( 0, T * new_pose->getJointTransform(0) );

					is_updated = temp_char.update();
				}
				while( is_updated );

				//
				temp_motion.exportIntoBVH( PATH_EDITED );

				//
				is_playing = true;
				is_tracking = false;
				
				modifyLookAt( math::position(0, current_lookat.y(), 0) );
			}
		}
		break;
	case 32:	// space
		{
			is_playing = !is_playing;
		}
		break;
	case '1':
		{
			is_neighbors_displayed = !is_neighbors_displayed;
		}
		break;
	case '2':
		{
			is_search_tree_displayed = !is_search_tree_displayed;
		}
		break;
	case '3':
		{
			is_spatial_partition_displayed = !is_spatial_partition_displayed;
		}
		break;
	case '4':
		{
			is_tree_path_displayed = !is_tree_path_displayed;
		}
		break;
	}
	glutPostRedisplay();
}

void special( int key, int x, int y )
{
	switch(key){
	case GLUT_KEY_F1:		view_mode = NEAREST;	break;
	case GLUT_KEY_F2:		view_mode = BLENDED;	break;
	case GLUT_KEY_F3:		view_mode = DEFORMED;	break;

	case GLUT_KEY_LEFT:		break;
	case GLUT_KEY_DOWN:		break;
	case GLUT_KEY_RIGHT:	break;
	case GLUT_KEY_UP:		break;
	}

	glutPostRedisplay();
}

void mouse( int button, int state, int x, int y )
{
	if( button == GLUT_LEFT_BUTTON ) 
	{
		if( state == GLUT_DOWN ) 
		{
			is_lbutton_down = true;
			track_x = x;
			track_y = y;
		}
		else 
		{
			is_lbutton_down = false;
		}
		return;
	}
	if( button == GLUT_RIGHT_BUTTON ) 
	{
		if( state == GLUT_DOWN ) 
		{
			is_rbutton_down = true;
			track_x = x;
			track_y = y;
		}
		else 
		{
			is_rbutton_down = false;
		}
		return;
	}
}

void motion( int x, int y )
{
	if( is_lbutton_down | is_rbutton_down ) 
	{
		if( is_lbutton_down ) 
		{	
			track_ball.Move( track_x, track_y, x, y );
		}
		else 
		{
			view_distance += (double)( y - track_y ) / win_height * 100.0f;
		}
		track_x = x;
		track_y = y;

		glutPostRedisplay();

		return;
	}
}
