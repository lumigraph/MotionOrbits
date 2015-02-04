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
#include "Skeleton.h"
#include "Joint.h"
#include "Human.h"

#include "MotionGraph.h"
#include "OrbitGraph.h"
#include "Character.h"

#include "PoseData.h"
#include "PoseIK.h"
#include "PoseConstraint.h"

#include "mathclass/QmApproximate.h"
#include "mathclass/QmGeodesic.h"
#include "mathclass/transq.h"

/*
#define PATH_BVH	"../data/boxing/boxing_shadow_m_edit.bvh"
#define PATH_GRAPH	"../data/boxing/graph.txt"
#define PATH_ORBIT	"../data/boxing/orbit.txt"
*/

/*
#define PATH_BVH	"../data/basketball/shooting_refined.bvh"
#define PATH_GRAPH	"../data/basketball/graph.txt"
#define PATH_ORBIT	"../data/basketball/orbit.txt"
*/

#define PATH_BVH	"../data/b-boy/B_boy.bvh"
#define PATH_GRAPH	"../data/b-boy/graph.txt"
#define PATH_ORBIT	"../data/b-boy/orbit.txt"

//
CRITICAL_SECTION cs;


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

//
static bool is_lbutton_down = false;
static bool is_rbutton_down = false;
static int track_x, track_y;

static unsigned int time_interval = 1;
static bool is_playing = false;

//
static QTrackBall		track_ball;

static SkeletalMotion	motion_data;
static MotionGraph		motion_graph;
static OrbitGraph		orbit_graph( &motion_graph );
static Character		character;

extern DrawingTool		drawing_tool;

enum
{
	CYCLING_IN_ORBIT = 0,
	LEAVING_FROM_ORBIT,
	ENTERING_INTO_ORBIT,
};

static OrbitGraph::Node* current_orbit = 0;
static OrbitGraph::Edge* current_transit = 0;
static unsigned int moving_mode = CYCLING_IN_ORBIT;

//
using namespace Leap;

static LeapListener listener;
static Controller controller;

//
enum {
	NONE = 0,
	RIGHT_HAND,
	LEFT_HAND,
	RIGHT_FOOT,
	LEFT_FOOT,
	WHOLE_BODY,
};

static bool is_pinched = false;
static Vector pinched_location(0,0,0);
static Vector dragged_location(0,0,0);
static math::vector joint_location(0,0,0);

static Character* pinched_character = 0;
static unsigned int pinched_part = NONE;
static unsigned int pinched_joint = 0;

static PoseData* pinched_pose = 0;
static PoseData* edited_pose = 0;
static math::transq edited_transform( math::quater(1,0,0,0), math::vector(0,0,0) );

static std::map< OrbitGraph::Node*, std::vector< math::vector >* > transition_trajectories;
static std::map< OrbitGraph::Node*, std::vector< std::pair<math::transq, PoseData*> >* > transition_poses;

static OrbitGraph::Node* selected_orbit = 0;
static unsigned int	selected_frame = 0;

static std::vector< math::vector >* selected_transition_trajectory = 0;
static std::vector< std::pair<math::transq, PoseData*> >* selected_transition_poses = 0;

//
static const unsigned int MAX_TRAIL_LENGTH = 1024;
static std::deque< math::position > movement_trail;


//
extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );


void startLeapController( int* argcp, char** argv )
{ 			
	atexit( finalize );

	glutInit( argcp, argv );
	glutInitWindowSize( win_width, win_height );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL );

//	glutCreateWindow( "Leap Controller" );
	glutGameModeString( "1024x768:32@60" );
	if( glutGameModeGet( GLUT_GAME_MODE_POSSIBLE ) )
	{
		std::cout << "The selected mode is not supported in full-screen..\n";
		glutCreateWindow( "Leap Controller" );
	}
	else
	{
		glutEnterGameMode();
	}

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

void initialize()
{
	motion_data.importFromBVH( PATH_BVH );
	motion_graph.load( PATH_GRAPH );
	orbit_graph.load( PATH_ORBIT );

	//setupBoxingSkeleton( motion_data.getSkeleton() );
	setupBboySkeleton( motion_data.getSkeleton() );
	//setupBasketballSkeleton( motion_data.getSkeleton() );

	//
	unsigned int num_orbits = orbit_graph.getNumNodes();
	unsigned int orbit_index = rand() % num_orbits;
	current_orbit = orbit_graph.getNode( orbit_index );
	current_transit = 0;
	moving_mode = CYCLING_IN_ORBIT;

	std::vector< MotionGraph::Node* >* orbit_cycle = current_orbit->getCycle();
	MotionGraph::Node* start_node = ( *orbit_cycle )[ 0 ];

	character.embody( &motion_data, &motion_graph );
	character.extendPath( start_node );
	character.place( 0, 0, 0 );

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

static void drawHands()
{
	Frame frame = controller.frame();

	HandList hands = frame.hands();
	for( HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl )
	{
		const Hand hand = *hl;

		//
		glEnable( GL_LIGHTING );

		Vector p = hand.palmPosition();
		glPushMatrix();
		glTranslatef( p.x, p.y, p.z );
		glutSolidSphere( 15, 10, 10 );
		glPopMatrix();

		//
		glDisable( GL_LIGHTING );

		if( hand.isLeft() )
		{
			glColor3f( 1, 0, 0 );
		}
		else
		{
			glColor3f( 0, 0, 1 );
		}

		glLineWidth( 10.0f );
		glBegin( GL_LINES );

		const FingerList fingers = hand.fingers();
		for( FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl )
		{
			const Finger finger = *fl;

			for( int b=0; b < 4; b++ )
			{
				Bone::Type bone_type = static_cast<Bone::Type>( b );
				Bone bone = finger.bone( bone_type );

				Vector start = bone.prevJoint();
				Vector end = bone.nextJoint();

				glVertex3f( start.x, start.y, start.z );
				glVertex3f( end.x, end.y, end.z );
			}
		}
		glEnd();
	}
}

static void drawTrail()
{
	drawing_tool.setColor( 0, 0, 0, 1 );

	std::deque< math::position >::iterator itor_t = movement_trail.begin();
	while( itor_t != movement_trail.end() )
	{
		math::position p = ( *itor_t ++ );
		p.set_y( 1 );

		drawing_tool.drawSphere( p, 2 );
	}
}

static void drawFloor()
{
	drawing_tool.setColor( 0.8, 0.8, 0.8, 0.4 );
	drawing_tool.drawBox( math::position(0,0,0), math::vector(1000,1,1000) );
}

static void drawCharacter()
{
	float thickness = 5.0f;

	switch( pinched_part ) {
	case NONE:
		drawing_tool.setColor( 1, 0.4, 0, 1 );
		break;

	case WHOLE_BODY:
		drawing_tool.setColor( 0, 0.4, 1, 1 );
		break;

	case LEFT_HAND:
	case LEFT_FOOT:
	case RIGHT_HAND:
	case RIGHT_FOOT:
		drawing_tool.setColor( 0.4, 1, 0.4, 1 );
		break;
	}

	if( character.isInBlend() )
	{
		SkeletalMotion* blend_motion = character.getBlendMotion();
		math::transq T = character.getTransform();
		unsigned int f = character.getBlendFrame();

		drawing_tool.drawPose( blend_motion, f, thickness, T );
	}
	else
	{
		SkeletalMotion* skeletal_motion = character.getSkeletalMotion();
		math::transq T = character.getTransform();
		unsigned int f = character.getFrame();

		drawing_tool.drawPose( skeletal_motion, f, thickness, T );
	}
}

static void drawTrajectory( std::vector< math::vector >* trajectory, bool is_connected=false )
{
	float thickness = 1.0f;

	math::vector p_first(0,0,0);
	math::vector p_old(0,0,0);
	std::vector< math::vector >::iterator itor_p = trajectory->begin();
	while( itor_p != trajectory->end() )
	{
		math::vector p = ( *itor_p ++ );

		if( !is_connected )
		{
			drawing_tool.drawSphere( math::position(p.x(), p.y(), p.z()), thickness );
		}
		else
		{
			if( p_old == math::vector(0,0,0) )
			{
				drawing_tool.drawSphere( math::position(p.x(), p.y(), p.z()), thickness );
				p_first = p;
			}
			else
			{
				drawing_tool.drawCylinder( math::position(p_old.x(), p_old.y(), p_old.z()), math::position(p.x(),p.y(),p.z()), thickness );
			}
		}
		p_old = p;
	}
//	drawing_tool.drawCylinder( math::position(p_old.x(), p_old.y(), p_old.z()), math::position(p_first.x(), p_first.y(), p_first.z()), thickness );
}

static void drawPinchedInfo()
{
	float thickness = 5.0f;

	if( !is_pinched )
	{
		return;
	}

	//
	glEnable( GL_BLEND );

	if( !selected_orbit )
	{
		PoseData* pose = pinched_character->getCurrentPose();
		math::transq T = pinched_character->getTransform();

//		drawing_tool.setColor( 0.5, 0.5, 0.5, 0.25 );
//		drawing_tool.drawPose( motion_data.getSkeleton(), pose, thickness, T );

		drawing_tool.setColor( 0.5, 0.5, 0.5, 0.25 );
		drawing_tool.drawPose( motion_data.getSkeleton(), edited_pose, thickness, edited_transform );
	}
	else
	{
		drawing_tool.setColor( 1, 0, 0, 0.5 );

		std::map< OrbitGraph::Node*, std::vector< std::pair<math::transq, PoseData*> >* >::iterator itor_p = transition_poses.find( selected_orbit );
		if( itor_p != transition_poses.end() )
		{
			std::vector< std::pair<math::transq, PoseData*> >* selected_poses = itor_p->second;
				
			if( selected_frame <= selected_poses->size()-1 )
			{
				math::transq T = ( *selected_poses )[ selected_frame ].first;
				PoseData* pose = ( *selected_poses )[ selected_frame ].second;

				drawing_tool.drawPose( motion_data.getSkeleton(), pose, thickness, T );
			}
		}

		//
		if( selected_transition_poses )
		{
			drawing_tool.setColor( 0.5, 0.5, 0.5, 0.5 );

			unsigned int f = 0;

			std::vector< std::pair<math::transq, PoseData*> >::iterator itor_p = selected_transition_poses->begin();
			while( itor_p != selected_transition_poses->end() )
			{
				math::transq T = itor_p->first;
				PoseData* pose = itor_p->second;
				itor_p ++;
				f ++;

				if( f == 1 )
				{
//					drawing_tool.drawPose( motion_data.getSkeleton(), pose, thickness, T );
				}
				if( itor_p == selected_transition_poses->end() )
				{
//					drawing_tool.drawPose( motion_data.getSkeleton(), pose, thickness, T );
				}
			}
		}

		//
		if( selected_transition_trajectory )
		{
			math::vector start_color( 1, 1, 1 );
			math::vector end_color( 1, 0, 0 );
			
			unsigned int num_frames = selected_transition_trajectory->size(), f;
			
			for( f=0; f < num_frames-1; f++ )
			{
				math::vector p0 = ( *selected_transition_trajectory )[ f+0 ];
				math::vector p1 = ( *selected_transition_trajectory )[ f+1 ];

				float t = (float)f / (float)( num_frames-2 );
				math::vector color = math::interpolate( t, start_color, end_color );

				drawing_tool.setColor( color.x(), color.y(), color.z(), 1 );
				drawing_tool.drawCylinder( math::position(p0.x(),p0.y(),p0.z()), math::position(p1.x(),p1.y(),p1.z()), 1.0f );
			}
		}
	}

	glDisable( GL_BLEND );

	//
	float angle = pinched_character->getAngle();
	math::quater rot_quat = math::rotate_transq( M_PI/2 + angle, math::y_axis ).rotation;

	Vector dV( dragged_location - pinched_location );	
	math::vector dv( dV.x, dV.y, dV.z );
	dv = math::rotate( rot_quat, dv );

	math::vector cursor = joint_location + dv;
	
	drawing_tool.setColor( 1, 0, 0, 1 );
	drawing_tool.drawSphere( math::position( cursor.x(), cursor.y(), cursor.z() ), thickness * 1.0 );

	//
	glEnable( GL_LIGHT1 );

	GLfloat position[] = { cursor.x(), cursor.y(), cursor.z(), 1.0f };
	GLfloat diffuse[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	GLfloat specular[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	GLfloat ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat shininess = 10.0f;

	glLightfv( GL_LIGHT1, GL_POSITION, position );
	glLightfv( GL_LIGHT1, GL_DIFFUSE, diffuse );
	glLightfv( GL_LIGHT1, GL_SPECULAR, specular );
	glLightfv( GL_LIGHT1, GL_AMBIENT, ambient );
	glLightf( GL_LIGHT1, GL_SHININESS, shininess );
	
	glLightf( GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0 );
	glLightf( GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.01 );
	glLightf( GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.001 );

	//
	std::map< OrbitGraph::Node*, std::vector< math::vector >* >::iterator itor_t = transition_trajectories.begin();
	while( itor_t != transition_trajectories.end() )
	{
		OrbitGraph::Node* orbit = itor_t->first;
		std::vector< math::vector >* trajectory = itor_t->second;

		drawing_tool.setColor( 0.5, 0.5, 0.5, 1 );
		drawTrajectory( trajectory );

		itor_t ++;
	}

	//
	glDisable( GL_LIGHT1 );
}

void display()
{
	glClearDepth( 1 );
	glClearColor( 1, 1, 1, 1 );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	setupLight();

	if( !is_pinched )
	{
		float x = character.getX();
		float y = 100.0;
		float z = character.getZ();
		float angle = character.getAngle();

		glTranslatef( 0, 0, -view_distance );

		float rot_mat[16];
		track_ball.BuildRotationMatrix( (float *)rot_mat );
		glMultMatrixf( (float *)rot_mat );
	
//		glTranslatef( -x, -y, -z );
	}
	else
	{
		float x = joint_location.x();
		float y = joint_location.y();
		float z = joint_location.z();
		float angle = pinched_character->getAngle();

		/*
		math::quater rot_quat = math::rotate_transq( M_PI/2 + angle, math::y_axis ).rotation;
		math::vector dv(0, 0, 10);
		dv = math::rotate( rot_quat, dv );
		*/

		glTranslatef( 0, 0, -300 );
		glRotatef( 45.0, 1, 0, 0 );
		glRotatef( -( 90.0 + angle*180.0/M_PI), 0, 1, 0 );
		glTranslatef( -x, -y, -z );
	}

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

	if( !is_pinched )
	{
		drawCharacter();
	}
	else
	{
		drawPinchedInfo();
	}

	glPopMatrix();

	glDisable( GL_STENCIL_TEST );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	
	drawFloor();

	glDisable( GL_BLEND );

	if( !is_pinched )
	{
		drawCharacter();
	}
	else
	{
		drawPinchedInfo();
	}

	//
	drawTrail();
	drawHands();

	//
	MotionGraph::Node* curr_node = character.getPathNode( 0 );

	char frame_str[128];

	switch( moving_mode ) {
	case CYCLING_IN_ORBIT:
		{
			unsigned int orbit_index = orbit_graph.getNodeIndex( current_orbit );
			unsigned int cycle_size = current_orbit->getCycleSize();
			unsigned int cycle_phase = current_orbit->getPhase( curr_node );

			sprintf( frame_str, "CYCLING IN ORBIT: orbit = %d, phase = %d/%d", orbit_index, cycle_phase, cycle_size );
		}
		break;

	case LEAVING_FROM_ORBIT:
		{
			unsigned int orbit_index = orbit_graph.getNodeIndex( current_orbit );
			unsigned int curr_phase = current_orbit->getPhase( curr_node );
			unsigned int dest_phase = current_orbit->getPhase( current_transit->startInPath() );

			sprintf( frame_str, "LEAVING FROM ORBIT: orbit = %d, phase = %d -> %d", orbit_index, curr_phase, dest_phase );
		}
		break;

	case ENTERING_INTO_ORBIT:
		{
			unsigned int from_orbit = orbit_graph.getNodeIndex( current_transit->getFromNode() );
			unsigned int to_orbit = orbit_graph.getNodeIndex( current_transit->getToNode() );
			unsigned int path_length = current_transit->getPathLength();
			unsigned int path_phase = current_transit->getPhase( curr_node );

			sprintf( frame_str, "ENTERING INTO ORBIT: prev orbit = %d, next orbit = %d, phase = %d/%d", from_orbit, to_orbit, path_phase, path_length );
		}
		break;
	}
	drawing_tool.drawText( 0, 0, GLUT_BITMAP_TIMES_ROMAN_24, frame_str );

	//
	glutSwapBuffers();
}

static inline bool sortOrbitEdges( OrbitGraph::Edge* lhs, OrbitGraph::Edge* rhs )
{
	return ( lhs->getPathLength() < rhs->getPathLength() );
}

static void collectCycle( float x, float z, float angle, unsigned int joint_index, OrbitGraph::Node* orbit, std::vector< math::vector >* traj, std::vector< std::pair<math::transq, PoseData*> >* pose_list, bool is_moving=false, unsigned int shift=0 )
{
	Character ch;
	ch.embody( &motion_data, &motion_graph );

	std::vector< MotionGraph::Node* >* cycle = orbit->getCycle();
	MotionGraph::Node* start_node = ( *cycle )[0];
	unsigned int start_frame = start_node->getSegment( 0 ).first;
	unsigned int root_index = motion_data.getSkeleton()->getRootJoint()->getIndex();

	ch.extendPath( start_node );
	ch.place( x, z, angle );

	MotionGraph::Node* node = start_node;
	unsigned int frame = start_frame;

	unsigned int f;
	for( f=0; f < shift; f++ )
	{
		unsigned int node_path_len = ch.getNodePathLength();
		if( node_path_len <= 1 )
		{
			MotionGraph::Node* next_node = orbit->stepInCycle( node );
			ch.extendPath( next_node );
		}

		ch.update();
		if( joint_index != root_index )
		{
			ch.place( x, z, angle );
		}
		node = ch.getPathNode( 0 );
		frame = ch.getFrame();
	}

	start_node = node;
	start_frame = frame;

	do
	{
		math::transq joint_transq = ch.getGlobalTransform( joint_index );
		traj->push_back( joint_transq.translation );

		math::transq T = ch.getTransform();
		PoseData* pose = new PoseData;
		pose->initialize( motion_data.getNumJoints() );
		pose->copy( ch.getCurrentPose() );
		pose_list->push_back( std::make_pair(T, pose) );

		unsigned int node_path_len = ch.getNodePathLength();
		if( node_path_len <= 1 )
		{
			MotionGraph::Node* next_node = orbit->stepInCycle( node );
			ch.extendPath( next_node );
		}

		ch.update();

		if( joint_index != root_index && !is_moving )
		{
		//	ch.place( x, z, angle );
		}
		
		node = ch.getPathNode( 0 );
		frame = ch.getFrame();
	}
	while( !( node == start_node && frame == start_frame ) );
}

static void startPinch( unsigned int part, Vector position )
{
	is_pinched = true;
	pinched_location = position;
	pinched_part = part;
	pinched_character = character.clone();
	
	pinched_pose = new PoseData;
	pinched_pose->initialize( motion_data.getNumJoints() );
	pinched_pose->copy( pinched_character->getCurrentPose() );

	edited_pose = new PoseData;
	edited_pose->initialize( motion_data.getNumJoints() );
	edited_pose->copy( pinched_character->getCurrentPose() );
	
	edited_transform = pinched_character->getTransform();

	unsigned int human_id = 0;
	switch( pinched_part ) {
	case WHOLE_BODY:	human_id = Human::PELVIS;		break;
	case LEFT_HAND:		human_id = Human::LEFT_PALM;	break;
	case LEFT_FOOT:		human_id = Human::LEFT_FOOT;	break;
	case RIGHT_HAND:	human_id = Human::RIGHT_PALM;	break;
	case RIGHT_FOOT:	human_id = Human::RIGHT_FOOT;	break;
	}
	pinched_joint = motion_data.getSkeleton()->getHumanJoint( human_id )->getIndex();
	joint_location = character.getGlobalTransform( pinched_joint ).translation;
	
	float x = pinched_character->getX();
	float z = pinched_character->getZ();
	float angle = pinched_character->getAngle();

	//
	std::vector< OrbitGraph::Edge* >* adj_edges = current_orbit->getNextEdges();
	std::sort( adj_edges->begin(), adj_edges->end(), sortOrbitEdges );

	std::vector< OrbitGraph::Node* > transition_nodes;
	std::vector< OrbitGraph::Edge* >::iterator itor_e = adj_edges->begin();
	while( itor_e != adj_edges->end() && transition_nodes.size() <= 20 )
	{
		OrbitGraph::Edge* adj_edge = ( *itor_e ++ );
		OrbitGraph::Node* adj_orbit = adj_edge->getToNode();
		transition_nodes.push_back( adj_orbit );

		//
		std::vector< math::vector >* trajectory = new std::vector< math::vector >;
		std::vector< std::pair<math::transq, PoseData*> >* pose_list = new std::vector< std::pair<math::transq, PoseData*> >;

		collectCycle( x, z, angle, pinched_joint, adj_orbit, trajectory, pose_list );
	
		transition_trajectories[ adj_orbit ] = trajectory;
		transition_poses[ adj_orbit ] = pose_list;
	}
	selected_orbit = 0;

	//
	is_playing = false;
}

static void removeSelection()
{
	selected_orbit = 0;
	selected_frame = 0;

	if( selected_transition_trajectory )
	{
		delete selected_transition_trajectory;
		selected_transition_trajectory = 0;
	}

	if( selected_transition_poses )
	{
		std::vector< std::pair<math::transq, PoseData*> >::iterator itor_p = selected_transition_poses->begin();
		while( itor_p != selected_transition_poses->end() )
		{
			PoseData* pose = itor_p->second;
			delete pose;
			itor_p ++;
		}
		delete selected_transition_poses;
		selected_transition_poses = 0;
	}
}

static void endPinch()
{
	if( selected_orbit )
	{
		std::vector< OrbitGraph::Edge* >* transition_edges = orbit_graph.findEdges( current_orbit, selected_orbit );
		current_transit = ( *transition_edges )[ 0 ];
		moving_mode = LEAVING_FROM_ORBIT;
	}

	//
	is_pinched = false;
	pinched_location = Vector(0,0,0);
	dragged_location = Vector(0,0,0);

	pinched_part = NONE;
	pinched_joint = 0;
	joint_location = math::vector(0,0,0);

	delete pinched_character;
	pinched_character = 0;

	delete pinched_pose;
	pinched_pose = 0;

	delete edited_pose;
	edited_pose = 0;

	edited_transform = math::transq( math::quater(1,0,0,0), math::vector(0,0,0) );

	std::map< OrbitGraph::Node*, std::vector< math::vector >* >::iterator itor_t = transition_trajectories.begin();
	while( itor_t != transition_trajectories.end() )
	{
		std::vector< math::vector >* trajectory = itor_t->second;
		delete trajectory;

		itor_t ++;
	}
	transition_trajectories.clear();

	std::map< OrbitGraph::Node*, std::vector< std::pair<math::transq, PoseData*> >* >::iterator itor_pl = transition_poses.begin();
	while( itor_pl != transition_poses.end() )
	{
		std::vector< std::pair<math::transq, PoseData*> >* pose_list = itor_pl->second;

		std::vector< std::pair<math::transq, PoseData*> >::iterator itor_p = pose_list->begin();
		while( itor_p != pose_list->end() )
		{
			PoseData* pose = itor_p->second;
			delete pose;
			itor_p ++;
		}
		pose_list->clear();

		itor_pl ++;
	}
	transition_poses.clear();

	removeSelection();

	//
	is_playing = true;
}

static void inPinch( Vector position )
{
	Vector old_drag = dragged_location;
	Vector new_drag = position;

	//
	Vector dV( new_drag - pinched_location );	
	math::vector dv( dV.x, dV.y, dV.z );

	float angle = pinched_character->getAngle();
	math::quater rot_quat = math::rotate_transq( M_PI/2 + angle, math::y_axis ).rotation;
	dv = math::rotate( rot_quat, dv );

	math::vector cursor = joint_location + dv;

	OrbitGraph::Node* nearest_orbit = 0;
	unsigned int nearest_frame = 0;
	float nearest_dist = +FLT_MAX;

	std::map< OrbitGraph::Node*, std::vector<math::vector>* >::iterator itor_t = transition_trajectories.begin();
	while( itor_t != transition_trajectories.end() )
	{
		OrbitGraph::Node* orbit = itor_t->first;
		std::vector< math::vector >* trajectory = itor_t->second;
		itor_t ++;

		unsigned int f = 0;
		std::vector< math::vector >::iterator itor_p = trajectory->begin();
		while( itor_p != trajectory->end() )
		{
			math::vector point = ( *itor_p ++ );

			float dist = math::vector( cursor-point ).length();
			if( dist < nearest_dist )
			{
				nearest_orbit = orbit;
				nearest_frame = f;
				nearest_dist = dist;
			}
			f ++;
		}
	}
	if( nearest_orbit /*nearest_dist < 10.0f*/ )
	{
		if( selected_orbit != nearest_orbit || selected_frame != nearest_frame )
		{
			removeSelection();

			selected_orbit = nearest_orbit;
			selected_frame = nearest_frame;
			
			selected_transition_trajectory = new std::vector< math::vector >;
			selected_transition_poses = new std::vector< std::pair<math::transq, PoseData*> >;

			float x = pinched_character->getX();
			float z = pinched_character->getZ();
			float angle = pinched_character->getAngle();

			collectCycle( x, z, angle, pinched_joint, selected_orbit, selected_transition_trajectory, selected_transition_poses, true, 0 );	//selected_frame ); 
			
			//
			std::map< OrbitGraph::Node*, std::vector< std::pair<math::transq, PoseData*> >* >::iterator itor_pl = transition_poses.find( selected_orbit );
			if( itor_pl != transition_poses.end() )
			{
				std::vector< std::pair<math::transq, PoseData*> >* pose_list = itor_pl->second;
				if( selected_frame < pose_list->size() )
				{
					PoseData* selected_pose = ( *pose_list )[ selected_frame ].second;
					edited_pose->copy( selected_pose );
				
					edited_transform = ( *pose_list )[ selected_frame ].first;
				}
			}
		}
	}
	else
	{
		removeSelection();
	}

	//
	if( old_drag != Vector(0,0,0) )
	{
		Vector Disp( new_drag - old_drag );
		math::vector disp( Disp.x, Disp.y, Disp.z );
		disp = math::rotate( rot_quat, disp );

		if( pinched_part == WHOLE_BODY )
		{
			float x = edited_transform.translation.x();
			float z = edited_transform.translation.z();

			edited_transform.translation.set_x( x+disp.x() );
			edited_transform.translation.set_z( z+disp.z() );
		}
		else
		{
			QmGeodesic g( math::quater(1,0,0,0), math::y_axis );
			math::quater edited_rotation_inv = edited_transform.rotation.inverse();
			edited_rotation_inv = g.nearest( edited_rotation_inv );

			disp = math::rotate( edited_rotation_inv, disp );

			//
			math::transq root_transq = pinched_character->getTransform();

			unsigned int left_hand = motion_data.getSkeleton()->getHumanJoint( Human::LEFT_PALM )->getIndex();
			unsigned int right_hand = motion_data.getSkeleton()->getHumanJoint( Human::RIGHT_PALM )->getIndex();
			unsigned int left_foot = motion_data.getSkeleton()->getHumanJoint( Human::LEFT_FOOT )->getIndex();
			unsigned int right_foot = motion_data.getSkeleton()->getHumanJoint( Human::RIGHT_FOOT )->getIndex();

			math::transq lh_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), left_hand );
			math::transq rh_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), right_hand );
			math::transq lf_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), left_foot );
			math::transq rf_transq = edited_pose->getGlobalTransform( motion_data.getSkeleton(), right_foot );

			PoseData temp_pose;
			temp_pose.initialize( motion_data.getNumJoints() );
			temp_pose.copy( edited_pose );

			PoseConstraint pose_constraint;
			pose_constraint.addConstraint( Human::LEFT_PALM, lh_transq );
			pose_constraint.addConstraint( Human::RIGHT_PALM, rh_transq );
			pose_constraint.addConstraint( Human::LEFT_FOOT, lf_transq );
			pose_constraint.addConstraint( Human::RIGHT_FOOT, rf_transq );

			switch( pinched_part ) {
			case LEFT_HAND:
				{
					pose_constraint.removeConstraint( Human::LEFT_PALM );
					pose_constraint.addConstraint( Human::LEFT_PALM, lh_transq.translation + disp );
				}
				break;
			case RIGHT_HAND:
				{
					pose_constraint.removeConstraint( Human::RIGHT_PALM );
					pose_constraint.addConstraint( Human::RIGHT_PALM, rh_transq.translation + disp );
				}
				break;
			case LEFT_FOOT:
				{
					pose_constraint.removeConstraint( Human::LEFT_FOOT );
					pose_constraint.addConstraint( Human::LEFT_FOOT, lf_transq.translation + disp );
				}
				break;
			case RIGHT_FOOT:
				{
					pose_constraint.removeConstraint( Human::RIGHT_FOOT );
					pose_constraint.addConstraint( Human::RIGHT_FOOT, rf_transq.translation + disp );
				}
				break;
			}

			PoseIK pose_ik;
			pose_ik.initialize();
			pose_ik.ik_body( motion_data.getSkeleton(), &temp_pose, edited_pose, &pose_constraint, 0.0 ); 
		}
	}
	//
	dragged_location = position;
}

static unsigned int checkPinchedPart( Vector* position )
{
	Frame frame = controller.frame();
	InteractionBox box = frame.interactionBox();
	Vector box_center = box.center();

	HandList hands = frame.hands();
	if( hands.count() == 1 )
	{
		Hand hand = ( *hands.begin() );
		Vector hand_pos = hand.palmPosition();
		float dist_to_center = hand_pos.distanceTo( box_center );

		FingerList fingers = hand.fingers();
		int num_fingers = fingers.count();

		if( num_fingers >= 2 )
		{
			Vector avg_pos(0,0,0);

			for( FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl )
			{
				const Finger finger = *fl;
				Vector finger_pos = finger.tipPosition();
				avg_pos += finger_pos;
			}
			avg_pos /= num_fingers;

			//
			float max_radius = 0.0f;

			for( FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl )
			{
				const Finger finger = *fl;
				Vector finger_pos = finger.tipPosition();
				float radius = finger_pos.distanceTo( avg_pos );
				if( radius > max_radius )
				{
					max_radius = radius;
				}
			}

			//
			if( max_radius < 40 )
			{
				*position = avg_pos;

				Vector palm_normal = hand.palmNormal();
				Vector down_direction(0,-1,0);
				float dot = palm_normal.dot( down_direction );
				float orthogonal = 1 - fabsf( dot - 0 );
				float parallel = 1 - fabsf( dot - 1 );

				if( hand.isLeft() )
				{
					if( orthogonal > parallel )	return LEFT_HAND;
					else						return LEFT_FOOT;
				}
				else
				{
					if( orthogonal > parallel )	return RIGHT_HAND;
					else						return RIGHT_FOOT;
				}

//				return WHOLE_BODY;
			}

			/*
			else
			{
				float min_dist = +FLT_MAX;
				Vector min_pos(0,0,0);

				FingerList::const_iterator fl1 = fingers.begin();
				while( fl1 != fingers.end() )
				{
					const Finger finger1 = *fl1;
					fl1 ++;

					Vector finger1_pos = finger1.tipPosition();

					FingerList::const_iterator fl2 = fl1;
					while( fl2 != fingers.end() )
					{
						const Finger finger2 = *fl2;
						fl2 ++;

						Vector finger2_pos = finger2.tipPosition();

						double dist = finger1_pos.distanceTo( finger2_pos );
						if( dist < min_dist )
						{
							min_dist = dist;
							min_pos = Vector( finger1_pos + finger2_pos ) / 2;
						}
					}
				}

				if( min_dist < 20 )
				{
					*position = min_pos;

					Vector palm_normal = hand.palmNormal();
					Vector down_direction(0,-1,0);
					float dot = palm_normal.dot( down_direction );
					float orthogonal = 1 - fabsf( dot - 0 );
					float parallel = 1 - fabsf( dot - 1 );

					if( hand.isLeft() )
					{
						if( orthogonal > parallel )	return LEFT_HAND;
						else						return LEFT_FOOT;
					}
					else
					{
						if( orthogonal > parallel )	return RIGHT_HAND;
						else						return RIGHT_FOOT;
					}
				}
			}
			*/
		}
	}

	*position = Vector(0,0,0);
	return NONE;
}

static bool checkTapGesture()
{
	Frame frame = controller.frame();	
	GestureList gestures = frame.gestures();
	for( int g=0; g < gestures.count(); g++ )
	{
		if( gestures[g].type() == Gesture::Type::TYPE_KEY_TAP &&
			gestures[g].state() == Gesture::State::STATE_STOP )
		{
			return true;
		}
	}
	return false;
}

static void leapControl()
{
	if( moving_mode != CYCLING_IN_ORBIT )
	{
		return;
	}

	unsigned int part = NONE;
	Vector position(0,0,0);

	part = checkPinchedPart( &position );

	if( !is_pinched )
	{
		if( part != NONE )
		{
			startPinch( part, position );
		}
	}
	else
	{
		if( part != NONE )
		{
			inPinch( position );
		}
		else
		{
			endPinch();
		}
	}
}

void idle()
{
	static unsigned int count = 0;

	//
	//leapControl();

	if( is_pinched )
	{
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
					Vector position = finger.tipPosition();
					inPinch( position );
					break;
				}
			}
		}
	}

	//
	if( is_playing )
	{
		if( count % 1 == 0 )
		{
			unsigned int node_path_len = character.getNodePathLength();
			if( node_path_len <= 1 )
			{
				MotionGraph::Node* curr_node = character.getPathNode( 0 );
				MotionGraph::Node* next_node = 0;

				switch( moving_mode ) {
				case CYCLING_IN_ORBIT:
					{
						next_node = current_orbit->stepInCycle( curr_node );
					}
					break;
					
				case LEAVING_FROM_ORBIT:
					{
						MotionGraph::Node* exit_node = current_transit->startInPath();
						if( curr_node == exit_node )
						{
							moving_mode = ENTERING_INTO_ORBIT;
							current_orbit = 0;
							next_node = current_transit->stepInPath( curr_node );
						}
						else
						{
							next_node = current_orbit->stepInCycle( curr_node );
						}
					}
					break;

				case ENTERING_INTO_ORBIT:
					{
						MotionGraph::Node* enter_node = current_transit->endInPath();
						if( curr_node == enter_node )
						{
							moving_mode = CYCLING_IN_ORBIT;
							current_orbit = current_transit->getToNode();
							current_transit = 0;
							next_node = current_orbit->stepInCycle( curr_node );
						}
						else
						{
							next_node = current_transit->stepInPath( curr_node );
						}
					}
					break;
				}
				character.extendPath( next_node );
			}
			else
			{
				character.update();

				//
				double x = character.getX();
				double z = character.getZ();
				movement_trail.push_back( math::position(x,0,z) );

				if( movement_trail.size() > MAX_TRAIL_LENGTH )
				{
					movement_trail.pop_front();
				}
			}
		}
		count ++;
	}

	glutPostRedisplay();
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
			if( moving_mode == CYCLING_IN_ORBIT )
			{
				std::vector< OrbitGraph::Edge* >* next_edges = current_orbit->getNextEdges();
				unsigned int num_next_edges = (unsigned int)next_edges->size();
				unsigned int edge_index = rand() % num_next_edges;
				current_transit = ( *next_edges )[ edge_index ];

				moving_mode = LEAVING_FROM_ORBIT;

				if( is_pinched )
				{
					endPinch();
				}
			}
		}
		break;
	case 13:	// enter
		{
			character.place( 0, 0, character.getAngle() );
		}
		break;
	case 32:	// space
		{
			is_playing = !is_playing;
		}
		break;

	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
		{
			if( moving_mode == CYCLING_IN_ORBIT )
			{
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
							Vector position = finger.tipPosition();
							startPinch( key-'1'+1, position );
							break;
						}
					}
				}
			}
		}
		break;

	case 'Q':
	case 'q':
	case 'W':
	case 'w':
	case 'A':
	case 'a':
	case 'S':
	case 's':
		{
			int part = 0;
			if( key == 'W' || key == 'w' )		part = 1;	// right hand
			else if( key == 'Q' || key == 'q' )	part = 2;	// left hand
			else if( key == 'S' || key == 's' )	part = 3;	// right foot
			else if( key == 'A' || key == 'a' )	part = 4;	// left foot

			if( moving_mode == CYCLING_IN_ORBIT )
			{
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
							Vector position = finger.tipPosition();
							startPinch( part, position );
							break;
						}
					}
				}
			}
		}
		break;
	}

	glutPostRedisplay();
}

void special( int key, int x, int y )
{
	if( moving_mode != CYCLING_IN_ORBIT )
	{
		return;
	}

	switch(key){
		case GLUT_KEY_LEFT:
		case GLUT_KEY_DOWN:
		case GLUT_KEY_RIGHT:
		case GLUT_KEY_UP:
			break;
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

