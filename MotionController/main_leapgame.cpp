#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdarg.h>
#include <windows.h>

#include <vector>
#include <algorithm>

#include <GL/glew.h>
#include <GL/glut.h>

#include "DrawingTool.h"
#include "trackball.h"

#include "LeapBrowser.h"

#include "BulletWorld.h"
#include "BulletCompound.h"
#include "BulletObject.h"

#include "Character.h"
#include "SkeletalMotion.h"
#include "PoseData.h"
#include "Skeleton.h"
#include "Joint.h"
#include "Human.h"

#include "mathclass/transq.h"
#include "mathclass/QmGeodesic.h"

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

#include "EvaluationLog.h"

//
#define PATH_BOXING_BVH			"../data/boxing/boxing_shadow_m_edit.bvh"
#define PATH_BOXING_GRAPH		"../data/boxing/graph.txt"
#define PATH_BOXING_LOG			"../data/boxing/log.txt"

#define PATH_BASKETBALL_BVH		"../data/basketball/shooting_refined.bvh"
#define PATH_BASKETBALL_GRAPH	"../data/basketball/graph.txt"
#define PATH_BASKETBALL_LOG		"../data/basketball/log.txt"

#define PATH_BBOY_BVH			"../data/b-boy/B_boy.bvh"
#define PATH_BBOY_GRAPH			"../data/b-boy/graph.txt"
#define PATH_BBOY_LOG			"../data/b-boy/log.txt"

#define PATH_EDITED_BVH			"../data/edited_motion.bvh"


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
static float view_distance = 750.0f;

//
static bool is_lbutton_down = false;
static bool is_rbutton_down = false;
static int track_x = 0, track_y = 0;

//
static unsigned int time_interval = 1;
static bool is_playing = false;

//
static QTrackBall		track_ball;
extern DrawingTool		drawing_tool;

//
static LeapBrowser		leap_browser;
static BulletWorld		bullet_world;

//
enum {
	BOXING = 0,
	BASKETBALL,
	BBOY,
};

enum {
	FREE_EDITING = 0,
	GAME_PLAYING,
};

enum {
	NEAREST = 0,
	BLENDED,
	DEFORMED
};

static int motion_type = BASKETBALL;
static int interaction_type = FREE_EDITING;
static int view_mode = NEAREST;

static bool is_neighbors_displayed = false;
static bool is_search_tree_displayed = false;
static bool is_spatial_partition_displayed = false;
static bool is_tree_path_displayed = false;

static BulletObject* ball = 0;
static const int MAX_TRAJECTORY_LENGTH = 600;
static std::deque< math::position > ball_trajectory;

static const double FLOOR_SIZE = 1000;

static EvaluationLog evaluation_log;

static bool is_evaluation_started = false;
static bool is_evaluation_ended = false;

static unsigned int start_time = 0;
static unsigned int end_time = 0;

static int num_total_targets = 0;
static int num_solved_targets = 0;

//
extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupCMUSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );


//
void startLeapGame( int* argcp, char** argv )
{ 			
	atexit( finalize );

	glutInit( argcp, argv );
	glutInitWindowSize( win_width, win_height );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL );
    glutCreateWindow( "Leap Game" );

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
	finalize();

	std::string motion_path = "";
	std::string graph_path = "";
	std::string log_path = "";
	void ( *setup_skeleton )( Skeleton* ) = 0;

	switch( motion_type ) {
	case BOXING:
		motion_path = PATH_BOXING_BVH;
		graph_path = PATH_BOXING_GRAPH;
		log_path = PATH_BOXING_LOG;
		setup_skeleton = &setupBoxingSkeleton;
		break;

	case BASKETBALL:
		motion_path = PATH_BASKETBALL_BVH;
		graph_path = PATH_BASKETBALL_GRAPH;
		log_path = PATH_BASKETBALL_LOG;
		setup_skeleton = &setupBasketballSkeleton;
		break;

	case BBOY:
		motion_path = PATH_BBOY_BVH;
		graph_path = PATH_BBOY_GRAPH;
		log_path = PATH_BBOY_LOG;
		setup_skeleton = &setupBboySkeleton;
		break;
	};

	double x = 0, z = 0, angle = 0;
	leap_browser.initialize( motion_path, graph_path, setup_skeleton, x, z, angle );

	//
	bullet_world.initialize();

	BulletObject* floor = bullet_world.addBox( math::position(0,0,0), math::quater(1,0,0,0), math::vector(FLOOR_SIZE,1,FLOOR_SIZE), 0 );
	floor->hide();

	int num_stands = 10;
	double da = 2 * M_PI / num_stands;
	double radius = FLOOR_SIZE/2 * 0.8;

	for( int i=0; i < num_stands; i++ )
	{
		double angle = i * da;				//(double)rand() / (double)RAND_MAX * M_PI * 2;
		double x = radius * cos( angle );	//(double)rand() / (double)RAND_MAX * FLOOR_SIZE - FLOOR_SIZE/2;
		double z = radius * sin( angle );	//(double)rand() / (double)RAND_MAX * FLOOR_SIZE - FLOOR_SIZE/2;

//		double h = (double)rand() / (double)RAND_MAX * 50 + 50;
//		double r = 10;

		double h = 300;	//(double)rand() / (double)RAND_MAX * 50 + 250;
		double r = 20;

//		BulletCompound* punch_target = bullet_world.addPunchTarget( math::position(x,0.5,z), r, h, 100 );
		BulletCompound* basket_stand = bullet_world.addBasketStand( math::position(x,0.5,z), -angle-M_PI/2, r, h, 0 );
	}

	//
	num_total_targets = num_stands;
	num_solved_targets = 0;

	is_evaluation_started = false;
	is_evaluation_ended = false;

	start_time = 0;
	end_time = 0;

	//
	float q[4] = { 0, 0, 0, 1 };
	track_ball.SetQuat( q );

	//
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_NORMALIZE );
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}

void finalize()
{
	bullet_world.finalize();
	leap_browser.finalize();

	//
	std::string log_path = "";

	switch( motion_type ) {
	case BOXING:
		log_path = PATH_BOXING_LOG;
		break;

	case BASKETBALL:
		log_path = PATH_BASKETBALL_LOG;
		break;

	case BBOY:
		log_path = PATH_BBOY_LOG;
		break;
	};
	evaluation_log.save( log_path );
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
	drawing_tool.drawBox( math::position(0,0,0), math::vector(FLOOR_SIZE,1,FLOOR_SIZE) );
}

static void drawScene()
{
	leap_browser.drawHands();
	leap_browser.drawCharacter();

	switch( view_mode ) {
	case NEAREST:	leap_browser.drawTrackedPose();		break;
	case BLENDED:	leap_browser.drawBlendedPose();		break;
	case DEFORMED:	leap_browser.drawEditedPose();		break;
	}

	bullet_world.render();

	if( !ball_trajectory.empty() )
	{
		drawing_tool.setColor( 0, 0, 0, 1 );

		std::deque< math::position >::iterator itor_p = ball_trajectory.begin();
		while( itor_p != ball_trajectory.end() )
		{
			math::position p = ( *itor_p ++ );
			drawing_tool.drawSphere( p, 1 );
		}
	}
}

static void drawVisualAids()
{
	if( is_spatial_partition_displayed )	leap_browser.drawSpatialPartition();
	if( is_search_tree_displayed )			leap_browser.drawSearchTree();
	if( is_neighbors_displayed )			leap_browser.drawTrackedNeighbors();	
	if( is_tree_path_displayed )			leap_browser.drawTreePath();
}

static void drawEvaluationInfo()
{
	char frame_str[128];

	drawing_tool.setColor( 0, 0, 0, 1 );

	int num_remaining_targets = num_total_targets - num_solved_targets;
	sprintf( frame_str, "Number of remaining targets: %d", num_remaining_targets );
	drawing_tool.drawText( 0, 0, GLUT_BITMAP_TIMES_ROMAN_24, frame_str );

	if( is_evaluation_started )
	{
		unsigned int current_time;
		
		if( is_evaluation_ended )
		{
			current_time = end_time;
		}
		else
		{
			current_time = timeGetTime();
		}
		unsigned int milliseconds = current_time - start_time;

		int seconds = (int) (milliseconds / 1000) % 60 ;
		int minutes = (int) ((milliseconds / (1000*60)) % 60);
		int hours   = (int) ((milliseconds / (1000*60*60)) % 24);

		sprintf( frame_str, "Elapsed time: %d min. %d sec.", minutes, seconds );
		drawing_tool.drawText( 0, 30, GLUT_BITMAP_TIMES_ROMAN_24, frame_str );
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

	math::position current_lookat = leap_browser.getCurrentLookAt();
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

//	drawScene();

	glPopMatrix();

	glDisable( GL_STENCIL_TEST );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	
	drawFloor();

	glDisable( GL_BLEND );

	drawScene();

	drawVisualAids();

	drawEvaluationInfo();

	//
	glutSwapBuffers();
}

static void checkBoxingActionEvent()
{
}

static void checkBoxingGameEvent()
{
}

static void checkBasketballActionEvent()
{
	SkeletalMotion* recent_motion = leap_browser.getPlayedBackMotion();
	if( !recent_motion )
	{
		return;
	}

	// check shooting event
	int N = recent_motion->getNumFrames();
	if( N < 3 )
	{
		return;
	}

	Skeleton* skeleton = recent_motion->getSkeleton();

	int h_index = skeleton->getHumanJoint( Human::HEAD )->getIndex();
	int rh_index = skeleton->getHumanJoint( Human::RIGHT_PALM )->getIndex();
	int lh_index = skeleton->getHumanJoint( Human::LEFT_PALM )->getIndex();

	math::transq h0 = recent_motion->getGlobalTransform( N-3, h_index );
	math::transq h1 = recent_motion->getGlobalTransform( N-2, h_index );
	math::transq h2 = recent_motion->getGlobalTransform( N-1, h_index );

	math::transq rh0 = recent_motion->getGlobalTransform( N-3, rh_index );
	math::transq rh1 = recent_motion->getGlobalTransform( N-2, rh_index );
	math::transq rh2 = recent_motion->getGlobalTransform( N-1, rh_index );

	math::transq lh0 = recent_motion->getGlobalTransform( N-3, lh_index );
	math::transq lh1 = recent_motion->getGlobalTransform( N-2, lh_index );
	math::transq lh2 = recent_motion->getGlobalTransform( N-1, lh_index );

	double h0_y = h0.translation.y();
	double h1_y = h1.translation.y();
	double h2_y = h2.translation.y();

	double rh0_y = rh0.translation.y();
	double rh1_y = rh1.translation.y();
	double rh2_y = rh2.translation.y();

	double lh0_y = lh0.translation.y();
	double lh1_y = lh1.translation.y();
	double lh2_y = lh2.translation.y();

	if( rh0_y > h0_y && lh0_y > h0_y && rh1_y > h1_y && lh1_y > h1_y && rh2_y > h2_y && lh2_y > h2_y )
	{
		if( rh1_y > rh0_y && rh1_y > rh2_y )
		{
			// shooting direction
			int lower_index = skeleton->getHumanJoint( Human::LOWER_RIGHT_ARM )->getIndex();
			int upper_index = skeleton->getHumanJoint( Human::UPPER_RIGHT_ARM )->getIndex();

			math::transq lower_arm = recent_motion->getGlobalTransform( N-2, lower_index );
			math::transq upper_arm = recent_motion->getGlobalTransform( N-2, upper_index );

			math::vector v = lower_arm.translation - upper_arm.translation;
			v = math::normalize( v );

			// shooting position
			math::position p = math::position(0,0,0) + rh1.translation + v * 10;  

			// shooting impulse
			double m = 100.0;
			math::vector impulse = m * v * 50;
			btVector3 bt_impulse( impulse.x(), impulse.y(), impulse.z() );

			// SHOOT!
			BulletObject* bullet_obj = bullet_world.addSphere( p, 10.0, m );
			bullet_obj->setColor( 1, 0, 0 );

			btRigidBody* rigid_body = bullet_obj->getRigidBody();
			rigid_body->applyCentralImpulse( bt_impulse );

			//
			if( ball )
			{
				bullet_world.removeObject( ball );
				ball_trajectory.clear();
			}
			ball = bullet_obj;
		}
	}
}

static void checkBasketballGameEvent()
{
	// check goal-in event
	if( ball )
	{
		math::position p = ball->getLocation();
		ball_trajectory.push_back( p );

		if( ball_trajectory.size() > MAX_TRAJECTORY_LENGTH )
		{
			ball_trajectory.pop_front();
		}

		if( p.y() < 0 )
		{
			bullet_world.removeObject( ball );
			ball_trajectory.clear();
			ball = 0;
		}

		//
		std::vector< BulletCompound* >* compounds = bullet_world.getBulletCompounds();
		std::vector< BulletCompound* >::iterator itor_c = compounds->begin();
		while( itor_c != compounds->end() )
		{
			BulletCompound* comp = ( *itor_c ++ );
			if( comp->isCollided() )
			{
				std::vector< BulletObject* >* objects = comp->getBulletObjects();

				BulletObject* board = ( *objects )[ objects->size()-1 ];
				if( board->isCollided() )
				{
					math::vector color = board->getColor();
					if( color != math::vector(0.5, 0.5, 0.5) )
					{
						board->setColor( 0.5, 0.5, 0.5 );
						num_solved_targets ++;
					}

					if( num_solved_targets == num_total_targets )
					{
						end_time = timeGetTime();
						is_evaluation_ended = true;
					}
				}
			}
		}
	}
}

static void checkBBoyActionEvent()
{
}

static void checkBBoyGameEvent()
{
}

static void checkActionEvent()
{
	switch( motion_type ) {
	case BOXING:
		{
			checkBoxingActionEvent();
		}
		break;

	case BASKETBALL:
		{
			checkBasketballActionEvent();
		}
		break;

	case BBOY:
		{
			checkBBoyActionEvent();
		}
		break;
	}
}

static void checkGameEvent()
{
	switch( motion_type ) {
	case BOXING:
		{
			checkBoxingGameEvent();
		}
		break;

	case BASKETBALL:
		{
			checkBasketballGameEvent();
		}
		break;

	case BBOY:
		{
			checkBBoyGameEvent();
		}
		break;
	}
}

void idle()
{
	float q[4];
	track_ball.GetQuat( q );

	math::quater rot( q[3], q[0], q[1], q[2] );
	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );
	math::quater rotY1 = g.nearest( rot );
	leap_browser.setLeapRotation( rotY1 );

	//
	leap_browser.update();

	bool is_animating = leap_browser.isAnimatingCharacter();
	if( is_animating )
	{
		checkActionEvent();
	}
	checkGameEvent();

	for( int i=0; i < 6; i++ )
	{
		bullet_world.update();
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
			leap_browser.moveToNext();
			leap_browser.sequenceMotion();
		}
		break;
	case 13:	// enter
		{
			leap_browser.replayFromFirst();
			leap_browser.exportSequencedMotion( PATH_EDITED_BVH );
		}
		break;
	case 32:	// space
		{
			start_time = timeGetTime();
			is_evaluation_started = true;
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

	case GLUT_KEY_UP:		view_distance -= 10;	break;
	case GLUT_KEY_DOWN:		view_distance += 10;	break;
	case GLUT_KEY_LEFT:		break;
	case GLUT_KEY_RIGHT:	break;
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
