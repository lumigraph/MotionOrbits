#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdarg.h>
#include <windows.h>

#include <GL/glew.h>
#include <GL/glut.h>

#include "DrawingTool.h"
#include "trackball.h"

#include "SkeletalMotion.h"
#include "MotionData.h"
#include "MotionEdit.h"
#include "MotionCluster.h"
#include "Skeleton.h"
#include "Joint.h"

#include "Human.h"
#include "PoseConstraint.h"
#include "PoseData.h"
#include "PoseIK.h"

//#define PATH_BVH	"../data/boxing/boxing_shadow_m_edit.bvh"
//#define PATH_BVH	"../data/b-boy/B_boy.bvh"
#define PATH_BVH	"../data/basketball/basketball.bvh"

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
enum {
	PERSPECTIVE_VIEW = 0,
	TOP_VIEW
};
		
static unsigned int view_mode = PERSPECTIVE_VIEW;

//
static bool is_lbutton_down = false;
static bool is_rbutton_down = false;
static int track_x, track_y;

static unsigned int current_frame = 0;
static unsigned int time_interval = 1;
static bool is_playing = false;
static bool is_simplified = false;

//
static QTrackBall		track_ball;

static SkeletalMotion	motion_data;
static MotionEdit		motion_edit;

extern DrawingTool		drawing_tool;

//
static std::vector< MotionSegment* > segment_list;
static unsigned int current_segment = 0;
static const unsigned int MIN_SEGMENT_LENGTH = 10;


//
extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupCMUSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );

// experimenting with segmentation

void startBVHPlayer( int* argcp, char** argv )
{ 			
	atexit( finalize );

	glutInit( argcp, argv );
	glutInitWindowSize( win_width, win_height );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL );
    glutCreateWindow( "Motion Viewer" );

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
	glutTimerFunc( time_interval, timer, 1 );
	glutIdleFunc( idle );
	
	glutMainLoop();
} 

void segmentMotion()
{
	int num_frames = motion_data.getNumFrames();
	if( num_frames > 0 )
	{
		bool* is_lf_contact = new bool[ num_frames ];
		bool* is_rf_contact = new bool[ num_frames ];

		//
		Joint* lf = motion_data.getHumanJoint( Human::LEFT_FOOT );
		Joint* rf = motion_data.getHumanJoint( Human::RIGHT_FOOT );
		Joint* lt = motion_data.getHumanJoint( Human::LEFT_TOE );
		Joint* rt = motion_data.getHumanJoint( Human::RIGHT_TOE );

		double hl = 10, sl = 0.5;
		int f;

		for( f=0; f < num_frames-1; f++ )
		{
			math::position lf_p = motion_data.getPosition( f, lf->getIndex() );
			math::position rf_p = motion_data.getPosition( f, rf->getIndex() );

			math::vector lf_v0 = motion_data.getLinearVelocity( f, lf->getIndex() );
			math::vector rf_v0 = motion_data.getLinearVelocity( f, rf->getIndex() );

			math::vector lf_v1 = motion_data.getLinearVelocity( f+1, lf->getIndex() );
			math::vector rf_v1 = motion_data.getLinearVelocity( f+1, rf->getIndex() );

			//if( lf_a.length() < sl )
			if( lf_p.y() < hl && lf_v0.y() < 0 && lf_v1.y() > 0 )
			{
				is_lf_contact[ f ] = true;
			}
			else
			{
				is_lf_contact[ f ] = false;
			}

			//if( rf_a.length() < sl )
			if( rf_p.y() < hl && rf_v0.y() < 0 && rf_v1.y() > 0 )
			{
				is_rf_contact[ f ] = true;
			}
			else
			{
				is_rf_contact[ f ] = false;
			}
		}

		//
		unsigned int prev_f = 0;
		unsigned int num_segments = 0;

		for( f=1; f < num_frames; f++ )
		{
			if( ( is_rf_contact[ f-1 ] != is_rf_contact[ f ] || is_lf_contact[ f-1 ] != is_lf_contact[ f ] )
				&& f-prev_f >= MIN_SEGMENT_LENGTH )
			{
				segment_list.push_back( new MotionSegment( prev_f, f-1 ) );
				std::cout << "segment[ " << num_segments++ << " ]: " << f-prev_f << " frames ( " << prev_f << "~" << f-1 << " )\n";

				prev_f = f;
			}
		}
	}
}

void initialize()
{
	motion_data.importFromBVH( PATH_BVH );
	
	//setupBoxingSkeleton( motion_data.getSkeleton() );
	//setupBboySkeleton( motion_data.getSkeleton() );
	//setupCMU14Skeleton( motion_data.getSkeleton() );
	setupBasketballSkeleton( motion_data.getSkeleton() );

	//
	segmentMotion();

	//
	float q[4] = { 0, 0, 0, 1 };
	track_ball.SetQuat( q );

	//
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glEnable( GL_LIGHT0 );
	glEnable( GL_NORMALIZE );
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

	GLfloat position[] = { 0.0f, 0.0f, -1.0f, 0.0f };		// directional source
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

void finalize()
{
	std::vector< MotionSegment* >::iterator itor_s = segment_list.begin();
	while( itor_s != segment_list.end() )
	{
		MotionSegment* segment = ( *itor_s ++ );
		delete segment;
	}
	segment_list.clear();
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

void display()
{
	glClearDepth( 1 );
	glClearColor( 1, 1, 1, 1 );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();

	setupLight();

	glTranslatef( 0, 0, -view_distance );

	switch( view_mode ) {
	case PERSPECTIVE_VIEW:
		{
			float rot_mat[16];
			track_ball.BuildRotationMatrix( (float *)rot_mat );
			glMultMatrixf( (float *)rot_mat );
		}
		break;
	case TOP_VIEW:
		{
			glRotatef( 90, 1, 0, 0 );
		}
		break;
	}

	//
	unsigned int num_frames = motion_data.getNumFrames();
	unsigned int f = current_frame % num_frames;
	float thickness = 5.0f;
	//float thickness = 1.0f;

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

//	setupLight();

	drawing_tool.setColor( 1.0, 0.4, 0.0, 1 );
	drawing_tool.drawPose( &motion_data, f, thickness );

	glPopMatrix();

	glDisable( GL_STENCIL_TEST );

//	setupLight();

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	
	drawFloor();

	glDisable( GL_BLEND );

	drawing_tool.setColor( 1.0, 0.4, 0.0, 1 );
	drawing_tool.drawPose( &motion_data, f, thickness );

	char frame_str[128];
	sprintf( frame_str, "Current frame: %5d", f );
	//drawing_tool.drawText( 0, 0, GLUT_BITMAP_TIMES_ROMAN_24, frame_str );

	//
	Joint* lf = motion_data.getHumanJoint( Human::LEFT_FOOT );
	Joint* rf = motion_data.getHumanJoint( Human::RIGHT_FOOT );
	Joint* lt = motion_data.getHumanJoint( Human::LEFT_TOE );
	Joint* rt = motion_data.getHumanJoint( Human::RIGHT_TOE );

	math::vector lf_v = motion_data.getLinearVelocity( f, lf->getIndex() );
	math::vector rf_v = motion_data.getLinearVelocity( f, rf->getIndex() );

	math::position lf_p = motion_data.getPosition( f, lf->getIndex() );
	math::position rf_p = motion_data.getPosition( f, rf->getIndex() );

	math::vector lt_v = motion_data.getLinearVelocity( f, lt->getIndex() );
	math::vector rt_v = motion_data.getLinearVelocity( f, rt->getIndex() );

	math::position lt_p = motion_data.getPosition( f, lt->getIndex() );
	math::position rt_p = motion_data.getPosition( f, rt->getIndex() );

	std::cout << "Foot height: Left( " << lf_p.y() << ", " << lf_v.y() << "), Right( " << rf_p.y() << ", " << rf_v.y() << ")\n";

//	double hl = 6, sl = 0.3;
	double hl = 20, sl = 3.0;

	if( lf_p.y() < hl && fabs(lf_v.y()) < sl && lt_p.y() < hl && fabs(lt_v.y()) < sl )
	{
		drawing_tool.drawText( 0, 0, GLUT_BITMAP_TIMES_ROMAN_24, "Left foot CONTACT" );
	}
	else
	{
		drawing_tool.drawText( 0, 0, GLUT_BITMAP_TIMES_ROMAN_24, "Left foot FLYING" );
	}

	if( rf_p.y() < hl && fabs(rf_v.y()) < sl && rt_p.y() < hl && fabs(rt_v.y()) < sl )
	{
		drawing_tool.drawText( 0, 30, GLUT_BITMAP_TIMES_ROMAN_24, "Right foot CONTACT" );
	}
	else
	{
		drawing_tool.drawText( 0, 30, GLUT_BITMAP_TIMES_ROMAN_24, "Right foot FLYING" );
	}

	//
	glutSwapBuffers();
}

void timer( int timer_id )
{
}

void idle()
{
	static unsigned int count = 0;

	if( is_playing )
	{
		if( count % 10 == 0 )
		{
			current_frame ++;

			//
			MotionSegment* segment = segment_list[ current_segment ];
			if( segment->getEndFrame() == current_frame )
			{
				current_segment ++;
				if( current_segment == segment_list.size() )
				{
					current_segment = 0;
				}
				is_playing = false;
			}
			
			glutPostRedisplay();
		}
		count ++;
	}
}

void keyboard( unsigned char key, int x, int y )
{
	switch( key ) {
	case 9:	// tab
		{
			if( view_mode == PERSPECTIVE_VIEW )	view_mode = TOP_VIEW;
			else								view_mode = PERSPECTIVE_VIEW;
		}
		break;
	case 13:	// enter
		{
			current_frame = 0;
		}
		break;
	case 32:	// space
		{
			is_playing = !is_playing;
		}
	}
	glutPostRedisplay();
}

void special( int key, int x, int y )
{
	switch(key){
		case GLUT_KEY_LEFT:
			{
				current_frame -= 1;
			}
			break;
		case GLUT_KEY_RIGHT:
			{
				current_frame += 1;
			}
			break;
		case GLUT_KEY_UP:
			{
				current_frame -= 10;

				/*
				if( view_distance > 50 )
				{
					view_distance -= 10;
				}
				*/

			}
			break;
		case GLUT_KEY_DOWN:
			{
				current_frame += 10;
				/*
				if( view_distance < 1000 )
				{
					view_distance += 10;
				}
				*/
			}
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
