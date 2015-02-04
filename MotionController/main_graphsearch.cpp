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

#include "SkeletalMotion.h"
#include "MotionData.h"
#include "MotionEdit.h"
#include "MotionCluster.h"
#include "MotionGraph.h"
#include "Skeleton.h"
#include "Joint.h"

#include "Human.h"
#include "PoseConstraint.h"
#include "PoseData.h"
#include "PoseIK.h"

#include "Character.h"
#include "SearchTree.h"


/*
#define PATH_BVH	"../data/boxing/boxing_shadow_m_edit.bvh"
#define PATH_GRAPH	"../data/boxing/graph.txt"
*/

#define PATH_BVH	"../data/basketball/shooting_refined.bvh"
#define PATH_GRAPH	"../data/basketball/graph.txt"

/*
#define PATH_BVH	"../data/b-boy/B_boy.bvh"
#define PATH_GRAPH	"../data/b-boy/graph.txt"
*/

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
static bool is_simplified = false;

//
static QTrackBall		track_ball;

static SkeletalMotion	motion_data;
static MotionGraph		motion_graph;
static SearchTree		search_tree;
static Character		character;

extern DrawingTool		drawing_tool;


//
extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupCMUSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );


void startGraphSearch( int* argcp, char** argv )
{ 			
	atexit( finalize );

	glutInit( argcp, argv );
	glutInitWindowSize( win_width, win_height );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL );
    glutCreateWindow( "Graph Search" );

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
	
	//setupBoxingSkeleton( motion_data.getSkeleton() );
	//setupBboySkeleton( motion_data.getSkeleton() );
	//setupCMU14Skeleton( motion_data.getSkeleton() );
	setupBasketballSkeleton( motion_data.getSkeleton() );

	//
	character.embody( &motion_data, &motion_graph );
	character.extendPathRandomly( 1 );
	character.place( 0, 0, 0 );

	//
	search_tree.initialize( &motion_data, &motion_graph, character.getPathNode(0), 0, 0, 0, 0, Human::RIGHT_PALM );

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

	if( character.isInBlend() )
	{
		SkeletalMotion* blend_motion = character.getBlendMotion();
		math::transq T = character.getTransform();
		unsigned int f = character.getBlendFrame();

		drawing_tool.setColor( 0, 0.4, 1, 1 );
		drawing_tool.drawPose( blend_motion, f, thickness, T );
	}
	else
	{
		SkeletalMotion* skeletal_motion = character.getSkeletalMotion();
		math::transq T = character.getTransform();
		unsigned int f = character.getFrame();

		drawing_tool.setColor( 1, 0.4, 0, 1 );
		drawing_tool.drawPose( skeletal_motion, f, thickness, T );
	}
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
	
	glColor3f( 0, 0, 0 );
	glBegin( GL_POINTS );
	glVertex3f( x1, 1, z1 );
	glVertex3f( xN, 1, zN );
	glEnd();

	glColor3f( 0, 0, 1 );
	glBegin( GL_LINES );
	glVertex3f( x1, 1, z1 );
	glVertex3f( xN, 1, zN );
	glEnd();

	glColor3f( 1, 0, 0 );
	glBegin( GL_POINTS );
	glVertex3f( p.x(), p.y(), p.z() );
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

static void drawSearchTree()
{
	SearchTree::Node* root = search_tree.getRootNode();
	drawSearchNode( root );
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

	drawSearchTree();

	glPopMatrix();

	glDisable( GL_STENCIL_TEST );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	
	drawFloor();

	glDisable( GL_BLEND );

	drawCharacter();


	//
	glutSwapBuffers();
}

void idle()
{
	static unsigned int count = 0;

	if( is_playing )
	{
		if( count % 10 == 0 )
		{
			unsigned int node_path_len = character.getNodePathLength();
			if( node_path_len <= 1 )
			{
				character.extendPathRandomly( 1 );
			}
			character.update();

			glutPostRedisplay();
		}
		count ++;
	}
	else
	{
		search_tree.extend( math::position(100, 100, 100) );
		search_tree.shrink( math::position(100, 100, 100) );
		
		glutPostRedisplay();
	}
}

void timer( int timer_id )
{
	glutTimerFunc( time_interval, timer, timer_id );
}

static void refreshCharacter()
{
	character.embody( &motion_data, &motion_graph );
	character.extendPathRandomly( 1 );
	character.place( 0, 0, 0 );
}

void keyboard( unsigned char key, int x, int y )
{
	switch( key ) {
	case 9:	// tab
		{
//			search_tree.extend( 100, 100 );
		}
		break;
	case 13:	// enter
		{
//			search_tree.shrink( 100, 100 );
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
