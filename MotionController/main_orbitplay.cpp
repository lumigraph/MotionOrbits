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
#include "Skeleton.h"
#include "Joint.h"
#include "Human.h"

#include "MotionGraph.h"
#include "OrbitGraph.h"
#include "Character.h"

/*
#define PATH_BVH	"../data/boxing/boxing_shadow_m_edit.bvh"
#define PATH_GRAPH	"../data/boxing/graph.txt"
#define PATH_ORBIT	"../data/boxing/orbit.txt"
*/
/*
#define PATH_BVH	"../data/basketball/basketball.bvh"
#define PATH_GRAPH	"../data/basketball/graph.txt"
#define PATH_ORBIT	"../data/basketball/orbit.txt"
*/

#define PATH_BVH	"../data/b-boy/B_boy.bvh"
#define PATH_GRAPH	"../data/b-boy/graph.txt"
#define PATH_ORBIT	"../data/b-boy/orbit.txt"


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

static OrbitGraph::Node*	current_orbit = 0;
static OrbitGraph::Edge*	current_transit = 0;
static unsigned int			moving_mode = CYCLING_IN_ORBIT;

static std::vector< float >	orbit_energy_levels;

//
extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupCMUSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );


void startOrbitPlayer( int* argcp, char** argv )
{ 			
	atexit( finalize );

	glutInit( argcp, argv );
	glutInitWindowSize( win_width, win_height );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL );
    glutCreateWindow( "Orbit Player" );

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

static void calcOrbitEnergyLevels()
{
	unsigned int pelvis = motion_data.getSkeleton()->getHumanJoint( Human::PELVIS )->getIndex();
	unsigned int lhand = motion_data.getSkeleton()->getHumanJoint( Human::LEFT_PALM )->getIndex();
	unsigned int rhand = motion_data.getSkeleton()->getHumanJoint( Human::RIGHT_PALM )->getIndex();
	unsigned int lfoot = motion_data.getSkeleton()->getHumanJoint( Human::LEFT_FOOT )->getIndex();
	unsigned int rfoot = motion_data.getSkeleton()->getHumanJoint( Human::RIGHT_FOOT )->getIndex();

	std::vector< OrbitGraph::Node* >* orbits = orbit_graph.getNodeList();
	std::vector< OrbitGraph::Node* >::iterator itor_o = orbits->begin();
	while( itor_o != orbits->end() )
	{
		OrbitGraph::Node* orbit = ( *itor_o ++ );

		float e = 0.0;
		unsigned int len = 0;

		std::vector< MotionGraph::Node* >* cycle = orbit->getCycle();
		std::vector< MotionGraph::Node* >::iterator itor_n = cycle->begin();
		while( itor_n != cycle->end() )
		{
			MotionGraph::Node* node = ( *itor_n ++ );
			std::pair<unsigned int, unsigned int> segment = node->getSegment( 0 );
			
			unsigned int f1 = segment.first;
			unsigned int fN = segment.second;
		
			/*
			for( unsigned int f=f1; f <= fN; f++ )
			{
				math::vector v_p = motion_data.getLinearVelocity( f, pelvis );
				math::vector v_lh = motion_data.getLinearVelocity( f, lhand );
				math::vector v_rh = motion_data.getLinearVelocity( f, rhand );
				math::vector v_lf = motion_data.getLinearVelocity( f, lfoot );
				math::vector v_rf = motion_data.getLinearVelocity( f, rfoot );

				float sp = v_p.length();
				float slh = v_lh.length();
				float srh = v_rh.length();
				float slf = v_lf.length();
				float srf = v_rf.length();

				e += ( sp*sp + slh*slh + srh*srh + slf*slf + srf*srf );
			}
			*/
			math::vector disp = 
				motion_data.getGlobalTransform( f1, pelvis ).translation 
				- motion_data.getGlobalTransform( fN, pelvis ).translation;
			e += disp.length();
			len += ( fN-f1+1 );
		}
		orbit_energy_levels.push_back( e / (float)len );
	}
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
	calcOrbitEnergyLevels();

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

	glPopMatrix();

	glDisable( GL_STENCIL_TEST );

	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	
	drawFloor();

	glDisable( GL_BLEND );

	drawCharacter();

	//
	MotionGraph::Node* curr_node = character.getPathNode( 0 );

	char frame_str[128];

	switch( moving_mode ) {
	case CYCLING_IN_ORBIT:
		{
			unsigned int orbit_index = orbit_graph.getNodeIndex( current_orbit );
			unsigned int cycle_size = current_orbit->getCycleSize();
			unsigned int cycle_phase = current_orbit->getPhase( curr_node );

			sprintf( frame_str, "CYCLING IN ORBIT: orbit = %d, phase = %d/%d, energy = %f", orbit_index, cycle_phase, cycle_size, orbit_energy_levels[ orbit_index ] );
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

	/*
	unsigned int orbit_index = orbit_graph.getNodeIndex( current_orbit );
	unsigned int cycle_size = current_orbit->getCycleSize();
	unsigned int phase = current_orbit->getPhase( current_node );

	char frame_str[128];
	sprintf( frame_str, "Current orbit: %d, current phase: %d/%d", orbit_index, phase, cycle_size );
	drawing_tool.drawText( 0, 0, GLUT_BITMAP_TIMES_ROMAN_24, frame_str );
	*/

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
			}

			glutPostRedisplay();
		}
		count ++;
	}
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
				/*
				unsigned int min_dist = (unsigned int)-1;
				OrbitGraph::Edge* min_edge = 0;

				std::vector< OrbitGraph::Edge* >* next_edges = current_orbit->getNextEdges();
				std::vector< OrbitGraph::Edge* >::iterator itor_e = next_edges->begin();
				while( itor_e != next_edges->end() )
				{
					OrbitGraph::Edge* edge = ( *itor_e ++ );
					if( edge->getFromNode() == edge->getToNode() )	continue;

					unsigned int dist = edge->getPathLength();
					if( dist < min_dist )
					{
						min_dist = dist;
						min_edge = edge;
					}
				}
				current_transit = min_edge;
				*/
				
				std::vector< OrbitGraph::Edge* >* next_edges = current_orbit->getNextEdges();
				unsigned int num_next_edges = (unsigned int)next_edges->size();
				unsigned int edge_index = rand() % num_next_edges;
				current_transit = ( *next_edges )[ edge_index ];

				moving_mode = LEAVING_FROM_ORBIT;
			}
		}
		break;
	case 13:	// enter
		{
			character.place( 0, 0, 0 );
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
	if( moving_mode != CYCLING_IN_ORBIT )
	{
		return;
	}

	switch(key){
		case GLUT_KEY_LEFT:
		case GLUT_KEY_DOWN:
		case GLUT_KEY_RIGHT:
		case GLUT_KEY_UP:
			{
				unsigned int from_node_index = orbit_graph.getNodeIndex( current_orbit );
				float from_node_energy = orbit_energy_levels[ from_node_index ];

				std::vector< OrbitGraph::Edge* > candidate_edges;

				std::vector< OrbitGraph::Edge* >* next_edges = current_orbit->getNextEdges();
				std::vector< OrbitGraph::Edge* >::iterator itor_e = next_edges->begin();
				while( itor_e != next_edges->end() )
				{
					OrbitGraph::Edge* edge = ( *itor_e ++ );
					if( edge->getFromNode() == edge->getToNode() )	continue;

					unsigned int to_node_index = orbit_graph.getNodeIndex( edge->getToNode() );
					float to_node_energy = orbit_energy_levels[ to_node_index ];

					if( key == GLUT_KEY_LEFT || key == GLUT_KEY_DOWN )
					{
						if( to_node_energy < from_node_energy )
						{
							candidate_edges.push_back( edge );
						}
					}
					else
					{
						if( to_node_energy > from_node_energy )
						{
							candidate_edges.push_back( edge );
						}
					}
				}

				if( !candidate_edges.empty() )
				{
					unsigned int num_edges = (unsigned int)candidate_edges.size();
					unsigned int index = rand() % num_edges;

					current_transit = candidate_edges[ index ];
					moving_mode = LEAVING_FROM_ORBIT;
				}
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
