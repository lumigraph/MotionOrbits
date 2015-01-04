#include "DrawingTool.h"
#include "mathclass/matrix.h"

#include "Skeleton.h"
#include "Joint.h"
#include "PoseData.h"

#include <GL/glut.h>

DrawingTool::DrawingTool()
{
	quadric = gluNewQuadric();

	red = 0.0f;
	green = 0.0f;
	blue = 0.0f;
	alpha = 1.0f;

	win_width = 100;
	win_height = 100;
}

DrawingTool::~DrawingTool()
{
	gluDeleteQuadric( quadric );
}

void DrawingTool::drawText( float x, float y, void* font, char* string )
{
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	glOrtho( 0, win_width, 0, win_height, -100, 100 );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	glDisable( GL_LIGHTING );

	glColor3f( red, green, blue );

	char* c;
	glRasterPos3f( x, y, 0 );
	for( c = string; *c != '\0'; c++ )
	{
		glutBitmapCharacter( font, *c );
	}

	glEnable( GL_LIGHTING );

	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
}

void DrawingTool::drawSphere( const math::position& center, float radius )
{
	applyColor();

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	glTranslatef( center.x(), center.y(), center.z() );
	glutSolidSphere( radius, 20, 20 );

	glPopMatrix();
}

void DrawingTool::drawTransbox( const math::position &p1, const math::position &p2, const math::position &p3, float size)
{
	float mat[16];
	math::vector t = p3 - p1; math::vector nt = math::normalize(t); double length = t.length(); double center = length/2;	
	math::vector s = p2 - p1;
	math::vector n = math::normalize(s*t);
	math::vector q = math::normalize(t*n);
	math::matrix m = math::matrix(nt, q, n);
	calcMatrix(m, mat);

	transColor();
	applyColor();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef((p1.x() + p3.x())/2,(p1.y() + p3.y())/2,(p1.z() + p3.z())/2);
	glMultMatrixf(mat);
	glScalef(length, size, size);
	glutSolidCube(1);
		glPushMatrix();
			glDisable(GL_LIGHTING);
				glScalef(1/length, 1/size, 1/size);
				drawAxis(2.0f, 30.0f);
			glEnable(GL_LIGHTING);
		glPopMatrix();
	glPopMatrix();
}

void DrawingTool::drawAxis(float width, float size)
{
	glLineWidth(width);

	glColor3f(1.0,0.0,0.0);
	glBegin(GL_LINE_STRIP);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(size, 0.0f, 0.0f);
	glEnd();

	glColor3f(0.0,1.0,0.0);
	glBegin(GL_LINE_STRIP);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, size, 0.0f);
	glEnd();

	glColor3f(0.0,0.0,1.0);
	glBegin(GL_LINE_STRIP);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, size);
	glEnd();
}

void DrawingTool::transColor()
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ZERO);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}
void DrawingTool::drawCylinder( const math::position& start, const math::position& end, float radius )
{
	if( start == end )
	{
		return;
	}

	math::vector dir( end-start );
	float length = dir.length();
	dir /= length;

	math::quater q;
	if( dir.x()==0 && dir.y()==0 )
	{
		if( dir.z() > 0 )	q = math::quater(1,0,0,0);
		else				q = math::quater(0,0,1,0);
	}
	else
	{
		q = math::quater( math::vector(0,0,1), dir );
	}

	float mat[16];
	calcMatrix( q, mat );
	
	
	applyColor();

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	glTranslatef( start.x(), start.y(), start.z() );
	glMultMatrixf( mat );	
	gluCylinder( quadric, radius, radius, length, 20, 1 );		// bottom_z=0, top_z=+length
	//glutWireCube(1);
	glPopMatrix();
}

void DrawingTool::drawBox( const math::position& center, const math::vector& size )
{
	applyColor();

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	glTranslatef( center.x(), center.y(), center.z() );
	glScalef( size.x(), size.y(), size.z() );
	glutSolidCube( 1 );

	glPopMatrix();
}

void DrawingTool::setColor( float r, float g, float b, float a )
{
	red = r;
	green = g;
	blue = b;
	alpha = a; 
}

void DrawingTool::setWindowSize( int w, int h )
{
	win_width = w;
	win_height = h;
}

void DrawingTool::applyColor()
{
	GLfloat ambient[4] = { red * 0.5f, green * 0.5f, blue * 0.5f, 1.0f };
	GLfloat diffuse[4] = { red, green, blue, alpha };
	GLfloat specular[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat shininess = 100.0f;

	glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, ambient );
	glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse );
	glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, specular );
	glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, shininess );
}

void DrawingTool::calcMatrix( const math::quater& q, float* mat )
{
	math::matrix m = math::Quater2Matrix( q );
	calcMatrix( m, mat );
}

void DrawingTool::calcMatrix( const math::matrix& m, float* mat )
{
	for( unsigned int c=0; c < 4; c++ )
	{
		for( unsigned int r=0; r < 4; r++ )
		{
			unsigned int i = c * 4 + r;
			float v = 0.0f;
			if( r < 3 && c < 3 )
			{
				v = m.getValue( c, r );
			}
			else
			{
				v = ( r==3 && c==3 ? 1.0f : 0.0f );
			}
			mat[i] = v;
		}
	}
}

void DrawingTool::drawPose( SkeletalMotion* motion, unsigned int f, float thickness, const math::transq& transform )
{
	unsigned int num_joints = motion->getNumJoints();
	unsigned int i;
	for( i= 0; i < num_joints; i++ )
	{
		Joint* child = motion->getJointByIndex( i );
		math::position pi(0,0,0);
		math::transq ti = motion->getGlobalTransform( f, i, transform );
		pi *= ti;

		drawSphere( pi, thickness );

		Joint* parent = child->getParent();
		if( parent )
		{
			unsigned int j = parent->getIndex();
			math::position pj(0,0,0);
			math::transq tj = motion->getGlobalTransform( f, j, transform );
			pj *= tj;
			drawCylinder( pi, pj, thickness );
		}
	}
}

void DrawingTool::drawPose( Skeleton* skeleton, PoseData* pose, float thickness, const math::transq& transform )
{
	unsigned int num_joints = pose->getNumJoints();
	unsigned int i;
	for( i= 0; i < num_joints; i++ )
	{
		Joint* child = skeleton->getJointByIndex( i );
		math::position pi(0,0,0);
		math::transq ti = pose->getGlobalTransform( skeleton, i, transform );
		pi *= ti;

		drawSphere( pi, thickness );

		Joint* parent = child->getParent();
		if( parent )
		{
			unsigned int j = parent->getIndex();
			math::position pj(0,0,0);
			math::transq tj = pose->getGlobalTransform( skeleton, j, transform );
			pj *= tj;
			drawCylinder( pi, pj, thickness );
		}
	}
}

//

void DrawingTool::strokeLine( float x0, float y0, float x1, float y1 )
{
	glBegin( GL_LINES );
	glColor3f( red, green, blue );
	glVertex3f( x0, y0, 0 );
	glVertex3f( x1, y1, 0 );
	glEnd();
}

void DrawingTool::fillCircle( float cx, float cy, float r )
{
	unsigned int i = 0, num_sides = 30;
	float angle = 0, da = 2 * M_PI / (float)num_sides;

	glBegin( GL_TRIANGLE_FAN );
	glColor3f( red, green, blue );

	glVertex3f( cx, cy, 0 );
	for( i=0; i < num_sides+1; i++ )
	{
		float x = cx + r * cosf( angle );
		float y = cy + r * sinf( angle );
		glVertex3f( x, y, 0 );

		angle += da;
	}
	glEnd();
}

void DrawingTool::strokeCircle( float cx, float cy, float r )
{
	unsigned int i = 0, num_sides = 30;
	float angle = 0, da = 2 * M_PI / (float)num_sides;

	glBegin( GL_LINE_STRIP );
	glColor3f( red, green, blue );

	for( i=0; i < num_sides+1; i++ )
	{
		float x = cx + r * cosf( angle );
		float y = cy + r * sinf( angle );
		glVertex3f( x, y, 0 );

		angle += da;
	}
	glEnd();
}

void DrawingTool::fillTriangle( float x0, float y0, float x1, float y1, float x2, float y2 )
{
	glBegin( GL_TRIANGLES );
	glColor3f( red, green, blue );

	glVertex3f( x0, y0, 0 );
	glVertex3f( x1, y1, 0 );
	glVertex3f( x2, y2, 0 );

	glEnd();
}

void DrawingTool::strokeTriangle( float x0, float y0, float x1, float y1, float x2, float y2 )
{
	glBegin( GL_LINE_STRIP );
	glColor3f( red, green, blue );

	glVertex3f( x0, y0, 0 );
	glVertex3f( x1, y1, 0 );
	glVertex3f( x2, y2, 0 );
	glVertex3f( x0, y0, 0 );

	glEnd();
}

void DrawingTool::fillRectangle( float cx, float cy, float w, float h, float a )
{
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	glTranslatef( cx, cy, 0 );
	glRotatef( a, 0, 0, 1 );
	glScalef( w/2, h/2, 1 );
	
	glBegin( GL_QUADS );
	glColor3f( red, green, blue );

	glVertex3f( 1, 1, 0 );
	glVertex3f( -1, 1, 0 );
	glVertex3f( -1, -1, 0 );
	glVertex3f( 1, -1, 0 );

	glEnd();

	glPopMatrix();
}

void DrawingTool::strokeRectangle( float cx, float cy, float w, float h, float a )
{
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();

	glTranslatef( cx, cy, 0 );
	glRotatef( a, 0, 0, 1 );
	glScalef( w/2, h/2, 1 );
	
	glBegin( GL_LINE_STRIP );
	glColor3f( red, green, blue );

	glVertex3f( 1, 1, 0 );
	glVertex3f( -1, 1, 0 );
	glVertex3f( -1, -1, 0 );
	glVertex3f( 1, -1, 0 );
	glVertex3f( 1, 1, 0 );

	glEnd();

	glPopMatrix();
}
