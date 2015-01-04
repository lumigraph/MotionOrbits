#pragma once

#include "mathclass/position.h"
#include "mathclass/vector.h"
#include "mathclass/quater.h"
#include "mathclass/transq.h"
#include "SkeletalMotion.h"

#include <GL/glut.h>

class DrawingTool
{
public:
	DrawingTool();
	virtual ~DrawingTool();

	void drawSphere( const math::position& center, float radius );
	void drawCylinder( const math::position& start, const math::position& end, float radius );
	void drawTransbox ( const math::position& p1, const math::position& p2, const math::position& p3, float size );
	void drawBox( const math::position& center, const math::vector& size );
	void drawAxis(float width, float size);

	void drawText( float x, float y, void* font, char* string );
	void drawPose( SkeletalMotion* motion, unsigned int f, float thickness, const math::transq& transform = math::identity_transq );
	void drawPose( Skeleton* skeleton, PoseData* pose, float thickness, const math::transq& transform = math::identity_transq );

	void strokeLine( float x0, float y0, float x1, float y1 );
	void strokeCircle( float cx, float cy, float r );
	void strokeTriangle( float x0, float y0, float x1, float y1, float x2, float y2 );
	void strokeRectangle( float cx, float cy, float w, float h, float a );

	void fillCircle( float cx, float cy, float r );
	void fillTriangle( float x0, float y0, float x1, float y1, float x2, float y2 );
	void fillRectangle( float cx, float cy, float w, float h, float a );

	void setColor( float r, float g, float b, float a );
	void setWindowSize( int w, int h );

	void calcMatrix( const math::transq& transform, float* mat );
	void calcMatrix( const math::vector& translation, float* mat );
	void calcMatrix( const math::quater& rotation, float* mat );
	void calcMatrix( const math::matrix& matrix, float* mat );

protected:
	void applyColor();
	void transColor();

	GLUquadric* quadric;
	float red, green, blue, alpha;
	int win_width, win_height;
};
