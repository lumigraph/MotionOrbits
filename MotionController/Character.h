#pragma once

#include "mathclass/position.h"
#include "mathclass/vector.h"
#include "mathclass/quater.h"
#include "mathclass/transq.h"

#include "SkeletalMotion.h"
#include "MotionGraph.h"

#include <deque>

class Character
{
public:
	Character();
	virtual ~Character();

	void embody( SkeletalMotion* m, MotionGraph* g );
	void place( double x, double z, double angle );
	
	void clearPath();										// set N = 0
	unsigned int extendPath( MotionGraph::Node* to_node );	// increase N by k (k is the length of shortest path)
	unsigned int extendPathRandomly( unsigned int k );		// increase N by k with a random path
	unsigned int shortenPath( unsigned int k );				// decrease N by k

	bool update();

	void whereToGo( double* to_x, double* to_z, double* to_angle, MotionGraph::Node* to_node = 0 );

	Character* clone();
	math::transq getGlobalTransform( unsigned int joint_index );
	PoseData* getCurrentPose();

	inline double getX()		{ return x; }
	inline double getZ()		{ return z; }
	inline double getAngle()	{ return angle; }
	inline math::transq getTransform()	{ return transform; }

	inline unsigned int getFrame()		{ return frame; }
	inline unsigned int getStartFrame()	{ return f1; }
	inline unsigned int getEndFrame()	{ return fN; }

	inline unsigned int getNodePathLength()	{ return (unsigned int)node_path.size(); }
	inline MotionGraph::Node* getPathNode( unsigned int i )	{ return node_path[ i ]; }

	inline unsigned int getSegmentPathLength()	{ return (unsigned int)segment_path.size(); }
	inline unsigned int getPathSegment( unsigned int i )	{ return segment_path[ i ]; }

	inline SkeletalMotion*	getSkeletalMotion()	{ return skeletal_motion; }
	inline MotionGraph*		getMotionGraph()	{ return motion_graph; }

	inline bool isInBlend()					{ return is_in_blend; }
	inline unsigned int getBlendFrame()		{ return blend_frame; }
	inline SkeletalMotion* getBlendMotion()	{ return blend_motion; }

protected:
	void proceedToNextFrame();
	void proceedToNextNode();

	void placeFromTrans( SkeletalMotion* m, unsigned int f, math::transq t, double* x, double* z, double* angle );
	void transFromPlace( SkeletalMotion* m, unsigned int f, math::transq* t, double x, double z, double angle );

	void connectSegments();
	void blendMotions();

	void updatePlacement();
	void updateBlendFrame();

	SkeletalMotion* skeletal_motion;
	MotionGraph*	motion_graph;

	double			x, z;
	double			angle;
	unsigned int	frame;
	unsigned int	f1, fN;
	math::transq	transform;

	bool			is_in_blend;
	unsigned int	blend_length;
	unsigned int	blend_frame;
	SkeletalMotion*	blend_motion;

	std::deque< MotionGraph::Node* >	node_path;		// N
	std::deque< unsigned int >			segment_path;	// N
};
