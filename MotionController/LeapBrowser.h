#pragma once

#include "Leap.h"

#include <vector>
#include "MotionGraph.h"
#include "SearchTree.h"

#include "mathclass/transq.h"

class SkeletalMotion;
class SearchTree;
class Character;
class PoseData;
class Skeleton;

class LeapListener;

namespace Leap
{
	class Controller;
};

class LeapBrowser
{
public:
	LeapBrowser();
	virtual ~LeapBrowser();

	void initialize( const std::string& path_motion, const std::string& path_graph, void (*setup_skeleton)( Skeleton* ), float x, float z, float angle, int node_index=0 );
	void update();
	void finalize();

	void moveToNext();
	void replayFromFirst();
	void sequenceMotion();
	void exportSequencedMotion( const std::string& path_sequenced );

	//
	void drawHands();

	void drawCharacter();
	void drawSearchTree();
	void drawSpatialPartition();

	void drawTrackedPose();
	void drawTrackedNeighbors();
	void drawTreePath();

	void drawBlendedPose();
	void drawEditedPose();

	//
	inline void setLeapTransform( math::transq& T )		{ leap_transform = T; }
	inline void setLeapTranslation( math::vector& v )	{ leap_transform.translation = v; }
	inline void setLeapRotation( math::quater& q )		{ leap_transform.rotation = q; }

	inline math::transq getLeapTransform()		{ return leap_transform; }
	inline math::vector getLeapTranslation()	{ return leap_transform.translation; }
	inline math::quater getLeapRotation()		{ return leap_transform.rotation; }

	inline void enableTrack()		{ is_tracking_pose = true; }
	inline void disableTrack()		{ is_tracking_pose = false; }
	inline bool isTrackingPose()	{ return is_tracking_pose; }

	inline bool isAnimatingCharacter()	{ return is_animating_character; }
	inline Character* getCharacter()	{ return character; }

	inline bool isCursorLocated()					{ return is_cursor_located; }
	inline math::position getCursorLocation()		{ return cursor_location; }

	inline bool isModifyingLookAt()					{ return is_modifying_lookat; }
	inline math::position getCurrentLookAt()		{ return current_lookat; }

	//
	inline SkeletalMotion* getSequencedMotion()		{ return sequenced_motion; }
	inline SkeletalMotion* getPlayedBackMotion()	{ return played_back_motion; }

	//
	inline void setNumNeighborsToTrack( int k )		{ num_neighbors_to_track = k; }
	inline int getNumNeighborsToTrack()				{ return num_neighbors_to_track; }

	inline void setNumPosesToBlend( int k )			{ num_poses_to_blend = k; }
	inline int getNumPosesToBlend()					{ return num_poses_to_blend; }

	inline void setNumExtendsPerUpdate( int n )		{ num_extends_per_update = n; }
	inline int getNumExtendsPerUpdate()				{ return num_extends_per_update; }

	inline void setNumTracksPerUpdate( int n )		{ num_tracks_per_update = n; }
	inline int getNumTracksPerUpdate()				{ return num_tracks_per_update; }

protected:
	void updateCharacter();
	void updateCursorLocation();
	void updateLookAt();

	void updateSearchTree();
	void updateTrackedPose();
	
	void blendNeighborPoses();
	void editBlendedPose();

	double calcEffortForMovement( math::transq& T1, int f1, math::transq& T2, int f2 );
	void resetSearchTree();

	void modifyLookAt( math::position& new_lookat );
//	void interpolateLookAt();

	void startAnimation();
	void endAnimation();

	void drawSearchNodeTopDown( SearchTree::Node* node );
	void drawSearchNodeBottomUp( SearchTree::Node* node );

	//
//	LeapListener*		leap_listener;
	Leap::Controller	leap_controller;

	math::transq		leap_transform;
	math::position		cursor_location;

	SkeletalMotion*		motion_data;
	MotionGraph*		motion_graph;
	SearchTree*			search_tree;
	Character*			character;

	unsigned int		target_human_joint;
	unsigned int		target_joint_index;

	math::position		current_lookat;
	math::position		original_lookat;
	math::position		desired_lookat;

	bool	is_tracking_pose;
	bool	is_animating_character;
	bool	is_modifying_lookat;
	bool	is_cursor_located;

	SearchTree::Node*					tracked_node;
	int									tracked_frame;
	
	std::vector< SearchTree::Node* >										neighbor_nodes;
	std::vector< std::pair< std::pair<SearchTree::Node*, int>, double > >	neighbor_poses;

	PoseData*			blended_pose;
	PoseData*			edited_pose;

	std::vector< MotionGraph::Node*	>	motion_sequence;
	SkeletalMotion*		sequenced_motion;
	SkeletalMotion*		played_back_motion;

	//
	int		num_neighbors_to_track;
	int		num_poses_to_blend;

	int		num_extends_per_update;
	int		num_tracks_per_update;

	//
	int		lookat_animation_duration;
	int		lookat_animation_frame;
};
