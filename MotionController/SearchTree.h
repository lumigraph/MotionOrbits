#pragma once

#include <vector>
#include <set>

#include "MotionGraph.h"
#include "mathclass/position.h"
#include "mathclass/transq.h"

class SpacePartition;

class SearchTree
{
public:
	class Node
	{
	public:
		Node( MotionGraph::Node* graph_node, double x1, double z1, double a1, double xN, double zN, double aN, math::transq& T, math::position& P )
		{
			this->graph_node = graph_node;

			this->x1 = x1;
			this->z1 = z1;
			this->a1 = a1;

			this->xN = xN;
			this->zN = zN;
			this->aN = aN;

			this->T = T;
			this->P = P;

			this->parent_node = 0;
			this->dist_to_goal = 0;

			this->depth = 0;
		}
		virtual ~Node()
		{
			removeAllChildren();
		}

		inline MotionGraph::Node* getGraphNode()	{ return graph_node; }

		inline double getStartX()		{ return x1; }
		inline double getStartZ()		{ return z1; }
		inline double getStartAngle()	{ return a1; }

		inline double getEndX()			{ return xN; }
		inline double getEndZ()			{ return zN; }
		inline double getEndAngle()		{ return aN; }

		inline math::transq& getTransform()		{ return T; }
		inline math::position& getPosition()	{ return P; }

		inline int getStartFrame()	{ return graph_node->getSegment(0).first; }
		inline int getEndFrame()	{ return graph_node->getSegment(0).second; }

		//
		inline void setDistanceToGoal( double d )	{ dist_to_goal = d; }
		inline double getDistanceToGoal()			{ return dist_to_goal; }

		//
		inline void setParent( Node* n )	{ parent_node = n; if( parent_node ) depth = parent_node->getDepth()+1; else depth = 0; }
		inline Node* getParent()			{ return parent_node; }

		inline void addChild( Node* n )		{ child_nodes.push_back( n ); n->setParent( this ); }
		inline Node* getChild( int i )		{ if( i < getNumChildren() ) return child_nodes[i];	else return 0; }
		inline std::vector< Node* >* getChildren()	{ return &child_nodes; }

		inline int getNumChildren()		{ return (int)child_nodes.size(); }
		inline bool isLeaf()			{ return ( getNumChildren() == 0 ); }

		inline void removeChild( Node* n )
		{
			std::vector< Node* >::iterator itor_n = child_nodes.begin();
			while( itor_n != child_nodes.end() )
			{
				Node* child = ( *itor_n );
				if( child == n )
				{
					child_nodes.erase( itor_n );
					child->setParent( 0 );
					return;
				}
				itor_n ++;
			}
		}
		inline void removeAllChildren()
		{
			std::vector< Node* >::iterator itor_n = child_nodes.begin();
			while( itor_n != child_nodes.end() )
			{
				Node* child = ( *itor_n ++ );
				child->setParent( 0 );
			}
			child_nodes.clear();
		}

		inline int getDepth()			{ return depth; }

		inline bool isChildExtended( MotionGraph::Node* child_graph_node )
		{
			std::vector< Node* >::iterator itor_c = child_nodes.begin();
			while( itor_c != child_nodes.end() )
			{
				Node* child = ( *itor_c ++ );
				if( child->getGraphNode() == child_graph_node )
				{
					return true;
				}
			}
			return false;
		}

		inline bool isFullyExtended()
		{
			std::vector< MotionGraph::Edge* >* graph_edges = graph_node->getNextEdges();
			int num_graph_edges = (int)graph_edges->size();
			int num_children = this->getNumChildren();

			if( num_children == num_graph_edges )
			{
				return true;
			}
			else
			{
				return false;
			}
		}

	protected:
		MotionGraph::Node* graph_node;

		double x1, z1, a1;
		double xN, zN, aN;

		Node* parent_node;
		std::vector< Node* > child_nodes;
		int depth;

		double dist_to_goal;
		math::transq T;
		math::position P;
	};

	SearchTree();
	virtual ~SearchTree();

	void initialize(  SkeletalMotion* motion, MotionGraph* graph, MotionGraph::Node* graph_node, double start_x, double start_z, double start_angle, double randomness, unsigned int human_joint );
	void finalize();

	bool extend( math::position& goal_point );
	bool shrink( math::position& goal_point ); 

	int extendN( math::position& goal_point, int N );
	int shrinkN( math::position& goal_point, int N );

	inline Node* getRootNode()					{ return root_node; }
//	inline std::set< Node* >* getLeafNodes()	{ return &leaf_nodes; }
	
	void placeFromTrans( unsigned int f, math::transq t, double* x, double* z, double* angle );
	void transFromPlace( unsigned int f, math::transq* t, double x, double z, double angle );

	math::position calcTrackPoint( Node* node, int f );

	double kNN( double x, double z, int K, std::vector< SearchTree::Node* >* neighbors );

	inline SpacePartition* getSpacePartition()	{ return space_partition; }

	void calcLocalCoords( double x, double z, double* local_x, double* local_z );
	void calcGlobalCoords( double local_x, double local_z, double* x, double* z );

protected:
	Node* addNode( Node* parent, MotionGraph::Node* graph_node, double x=0, double z=0, double angle=0 );
	void removeNode( Node* node );

//	void addAsLeaf( Node* node );
//	bool removeAsLeaf( Node* node );
//	bool isLeaf( Node* node );

	void estimateExtendedConfig( Node* parent, MotionGraph::Node* graph_node, math::transq* T, double* x1, double* z1, double* a1, double* xN, double* zN, double* aN );
	double evaluateSamplingQuality( double x, double z );

	//
	SkeletalMotion* motion;
	MotionGraph* graph;

	double start_x, start_z;
	double start_angle;
	double randomness;

	unsigned int human_joint;
	unsigned int joint_index;

	//
	Node* root_node;
	int num_nodes;
//	std::set< Node* > leaf_nodes;

	//
	SpacePartition* space_partition;
};
