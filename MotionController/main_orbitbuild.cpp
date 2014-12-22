#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <stack>
#include <algorithm>

#include "SkeletalMotion.h"
#include "MotionData.h"
#include "Skeleton.h"
#include "Joint.h"
#include "Human.h"

#include "MotionGraph.h"
#include "OrbitGraph.h"

/*
#define PATH_BVH	"../data/boxing/boxing_shadow_m_edit.bvh"
#define PATH_GRAPH	"../data/boxing/graph.txt"
#define PATH_ORBIT	"../data/boxing/orbit.txt"
*/

#define PATH_BVH	"../data/basketball/basketball.bvh"
#define PATH_GRAPH	"../data/basketball/graph.txt"
#define PATH_ORBIT	"../data/basketball/orbit.txt"

/*
#define PATH_BVH	"../data/b-boy/B_boy.bvh"
#define PATH_GRAPH	"../data/b-boy/graph.txt"
#define PATH_ORBIT	"../data/b-boy/orbit.txt"
*/

#define MIN_CYCLE_SIZE	0
#define MAX_CYCLE_SIZE	10

extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );


void startOrbitBuilder( int* argcp, char** argv )
{ 
	SkeletalMotion motion_data;
	bool is_motion_loaded = motion_data.importFromBVH( PATH_BVH );
	if( !is_motion_loaded )
	{
		std::cout << "ERROR: failed to load motion" << std::endl;
		return;
	}

//	setupBboySkeleton( motion_data.getSkeleton() );
//	setupBoxingSkeleton( motion_data.getSkeleton() );
	setupBasketballSkeleton( motion_data.getSkeleton() );

	MotionGraph motion_graph;
	bool is_graph_loaded = motion_graph.load( PATH_GRAPH );
	if( !is_graph_loaded )
	{
		std::cout << "ERROR: failed to load graph" << std::endl;
		return;
	}

	unsigned int num_cycles = motion_graph.getNumCycles();
	if( num_cycles == 0 )
	{
		std::cout << "ERROR: graph has no cycles" << std::endl;
		return;
	}

	//
	OrbitGraph orbit_graph( &motion_graph );

	// step 1: create orbit nodes with the mediums-sized cycles of original motion graph 
	std::cout << "[1] create orbit nodes" << std::endl;

	unsigned int i, j;
	for( i=0; i < num_cycles; i++ )
	{
		std::vector< unsigned int >* cycle = motion_graph.getCycle( i );

		unsigned int cycle_size = (unsigned int)cycle->size();
		if( cycle_size >= MIN_CYCLE_SIZE && cycle_size <= MAX_CYCLE_SIZE )
		{
			std::vector< MotionGraph::Node* > orbit_cycle;
			
			std::vector< unsigned int >::iterator itor_c = cycle->begin();
			while( itor_c != cycle->end() )
			{
				unsigned int index = ( *itor_c ++ );
				MotionGraph::Node* node = motion_graph.getNode( index );
				if( !node )
				{
					std::cout << "ERROR: graph has invalid cycles" << std::endl;
					return;
				}
				orbit_cycle.push_back( node );
			}
			std::cout << "- node(" << (int)orbit_graph.getNumNodes() << ") has " << (int)orbit_cycle.size() << "-sized cycle" << std::endl;
			orbit_graph.addNode( &orbit_cycle );
		}
	}
	std::cout << std::endl;

	// step 2: connect any two orbit nodes by shortest paths of original motion graph between their corresponding cycles
	std::cout << "[2] connect orbit nodes" << std::endl;

	std::vector< OrbitGraph::Node* >* orbit_nodes = orbit_graph.getNodeList();
	
	i = 0;
	std::vector< OrbitGraph::Node* >::iterator itor_fn = orbit_nodes->begin();
	while( itor_fn != orbit_nodes->end() )
	{
		OrbitGraph::Node* from_node = ( *itor_fn ++ );
		std::vector< MotionGraph::Node* >* from_cycle = from_node->getCycle();

		j = 0;
		std::vector< OrbitGraph::Node* >::iterator itor_tn = orbit_nodes->begin();
		while( itor_tn != orbit_nodes->end() )
		{
			OrbitGraph::Node* to_node = ( *itor_tn ++ );
			std::vector< MotionGraph::Node* >* to_cycle = to_node->getCycle();

			//
			MotionGraph::Node* from_last_node = ( *from_cycle )[ from_cycle->size()-1 ];
			MotionGraph::Node* to_first_node = ( *to_cycle )[ 0 ];

			std::deque< MotionGraph::Node* > path;
			motion_graph.findPath( from_last_node, to_first_node, &path );

			std::vector< MotionGraph::Node* > edge_path;
			std::deque< MotionGraph::Node* >::iterator itor_p = path.begin();
			while( itor_p != path.end() )
			{
				MotionGraph::Node* path_node = ( *itor_p ++ );
				edge_path.push_back( path_node );
			}

			std::cout << "- edge(" << i << ", " << j << ") has " << (int)path.size() << "-sized path" << std::endl;

			orbit_graph.addEdge( from_node, to_node, &edge_path );

			j ++;
		}
		i ++;
	}

	orbit_graph.save( PATH_ORBIT );
}
