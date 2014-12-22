#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <stack>
#include <algorithm>

#include "SkeletalMotion.h"
#include "MotionData.h"
#include "MotionCluster.h"
#include "MotionGraph.h"
#include "MotionEdit.h"
#include "Skeleton.h"
#include "Joint.h"

#include "Human.h"
#include "PoseConstraint.h"
#include "PoseData.h"
#include "PoseIK.h"

/*
#define PATH_BVH	"../data/boxing/boxing_shadow_m_edit.bvh"
#define PATH_GRAPH	"../data/boxing/graph.txt"

#define MIN_SEGMENT_LENGTH	20
#define MAX_POSE_DISTANCE	2.5f
#define MAX_ROOT_DISTANCE	10.0f
*/

#define PATH_BVH	"../data/basketball/basketball.bvh"
#define PATH_GRAPH	"../data/basketball/graph.txt"

#define MIN_SEGMENT_LENGTH	20
#define MAX_POSE_DISTANCE	3.0f
#define MAX_ROOT_DISTANCE	20.0f

/*
#define PATH_BVH	"../data/b-boy/B_boy.bvh"
#define PATH_GRAPH	"../data/b-boy/graph.txt"

#define MIN_SEGMENT_LENGTH	20
#define MAX_POSE_DISTANCE	3.0f
#define MAX_ROOT_DISTANCE	10.0f
*/


void segmentMotion( SkeletalMotion* motion_data, std::vector< MotionSegment* >* segment_list );
void initializeGraph( SkeletalMotion* motion_data, std::vector< MotionSegment* >* segment_list, MotionGraph* motion_graph ); 
void interweaveGraph( SkeletalMotion* motion_data, MotionGraph* motion_graph );
void leaveLargestSCC( MotionGraph* motion_graph, MotionGraph* connected_graph );
void enumerateCycles( MotionGraph* connected_graph, std::vector< MotionGraph* >* cycle_list );

void exportMotionGraph( MotionGraph* motion_graph );
void exportGraphCycles( std::vector< MotionGraph* >* cycle_list );

//
extern void setupBboySkeleton( Skeleton* s );
extern void setupBoxingSkeleton( Skeleton* s );
extern void setupCMUSkeleton( Skeleton* s );
extern void setupBasketballSkeleton( Skeleton* s );


void startGraphBuilder( int* argcp, char** argv )
{ 
	SkeletalMotion motion_data;
	bool is_loaded = motion_data.importFromBVH( PATH_BVH );
	if( !is_loaded )
	{
		std::cout << "ERROR: failed to load motion" << std::endl;
		return;
	}

//	setupBboySkeleton( motion_data.getSkeleton() );
//	setupBoxingSkeleton( motion_data.getSkeleton() );
	setupBasketballSkeleton( motion_data.getSkeleton() );

	/*
	// (1) segmentation
	std::cout << "[1] segmentation" << std::endl;

	std::vector< MotionSegment* > segment_list;
	segmentMotion( &motion_data, &segment_list );

	std::cout << "- # of segments = " << (int)segment_list.size() << std::endl << std::endl;

	// (2) make each segment a new ndoe & connect each pair of consecutive segments by a new edge
	std::cout << "[2] graph initialization" << std::endl;

	MotionGraph motion_graph;
	initializeGraph( &motion_data, &segment_list, &motion_graph );

	std::cout << "- # of nodes = " << (int)motion_graph.getNumNodes() << std::endl;
	std::cout << "- # of edges = " << (int)motion_graph.getNumEdges() << std::endl << std::endl;

	// (3) check n-by-n connectivity among all segments
	std::cout << "[3] graph interweaving" << std::endl;

	interweaveGraph( &motion_data, &motion_graph );

	std::cout << "- # of nodes = " << (int)motion_graph.getNumNodes() << std::endl;
	std::cout << "- # of edges = " << (int)motion_graph.getNumEdges() << std::endl << std::endl;

	// (4) leave only the largest strongly connected component
	std::cout << "[4] strongly connected component" << std::endl;

	MotionGraph connected_graph;
	leaveLargestSCC( &motion_graph, &connected_graph );

	std::cout << "- # of nodes = " << (int)connected_graph.getNumNodes() << std::endl;
	std::cout << "- # of edges = " << (int)connected_graph.getNumEdges() << std::endl << std::endl;

	connected_graph.save( PATH_GRAPH );
	return;
	*/
	
	MotionGraph connected_graph;
	connected_graph.load( PATH_GRAPH );

	// (5) enumerate all elementary circuits from the leargest component
	std::cout << "[5] cycle enumeration" << std::endl;
//	connected_graph.enumerateAllCycles();
	connected_graph.removeAllCycles();
	connected_graph.sampleRandomCycles( 100, 3 );
	connected_graph.sortCycles();


	// (6) export each circuit into a graph with the asceding order of its size
	std::cout << "[6] exporting graph and cycles" << std::endl;
	connected_graph.save( PATH_GRAPH );
	
}

void segmentMotion( SkeletalMotion* motion_data, std::vector< MotionSegment* >* segment_list )
{
	if( !motion_data )
	{
		return;
	}
	
	int num_frames = motion_data->getNumFrames();
	if( num_frames <= 0 )
	{
		return;
	}

	std::vector< MotionSegment* >::iterator itor_s = segment_list->begin();
	while( itor_s != segment_list->end() )
	{
		MotionSegment* segment = ( *itor_s ++ );
		delete segment;
	}
	segment_list->clear();

	//
	bool* is_lf_contact = new bool[ num_frames ];
	bool* is_rf_contact = new bool[ num_frames ];

	//
	Joint* lf = motion_data->getHumanJoint( Human::LEFT_FOOT );
	Joint* rf = motion_data->getHumanJoint( Human::RIGHT_FOOT );
	Joint* lt = motion_data->getHumanJoint( Human::LEFT_TOE );
	Joint* rt = motion_data->getHumanJoint( Human::RIGHT_TOE );

	double hl = 10, sl = 0.5;
	int f;

	for( f=0; f < num_frames-1; f++ )
	{
		math::position lf_p = motion_data->getPosition( f, lf->getIndex() );
		math::position rf_p = motion_data->getPosition( f, rf->getIndex() );

		math::vector lf_v0 = motion_data->getLinearVelocity( f, lf->getIndex() );
		math::vector rf_v0 = motion_data->getLinearVelocity( f, rf->getIndex() );

		math::vector lf_v1 = motion_data->getLinearVelocity( f+1, lf->getIndex() );
		math::vector rf_v1 = motion_data->getLinearVelocity( f+1, rf->getIndex() );

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
			segment_list->push_back( new MotionSegment( prev_f, f-1 ) );
			std::cout << "segment[ " << num_segments++ << " ]: " << f-prev_f << " frames ( " << prev_f << "~" << f-1 << " )\n";

			prev_f = f;
		}
	}
}

void initializeGraph( SkeletalMotion* motion_data, std::vector< MotionSegment* >* segment_list, MotionGraph* motion_graph )
{
	unsigned int root_index = motion_data->getHumanJoint( Human::PELVIS )->getIndex();
	
	motion_graph->initialize();

	std::vector< MotionSegment* >::iterator itor_s = segment_list->begin();
	while( itor_s != segment_list->end() )
	{
		MotionSegment* segment = ( *itor_s ++ );

		unsigned int f1 = segment->getStartFrame();
		unsigned int fN = segment->getEndFrame();

		bool is_continuous = true;
		for( unsigned int f=f1; f < fN; f++ )
		{
			unsigned int curr_f = f;
			unsigned int next_f = f+1;

			math::vector curr_t = motion_data->getGlobalTransform( curr_f, root_index ).translation;
			math::vector next_t = motion_data->getGlobalTransform( next_f, root_index ).translation;

			double pose_dist = SkeletalMotion::calcDistance( motion_data, curr_f, motion_data, next_f );
			double root_dist = math::vector( next_t-curr_t ).length();

			if( pose_dist > MAX_POSE_DISTANCE ||  root_dist > MAX_ROOT_DISTANCE )
			{
				is_continuous = false;
				break;
			}
		}

		if( is_continuous )
		{		
			std::cout << "- node[" << motion_graph->getNumNodes() << "] is created: " << f1 << "-" << fN << std::endl;
		
			MotionGraph::Node* node = motion_graph->addNode();
			node->addSegment( f1, fN );
		}
	}
	std::cout << std::endl;
}

void interweaveGraph( SkeletalMotion* motion_data, MotionGraph* motion_graph )
{
	std::vector< MotionGraph::Node* >* node_list = motion_graph->getNodeList();

	std::vector< MotionGraph::Node* >::iterator itor_n1 = node_list->begin();
	while( itor_n1 != node_list->end() )
	{
		MotionGraph::Node* node1 = ( *itor_n1 ++ );
		unsigned int i = motion_graph->getNodeIndex( node1 );

		std::vector< std::pair<unsigned int, unsigned int> >* segments1 = node1->getSegments();
		unsigned int s1 = ( *segments1 )[0].first;
		unsigned int e1 = ( *segments1 )[0].second;

		std::vector< MotionGraph::Node* >::iterator itor_n2 = node_list->begin();
		while( itor_n2 != node_list->end() )
		{
			MotionGraph::Node* node2 = ( *itor_n2 ++ );
			unsigned int j = motion_graph->getNodeIndex( node2 );

			if( node1 == node2 )
			{
				continue;
			}
			if( motion_graph->findEdges( node1, node2 ) != 0 )
			{
				continue;
			}

			std::vector< std::pair<unsigned int, unsigned int> >* segments2 = node2->getSegments();
			unsigned int s2 = ( *segments2 )[0].first;
			unsigned int e2 = ( *segments2 )[0].second;

			double dist = SkeletalMotion::calcDistance( motion_data, e1, motion_data, s2 );
			if( dist < MAX_POSE_DISTANCE )
			{
				std::cout << "- node[" << i << "] and node[" << j << "] are connected" << std::endl;

				motion_graph->addEdge( node1, node2 );
			}
		}
	}

	std::cout << std::endl;
}

void leaveLargestSCC( MotionGraph* original_graph, MotionGraph* purified_graph )
{
	original_graph->decompose();
	purified_graph->initialize();

	std::vector< MotionGraph::Node* >* largest_SCC = original_graph->getComponent( 0 );
	std::vector< MotionGraph::Node* >::iterator itor_n = largest_SCC->begin();
	while( itor_n != largest_SCC->end() )
	{
		MotionGraph::Node* node = ( *itor_n ++ );
		std::pair< int, int > segment = node->getSegment( 0 );

		MotionGraph::Node* new_node = purified_graph->addNode();
		new_node->addSegment( segment.first, segment.second );
	}

	int num_nodes = (int)largest_SCC->size();
	int i, j;
	for( i=0; i < num_nodes; i++ )
	{
		for( j=0; j < num_nodes; j++ )
		{
			MotionGraph::Node* original_source = ( *largest_SCC )[ i ];
			MotionGraph::Node* original_target = ( *largest_SCC )[ j ];

			if( original_graph->findEdges( original_source, original_target ) )
			{
				MotionGraph::Node* purified_source = purified_graph->getNode( i );
				MotionGraph::Node* purified_target = purified_graph->getNode( j );

				purified_graph->addEdge( purified_source, purified_target );
			}
		}
	}
}

//
typedef std::vector< std::list<unsigned int> > AdjListT;

void OUTPUT_CYCLE( std::vector<unsigned int>& node_stack, MotionGraph* graph, std::vector< MotionGraph* >* cycle_list )
{
	static unsigned int num_cycles = 0;
	std::cout << "- cycle[" << num_cycles ++ << "]: # of nodes = " << (int)node_stack.size() << std::endl;
	
	//
	MotionGraph* cycle = new MotionGraph;
	cycle_list->push_back( cycle );

	MotionGraph::Node* prev_node = 0;
	MotionGraph::Node* first_node = 0;

	std::vector< unsigned int >::iterator itor_n = node_stack.begin();
	while( itor_n != node_stack.end() )
	{
		unsigned int i = ( *itor_n ++ );
		
		MotionGraph::Node* node = graph->getNode( i );
		std::pair<unsigned int, unsigned int> segment = node->getSegment( 0 );

		MotionGraph::Node* new_node = cycle->addNode();
		new_node->addSegment( segment.first, segment.second );

		if( prev_node )
		{
			cycle->addEdge( prev_node, new_node );
		}

		if( !first_node )
		{
			first_node = new_node;
		}

		prev_node = new_node;
	}

	if( prev_node && first_node )
	{
		cycle->addEdge( prev_node, first_node );
	}
}

void UNBLOCK( unsigned int u, AdjListT& B, std::vector<unsigned int>& node_stack, std::vector<unsigned int>& blocked )
{
	blocked[u] = false;

	std::list< unsigned int >::iterator itor_w = B[u].begin();
	while( itor_w != B[u].end() )
	{
		unsigned int w = ( *itor_w );
		itor_w = B[u].erase( itor_w );

		if( blocked[w] )
		{
			UNBLOCK( w, B, node_stack, blocked );
		}
	}
}

bool CIRCUIT( unsigned int s, unsigned int v, AdjListT& A, AdjListT& B, std::vector<unsigned int>& node_stack, std::vector<unsigned int>& blocked, MotionGraph* graph, std::vector< MotionGraph* >* cycle_list )
{
    bool f = false;

    node_stack.push_back(v);
    blocked[v] = true;

	std::list< unsigned int >::iterator itor_w = A[v].begin();
	while( itor_w != A[v].end() )
	{
		unsigned int w = ( *itor_w ++ );

		if( w == s )
		{
			OUTPUT_CYCLE( node_stack, graph, cycle_list );
			f = true;
		}
		else
		{
			 if( !blocked[w] )
			 {
				 if( CIRCUIT( s, w, A, B, node_stack, blocked, graph, cycle_list ) )
				 {
					 f = true;
				 }
			 }
		}
	}

	if( f )
	{
		UNBLOCK( v, B, node_stack, blocked );
	}
	else
	{
		itor_w = A[v].begin();
		while( itor_w != A[v].end() )
		{
			unsigned int w = ( *itor_w ++ );

			if( std::find( B[w].begin(), B[w].end(), v ) == B[w].end() )
			{
				B[w].push_back( v );
			}
		}
	}

	node_stack.pop_back();

	return f;
}

void enumerateCycles( MotionGraph* connected_graph, std::vector< MotionGraph* >* cycle_list )
{
	AdjListT A, B;
	std::vector<unsigned int> node_stack;
	std::vector<unsigned int> blocked;

	unsigned int num_nodes = connected_graph->getNumNodes(), i, j;

    A.resize( num_nodes );
    B.resize( num_nodes );
    blocked.resize( num_nodes );

	for( i=0; i < num_nodes; i++ )
	{
		MotionGraph::Node* node = connected_graph->getNode( i );
		unsigned int u = i;

		std::vector< MotionGraph::Edge* >* edges = node->getNextEdges();
		unsigned int num_edges = (unsigned int)edges->size();

		for( j=0; j < num_edges; j++ )
		{
			MotionGraph::Node* next_node = ( *edges )[j]->getToNode();
			unsigned int v = connected_graph->getNodeIndex( next_node );

			A.at( u ).push_back( v );
		}

		blocked[i] = false;
	}

	for( i=0; i < num_nodes; i++ )
	{
		std::cout << "(*) node[ " << i << "/" << num_nodes << "] is processed" << std::endl;

		CIRCUIT( i, i, A, B, node_stack, blocked, connected_graph, cycle_list );

		A[i].clear();

		for( j=0; j < num_nodes; j++ )
		{
			A[j].remove( i );
		}

		for( j=0; j < num_nodes; j++ )
		{
			blocked[j] = false;
			B[j].clear();
		}
	}
}

void exportMotionGraph( MotionGraph* motion_graph )
{
	motion_graph->save( PATH_GRAPH );
}

/*
void exportGraphCycles( std::vector< MotionGraph* >* graph_cycles )
{
	int num_cycles = (int)graph_cycles->size(), i;

	for( i=0; i < num_cycles; i++ )
	{
		MotionGraph* cycle = ( *graph_cycles )[ i ];
		
		char path_cycle[ 128 ];
		sprintf( path_cycle, PATH_CYCLES, i );

		cycle->save( path_cycle );
	}
}
*/