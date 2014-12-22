#include "MotionGraph.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>

#include "SkeletalMotion.h"

#define DEFAULT_POSITION_THRESHOLD			(0.5f)
#define DEFAULT_ORIENTATION_THRESHOLD		(0.1f)


// graph
MotionGraph::MotionGraph()
{
	this->position_threshold = DEFAULT_POSITION_THRESHOLD;
	this->orientation_threshold = DEFAULT_ORIENTATION_THRESHOLD;

	initialize();
}

MotionGraph::~MotionGraph()
{
	finalize();
}

void MotionGraph::initialize()
{
	finalize();
}

void MotionGraph::finalize()
{
	std::vector< Node* >::iterator itor_node = node_list.begin();
	while( itor_node != node_list.end() )
	{
		Node* node = ( *itor_node ++ );
		delete node;
	}
	node_list.clear();

	std::vector< Edge* >::iterator itor_edge = edge_list.begin();
	while( itor_edge != edge_list.end() )
	{
		Edge* edge = ( *itor_edge ++ );
		delete edge;
	}
	edge_list.clear();

	removeAllComponents();
	removeAllCycles();
}

bool MotionGraph::load( std::string path )
{
	finalize();

	std::ifstream is( path.c_str(), std::ios::in );
	if( !is.is_open() )
	{
		return false;
	}

	// node list
	unsigned int num_nodes = 0, n;
	is >> num_nodes;

	for( n=0; n < num_nodes; n++ )
	{
		Node* node = addNode();

		//
		unsigned int num_segments = 0, s;
		is >> num_segments;

		for( s=0; s < num_segments; s++ )
		{
			unsigned int f1, fN;
			is >> f1 >> fN;

			node->addSegment( f1, fN );
		}
	}

	// edge list
	unsigned int num_edges = 0, e;
	is >> num_edges;

	for( e=0; e < num_edges; e++ )
	{
		unsigned int from_index = 0, to_index = 0;
		is >> from_index >> to_index;

		Node* from_node = getNode( from_index );
		Node* to_node = getNode( to_index );
		Edge* edge = addEdge( from_node, to_node );
	}

	// component list
	unsigned int num_components = 0, c;
	is >> num_components;

	for( c=0; c < num_components; c++ )
	{
		std::vector< Node* >* comp = new std::vector< Node* >;
		component_list.push_back( comp );

		unsigned int num_nodes = 0, n;
		is >> num_nodes;

		for( n=0; n < num_nodes; n++ )
		{
			unsigned int index = 0;
			is >> index;

			Node* node = getNode( index );
			node->setComponent( c );
			comp->push_back( node );
		}
	}

	// cycle list
	unsigned int num_cycles = 0, k;
	is >> num_cycles;

	for( k=0; k < num_cycles; k++ )
	{
		std::vector< unsigned int >* cycle = new std::vector< unsigned int >;
		cycle_list.push_back( cycle );

		unsigned int num_nodes = 0, n;
		is >> num_nodes;

		for( n=0; n < num_nodes; n++ )
		{
			unsigned int index = 0;
			is >> index;

			cycle->push_back( index );
		}
	}

	is.close();

	return true;
}

bool MotionGraph::save( std::string path )
{
	std::ofstream os( path.c_str(), std::ios::out );
	if( !os.is_open() )
	{
		return false;
	}

	// node list
	unsigned int num_nodes = (unsigned int)node_list.size();
	os << num_nodes << std::endl;

	std::vector< Node* >::iterator itor_node = node_list.begin();
	while( itor_node != node_list.end() )
	{
		Node* node = ( *itor_node ++ );
		
		//
		std::vector< std::pair<unsigned int, unsigned int> >* segments = node->getSegments();
		unsigned int num_segments = (unsigned int)segments->size();
		os << num_segments << std::endl;

		std::vector< std::pair<unsigned int, unsigned int> >::iterator itor_s = segments->begin();
		while( itor_s != segments->end() )
		{
			unsigned int f1 = itor_s->first;
			unsigned int fN = itor_s->second;
			itor_s ++;

			os << f1 << " " << fN << std::endl;
		}
		os << std::endl;
	}

	// edge list
	unsigned int num_edges = (unsigned int)edge_list.size();
	os << num_edges << std::endl;

	std::vector< Edge* >::iterator itor_edge = edge_list.begin();
	while( itor_edge != edge_list.end() )
	{
		Edge* edge = ( *itor_edge ++ );

		Node* from_node = edge->getFromNode();
		Node* to_node = edge->getToNode();
		unsigned int from_index = getNodeIndex( from_node );
		unsigned int to_index = getNodeIndex( to_node );

		os << from_index << " " << to_index << std::endl;
	}
	os << std::endl;

	// component list
	unsigned int num_components = (unsigned int)component_list.size();
	os << num_components << std::endl;

	std::vector< std::vector<Node*>* >::iterator itor_comp = component_list.begin();
	while( itor_comp != component_list.end() )
	{
		std::vector< Node* >* comp = ( *itor_comp ++ );
		
		unsigned int num_nodes = (unsigned int)comp->size();
		os << num_nodes << std::endl;
		
		std::vector< Node* >::iterator itor_n = comp->begin();
		while( itor_n != comp->end() )
		{
			Node* node = ( *itor_n ++ );
			unsigned int index = getNodeIndex( node );

			os << index << std::endl;
		}
		os << std::endl;
	}
	os << std::endl;

	// cycle list
	unsigned int num_cycles = (unsigned int)cycle_list.size();
	os << num_cycles << std::endl;

	std::vector< std::vector<unsigned int>* >::iterator itor_cycle = cycle_list.begin();
	while( itor_cycle != cycle_list.end() )
	{
		std::vector< unsigned int >* cycle = ( *itor_cycle ++ );

		unsigned int num_nodes = (unsigned int)cycle->size();
		os << num_nodes << std::endl;

		std::vector< unsigned int >::iterator itor_n = cycle->begin();
		while( itor_n != cycle->end() )
		{
			unsigned int index = ( *itor_n ++ );

			os << index << " ";
		}
		os << std::endl;
	}
	os << std::endl;

	os.close();

	return true;
}

MotionGraph::Node* MotionGraph::addNode( unsigned int type )
{
	Node* node = NULL;
	switch( type ) {
		case Node::Type::LOCAL :	node = new LocalNode();		break;
		case Node::Type::GLOBAL :	node = new GlobalNode();	break;
	}
	if( node )
	{
		node_list.push_back( node );
	}

	return node;
}

MotionGraph::Node* MotionGraph::getNode( unsigned int index )
{
	if( index < (unsigned int)node_list.size() )
	{
		return node_list[ index ];
	}
	return NULL;
}

unsigned int MotionGraph::getNodeIndex( Node* node )
{
	unsigned int index = 0;

	std::vector< Node* >::iterator itor_node = node_list.begin();
	while( itor_node != node_list.end() )
	{
		Node* n = ( *itor_node ++ );
		if( n == node )
		{
			break;
		}
		index ++;
	}
	return index;
}

bool MotionGraph::hasNode( Node* node )
{
	std::vector< Node* >::iterator itor_node = node_list.begin();
	while( itor_node != node_list.end() )
	{
		Node* n = ( *itor_node ++ );
		if( n == node )
		{
			return true;
		}
	}
	return false;
}

MotionGraph::Edge* MotionGraph::addEdge( Node* from_node, Node* to_node )
{
	Edge* edge = new Edge( from_node, to_node );
	edge_list.push_back( edge );

	//
	from_node->addNextEdge( edge );
	to_node->addPrevEdge( edge );

	//
	std::vector< Edge* >* edges = findEdges( from_node, to_node );
	if( !edges )
	{
		edges = new std::vector< Edge* >;
		edge_map[ std::make_pair(from_node, to_node) ] = edges;
	}
	edges->push_back( edge );

	//
	return edge;
}

MotionGraph::Edge* MotionGraph::getEdge( unsigned int index )
{
	if( index < (unsigned int)edge_list.size() )
	{
		return edge_list[ index ];
	}
	return NULL;
}

std::vector< MotionGraph::Edge* >* MotionGraph::findEdges( Node* from_node, Node* to_node )
{
	std::map< std::pair<Node*,Node*>, std::vector<Edge*>* >::iterator itor_edge = edge_map.find( std::make_pair(from_node,to_node) );
	if( itor_edge != edge_map.end() )
	{
		return itor_edge->second;
	}
	return NULL;
}

bool MotionGraph::removeEdges( Node* from_node, Node* to_node )
{
	std::map< std::pair<Node*,Node*>, std::vector<Edge*>* >::iterator itor_edge = edge_map.find( std::make_pair(from_node,to_node) );
	if( itor_edge != edge_map.end() )
	{
		std::vector< Edge* >* edges = itor_edge->second;
		std::vector< Edge* >::iterator itor_e = edges->begin();
		while( itor_e != edges->end() )
		{
			Edge* edge = ( *itor_e ++ );
			removeEdge( edge );
		}
		edge_map.erase( itor_edge );
		delete edges;

		return true;
	}
	else
	{
		return false;
	}
}

void MotionGraph::removeEdge( MotionGraph::Edge* edge )
{
	bool is_found = false;
	std::vector< Edge* >::iterator itor_e = edge_list.begin();
	while( itor_e != edge_list.end() )
	{
		Edge* e = ( *itor_e );
		if( edge == e )
		{
			edge_list.erase( itor_e );
			is_found = true;
			break;
		}
		itor_e ++;
	}

	if( is_found )
	{
		Node* from_node = edge->getFromNode();
		Node* to_node = edge->getToNode();

		from_node->removeNextEdge( edge );
		to_node->removePrevEdge( edge );

		delete edge;
	}
}

bool MotionGraph::hasEdge( MotionGraph::Edge* edge )
{
	std::vector< Edge* >::iterator itor_e = edge_list.begin();
	while( itor_e != edge_list.end() )
	{
		Edge* e = ( *itor_e );
		if( edge == e )
		{
			return true;
		}
	}
	return false;
}

//
static bool sort_node_finish( MotionGraph::Node*& rh, MotionGraph::Node*& lh )
{
	return ( rh->dfsGetFinishTime() > lh->dfsGetFinishTime() );
}

static bool sort_component( std::vector<MotionGraph::Node*>*& rh, std::vector<MotionGraph::Node*>*& lh )
{
	return ( (unsigned int)rh->size() > (unsigned int)lh->size() );
}

void MotionGraph::removeAllComponents()
{
	std::vector< std::vector<Node*>* >::iterator itor_c = component_list.begin();
	while( itor_c != component_list.end() )
	{
		std::vector< Node* >* comp = ( *itor_c ++ );
		delete comp;
	}
	component_list.clear();
}

void MotionGraph::decompose()
{
	removeAllComponents();

	// 1. Prepare to depth first search with normal direction
	std::vector< Node* >::iterator itor_node_init = node_list.begin();
	while( itor_node_init != node_list.end() )
	{
		Node* node = ( *itor_node_init ++ );
		node->dfsInit();
	}

	// 2. DFS with normal direction
	unsigned int step = 0;

	std::vector< Node* >::iterator itor_dfs = node_list.begin();
	while( itor_dfs != node_list.end() )
	{
		Node* node = ( *itor_dfs ++ );

		if( node->dfsIsWhite() )
		{
			dfsVisit( node, &step, NULL );
		}
	}

	// 3. Prepare to depth first search with inverse direction

	// a) sort graph with descending finish time
	std::sort( node_list.begin(), node_list.end(), sort_node_finish );

	// b) set the colors of all the nodes to white
	std::vector< Node* >::iterator itor_node_white = node_list.begin();
	while( itor_node_white != node_list.end() )
	{
		Node* node = ( *itor_node_white ++ );
		node->dfsWhite();
	}

	// 5. DFS with inverse direction while finding strongly connected components
	step = 0;

	std::vector< Node* >::iterator itor_dfs_inv = node_list.begin();
	while( itor_dfs_inv != node_list.end() )
	{
		Node* node = ( *itor_dfs_inv ++ );

		if( node->dfsIsWhite() )
		{
			std::vector< Node* >* component = new std::vector< Node* >;
			component_list.push_back( component );

			dfsVisitInverse( node, &step, component );
		}
	}

	// 6. Sort strongly connected components with descending size
	std::sort( component_list.begin(), component_list.end(), sort_component );

	// 7. Set the component indices of all the nodes
	unsigned int component_index = 0;

	std::vector< std::vector<Node*>* >::iterator itor_component = component_list.begin();
	while( itor_component != component_list.end() )
	{
		std::vector< Node* >* nodes = ( *itor_component ++ );

		std::vector< Node* >::iterator itor_node = nodes->begin();
		while( itor_node != nodes->end() )
		{
			Node* node = ( *itor_node ++ );
			node->setComponent( component_index );
		}

		component_index ++;
	}

	// debug
	unsigned int num_nodes = (unsigned int)node_list.size();
	unsigned int num_components = (unsigned int)component_list.size();
	std::cout << "# of components: " << num_components << "\n";

	unsigned int comp_index = 0;
	std::vector< std::vector<Node*>* >::iterator itor_comp = component_list.begin();
	while( itor_comp != component_list.end() )
	{
		std::vector< Node* >* node_list = ( *itor_comp ++ );
		std::cout << "component[" << comp_index++ << "] = " << (unsigned int)node_list->size() << "\n";
	}
}

void MotionGraph::dfsVisit( Node* node, unsigned int* step, std::vector<Node*>* forest )
{
	node->dfsGray();
	node->dfsDiscover( (*step) ++ );

	std::vector< Edge* >* next_edges = node->getNextEdges();
	std::vector< Edge* >::iterator itor_edge = next_edges->begin();
	while( itor_edge != next_edges->end() )
	{
		Edge* next_edge = ( *itor_edge ++ );
		Node* next_node = next_edge->getToNode();

		if( next_node->dfsIsWhite() )
		{
			next_node->dfsSetPred( node );
			dfsVisit( next_node, step, forest );
		}
	}

	node->dfsBlack();
	node->dfsFinish( (*step) ++ );

	if( forest )
	{
		forest->push_back( node );
	}
}

void MotionGraph::dfsVisitInverse( Node* node, unsigned int* step, std::vector<Node*>* forest )
{
	node->dfsGray();
	node->dfsDiscover( (*step) ++ );

	std::vector< Edge* >* prev_edges = node->getPrevEdges();
	std::vector< Edge* >::iterator itor_edge = prev_edges->begin();
	while( itor_edge != prev_edges->end() )
	{
		Edge* prev_edge = ( *itor_edge ++ );
		Node* prev_node = prev_edge->getFromNode();

		if( prev_node->dfsIsWhite() )
		{
			prev_node->dfsSetPred( node );
			dfsVisitInverse( prev_node, step, forest );
		}
	}

	node->dfsBlack();
	node->dfsFinish( (*step) ++ );

	if( forest )
	{
		forest->push_back( node );
	}
}


//

bool MotionGraph::findPath( Node* from_node, Node* to_node, std::deque< Node* >* path )
{
	// initialization
	dijkstraInit();

//	from_node->dijkstraSetDist( 0.0f );
//	from_node->dijkstraSetPred( NULL );
//	from_node->dijkstraGray();

	dijkstra_Q.clear();
	dijkstra_Q.push_back( from_node );
	std::make_heap( dijkstra_Q.begin(), dijkstra_Q.end(), dijkstraCompare );

	// find the shortest path to target node
	bool is_from_node_visited = false;
	while( !dijkstra_Q.empty() )
	{
		Node* node = dijkstraExtractMin();
		if( node == from_node )
		{
			if( !is_from_node_visited )
			{
				is_from_node_visited = true;
			}
			else
			{
				node->dijkstraBlack();
			}
		}
		else
		{
			node->dijkstraBlack();
		}
		dijkstraRelax( node );
	}

	// record the path
	if( to_node )
	{
		path->clear();

		Node* v = to_node;
		do
		{
			path->push_front( v );
			v = v->dijkstraGetPred();
		}
		while( v != from_node && v != NULL );

		if( v == from_node )
		{
			path->push_front( from_node );
			return true;
		}
		else
		{
			path->clear();
			return false;
		}
	}
	else
	{
		return false;
	}
}

//

bool MotionGraph::dijkstraCompare( Node* lh, Node* rh )
{
	if( lh->dijkstraGetDist() > rh->dijkstraGetDist() )
	{
		return true;
	}
	return false;
}

std::vector< MotionGraph::Node* >::iterator MotionGraph::dijkstraFindInQ( Node* node )
{
	std::vector< Node* >::iterator i = dijkstra_Q.begin();
	while( i != dijkstra_Q.end() )
	{
		Node* n = ( *i );
		if( n == node )
		{
			break;
		}
		i ++;
	}
	return i;
}

void MotionGraph::dijkstraInit()
{
	dijkstra_Q.clear();
	dijkstra_source = NULL;

	std::vector< Node* >::iterator itor_node = node_list.begin();
	while( itor_node != node_list.end() )
	{
		Node* node = ( *itor_node ++ );
		node->dijkstraInit();
	}
}

void MotionGraph::dijkstraRelax( Node* node )
{
	float node_dist = node->dijkstraGetDist();

	std::vector< Edge* > edges;
	node->getNextEdgesInsideComponent( &edges );

	std::vector< Node* > adjacent_nodes;
	std::vector< Edge* >::iterator itor_e = edges.begin();
	while( itor_e != edges.end() )
	{
		Edge* edge = ( *itor_e ++ );
		adjacent_nodes.push_back( edge->getToNode() );
	}

	std::vector< Node* >::iterator itor_n = adjacent_nodes.begin();
	while( itor_n != adjacent_nodes.end() )
	{
		Node* adj_node = ( *itor_n ++ );

		if( adj_node->dijkstraIsBlack() )
		{
			continue;
		}

		float dist = 1.0f;	//(float)math::vector( adj_pos-pos ).length();
		float new_dist = node_dist + dist;

		if( adj_node->dijkstraIsWhite() )
		{
			adj_node->dijkstraSetDist( new_dist );
			adj_node->dijkstraSetPred( node );
			adj_node->dijkstraGray();

			dijkstra_Q.push_back( adj_node );
			std::push_heap( dijkstra_Q.begin(), dijkstra_Q.end(), dijkstraCompare );
		}
		else
		{
			float old_dist = adj_node->dijkstraGetDist();
			if( old_dist > new_dist )
			{
				adj_node->dijkstraSetDist( new_dist );
				adj_node->dijkstraSetPred( node );

				std::vector< Node* >::iterator index_Q = dijkstraFindInQ( adj_node );
				if( index_Q != dijkstra_Q.end() )
				{
					std::push_heap( dijkstra_Q.begin(), index_Q+1, dijkstraCompare );
				}
				else
				{
					assert( false );
				}
			}
		}
	}
}

MotionGraph::Node* MotionGraph::dijkstraExtractMin()
{
	if( dijkstra_Q.empty() )
	{
		return NULL;
	}
	Node* n = dijkstra_Q.front();
	std::pop_heap( dijkstra_Q.begin(), dijkstra_Q.end(), dijkstraCompare );
	dijkstra_Q.pop_back();

	return n;
}


//
void MotionGraph::OUTPUT_CYCLE( std::vector<unsigned int>& node_stack )
{
	std::cout << "- cycle[" << (int)cycle_list.size() << "]: # of nodes = " << (int)node_stack.size() << std::endl;

	//
	std::vector< unsigned int >* cycle = new std::vector< unsigned int >;
	cycle_list.push_back( cycle );

	std::vector< unsigned int >::iterator itor_n = node_stack.begin();
	while( itor_n != node_stack.end() )
	{
		unsigned int i = ( *itor_n ++ );
		cycle->push_back( i );
	}
}

void MotionGraph::UNBLOCK( unsigned int u, AdjListT& B, std::vector<unsigned int>& node_stack, std::vector<unsigned int>& blocked )
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

bool MotionGraph::CIRCUIT( unsigned int s, unsigned int v, AdjListT& A, AdjListT& B, std::vector<unsigned int>& node_stack, std::vector<unsigned int>& blocked )
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
			OUTPUT_CYCLE( node_stack );
			f = true;
		}
		else
		{
				if( !blocked[w] )
				{
					if( CIRCUIT( s, w, A, B, node_stack, blocked ) )
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

void MotionGraph::removeAllCycles()
{
	std::vector< std::vector<unsigned int>* >::iterator itor_c = cycle_list.begin();
	while( itor_c != cycle_list.end() )
	{
		std::vector<unsigned int>* cycle = ( *itor_c ++ );
		delete cycle;
	}
	cycle_list.clear();
}

inline static bool sort_cycle( std::vector<unsigned int>* lhs, std::vector<unsigned int>* rhs )
{
	return ( lhs->size() < rhs->size() );
}

void MotionGraph::sortCycles()
{
	std::sort( cycle_list.begin(), cycle_list.end(), sort_cycle );
}

// this method assumes that the graph is a strongly connected component and doesn't include any nodes whose edges point to themselves
void MotionGraph::enumerateAllCycles()
{
	removeAllCycles();

	//
	AdjListT A, B;
	std::vector<unsigned int> node_stack;
	std::vector<unsigned int> blocked;

	unsigned int num_nodes = this->getNumNodes(), i, j;

    A.resize( num_nodes );
    B.resize( num_nodes );
    blocked.resize( num_nodes );

	for( i=0; i < num_nodes; i++ )
	{
		MotionGraph::Node* node = this->getNode( i );
		unsigned int u = i;

		std::vector< MotionGraph::Edge* >* edges = node->getNextEdges();
		unsigned int num_edges = (unsigned int)edges->size();

		for( j=0; j < num_edges; j++ )
		{
			MotionGraph::Node* next_node = ( *edges )[j]->getToNode();
			unsigned int v = this->getNodeIndex( next_node );

			A.at( u ).push_back( v );
		}

		blocked[i] = false;
	}

	for( i=0; i < num_nodes; i++ )
	{
		std::cout << "(*) node[ " << i << "/" << num_nodes << "] is processed" << std::endl;

		CIRCUIT( i, i, A, B, node_stack, blocked );

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

void MotionGraph::sampleRandomCycles( unsigned int n, unsigned int max_len )
{
	unsigned int num_nodes = getNumNodes();
	if( max_len > num_nodes )
	{
		return;
	}

	unsigned int k;
	for( k=0; k < n; k++ )
	{
		bool is_cycle_found = false;

		while( !is_cycle_found )
		{
			unsigned int start_index = rand() % num_nodes;
			MotionGraph::Node* start_node = getNode( start_index );

			std::vector< MotionGraph::Node* > stack_nodes;
			is_cycle_found = sampleRandomCycleFrom( start_node, &stack_nodes, max_len );

			if( is_cycle_found )
			{
				std::cout << "- cycle[" << k << "] is found: ";

				std::vector< unsigned int >* cycle = new std::vector< unsigned int >;
				std::vector< Node* >::iterator itor_sn = stack_nodes.begin();
				while( itor_sn != stack_nodes.end() )
				{
					Node* node = ( *itor_sn ++ );
					unsigned int index = getNodeIndex( node );
					cycle->push_back( index );

					std::cout << index << " ";
				}
				cycle_list.push_back( cycle );

				std::cout << std::endl;
			}
			else
			{
				std::cout << "- cycle[" << k << "] is not found" << std::endl;
			}
		}
	}
}

bool MotionGraph::sampleRandomCycleFrom( Node* start_node, std::vector< Node* >* stack_nodes, unsigned int max_len )
{
	if( !stack_nodes->empty() )
	{
		bool is_crossed = false;
		std::vector< Node* >::iterator itor_sn = stack_nodes->begin();
		while( itor_sn != stack_nodes->end() )
		{
			Node* stack_node = ( *itor_sn );
			if( start_node == stack_node )
			{
				is_crossed = true;
				break;
			}
			itor_sn ++;
		}

		if( is_crossed )
		{
			if( itor_sn == stack_nodes->begin() )
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}

	if( max_len == 0 )
	{
		return false;
	}

	//
	stack_nodes->push_back( start_node );

	//
	std::vector< Node* > adj_nodes;
	std::vector< Edge* >* adj_edges = start_node->getNextEdges();
	std::vector< Edge* >::iterator itor_e = adj_edges->begin();
	while( itor_e != adj_edges->end() )
	{
		Edge* adj_edge = ( *itor_e ++ );
		Node* adj_node = adj_edge->getToNode();
		adj_nodes.push_back( adj_node );
	}
	std::random_shuffle( adj_nodes.begin(), adj_nodes.end() );

	//
	std::vector< Node* >::iterator itor_an = adj_nodes.begin();
	while( itor_an != adj_nodes.end() )
	{
		Node* adj_node = ( *itor_an ++ );

		bool is_cycle_found = sampleRandomCycleFrom( adj_node, stack_nodes, max_len-1 );
		if( is_cycle_found )
		{
			return true;
		}
	}

	//
	stack_nodes->pop_back();

	return false;
}
