#pragma once

#include <vector>
#include <map>
#include <deque>
#include <set>
#include <list>

#include "mathclass/position.h"
#include "mathclass/quater.h"

typedef std::vector< std::list<unsigned int> > AdjListT;

class SkeletalMotion;

class MotionGraph
{
public:
	class Edge;

	class Node
	{
	public:
		enum Type
		{
			NONE = 0,
			LOCAL,
			GLOBAL,
		};

		Node()	
		{
			component = 0;
			type = NONE;
		}
		virtual ~Node()	
		{
		}

		inline void addSegment( unsigned int f1, unsigned int fN )	{ segments.push_back( std::make_pair(f1,fN) ); }
		inline std::pair<unsigned int, unsigned int> getSegment( unsigned int i )	{ return segments[ i ]; }
		inline std::vector< std::pair<unsigned int, unsigned int> >* getSegments()	{ return &segments; }
		inline unsigned int getNumSegments()	{ return (unsigned int)segments.size(); }

		inline void addNextEdge( Edge* edge )	{ next_edges.push_back( edge ); }
		inline void addPrevEdge( Edge* edge )	{ prev_edges.push_back( edge ); }

		inline std::vector< Edge* >*	getNextEdges()	{ return &next_edges; }
		inline std::vector< Edge* >*	getPrevEdges()	{ return &prev_edges; }

		inline Node* getNextNode( unsigned int i ) { if( i < (int)next_edges.size() ) return next_edges[i]->getToNode(); else return 0; }
		inline Node* getPrevNode( unsigned int i ) { if( i < (int)prev_edges.size() ) return prev_edges[i]->getFromNode(); else return 0; }

		inline bool removeNextEdge( Edge* edge )	
		{
			std::vector< Edge* >::iterator itor_next = next_edges.begin();
			while( itor_next != next_edges.end() )
			{
				Edge* e = ( *itor_next );
				if( e == edge )
				{
					next_edges.erase( itor_next );
					return true;
				}
				itor_next ++;
			}
			return false;
		}
		inline bool removePrevEdge( Edge* edge )
		{
			std::vector< Edge* >::iterator itor_prev = prev_edges.begin();
			while( itor_prev != prev_edges.end() )
			{
				Edge* e = ( *itor_prev );
				if( e == edge )
				{
					prev_edges.erase( itor_prev );
					return true;
				}
				itor_prev ++;
			}
			return false;
		}

		inline void getNextEdgesInsideComponent( std::vector< Edge* >* safe_edges )
		{
			std::vector< Edge* >::iterator itor_next = next_edges.begin();
			while( itor_next != next_edges.end() )
			{
				Edge* next_edge = ( *itor_next ++ );
				Node* next_node = next_edge->getToNode();

				if( this->getComponent() == next_node->getComponent() )
				{
					safe_edges->push_back( next_edge );
				}
			}
		}
		inline void getPrevEdgesInsideComponent( std::vector< Edge* >* safe_edges )
		{
			std::vector< Edge* >::iterator itor_prev = prev_edges.begin();
			while( itor_prev != prev_edges.end() )
			{
				Edge* prev_edge = ( *itor_prev ++ );
				Node* prev_node = prev_edge->getToNode();

				if( this->getComponent() == prev_node->getComponent() )
				{
					safe_edges->push_back( prev_edge );
				}
			}
		}

		// DFS
		inline void dfsInit() {
			dfs_color = 0;
			dfs_discover_time = 0;
			dfs_finish_time = 0;
			dfs_pred = NULL;
		}

		inline void dfsWhite()		{ dfs_color = 0; }
		inline void dfsGray()		{ dfs_color = 1; }
		inline void dfsBlack()		{ dfs_color = 2; }

		inline bool dfsIsWhite()	{ return dfs_color==0; }
		inline bool dfsIsGray()		{ return dfs_color==1; }
		inline bool dfsIsBlack()	{ return dfs_color==2; }

		inline void dfsDiscover( unsigned int t )	{ dfs_discover_time = t; }
		inline void dfsFinish( unsigned int t )		{ dfs_finish_time = t; }

		inline unsigned int dfsGetDiscoverTime()	{ return dfs_discover_time; }
		inline unsigned int dfsGetFinishTime()		{ return dfs_finish_time; }

		inline void dfsSetPred( Node* pred )	{ dfs_pred = pred; }
		inline Node* dfsGetPred()				{ return dfs_pred; }

		inline void setComponent( unsigned int c )	{ component = c; }
		inline unsigned int getComponent()			{ return component; }

		// DIJKSTRA
		inline void dijkstraInit() {
			dijkstra_pred = NULL;
			dijkstra_dist = +FLT_MAX;
			dijkstra_color = 0;
		}
		inline void dijkstraSetPred( Node* pred )	{ dijkstra_pred = pred; }
		inline void dijkstraSetDist( float dist )	{ dijkstra_dist = dist; }

		inline Node* dijkstraGetPred()				{ return dijkstra_pred; }
		inline float dijkstraGetDist()				{ return dijkstra_dist; }

		inline void dijkstraWhite()					{ dijkstra_color = 0; }
		inline void dijkstraGray()					{ dijkstra_color = 1; }
		inline void dijkstraBlack()					{ dijkstra_color = 2; }
		inline bool dijkstraIsWhite()				{ return dijkstra_color==0; }
		inline bool dijkstraIsGray()				{ return dijkstra_color==1; }
		inline bool dijkstraIsBlack()				{ return dijkstra_color==2; }

		//
		inline unsigned int getType()	{ return type; }

	protected:
		std::vector< std::pair<unsigned int, unsigned int> >	segments;

		std::vector< Edge* >	next_edges;
		std::vector< Edge* >	prev_edges;

		// DFS
		unsigned int component;

		Node*			dfs_pred;
		unsigned int	dfs_color;
		unsigned int	dfs_discover_time;
		unsigned int	dfs_finish_time;

		// DIJKSTRA (BFS)
		Node*			dijkstra_pred;
		float			dijkstra_dist;
		unsigned int	dijkstra_color;

		//
		unsigned int	type;
	};

	class LocalNode : public Node
	{
	public:
		LocalNode()	: Node()
		{
			type = Node::Type::LOCAL;
		}
		virtual ~LocalNode()
		{
		}
	};

	class GlobalNode : public Node
	{
	public:
		GlobalNode() : Node()
		{
			type = Node::Type::GLOBAL;

			position = math::position(0,0,0);
			orientation = math::quater(1,0,0,0);
		}
		virtual ~GlobalNode()
		{
		}

		inline void setPosition( const math::position& p )		{ position = p; }
		inline const math::position& getPosition()				{ return position; }

		inline void setOrientation( const math::quater& o )		{ orientation = o; }
		inline const math::quater& getOrientation()				{ return orientation; }

	protected:
		math::position	position;
		math::quater	orientation;
	};

	class Edge
	{
	public:
		Edge( Node* fn, Node* tn )	
		{
			from_node = fn;
			to_node = tn;
		}
		~Edge()	
		{
		}
		
		inline Node* getFromNode()	{ return from_node; }
		inline Node* getToNode()	{ return to_node; }

	protected:
		Node* from_node;
		Node* to_node;
	};

	MotionGraph();
	~MotionGraph();

	bool load( std::string path );
	bool save( std::string path );

	void initialize();
	void finalize();

	Node* addNode( unsigned int type = MotionGraph::Node::Type::LOCAL );
	Node* getNode( unsigned int index );
	unsigned int getNodeIndex( Node* node );
	bool hasNode( Node* node );

	Edge* addEdge( Node* from_node, Node* to_node );
	Edge* getEdge( unsigned int index );
	std::vector< Edge* >* findEdges( Node* from_node, Node* to_node );
	bool removeEdges( Node* from_node, Node* to_node );
	bool hasEdge( Edge* edge );

	void decompose();
	void removeAllComponents();

	void enumerateAllCycles();
	void removeAllCycles();
	void sortCycles();

	void sampleRandomCycles( unsigned int n, unsigned int max_len );
	bool sampleRandomCycleFrom( Node* start_node, std::vector< Node* >* stack_nodes, unsigned int max_len );
	void removeRedundantCycles();

	bool findPath( Node* from_node, Node* to_node, std::deque< Node* >* path );

	//
	inline std::vector< Node* >*				getNodeList()		{ return &node_list; }
	inline std::vector< Edge* >*				getEdgeList()		{ return &edge_list; }
	inline std::vector< std::vector<Node*>* >*	getComponentList()	{ return &component_list; }

	inline void setPositionThreshold( float t )		{ position_threshold = t; }
	inline void setOrientationThreshold( float t )	{ orientation_threshold = t; }

	inline float getPositionThreshold()		{ return position_threshold; }
	inline float getOrientationThreshold()	{ return orientation_threshold; }

	inline unsigned int getNumNodes()		{ return (unsigned int)node_list.size(); }
	inline unsigned int getNumEdges()		{ return (unsigned int)edge_list.size(); }
	inline unsigned int getNumComponents()	{ return (unsigned int)component_list.size(); }
	inline unsigned int getNumCycles()		{ return (unsigned int)cycle_list.size(); }

	//
	inline std::vector< Node* >* getComponent( unsigned int i )
	{
		if( i < getNumComponents() )
		{
			return component_list[ i ];
		}
		return NULL;
	}

	inline std::vector<unsigned int>* getCycle( unsigned int i )	
	{ 
		if( i < getNumCycles() )
		{
			return cycle_list[ i ];
		}
		return NULL;
	}

protected:
	void removeEdge( Edge* edge );

	void dfsVisit( Node* node, unsigned int* step, std::vector<Node*>* forest=NULL );
	void dfsVisitInverse( Node* node, unsigned int* step, std::vector<Node*>* forest=NULL );

	void OUTPUT_CYCLE( std::vector<unsigned int>& node_stack );
	void UNBLOCK( unsigned int u, AdjListT& B, std::vector<unsigned int>& node_stack, std::vector<unsigned int>& blocked );
	bool CIRCUIT( unsigned int s, unsigned int v, AdjListT& A, AdjListT& B, std::vector<unsigned int>& node_stack, std::vector<unsigned int>& blocked );

	//
	float	position_threshold;
	float	orientation_threshold;

	std::vector< Node* >				node_list;
	std::vector< Edge* >				edge_list;
	std::vector< std::vector<Node*>* >	component_list;

	std::map< std::pair<Node*,Node*>, std::vector<Edge*>* >	edge_map;

	std::vector< std::vector<unsigned int>* > cycle_list;

	//
	void dijkstraInit();
	void dijkstraRelax( Node* node );
	Node* dijkstraExtractMin();
	std::vector< Node* >::iterator dijkstraFindInQ( Node* node );
	static bool dijkstraCompare( Node* lh, Node* rh );

	Node*					dijkstra_source;
	Node*					dijkstra_target;
	std::set< Node* >		dijkstra_pool;
	std::vector< Node* >	dijkstra_Q;
};
