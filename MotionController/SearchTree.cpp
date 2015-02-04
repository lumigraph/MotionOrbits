#include "SearchTree.h"
#include "SkeletalMotion.h"
#include "Skeleton.h"
#include "Joint.h"
#include "mathclass/QmGeodesic.h"

#include "SpacePartition.h"

#include <algorithm>

//#define NUM_NEAREST_NEIGHBORS	(10)

static const int K = 10;
static const double DENSITY_THRESHOLD = 0.1;
static const int DEPTH_THRESHOLD = 20;

static const int NUM_HORIZONTAL_GRIDS = 50;
static const int NUM_VERTICAL_GRIDS = 50;

SearchTree::SearchTree()
{
	motion = 0;
	graph = 0;
	start_x = start_z = start_angle = 0;
	randomness = 0;
		
	human_joint = 0;
	joint_index = 0;

	root_node = 0;
	num_nodes = 0;

	space_partition = new SpacePartition;
	space_partition->initialize( 1000.0, 1000.0, NUM_HORIZONTAL_GRIDS, NUM_VERTICAL_GRIDS );
}

SearchTree::~SearchTree()
{
	delete space_partition;
}

void SearchTree::initialize(  SkeletalMotion* motion, MotionGraph* graph, MotionGraph::Node* graph_node, double start_x, double start_z, double start_angle, double randomness, unsigned int human_joint )
{
	finalize();

	this->motion = motion;
	this->graph = graph;
	this->start_x = start_x;
	this->start_z = start_z;
	this->start_angle = start_angle;
	this->randomness = randomness;

	this->human_joint = human_joint;
	this->joint_index = motion->getHumanJoint( human_joint )->getIndex();

	this->root_node = addNode( 0, graph_node, start_x, start_z, start_angle );
}

void SearchTree::finalize()
{
	motion = 0;
	graph = 0;
	start_x = start_z = start_angle = 0;
	randomness = 0;

	human_joint = 0;
	joint_index = 0;

	removeNode( root_node );
	root_node = 0;
	num_nodes = 0;

	space_partition->clearAllBins();
}

inline static bool sortLeafNodes( SearchTree::Node* lhs, SearchTree::Node* rhs )
{
	return ( lhs->getDistanceToGoal() < rhs->getDistanceToGoal() );
}

int SearchTree::extendN( math::position& goal_point, int N )
{
	int n;

	for( n=0; n < N; n++ )
	{
		bool is_extended = extend( goal_point );
		if( !is_extended )
		{
			break;
		}
	}
	return n;
}

int SearchTree::shrinkN( math::position& goal_point, int N )
{
	int n;

	for( n=0; n < N; n++ )
	{
		bool is_shrunken = shrink( goal_point );
		if( !is_shrunken )
		{
			break;
		}
	}
	return n;
}

bool SearchTree::extend( math::position& goal_point )
{
	double gx = goal_point.x();
	double gz = goal_point.z();

	std::vector< Node* > neighbors;
	double radius = kNN( goal_point.x(), goal_point.z(), K, &neighbors );
	int k = (int)neighbors.size();

	if( k >= K )
	{
		double density = k / radius;	// * 2 * M_PI

		if( density >= DENSITY_THRESHOLD )
		{
			return false;
			/*
			shrink( goal_point );

			neighbors.clear();
			kNN( goal_point.x(), goal_point.z(), K, &neighbors ); 
			*/
		}
	}

	//
	std::vector< Node* > open_nodes;
	std::vector< Node* >::iterator itor_n = neighbors.begin();
	while( itor_n != neighbors.end() )
	{
		Node* node = ( *itor_n ++ );

		while( node != 0 && ( node->isFullyExtended() || node->getDepth() >= DEPTH_THRESHOLD ) )
		{
			node = node->getParent();
		}

		if( node )
		{
			open_nodes.push_back( node );
		}
	}

	if( open_nodes.empty() )
	{
		return false;
	}
	else
	{
		Node* max_node = 0;
		int max_index = 0;
		double max_qual = -DBL_MAX;

		std::vector< Node* >::iterator itor_n = open_nodes.begin();
		while( itor_n != open_nodes.end() )
		{
			Node* node = ( *itor_n ++ );

			int index = 0;
			MotionGraph::Node* graph_node = node->getGraphNode();
			std::vector< MotionGraph::Edge* >* graph_edges = graph_node->getNextEdges();
			std::vector< MotionGraph::Edge* >::iterator itor_e = graph_edges->begin();
			while( itor_e != graph_edges->end() )
			{
				MotionGraph::Edge* graph_edge = ( *itor_e ++ );
				MotionGraph::Node* graph_next_node = graph_edge->getToNode();

				if( !node->isChildExtended( graph_next_node ) )
				{
					/*
					math::transq T;
					double x1, z1, a1;
					double xN, zN, aN;
					this->estimateExtendedConfig( node, graph_next_node, &T, &x1, &z1, &a1, &xN, &zN, &aN );

					double dx = xN - goal_point.x();
					double dz = zN - goal_point.z();
					double dist = dx*dx + dz*dz;

					double qual = 1/dist;
					*/

					
					Node* child_node = addNode( node, graph_next_node );
					
					//double qual = 1.0 / math::vector( child_node->getPosition() - goal_point ).length();

					double qual = evaluateSamplingQuality( goal_point.x(), goal_point.z() );
					
					removeNode( child_node );
					

					if( qual > max_qual )
					{
						max_qual = qual;
						max_node = node;
						max_index = index;
					}
				}
				index ++;
			}
		}

		Node* parent_node = max_node;
		MotionGraph::Node* parent_graph_node = parent_node->getGraphNode();

		MotionGraph::Node* child_graph_node = parent_graph_node->getNextNode( max_index );
		Node* child_node = addNode( parent_node, child_graph_node );

		return true;
	}
}

bool SearchTree::shrink( math::position& goal_point )
{
	double gx = goal_point.x();
	double gz = goal_point.z();

	std::vector< Node* > neighbors;
	int k = kNN( goal_point.x(), goal_point.z(), K, &neighbors );

	if( k < K )
	{
		return false;
	}
	else
	{
		std::vector< Node* > close_nodes;
		std::vector< Node* >::iterator itor_n = neighbors.begin();
		while( itor_n != neighbors.end() )
		{
			Node* node = ( *itor_n ++ );

			while( !node->isLeaf() )
			{
				std::vector< Node* >* children = node->getChildren();
				int num_children = (int)children->size();
				
				int index = rand() % num_children;
				node = ( *children )[ index ];
			}

			if( node != root_node )
			{
				close_nodes.push_back( node );
			}
		}

		//
		if( close_nodes.empty() )
		{
			return false;
		}
		else
		{
			double gx = goal_point.x();
			double gz = goal_point.z();

			Node* min_node = 0;
			double min_qual = +DBL_MAX;

			std::vector< Node* >::iterator itor_n = close_nodes.begin();
			while( itor_n != close_nodes.end() )
			{
				Node* node = ( *itor_n ++ );
				Node* parent_node = node->getParent();
				MotionGraph::Node* graph_node = node->getGraphNode();

				double nx = node->getPosition().x();
				double nz = node->getPosition().z();

				double dx = gx - nx;
				double dz = gz - nz;

				//double qual = dx*dx + dz*dz;
				double qual = evaluateSamplingQuality( nx, nz );

				if( qual < min_qual )
				{
					min_qual = qual;
					min_node = node;
				}
			}

			removeNode( min_node );
		
			return true;
		}
	}
}

double SearchTree::evaluateSamplingQuality( double x, double z )
{
	std::vector< Node* > neighbors;
	double radius = kNN( x, z, K, &neighbors );

	if( neighbors.empty() )
	{
		return -DBL_MAX;
	}
	else
	{
		int k = (int)neighbors.size();
		double density = k / radius;	// * 2 * M_PI

		//
		double dist = 0.0;

		if( k > 1 )
		{
			for( int p=0; p < k; p++ )
			{
				Node* node_p = neighbors[ p ];
				double x_p = node_p->getEndX();
				double z_p = node_p->getEndZ();
				double a_p = node_p->getEndAngle();
				
				int f_p = node_p->getEndFrame();
				math::transq t_p = motion->getLocalTransform( f_p, joint_index );

				for( int q=p+1; q < k; q++ )
				{
					Node* node_q = neighbors[ q ];
					double x_q = node_q->getEndX();
					double z_q = node_q->getEndZ();
					double a_q = node_q->getEndAngle();

					int f_q = node_q->getEndFrame();
					math::transq t_q = motion->getLocalTransform( f_q, joint_index );

					double dx = fabs( x_q - x_p );
					double dz = fabs( z_q - z_p );
				
					double da1 = fabs( a_q - a_p );
					double da2 = M_PI - da1;
					double da = ( da1 < da2 ? da1 : da2 );
				
					double position_dist = sqrt( dx*dx + dz*dz ) * exp( da / M_PI );
					double pose_dist = math::vector( t_q.translation-t_p.translation ).length();
					double alpha = 10.0;

					dist += ( position_dist + pose_dist * alpha );
				}
			}
			dist /= ( k * (k-1) / 2 );
		}

		//
		return density * dist;
	}
}

double SearchTree::kNN( double x, double z, int K, std::vector< SearchTree::Node* >* neighbors )
{
	if( K > num_nodes )
	{
		K = num_nodes;
	}

	double local_x, local_z;
	calcLocalCoords( x, z, &local_x, &local_z );

	int num_neighbors_collected = 0;
	double max_dist = -DBL_MAX;

	int n = 0;
	while( 1 )
	{
		std::vector< SpaceBin* > nth_ring;
		bool is_ring_found = space_partition->getNthRing( local_x, local_z, n, &nth_ring );
		if( !is_ring_found )
		{
			break;
		}

		std::vector< SpaceBin* >::iterator itor_b = nth_ring.begin();
		while( itor_b != nth_ring.end() )
		{
			SpaceBin* bin = ( *itor_b ++ );
			
			std::set< SearchTree::Node* >* nodes = bin->getNodes();
			std::set< SearchTree::Node* >::iterator itor_n = nodes->begin();
			while( itor_n != nodes->end() )
			{
				SearchTree::Node* node = ( *itor_n ++ );
				neighbors->push_back( node );

				double nx = node->getPosition().x();
				double nz = node->getPosition().z();

				double dx = ( nx - x );
				double dz = ( nz - z );

				double dist = dx*dx + dz*dz;
				node->setDistanceToGoal( dist );

				//
				if( dist > max_dist )
				{
					max_dist = dist;
				}
			}

			int num_nodes = bin->getNumNodes();
			num_neighbors_collected += num_nodes;

			if( num_neighbors_collected >= K )
			{
				break;
			}
		}

		if( num_neighbors_collected >= K )
		{
			break;
		}

		n++;
	}

	//
	if( !neighbors->empty() )
	{
//		std::sort( neighbors->begin(), neighbors->end(), sortLeafNodes );
	}

	return max_dist;	//num_neighbors_collected;
}

//

math::position SearchTree::calcTrackPoint( Node* node, int f )
{
	int f1 = node->getStartFrame();
	int fN = node->getEndFrame();

	if( f < f1 || f > fN )
	{
		return math::position(0,0,0);
	}

	math::transq T = node->getTransform();
	math::transq t = motion->getGlobalTransform( f, joint_index, T );
	math::position p( t.translation.x(), t.translation.y(), t.translation.z() );

	return p;
}

void SearchTree::estimateExtendedConfig( Node* parent, MotionGraph::Node* graph_node, math::transq* T, double* x1, double* z1, double* a1, double* xN, double* zN, double* aN )
{
	int f1 = graph_node->getSegment(0).first;
	int fN = graph_node->getSegment(0).second;

	if( parent )
	{
		*x1 = parent->getEndX();
		*z1 = parent->getEndZ();
		*a1 = parent->getEndAngle();
	}

	transFromPlace( f1, T, *x1, *z1, *a1 );
	placeFromTrans( fN, *T, xN, zN, aN );
}

SearchTree::Node* SearchTree::addNode( Node* parent, MotionGraph::Node* graph_node, double x/*=0*/, double z/*=0*/, double a/*=0*/ )
{
	int f1 = graph_node->getSegment(0).first;
	int fN = graph_node->getSegment(0).second;

	math::transq T;
	double x1, z1, a1;
	double xN, zN, aN;

	if( !parent )
	{
		x1 = x;
		z1 = z;
		a1 = a;
	}

	estimateExtendedConfig( parent, graph_node, &T, &x1, &z1, &a1, &xN, &zN, &aN );

	math::transq t = motion->getGlobalTransform( fN, joint_index, T );
	math::position P( t.translation.x(), t.translation.y(), t.translation.z() );

	//
	Node* node = new Node( graph_node, x1, z1, a1, xN, zN, aN, T, P );

	if( parent )
	{
		/*
		if( parent->isLeaf() )
		{
			removeAsLeaf( parent );
		}
		*/

		parent->addChild( node );
	}

//	addAsLeaf( node );

	//
	if( space_partition )
	{
		double local_x, local_z;
		calcLocalCoords( P.x(), P.z(), &local_x, &local_z );

		space_partition->addNode( node, local_x, local_z ); 
	}

	num_nodes ++;

	return node;
}

void SearchTree::removeNode( Node* node )
{
	if( !node )
	{
		return;
	}

	Node* parent = node->getParent();

	if( parent )
	{
		parent->removeChild( node );

		/*
		if( parent->isLeaf() )
		{
			addAsLeaf( parent );
		}
		*/
	}

	std::vector< Node* >* children = node->getChildren();
	std::vector< Node* >::iterator itor_c = children->begin();
	while( itor_c != children->end() )
	{
		Node* child = ( *itor_c );
		itor_c = children->erase( itor_c );

		removeNode( child );
	}

//	removeAsLeaf( node );

	if( node == this->root_node )
	{
		this->root_node = 0;
	}

	//
	if( space_partition )
	{
		space_partition->removeNode( node );
	}

	num_nodes --;

	delete node;
}

/*
void SearchTree::addAsLeaf( Node* node )
{
	leaf_nodes.insert( node );
}

bool SearchTree::removeAsLeaf( Node* node )
{
	std::set< Node* >::iterator itor_n = leaf_nodes.find( node );
	if( itor_n != leaf_nodes.end() )
	{
		leaf_nodes.erase( itor_n );
		return true;
	}
	else
	{
		return false;
	}
}

bool SearchTree::isLeaf( Node* node )
{
	std::set< Node* >::iterator itor_n = leaf_nodes.find( node );
	if( itor_n != leaf_nodes.end() )
	{
		return true;
	}
	else
	{
		return false;
	}
}
*/

void SearchTree::placeFromTrans( unsigned int f, math::transq t, double* x, double* z, double* angle )
{
	Skeleton* skeleton = motion->getSkeleton();
	Joint* root = skeleton->getRootJoint();
	math::transq T = motion->getGlobalTransform( f, root->getIndex(), t );

	*x = T.translation.x();
	*z = T.translation.z();

	//
	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );
	T.rotation = g.nearest( T.rotation );

	math::vector front(0,0,1);	// ?
	front = math::rotate( T.rotation, front );

	*angle = atan2( front.x(), front.z() );
}

void SearchTree::transFromPlace( unsigned int f, math::transq* t, double x, double z, double angle )
{
	QmGeodesic g( math::quater(1,0,0,0), math::y_axis );

	math::transq trans = math::translate_transq( x, 0, z );
	math::transq rot = math::rotate_transq( angle, math::vector(0,1,0) );
	math::transq desired_transq = trans * rot;

	Skeleton* skeleton = motion->getSkeleton();
	Joint* root = skeleton->getRootJoint();

	math::transq current_transq = motion->getGlobalTransform( f, root->getIndex(), math::identity_transq );
	current_transq.rotation = g.nearest( current_transq.rotation );

	*t = desired_transq * current_transq.inverse();
	t->rotation = g.nearest( t->rotation );
	t->translation.set_y( 0 );
}

void SearchTree::calcLocalCoords( double x, double z, double* local_x, double* local_z )
{
	double dx = x - start_x;
	double dz = z - start_z;
	
	math::transq rot = math::rotate_transq( -start_angle, math::vector(0,1,0) );

	math::vector dv( dx, 0, dz );
	dv = math::rotate( rot.rotation, dv );

	*local_x = dv.x();
	*local_z = dv.z();
}

void SearchTree::calcGlobalCoords( double local_x, double local_z, double* x, double* z )
{
	math::transq rot = math::rotate_transq( start_angle, math::vector(0,1,0) );

	math::vector lv( local_x, 0, local_z );
	lv = math::rotate( rot.rotation, lv );

	*x = lv.x() + start_x;
	*z = lv.z() + start_z;
}
