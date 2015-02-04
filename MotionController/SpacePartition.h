#pragma once

#include <vector>
#include <set>

#include "SearchTree.h"

class SpaceBin
{
public:
	SpaceBin( double x_min, double x_max, double z_min, double z_max )
	{
		this->x_min = x_min;
		this->x_max = x_max;
		this->z_min = z_min;
		this->z_max = z_max;
	}
	virtual ~SpaceBin()	{}

	inline void addNode( SearchTree::Node* node )		
	{ 
		node_set.insert( node ); 
	}
	inline bool removeNode( SearchTree::Node* node )	
	{
		std::set< SearchTree::Node* >::iterator itor_n = node_set.find( node );
		if( itor_n != node_set.end() )
		{
			node_set.erase( itor_n );
			return true;
		}
		return false;
	}
	inline void clear()
	{
		node_set.clear();
	}

	inline std::set< SearchTree::Node* >* getNodes()	{ return &node_set; }
	inline int getNumNodes()							{ return (int)node_set.size(); }

	inline double getMinX()	{ return x_min; }
	inline double getMaxX()	{ return x_max; }

	inline double getMinZ()	{ return z_min; }
	inline double getMaxZ()	{ return z_max; }

	inline bool isEmpty()	{ return node_set.empty(); }

protected:
	double x_min, x_max;
	double z_min, z_max;

	std::set< SearchTree::Node* > node_set;
};

class SpacePartition
{
public:
	SpacePartition()			{ width=height=bin_width=bin_height=0.0; num_horizontal_bins=num_vertical_bins=0; }
	virtual ~SpacePartition()	{ finalize(); }

	void initialize( double width, double height, int num_horizontal_x, int num_vertical_bins );
	void finalize();

	SpaceBin* addNode( SearchTree::Node* node, double x, double z );
	SpaceBin* addNode( SearchTree::Node* node, int i, int j );
	
	SpaceBin* findBin( SearchTree::Node* node );
	SpaceBin* removeNode( SearchTree::Node* node );

	SpaceBin* getBin( double x, double z );
	SpaceBin* getBin( int i, int j );

	void clearBin( int i, int j );
	void clearAllBins();

	bool getNthRing( double x, double z, int N, std::vector< SpaceBin* >* nth_ring );
	bool getNthRing( int i, int j, int N, std::vector< SpaceBin* >* nth_ring );

	bool getBinsInBox( double x, double z, double size, std::vector< SpaceBin* >* bins_in_box );
	bool getBinsInBox( int i, int j, int N, std::vector< SpaceBin* >* bins_in_box );

	inline std::vector< std::vector< SpaceBin* >* >* getGrid()	{ return &space_grid; }

protected:
	inline void calcGridCoords( double x, double z, int* i, int* j )
	{
		*i = ( x + width/2 ) / bin_width;
		*j = ( z + height/2 ) / bin_height;
	}

	double width;
	double height;
	
	int num_horizontal_bins;
	int num_vertical_bins;

	double bin_width;
	double bin_height;

	std::map< SearchTree::Node*, SpaceBin* > map_node_bin;
	std::vector< std::vector< SpaceBin* >* > space_grid;
};
