#include "SpacePartition.h"

void SpacePartition::initialize( double width, double height, int num_horizontal_bins, int num_vertical_bins )
{
	if( width <= 0 || height <= 0 )
	{
		return;
	}
	if( num_horizontal_bins <= 0 || num_vertical_bins <= 0 )
	{
		return;
	}

	finalize();

	this->width = width;
	this->height = height;

	this->num_horizontal_bins = num_horizontal_bins;
	this->num_vertical_bins = num_vertical_bins;

	this->bin_width = width / num_horizontal_bins;
	this->bin_height = height / num_vertical_bins;
	
	int i, j;

	for( j=0; j < num_vertical_bins; j++ )
	{
		std::vector< SpaceBin* >* space_bins = new std::vector< SpaceBin* >;
		space_grid.push_back( space_bins );

		for( i=0; i < num_horizontal_bins; i++ )
		{
			SpaceBin* bin = new SpaceBin( i*bin_width - width/2, (i+1)*bin_width - width/2, j*bin_height - height/2, (j+1)*bin_height - height/2 );
			space_bins->push_back( bin );
		}
	}
}

void SpacePartition::finalize()
{
	std::vector< std::vector< SpaceBin* >* >::iterator itor_bl = space_grid.begin();
	while( itor_bl != space_grid.end() )
	{
		std::vector< SpaceBin* >* space_bins = ( *itor_bl ++ );
		
		std::vector< SpaceBin* >::iterator itor_b = space_bins->begin();
		while( itor_b != space_bins->end() )
		{
			SpaceBin* bin = ( *itor_b ++ );
			delete bin;
		}
		delete space_bins;
	}
	space_grid.clear();

	//
	map_node_bin.clear();

	//
	this->width = 0.0;
	this->height = 0.0;
	
	this->num_horizontal_bins = 0;
	this->num_vertical_bins = 0;

	this->bin_width = 0;
	this->bin_height = 0;
}

SpaceBin* SpacePartition::addNode( SearchTree::Node* node, double x, double z )
{
	int i, j;
	calcGridCoords( x, z, &i, &j );
	return addNode( node, i, j );
}

SpaceBin* SpacePartition::addNode( SearchTree::Node* node, int i, int j )
{
	if( i < 0 || i >= num_horizontal_bins || j < 0 || j >= num_vertical_bins )
	{
		return 0;
	}

	std::map< SearchTree::Node*, SpaceBin* >::iterator itor_b = map_node_bin.find( node );
	if( itor_b != map_node_bin.end() )
	{
		return 0;
	}

	SpaceBin* bin = ( *space_grid[j] )[i];
	bin->addNode( node );

	map_node_bin[ node ] = bin;

	return bin;
}

SpaceBin* SpacePartition::removeNode( SearchTree::Node* node )
{
	std::map< SearchTree::Node*, SpaceBin* >::iterator itor_b = map_node_bin.find( node );
	if( itor_b == map_node_bin.end() )
	{
		return 0;
	}

	SpaceBin* bin = itor_b->second;
	bin->removeNode( node );

	map_node_bin.erase( itor_b );

	return bin;
}

SpaceBin* SpacePartition::findBin( SearchTree::Node* node )
{
	std::map< SearchTree::Node*, SpaceBin* >::iterator itor_b = map_node_bin.find( node );
	if( itor_b == map_node_bin.end() )
	{
		return 0;
	}
	else
	{
		return itor_b->second;
	}
}

SpaceBin* SpacePartition::getBin( double x, double z )
{
	int i, j;
	calcGridCoords( x, z, &i, &j );
	return getBin( i, j );
}

SpaceBin* SpacePartition::getBin( int i, int j )
{
	if( i < 0 || i >= num_horizontal_bins || j < 0 || j >= num_vertical_bins )
	{
		return 0;
	}

	SpaceBin* bin = ( *space_grid[j] )[i];
	return bin;
}

void SpacePartition::clearBin( int i, int j )
{
	SpaceBin* bin = getBin( i, j );

	if( bin )
	{
		bin->clear();
	}
}

void SpacePartition::clearAllBins()
{
	for( int i=0; i < num_horizontal_bins; i++ )
	{
		for( int j=0; j < num_vertical_bins; j++ )
		{
			SpaceBin* bin = getBin( i, j );
			bin->clear();
		}
	}
}

bool SpacePartition::getNthRing( double x, double z, int N, std::vector< SpaceBin* >* nth_ring )
{
	int i, j;
	calcGridCoords( x, z, &i, &j );
	return getNthRing( i, j, N, nth_ring );
}

bool SpacePartition::getNthRing( int i, int j , int N, std::vector< SpaceBin* >* nth_ring )
{
	if( i < 0 || i >= num_horizontal_bins || j < 0 || j >= num_vertical_bins )
	{
		return false;
	}
	if( ( i-N ) < 0 && ( i+N ) >= num_horizontal_bins && ( j-N ) < 0 && ( j+N ) >= num_vertical_bins )
	{
		return false;
	}
	if( N < 0 )
	{
		return false;
	}

	if( N == 0 )
	{
		SpaceBin* bin_center = getBin( i, j );
		nth_ring->push_back( bin_center );
		return true;
	}

	// N >= 1
	SpaceBin* bin_left_top = getBin( i-N, j-N );
	SpaceBin* bin_right_top = getBin( i+N, j-N );
	SpaceBin* bin_right_bottom = getBin( i+N, j+N );
	SpaceBin* bin_left_bottom = getBin( i-N, j+N );

	int p, q;

	if( bin_left_top && !bin_left_top->isEmpty() )
	{
		nth_ring->push_back( bin_left_top );
	}
	for( p=i-N+1, q=j-N; p <= i+N-1; p++ )
	{
		SpaceBin* bin_top = getBin( p, q );
		if( bin_top && !bin_top->isEmpty() )
		{
			nth_ring->push_back( bin_top );
		}
	}

	if( bin_right_top && !bin_right_top->isEmpty() )
	{
		nth_ring->push_back( bin_right_top );
	}
	for( p=i+N, q=j-N+1; q <= j+N-1; q++ )
	{
		SpaceBin* bin_right = getBin( p, q );
		if( bin_right && !bin_right->isEmpty() )
		{
			nth_ring->push_back( bin_right );
		}
	}

	if( bin_right_bottom && !bin_right_bottom->isEmpty() )
	{
		nth_ring->push_back( bin_right_bottom );
	}
	for( p=i+N-1, q=j+N; p >= i-N+1; p-- )
	{
		SpaceBin* bin_bottom = getBin( p, q );
		if( bin_bottom && !bin_bottom->isEmpty() )
		{
			nth_ring->push_back( bin_bottom );
		}
	}

	if( bin_left_bottom && !bin_left_bottom->isEmpty() )
	{
		nth_ring->push_back( bin_left_bottom );
	}
	for( p=i-N, q=j+N-1; q >= j-N+1; q-- )
	{
		SpaceBin* bin_left = getBin( p, q );
		if( bin_left && !bin_left->isEmpty() )
		{
			nth_ring->push_back( bin_left );
		}
	}

	return true;
}

bool SpacePartition::getBinsInBox( double x, double z, double size, std::vector< SpaceBin* >* bins_in_box )
{
	int i, j;
	calcGridCoords( x, z, &i, &j );

	int N = size / bin_width;

	return getBinsInBox( i, j, N, bins_in_box );
}

bool SpacePartition::getBinsInBox( int i, int j, int N, std::vector< SpaceBin* >* bins_in_box )
{
	int left = i - N;
	int right = i + N;
	int top = j - N;
	int bottom = j + N;
	
	if( left >= num_horizontal_bins || right < 0 || top >= num_vertical_bins || bottom < 0 )
	{
		return false;
	}

	for( int p=left; p <= right; p++ )
	{
		for( int q=top; q <= bottom; q++ )
		{
			SpaceBin* bin = getBin( p, q );
			if( bin )
			{
				bins_in_box->push_back( bin );
			}
		}
	}
	return true;
}
