#include "DrawingTool.h"

DrawingTool drawing_tool;

extern void startBVHPlayer( int* argcp, char** argv );
extern void startGraphBuilder( int* argcp, char** argv );
extern void startGraphPlayer( int* argcp, char** argv );
extern void startOrbitBuilder( int* argcp, char** argv );
extern void startOrbitPlayer( int* argcp, char** argv );
extern void startLeapController( int* argcp, char** argv );
extern void startInverseKinematics( int* argcp, char** argv );

void main(int argc, char **argv) 
{
//	startBVHPlayer( &argc, argv );
//	startGraphBuilder( &argc, argv );
//	startGraphPlayer( &argc, argv );
//	startOrbitBuilder( &argc, argv );
//	startOrbitPlayer( &argc, argv );
	startLeapController( &argc, argv );
//	startInverseKinematics( &argc, argv );
} 
 