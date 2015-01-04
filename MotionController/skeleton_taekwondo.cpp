#include "Human.h"
#include "Skeleton.h"
#include "Joint.h"

void setupTaekwondoSkeleton( Skeleton* skeleton )
{
	skeleton->setHumanJoint( "Hips", Human::PELVIS );
	skeleton->setHumanJoint( "Spine", Human::SPINE_1 );
	skeleton->setHumanJoint( "Spine1", Human::SPINE_2 );
	skeleton->setHumanJoint( "HEad", Human::HEAD );
	skeleton->setHumanJoint( "RightUpLeg", Human::UPPER_RIGHT_LEG );
	skeleton->setHumanJoint( "RightLeg", Human::LOWER_RIGHT_LEG );
	skeleton->setHumanJoint( "RightFoot", Human::RIGHT_FOOT );
	skeleton->setHumanJoint( "RightToes", Human::RIGHT_TOE );
	skeleton->setHumanJoint( "LeftUpLeg", Human::UPPER_LEFT_LEG );
	skeleton->setHumanJoint( "LeftLeg", Human::LOWER_LEFT_LEG );
	skeleton->setHumanJoint( "LeftFoot", Human::LEFT_FOOT );
	skeleton->setHumanJoint( "LeftToes", Human::LEFT_TOE );
	skeleton->setHumanJoint( "RightShoulder", Human::RIGHT_SHOULDER );
	skeleton->setHumanJoint( "RightArm", Human::UPPER_RIGHT_ARM );
	skeleton->setHumanJoint( "RightForeArm", Human::LOWER_RIGHT_ARM );
	skeleton->setHumanJoint( "RightHand", Human::RIGHT_PALM );
	skeleton->setHumanJoint( "LeftShoulder", Human::LEFT_SHOULDER );
	skeleton->setHumanJoint( "LeftArm", Human::UPPER_LEFT_ARM );
	skeleton->setHumanJoint( "LeftForeArm", Human::LOWER_LEFT_ARM );
	skeleton->setHumanJoint( "LeftHand", Human::LEFT_PALM );
}
