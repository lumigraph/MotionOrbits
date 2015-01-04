#include "PoseIK.h"

#include "Skeleton.h"
#include "PoseData.h"
#include "PoseConstraint.h"
#include "Joint.h"

#include "mathclass/vectorN.h"
#include "mathclass/optimize.h"

#include <cassert>

using namespace math;

Skeleton*	PoseIK::skeleton = 0;
PoseData*	PoseIK::source_pose = 0;
PoseData*	PoseIK::target_pose = 0;
PoseConstraint* PoseIK::pose_constraint = 0;

double	PoseIK::ErrorBound = 0.001;
double	PoseIK::Tolerance = 0.001;

double	PoseIK::DampingCoeff[ NUM_JOINTS_IN_HUMAN+1 ];
double	PoseIK::RigidityCoeff[ NUM_JOINTS_IN_HUMAN+1 ];

bool	PoseIK::IsDampingEnabled;

matrixN	PoseIK::J;
vectorN	PoseIK::b;

unsigned int PoseIK::num_equations;
unsigned int PoseIK::num_unknowns;
unsigned int PoseIK::num_dof;


PoseIK::PoseIK()
{
}

PoseIK::~PoseIK()
{
}

void PoseIK::initialize()
{
	ErrorBound = 0.001;
	Tolerance = 0.001;

	for( unsigned int j=0; j < NUM_JOINTS_IN_HUMAN+1; j++ )
	{
		DampingCoeff[ j ] = 0.1f;
		RigidityCoeff[ j ] = 0.1f;
	}
	
	IsDampingEnabled = true;

	/*
	RigidityCoeff[ Human::PELVIS ] = 10.0f;
	RigidityCoeff[ Human::UPPER_LEFT_LEG ] = 0.1f;
	RigidityCoeff[ Human::UPPER_RIGHT_LEG ] = 0.1f;
	RigidityCoeff[ Human::LOWER_LEFT_LEG ] = 0.1f;
	RigidityCoeff[ Human::LOWER_RIGHT_LEG ] = 0.1f;
	RigidityCoeff[ Human::LEFT_FOOT ] = 10.0f;
	RigidityCoeff[ Human::RIGHT_FOOT ] = 10.0f;
	RigidityCoeff[ Human::LEFT_TOE ] = 100.0f;
	RigidityCoeff[ Human::RIGHT_TOE ] = 100.0f;
	*/
}

int PoseIK::ik_body( Skeleton* skeleton, PoseData* source_pose, PoseData* target_pose, PoseConstraint* pose_constraint, double roll )
{
	PoseIK::skeleton = skeleton;
	PoseIK::source_pose = source_pose;
	PoseIK::target_pose = target_pose;
	PoseIK::pose_constraint = pose_constraint;

	vectorN d;
	unsigned int dof = skeleton->calcDOF();

	d.setSize( dof );
	for( unsigned int i=0; i < dof; i++ )	d[i] = 0.0;

	int iter = 0;
	double f = 0.0f;

	if( energyFunc(d) > ErrorBound )
	{
		math::gradient_descent( d, dof, Tolerance, iter, f, energyFunc, gradientFunc, 10 );
	}

	/*
	ik_right_hand( pSkeleton, tPosture, posture_constraint, roll );
	ik_left_hand( pSkeleton, tPosture, posture_constraint, roll );
	ik_right_foot( pSkeleton, tPosture, posture_constraint, roll );
	ik_left_foot( pSkeleton, tPosture, posture_constraint, roll );
	*/

	return iter;
}

double PoseIK::energyFunc( const math::vectorN& d )
{
	target_pose->copy( source_pose );
	target_pose->addDisplacement( skeleton, d );

	//
	double dist = 0.0f;

	vector	dv, dq, dc;
	quater	q;

	JointConstraint* joint_constraint = 0;

	for( unsigned int i=0; i < NUM_JOINTS_IN_HUMAN; i++ )
	{
		Joint* joint = skeleton->getHumanJoint( i );
		if( !joint )	continue;

		bool is_constrained = pose_constraint->getConstraint( i, &joint_constraint );
		if( !is_constrained )	continue;

		unsigned int joint_index = joint->getIndex();
		unsigned int joint_dof = joint->getDOF();

		math::transq jT = target_pose->getGlobalTransform( skeleton, joint_index );
		math::transq cT = joint_constraint->getTransform();

		switch( joint_constraint->getType() ) {
			case JointConstraint::Type::POSITION :
			{
				dv = jT.translation - cT.translation;
				dist += dv % dv;
			}
			break;

			case JointConstraint::Type::ORIENTATION :
			{
				dv = difference( jT.rotation, cT.rotation );
				dist += dv % dv;
			}
			break;

			case JointConstraint::Type::TRANSQ :
			{
				dv = jT.translation - cT.translation;
				dist += dv % dv;

				dv = difference( jT.rotation, cT.rotation );
				dist += dv % dv;
			}
			break;

			default :
			{
				assert( false );
			}
			break;
		}
	}

	if( IsDampingEnabled )
	{
		unsigned int jj=0;

		for( unsigned int j=0; j < NUM_JOINTS_IN_HUMAN; j++ )
		{
			Joint* joint = skeleton->getHumanJoint( j );
			if( joint )
			{
				double c = DampingCoeff[ j+1 ];
				unsigned int joint_dof = joint->getDOF();

				switch( joint_dof ) {
				case 6 :
					{
						dist += c*d[jj]*d[jj];	jj++;
						dist += c*d[jj]*d[jj];	jj++;
						dist += c*d[jj]*d[jj];	jj++;

						dist += c*d[jj]*d[jj];	jj++;
						dist += c*d[jj]*d[jj];	jj++;
						dist += c*d[jj]*d[jj];	jj++;
					}
					break;
				case 3 :
					{
						dist += c*d[jj]*d[jj];	jj++;
						dist += c*d[jj]*d[jj];	jj++;
						dist += c*d[jj]*d[jj];	jj++;
					}
					break;
				case 1 :
					{
						dist += c*d[jj]*d[jj];	jj++;
					}
					break;
				default :
					break;
				}
			}
		}
	}

	return dist;

}

double PoseIK::gradientFunc( const math::vectorN& d, math::vectorN& dp )
{
	target_pose->copy( source_pose );
	target_pose->addDisplacement( skeleton, d );

	double dist = computeJacobian();
	scalingJacobian( J );

	dp.solve( J, b, 1.0e-2 );

	/*
	if ( PenvIKsolverMethod == PENV_IK_LS_SVD_METHOD )
		dp.solve( J, b, 1.0e-2 );
	else
	{
		static matrixN Jt; Jt.transpose( J );
		static matrixN Jp; Jp.mult( Jt, J );
		static vectorN bp; bp.mult( Jt, b );

		for( int k=0; k<Jp.row(); k++ ) Jp[k][k] += DAMPING;

		dp.solve( Jp, bp );
	}
	*/

	scalingCoefficients( dp );

	if( PoseIK::IsDampingEnabled ) {
		dist += incorporateDamping( d, dp );
	}

	return dist;
}

double PoseIK::computeJacobian()
{
	double dist = 0.0f;

	transq	t;
	vector	dv, dq, dc;
	vector	endTrans;
	quater	endRot;

	unsigned int i, j;
	vector	w1, w2, w3;

	num_dof = skeleton->calcDOF();
	num_equations = MAX( pose_constraint->getDOC(), num_dof );
	num_unknowns  = num_dof;

	J.setSize( num_equations, num_unknowns );
	b.setSize( num_equations );

	for( i=0; i < num_equations; i++ )
	{
		b[i] = 0.0;
		for( j=0; j < num_unknowns; j++ )
		{
			J[i][j] = 0.0;
		}
	}

	unsigned int ii = 0;

	JointConstraint* joint_constraint = 0;

	// 
	for( i=0; i < NUM_JOINTS_IN_HUMAN; i++ )
	{
		bool is_constrained = pose_constraint->getConstraint( i, &joint_constraint );

		if( !is_constrained )
		{
			continue;
		}

		Joint* joint = skeleton->getHumanJoint( i );
		unsigned int joint_id = joint->getIndex();
		unsigned int constraint_type = joint_constraint->getType();

		t = target_pose->getGlobalTransform( skeleton, joint_id );
		endTrans = t.translation;
		endRot   = t.rotation;

		unsigned int temp = ii;

		switch( constraint_type ) {
		case JointConstraint::Type::POSITION :
			{
				dv = joint_constraint->getTransform().translation - endTrans;
				dist += dv % dv;

				b[ii++] = dv.x();
				b[ii++] = dv.y();
				b[ii++] = dv.z();
			}
			break;
		case JointConstraint::Type::ORIENTATION :
			{
				dq = math::difference( joint_constraint->getTransform().rotation, endRot );
				dist += dq % dq;

				b[ii++] = dq.x();
				b[ii++] = dq.y();
				b[ii++] = dq.z();
			}
			break;
		case JointConstraint::Type::TRANSQ :
			{
				dv = joint_constraint->getTransform().translation - endTrans;
				dist += dv % dv;

				b[ii++] = dv.x();
				b[ii++] = dv.y();
				b[ii++] = dv.z();

				dq = math::difference( joint_constraint->getTransform().rotation, endRot );
				dist += dq % dq;

				b[ii++] = dq.x();
				b[ii++] = dq.y();
				b[ii++] = dq.z();
			}
			break;
		default :
			break;
		}

		// calc derivatives of all ancestors of this constrained joint 



		unsigned int jj = 0;

		for( j=0; j < NUM_JOINTS_IN_HUMAN; j++ )
		{
			Joint* other_joint = skeleton->getHumanJoint( j );
			if( !other_joint )
			{
				continue;
			}
			unsigned int other_id = other_joint->getIndex();
			unsigned int other_dof = other_joint->getDOF();

			if( skeleton->isAncestor( other_id, joint_id ) )
			{
				unsigned int iii = temp;

				switch( other_dof ) 
				{
					case 6 :
					{
						switch( joint_constraint->getType() ) 
						{
							case JointConstraint::Type::POSITION :
							{
								J[iii  ][jj  ] = 1;
								J[iii+1][jj+1] = 1;
								J[iii+2][jj+2] = 1;

								t  = target_pose->getGlobalTransform( skeleton, other_id );
								dv = endTrans - t.translation;

								w1 = vector(1,0,0) * t * dv;
								w2 = vector(0,1,0) * t * dv;
								w3 = vector(0,0,1) * t * dv;

								J[iii  ][jj+3] = w1[0];
								J[iii  ][jj+4] = w2[0];
								J[iii  ][jj+5] = w3[0];

								J[iii+1][jj+3] = w1[1];
								J[iii+1][jj+4] = w2[1];
								J[iii+1][jj+5] = w3[1];

								J[iii+2][jj+3] = w1[2];
								J[iii+2][jj+4] = w2[2];
								J[iii+2][jj+5] = w3[2];

								iii += 3;

								break;
							}

							case JointConstraint::Type::ORIENTATION :
							{
								J[iii  ][jj+3] = 1;
								J[iii+1][jj+4] = 1;
								J[iii+2][jj+5] = 1;

								break;
							}

							case JointConstraint::Type::TRANSQ :
							{
								J[iii  ][jj  ] = 1;
								J[iii+1][jj+1] = 1;
								J[iii+2][jj+2] = 1;

								t  = target_pose->getGlobalTransform( skeleton, other_id );
								dv = endTrans - t.translation;

								w1 = vector(1,0,0) * t * dv;
								w2 = vector(0,1,0) * t * dv;
								w3 = vector(0,0,1) * t * dv;

								J[iii  ][jj+3] = w1[0];
								J[iii  ][jj+4] = w2[0];
								J[iii  ][jj+5] = w3[0];

								J[iii+1][jj+3] = w1[1];
								J[iii+1][jj+4] = w2[1];
								J[iii+1][jj+5] = w3[1];

								J[iii+2][jj+3] = w1[2];
								J[iii+2][jj+4] = w2[2];
								J[iii+2][jj+5] = w3[2];

								iii += 3;

								J[iii  ][jj+3] = 1;
								J[iii+1][jj+4] = 1;
								J[iii+2][jj+5] = 1;

								break;
							}
							default :
								break;
						}
					}
					break;

					case 3 :
					{
						switch( joint_constraint->getType() )
						{
							case JointConstraint::Type::POSITION :
							{
								t  = target_pose->getGlobalTransform( skeleton, other_id );
								dv = endTrans - t.translation;

								w1 = vector(1,0,0) * t * dv;
								w2 = vector(0,1,0) * t * dv;
								w3 = vector(0,0,1) * t * dv;

								J[iii  ][jj  ] = w1[0];
								J[iii  ][jj+1] = w2[0];
								J[iii  ][jj+2] = w3[0];

								J[iii+1][jj  ] = w1[1];
								J[iii+1][jj+1] = w2[1];
								J[iii+1][jj+2] = w3[1];

								J[iii+2][jj  ] = w1[2];
								J[iii+2][jj+1] = w2[2];
								J[iii+2][jj+2] = w3[2];

								iii += 3;

								break;
							}

							case JointConstraint::Type::ORIENTATION :
							{
								J[iii  ][jj  ] = 1;
								J[iii+1][jj+1] = 1;
								J[iii+2][jj+2] = 1;

								break;
							}

							case JointConstraint::Type::TRANSQ :
							{
								t  = target_pose->getGlobalTransform( skeleton, other_id );
								dv = endTrans - t.translation;

								w1 = vector(1,0,0) * t * dv;
								w2 = vector(0,1,0) * t * dv;
								w3 = vector(0,0,1) * t * dv;

								J[iii  ][jj  ] = w1[0];
								J[iii  ][jj+1] = w2[0];
								J[iii  ][jj+2] = w3[0];

								J[iii+1][jj  ] = w1[1];
								J[iii+1][jj+1] = w2[1];
								J[iii+1][jj+2] = w3[1];

								J[iii+2][jj  ] = w1[2];
								J[iii+2][jj+1] = w2[2];
								J[iii+2][jj+2] = w3[2];

								iii += 3;

								J[iii  ][jj  ] = 1;
								J[iii+1][jj+1] = 1;
								J[iii+2][jj+2] = 1;

								break;
							}
							default :
								break;
						}
					}
					break;

					case 1 :
					{
						switch( joint_constraint->getType() )
						{
							case JointConstraint::Type::POSITION :
							{
								t  = target_pose->getGlobalTransform( skeleton, other_id );
								dv = vector(1,0,0) * t * ( endTrans - t.translation );

								J[iii  ][jj] = dv[0];
								J[iii+1][jj] = dv[1];
								J[iii+2][jj] = dv[2];

								iii += 3;

								break;
							}
								
							case JointConstraint::Type::ORIENTATION :
							{
								J[iii  ][jj] = 1;
								J[iii+1][jj] = 0;
								J[iii+2][jj] = 0;

								break;
							}

							case JointConstraint::Type::TRANSQ :
							{
								t  = target_pose->getGlobalTransform( skeleton, other_id );
								dv = vector(1,0,0) * t * ( endTrans - t.translation );

								J[iii  ][jj] = dv[0];
								J[iii+1][jj] = dv[1];
								J[iii+2][jj] = dv[2];

								iii += 3;

								J[iii  ][jj] = 1;
								J[iii+1][jj] = 0;
								J[iii+2][jj] = 0;

								break;
							}

							default :
								break;
						}
					}
					break;
				}
			}
			jj += other_dof;
		}
	}

	return dist;
}

void PoseIK::scalingJacobian( math::matrixN& J )
{
	unsigned int i, j;
	double c = 0.0f;

	unsigned int jj = 0;

	for( j=0; j < NUM_JOINTS_IN_HUMAN; j++ )
	{
		Joint* joint = skeleton->getHumanJoint( j );

		if( joint )
		{
			unsigned int dof = joint->getDOF();

			switch( dof )
			{
			case 6 :
				c = PoseIK::RigidityCoeff[ j ];
				for( i=0; i < num_equations; i++ )
				{
					J[i][jj  ] *= c;
					J[i][jj+1] *= c;
					J[i][jj+2] *= c;
				}
				c = PoseIK::RigidityCoeff[ j ];
				for( i=0; i < num_equations; i++ )
				{
					J[i][jj+3] *= c;
					J[i][jj+4] *= c;
					J[i][jj+5] *= c;
				}
				break;

			case 3 :
				c = PoseIK::RigidityCoeff[ j ];
				for( i=0; i < num_equations; i++ )
				{
					J[i][jj  ] *= c;
					J[i][jj+1] *= c;
					J[i][jj+2] *= c;
				}
				break;

			case 1 :
				c = PoseIK::RigidityCoeff[ j ];
				for( i=0; i < num_equations; i++ )
				{
					J[i][jj  ] *= c;
				}
				break;

			default :
				break;
			}

			jj += dof;
		}
	}
}

void PoseIK::scalingCoefficients( math::vectorN& x )
{
	double c = 0.0f;

	unsigned int jj = 0;

	for( unsigned int j=0; j < NUM_JOINTS_IN_HUMAN; j++ )
	{
		Joint* joint = skeleton->getHumanJoint( j );

		if( joint )
		{
			unsigned int dof = joint->getDOF();

			switch( dof ) {
			case 6 :
			{
				c = PoseIK::RigidityCoeff[ j ];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;

				c = PoseIK::RigidityCoeff[ j ];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;

				break;
			}
			case 3 :
			{
				c = PoseIK::RigidityCoeff[ j ];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;

				break;
			}
			case 1 :
			{
				c = PoseIK::RigidityCoeff[ j ];
				x[jj++] *= c;

				break;
			}
			default :
				break;
			}
		}
	}
}

double PoseIK::incorporateDamping( const math::vectorN& d, math::vectorN& dp )
{
	double dist = 0.0f;
	double c1, c2;

	unsigned int jj = 0;

	for( unsigned int j=0; j < NUM_JOINTS_IN_HUMAN; j++ )
	{
		Joint* joint = skeleton->getHumanJoint( j );

		if( joint )
		{
			c1 = PoseIK::DampingCoeff[ j ];
			c2 = 2.0 * c1;

			unsigned int dof = joint->getDOF();

			switch( dof ) {
			case 6 :
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;

				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;

				break;
			}
			case 3 :
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;

				break;
			}
			case 1 :
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;

				break;
			}

			default :
				break;
			}
		}
	}

	return dist;
}

