/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Commonplace Robotics GmbH
 *  http://www.commonplacerobotics.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name Commonplace Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Last change: May 16th, 2013


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <stdexcept>

#include "cpr_Matrix4.h"

using namespace std;


// the struct stores information about the current robots state:
// joint positions, cartesian position and error code
struct robotState
{
	double p[6];		// cart position
	double j[4];		// joint position
	int errorCode[4];
};


// Provides the necessary kinematic information and functionalities to enable cartesian motion
class cpr_KinematicMover
{
public:
  	cpr_KinematicMover();
	
  	void SetMotionVec(double *vec);
  	int moveJoint();
  	int moveCart();
  	int forwardKin(robotState s);
	int inverseKin(robotState s);
    void setJointState(double* joints);
  	double computeJointPos(int, int);
  	int computeTics(int, double);

  	robotState setPointState;
  	robotState currState;
	cpr_Matrix4 lastPosition;

private:

  	double v[4];

  	double motionVec[6];
	double gearScale[4];
	

  	const static constexpr double deg2rad = 3.14159 / 180.0;
    const static constexpr double rad2deg = 180.0 / 3.14159;

	// kinematic length of the Mover4 robot arm in mm
    const static constexpr double lz0 = 130.0;	// base
    const static constexpr double lz1 = 62.5;		// first joint
    const static constexpr double lz2 = 190.0;	// upper arm
    const static constexpr double lz3 = 220.0;	// lower arm
    const static constexpr double lz4 = 48.0;		// hand

    const static constexpr double EpsilonCenter = 5.0;		// forbidden radius around the z axis in mm

	bool flagSymmetryEllbowUp;				// for one cartesian position there are typically two joint positions. This flag stores which one to use


};






