#ifndef BONENODE_H
#define BONENODE_H

#include <string>
#include <vector>
#include <cmath>

class BoneNode
{
public:

	float DEGREES_PER_RADIAN;//I cannot believe C++ does not let you declare const floats in class files
	float RADIANS_PER_DEGREE;

	std::vector<float> xTranslations;
	std::vector<float> yTranslations;
	std::vector<float> zTranslations;

	std::vector<float> xRotations; //(in degrees)
	std::vector<float> yRotations;
	std::vector<float> zRotations;

	std::string name;
	std::vector<BoneNode*> childrenNodes;
	std::vector<std::string> channels;

	float xOffset;
	float yOffset;
	float zOffset;

	float xTranslation;
	float yTranslation;
	float zTranslation;

	//Euler representation of the current orientation (in degrees)
	float xRotation;
	float yRotation;
	float zRotation;

	//Quaternion representation of the current orientation
	float qw;
	float qx;
	float qy;
	float qz;

	BoneNode(void)
	{
		name = "Unnamed Bone";
		DEGREES_PER_RADIAN = (180.0f/3.1415926f);
		RADIANS_PER_DEGREE = (3.1415926f/180.0f);

		xOffset = 0;
		yOffset = 0;
		zOffset = 0;

		xTranslation = 0;
		yTranslation = 0;
		zTranslation = 0;

		xRotation = 0;
		yRotation = 0;
		zRotation = 0;

		qw = 1;//i.e. identity quaternion
		qx = 0;
		qy = 0;
		qz = 0;
	}

	void refreshQuaternionRepresentation(){
		convertEulerAnglesToQuaternion(xRotation*RADIANS_PER_DEGREE,yRotation*RADIANS_PER_DEGREE,zRotation*RADIANS_PER_DEGREE,
			this->qw,this->qx,this->qy,this->qz);
	}

	//Code is based off of:
	//http://www.gamasutra.com/view/feature/131686/rotating_objects_using_quaternions.php?page=2
	//NOTE: The Euler angles should be passed in as RADIANS
	static void convertEulerAnglesToQuaternion(float roll_radians, float pitch_radians, float yaw_radians, float& qw, float& qx, float& qy, float& qz){
		float cr, cp, cy, sr, sp, sy, cpcy, spsy;
		
		cr = cos(roll_radians/2.0f);
		cp = cos(pitch_radians/2.0f);
		cy = cos(yaw_radians/2.0f);
		sr = sin(roll_radians/2.0f);
		sp = sin(pitch_radians/2.0f);
		sy = sin(yaw_radians/2.0f);
		cpcy = cp * cy;
		spsy = sp * sy;
		qw = cr * cpcy + sr * spsy;
		qx = sr * cpcy - cr * spsy;
		qy = cr * sp * cy + sr * cp * sy;
		qz = cr * cp * sy - sr * sp * cy;
	}

	//Code is based off of:
	//http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
	static void convertQuaternionToAngleAndAxis(float qw, float qx, float qy, float qz, float& angle_radians, float& axis_x, float& axis_y, float& axis_z){
		angle_radians = 2.0f * acos(qw);
		double s = sqrt(1-qw*qw); // assuming quaternion normalised then w is less than 1, so term always positive.
		if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
			// if s close to zero then direction of axis not important
			axis_x = qx; // if it is important that axis is normalised then replace with x=1; y=z=0;
			axis_y = qy;
			axis_z = qz;
		} else {
			axis_x = qx / s; // normalise axis
			axis_y = qy / s;
			axis_z = qz / s;
		}
	}

	//Code is based off of:
	//http://www.gamasutra.com/view/feature/131686/rotating_objects_using_quaternions.php?page=2
	static void quaternionSlerp(float from_w,float from_x,float from_y,float from_z, 
								float to_w,  float to_x,  float to_y,  float to_z, 
								float t,
								float& result_w,float& result_x,float& result_y,float& result_z){
		
		const float DELTA = 0.0001f;
		float           to1[4];
		double        omega, cosom, sinom, scale0, scale1;
		// calc cosine
		cosom = from_x*to_x + from_y*to_y + from_z*to_z + from_w*to_w;
		// adjust signs (if necessary)
		if ( cosom <0.0 ){ cosom = -cosom; to1[0] = - to_x;
			to1[1] = - to_y;
			to1[2] = - to_z;
			to1[3] = - to_w;
		} else  {
			to1[0] = to_x;
			to1[1] = to_y;
			to1[2] = to_z;
			to1[3] = to_w;
		}
		// calculate coefficients
		if ( (1.0 - cosom) > DELTA ) {
		// standard case (slerp)
			omega = acos(cosom);
			sinom = sin(omega);
			scale0 = sin((1.0 - t) * omega) / sinom;
			scale1 = sin(t * omega) / sinom;
		} else {        
			// "from" and "to" quaternions are very close so we can do a linear interpolation
			scale0 = 1.0 - t;
			scale1 = t;
		}

		// calculate final values
		result_x = scale0 * from_x + scale1 * to1[0];
		result_y = scale0 * from_y + scale1 * to1[1];
		result_z = scale0 * from_z + scale1 * to1[2];
		result_w = scale0 * from_w + scale1 * to1[3];
	}

	~BoneNode(void)
	{
	}
};

#endif
