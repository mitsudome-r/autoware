/*
 * utilities.cpp
 *
 *  Created on: Jul 31, 2018
 *      Author: sujiwo
 */

#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "utilities.h"


using namespace std;
using namespace Eigen;


#define dPrecision 3


class VStream: public std::stringstream
{
public:
VStream()
{
	setf(ios::fixed, ios::floatfield);
	precision(dPrecision);
}
};


Quaterniond fromRPY (double roll, double pitch, double yaw)
{
	roll /= 2.0;
	pitch /= 2.0;
	yaw /= 2.0;
	double
		ci = cos(roll),
		si = sin(roll),
		cj = cos(pitch),
		sj = sin(pitch),
		ck = cos(yaw),
		sk = sin(yaw),
		cc = ci*ck,
		cs = ci*sk,
		sc = si*ck,
		ss = si*sk;
	Quaterniond q;
	q.x() = cj*sc - sj*cs;
	q.y() = cj*ss + sj*cc;
	q.z() = cj*cs - sj*sc;
	q.w() = cj*cc + sj*ss;
	q.normalize();
	return q;
}


Vector3d quaternionToRPY (const Quaterniond &q)
{
	Eigen::Matrix3d m = q.toRotationMatrix();

	double cy = sqrt(m(0,0)*m(0,0) + m(1,0)*m(1,0));
	double ax, ay, az;
	if (cy > std::numeric_limits<double>::epsilon()*4.0) {
		ax = atan2( m(2, 1),  m(2, 2));
		ay = atan2(-m(2, 0),  cy);
		az = atan2( m(1, 0),  m(0,0));
	}
	else {
        ax = atan2(-m(1, 2),  m(1, 1));
        ay = atan2(-m(2, 0),  cy);
        az = 0.0;
	}
	return Vector3d(ax, ay, az);
}


TTransform::TTransform(
	const double X, const double Y, const double Z,
	const double QX, const double QY, const double QZ, const double QW) :
		Affine3d(
			Eigen::Translation3d(X,Y,Z) * Eigen::Quaterniond(QW, QX, QY, QZ))
{}

TTransform::TTransform(
	const double X, const double Y, const double Z,
	const double roll, const double pitch, const double yaw) :
		Affine3d(
			Eigen::Translation3d(X,Y,Z) * fromRPY(roll, pitch, yaw))
{}



TTransform
TTransform::from_XYZ_RPY (
	const Eigen::Vector3d &pos,
	double roll, double pitch, double yaw)
{
	Quaterniond q = fromRPY(roll, pitch, yaw);
	return TTransform::from_Pos_Quat(pos, q);
}


TTransform
TTransform::from_Pos_Quat(const Vector3d &pos, const Quaterniond &orient)
{
	Affine3d t;
	t = Eigen::Translation3d(pos) * orient;
	return t;
}


#include <sstream>

string
TTransform::str() const
{
	stringstream ss;
	ss << "x=" << position().x()
		<< " y=" << position().y()
		<< " z=" << position().z()
		<< ", qx=" << orientation().x()
		<< " qy=" << orientation().y()
		<< " qz=" << orientation().z()
		<< " qw=" << orientation().w();
	return ss.str();
}


Vector4d vectorFromQuaternion (const Quaterniond &q)
{
	return Vector4d(q.x(), q.y(), q.z(), q.w());
}


void
TTransform::displacement (
	const TTransform &other,
	double &linear, double &angular) const
{
	linear = (this->position()-other.position()).norm();

	Vector4d q1 = vectorFromQuaternion(this->orientation().normalized()),
		q2 = vectorFromQuaternion(other.orientation().normalized());
	angular = acos(
		q1.head(3).dot(q2.head(3)) /
		(q1.head(3).norm() * q2.head(3).norm())
	);
}


TTransform
TTransform::interpolate(const TTransform &T1, const TTransform &T2, const double ratio)
{
	assert (0<=ratio and ratio<=1);

	Vector3d
		p1 = T1.position(),
		p2 = T2.position(),
		p3;
	Quaterniond
		q1 = T1.orientation(),
		q2 = T2.orientation(),
		q3;

	p3 = p1 + ratio * (p2 - p1);
	q3 = q1.slerp(ratio, q2);
	return TTransform::from_Pos_Quat(p3, q3);
}


using Eigen::VectorXd;

VectorXd
cdf (const cv::Mat &grayImage, const cv::Mat &mask)
{
	VectorXd rcdf = VectorXd::Zero(256);
	cv::MatND hist;
	int histSize = 256;
	float range[] = {0,255};
	const float *ranges[] = {range};
	cv::calcHist (&grayImage, 1, 0, cv::Mat(), hist, 1, &histSize, ranges, true, false);
	// cumulative sum
	rcdf[0] = hist.at<float>(0);
	for (int i=1; i<histSize; i++) {
		rcdf[i] = rcdf[i-1] + hist.at<float>(i);
	}

	rcdf = rcdf / cv::sum(hist)[0];
	return rcdf;
}


string
dumpVector(const Vector3d &v)
{
	VStream s;
	s << v.x() << " " << v.y() << " " << v.z();
	return s.str();
}


string
dumpVector(const Quaterniond &v)
{
	VStream s;
	s << v.x() << " " << v.y() << " " << v.z() << ' ' << v.w();
	return s.str();
}


std::string
dumpVector(const TTransform &P)
{
	Vector3d xyz = P.position();
	TQuaternion qxyz = P.orientation();
	Vector3d RPY = quaternionToRPY(qxyz);

	VStream s;
	s << "XYZ: " << dumpVector(xyz) << endl;
	s << "RPY (rad): " << dumpVector(RPY) << endl;
	s << "QXYZW: " << dumpVector(qxyz) << endl;

	return s.str();
}


void debugMsg(const string &s, double is_error)
{
	if (!is_error)
		cerr << s << endl << flush;
	else
		cout << s << endl << flush;
}