/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/DynamicObjectSLAM>
*
**/

#include "types_dyn_slam3d.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

using namespace std;
using namespace Eigen;

// ************************************************************************************************

LandmarkMotionTernaryEdge::LandmarkMotionTernaryEdge() : BaseMultiEdge<3,Vector3>()
{
    resize(3);
    J.fill(0);
    J.block<3,3>(0,0) = Matrix3::Identity();
}

bool LandmarkMotionTernaryEdge::read(std::istream& is)
{
    Vector3d meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);

    for (int i=0; i<3; i++)
      for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i) = information()(i,j);
      }
    return true;

}

bool LandmarkMotionTernaryEdge::write(std::ostream& os) const
{
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<3; ++i)
      for (int j=i; j<3; ++j)
        os <<  information()(i, j) << " " ;
    return os.good();
}

void LandmarkMotionTernaryEdge::computeError()
{
    VertexPointXYZ* point1 = dynamic_cast<VertexPointXYZ*>(_vertices[0]);
    VertexPointXYZ* point2 = dynamic_cast<VertexPointXYZ*>(_vertices[1]);
    VertexSE3* H = static_cast<VertexSE3*>(_vertices[2]);

    Vector3 expected = point1->estimate() - H->estimate().inverse()*point2->estimate();
    _error = expected - _measurement;
}

void LandmarkMotionTernaryEdge::linearizeOplus()
{

    //VertexPointXYZ* point1 = dynamic_cast<VertexPointXYZ*>(_vertices[0]);
    VertexPointXYZ* point2 = dynamic_cast<VertexPointXYZ*>(_vertices[1]);
    VertexSE3* H = static_cast<VertexSE3*>(_vertices[2]);

    Vector3 invHl2 = H->estimate().inverse()*point2->estimate();

    // jacobian wrt H
    J(0,4) =  invHl2(2);
    J(0,5) = -invHl2(1);
    J(1,3) = -invHl2(2);
    J(1,5) =  invHl2(0);
    J(2,3) =  invHl2(1);
    J(2,4) = -invHl2(0);

    Eigen::Matrix<number_t,3,6,Eigen::ColMajor> Jhom = J;

    _jacobianOplus[0] = Matrix3::Identity();
    _jacobianOplus[1] = -H->estimate().inverse().rotation();
    _jacobianOplus[2] = Jhom;
}

// ************************************************************************************************

EdgeSE3Altitude::EdgeSE3Altitude() : BaseUnaryEdge<1, double, VertexSE3>()
{
   resize(1);
   J.fill(0);
   J[1] = 1;
}

bool EdgeSE3Altitude::read(std::istream& is)
{
    double meas;
    is >> meas;
    setMeasurement(meas);
    is >> information()(0);
    return true;
}

bool EdgeSE3Altitude::write(std::ostream& os) const
{
    os  << measurement() << " ";
    os << information()(0) << " ";
    return os.good();
}

void EdgeSE3Altitude::computeError()
{
    VertexSE3* v = static_cast<VertexSE3*>(_vertices[0]);
    _error << v->estimate().translation()[1] - _measurement;
}

void EdgeSE3Altitude::linearizeOplus()
{
    Eigen::Matrix<number_t,1,6,Eigen::RowMajor> Jhom = J;
    _jacobianOplusXi = Jhom;
}

// ************************************************************************************************

} // end namespace
