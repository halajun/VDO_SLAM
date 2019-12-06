// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_six_dof_expmap.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

using namespace std;


Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3d unproject2d(const Vector2d& v)  {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7d est;
  for (int i=0; i<7; i++)
    is  >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(estimate().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  return os.good();
}


EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


void EdgeSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  Matrix<double,2,3> tmp;
  tmp(0,0) = fx;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*fx;

  tmp(1,0) = 0;
  tmp(1,1) = fy;
  tmp(1,2) = -y/z*fy;

  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;
}

Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}


Vector3d EdgeStereoSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz, const float &bf) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}

EdgeStereoSE3ProjectXYZ::EdgeStereoSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeStereoSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeStereoSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  const Matrix3d R =  T.rotation().toRotationMatrix();

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  _jacobianOplusXi(0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
  _jacobianOplusXi(0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
  _jacobianOplusXi(0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;

  _jacobianOplusXi(1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
  _jacobianOplusXi(1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
  _jacobianOplusXi(1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;

  _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*R(2,0)/z_2;
  _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)-bf*R(2,1)/z_2;
  _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2)-bf*R(2,2)/z_2;

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;

  _jacobianOplusXj(2,0) = _jacobianOplusXj(0,0)-bf*y/z_2;
  _jacobianOplusXj(2,1) = _jacobianOplusXj(0,1)+bf*x/z_2;
  _jacobianOplusXj(2,2) = _jacobianOplusXj(0,2);
  _jacobianOplusXj(2,3) = _jacobianOplusXj(0,3);
  _jacobianOplusXj(2,4) = 0;
  _jacobianOplusXj(2,5) = _jacobianOplusXj(0,5)-bf/z_2;
}


//Only Pose

bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0/xyz_trans[2];
  double invz_2 = invz*invz;

  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXi(0,2) = y*invz *fx;
  _jacobianOplusXi(0,3) = -invz *fx;
  _jacobianOplusXi(0,4) = 0;
  _jacobianOplusXi(0,5) = x*invz_2 *fx;

  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXi(1,2) = -x*invz *fy;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -invz *fy;
  _jacobianOplusXi(1,5) = y*invz_2 *fy;
}

Vector2d EdgeSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}


Vector3d EdgeStereoSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}


bool EdgeStereoSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0/xyz_trans[2];
  double invz_2 = invz*invz;

  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXi(0,2) = y*invz *fx;
  _jacobianOplusXi(0,3) = -invz *fx;
  _jacobianOplusXi(0,4) = 0;
  _jacobianOplusXi(0,5) = x*invz_2 *fx;

  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXi(1,2) = -x*invz *fy;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -invz *fy;
  _jacobianOplusXi(1,5) = y*invz_2 *fy;

  _jacobianOplusXi(2,0) = _jacobianOplusXi(0,0)-bf*y*invz_2;
  _jacobianOplusXi(2,1) = _jacobianOplusXi(0,1)+bf*x*invz_2;
  _jacobianOplusXi(2,2) = _jacobianOplusXi(0,2);
  _jacobianOplusXi(2,3) = _jacobianOplusXi(0,3);
  _jacobianOplusXi(2,4) = 0;
  _jacobianOplusXi(2,5) = _jacobianOplusXi(0,5)-bf*invz_2;
}

// ************************************************************************************************

bool EdgeSE3ProjectXYZOnlyObjMotion::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZOnlyObjMotion::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectXYZOnlyObjMotion::cam_project(const Vector3d & trans_xyz) const{

  double m1 = P(0,0)*trans_xyz[0] + P(0,1)*trans_xyz[1] + P(0,2)*trans_xyz[2] + P(0,3);
  double m2 = P(1,0)*trans_xyz[0] + P(1,1)*trans_xyz[1] + P(1,2)*trans_xyz[2] + P(1,3);
  double m3 = P(2,0)*trans_xyz[0] + P(2,1)*trans_xyz[1] + P(2,2)*trans_xyz[2] + P(2,3);
  double invm3 = 1.0/m3;

  Vector2d res;
  res[0] = m1*invm3;
  res[1] = m2*invm3;

  return res;
}

void EdgeSE3ProjectXYZOnlyObjMotion::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

  double m1 = P(0,0)*x + P(0,1)*y + P(0,2)*z + P(0,3);
  double m2 = P(1,0)*x + P(1,1)*y + P(1,2)*z + P(1,3);
  double m3 = P(2,0)*x + P(2,1)*y + P(2,2)*z + P(2,3);
  double invm3 = 1.0/m3;
  double invm3_2 = invm3*invm3;

  Matrix<double,2,3> tmp;
  tmp(0,0) = invm3_2*(P(0,0)*m3-P(2,0)*m1);
  tmp(0,1) = invm3_2*(P(0,1)*m3-P(2,1)*m1);
  tmp(0,2) = invm3_2*(P(0,2)*m3-P(2,2)*m1);
  tmp(1,0) = invm3_2*(P(1,0)*m3-P(2,0)*m2);
  tmp(1,1) = invm3_2*(P(1,1)*m3-P(2,1)*m2);
  tmp(1,2) = invm3_2*(P(1,2)*m3-P(2,2)*m2);

  _jacobianOplusXi(0,0) = -1.0*( y*tmp(0,2)-z*tmp(0,1) );
  _jacobianOplusXi(0,1) = -1.0*( z*tmp(0,0)-x*tmp(0,2) );
  _jacobianOplusXi(0,2) = -1.0*( x*tmp(0,1)-y*tmp(0,0) );
  _jacobianOplusXi(0,3) = -1.0*tmp(0,0);
  _jacobianOplusXi(0,4) = -1.0*tmp(0,1);
  _jacobianOplusXi(0,5) = -1.0*tmp(0,2);

  _jacobianOplusXi(1,0) = -1.0*( y*tmp(1,2)-z*tmp(1,1) );
  _jacobianOplusXi(1,1) = -1.0*( z*tmp(1,0)-x*tmp(1,2) );
  _jacobianOplusXi(1,2) = -1.0*( x*tmp(1,1)-y*tmp(1,0) );
  _jacobianOplusXi(1,3) = -1.0*tmp(1,0);
  _jacobianOplusXi(1,4) = -1.0*tmp(1,1);
  _jacobianOplusXi(1,5) = -1.0*tmp(1,2);
}

// ************************************************************************************************

bool EdgeXYZPrior2::read(std::istream& is){
  for (int i=0; i<3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeXYZPrior2::write(std::ostream& os) const {

  for (int i=0; i<3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeXYZPrior2::linearizeOplus(){
    _jacobianOplusXi = -1.0*Matrix3d::Identity();
}


// ************************************************************************************************

bool EdgeSE3ProjectXYZOnlyPoseBack::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPoseBack::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectXYZOnlyPoseBack::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

// void EdgeSE3ProjectXYZOnlyPoseBack::linearizeOplus() {
//   VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
//   Vector3d xyz_trans = vi->estimate().map_2(Xw);

//   double x = xyz_trans[0];
//   double y = xyz_trans[1];
//   double invz = 1.0/xyz_trans[2];
//   double invz_2 = invz*invz;

//   _jacobianOplusXi(0,0) =  -1.0*x*y*invz_2 *fx;
//   _jacobianOplusXi(0,1) = (1.0+(x*x*invz_2)) *fx;
//   _jacobianOplusXi(0,2) = -1.0*y*invz *fx;
//   _jacobianOplusXi(0,3) = invz *fx;
//   _jacobianOplusXi(0,4) = 0;
//   _jacobianOplusXi(0,5) = -1.0*x*invz_2 *fx;

//   _jacobianOplusXi(1,0) = -1.0*(1.0+y*y*invz_2) *fy;
//   _jacobianOplusXi(1,1) = x*y*invz_2 *fy;
//   _jacobianOplusXi(1,2) = x*invz *fy;
//   _jacobianOplusXi(1,3) = 0;
//   _jacobianOplusXi(1,4) = invz *fy;
//   _jacobianOplusXi(1,5) = -1.0*y*invz_2 *fy;
// }

// ************************************************************************************************

bool EdgeSE3ProjectFlowDepth::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectFlowDepth::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectFlowDepth::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

void EdgeSE3ProjectFlowDepth::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAFlowDepth* vi = static_cast<VertexSBAFlowDepth*>(_vertices[0]);
  Vector3d uvd = vi->estimate();
  Vector3d Xw;
  Xw << (meas(0)-uvd(0)-cx)*uvd(2)/fx, (meas(1)-uvd(1)-cy)*uvd(2)/fy, uvd(2);
  Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
  Vector3d xyz_trans = T.map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;

  Matrix<double,2,3> K;
  K(0,0) = fx; K(0,1) = 0;  K(0,2) = cx;
  K(1,0) = 0;  K(1,1) = fy; K(1,2) = cy;

  Matrix<double,3,4> T_mat;
  T_mat.block(0,0,3,3) = T.rotation().toRotationMatrix();
  T_mat.col(3) = T.translation();

  Matrix<double,2,4> A;
  A = K*T_mat*Twl;

  _jacobianOplusXi(0,0) = A(0,0)*uvd(2)*invfx;
  _jacobianOplusXi(0,1) = A(0,1)*uvd(2)*invfy;
  _jacobianOplusXi(0,2) = -1.0*( A(0,0)*(meas(0)-uvd(0)-cx)*invfx + A(0,1)*(meas(1)-uvd(1)-cy)*invfy + A(0,2) );

  _jacobianOplusXi(1,0) = A(1,0)*uvd(2)*invfx;
  _jacobianOplusXi(1,1) = A(1,1)*uvd(2)*invfy;
  _jacobianOplusXi(1,2) = -1.0*( A(1,0)*(meas(0)-uvd(0)-cx)*invfx + A(1,1)*(meas(1)-uvd(1)-cy)*invfy + A(1,2) );

}

// ************************************************************************************************

bool EdgeFlowDepthPrior::read(std::istream& is){
  for (int i=0; i<3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeFlowDepthPrior::write(std::ostream& os) const {

  for (int i=0; i<3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeFlowDepthPrior::linearizeOplus(){
    _jacobianOplusXi = -1.0*Matrix3d::Identity();
}

// *****************************************************************************************************************

bool EdgeSE3ProjectFlow::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectFlow::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectFlow::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

void EdgeSE3ProjectFlow::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAFlow* vi = static_cast<VertexSBAFlow*>(_vertices[0]);
  Vector2d f_uv = vi->estimate();
  Vector3d Xw;
  Xw << (meas(0)-f_uv(0)-cx)*depth/fx, (meas(1)-f_uv(1)-cy)*depth/fy, depth;
  Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
  Vector3d xyz_trans = T.map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;

  Matrix<double,2,3> K;
  K(0,0) = fx; K(0,1) = 0;  K(0,2) = cx;
  K(1,0) = 0;  K(1,1) = fy; K(1,2) = cy;

  Matrix<double,3,4> T_mat;
  T_mat.block(0,0,3,3) = T.rotation().toRotationMatrix();
  T_mat.col(3) = T.translation();

  Matrix<double,2,4> A;
  A = K*T_mat*Twl;

  _jacobianOplusXi(0,0) = A(0,0)*depth*invfx;
  _jacobianOplusXi(0,1) = A(0,1)*depth*invfy;

  _jacobianOplusXi(1,0) = A(1,0)*depth*invfx;
  _jacobianOplusXi(1,1) = A(1,1)*depth*invfy;

}

// ************************************************************************************************

bool EdgeFlowPrior::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeFlowPrior::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeFlowPrior::linearizeOplus(){
  // _jacobianOplusXi = -1.0*Matrix2d::Identity();
  _jacobianOplusXi = Matrix2d::Identity();
}

// ************************************************************************************************

bool EdgeSE3ProjectFlow2::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectFlow2::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectFlow2::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

void EdgeSE3ProjectFlow2::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAFlow* vi = static_cast<VertexSBAFlow*>(_vertices[0]);
  Vector2d obs(_measurement);
  Vector2d f_uv = vi->estimate();
  Vector3d Xw;
  Xw << (obs(0)-cx)*depth/fx, (obs(1)-cy)*depth/fy, depth;
  Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
  Vector3d xyz_trans = T.map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;

  _jacobianOplusXi = Matrix2d::Identity();

}

// ************************************************************************************************

bool EdgeSE3ProjectFlowDepth2::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectFlowDepth2::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectFlowDepth2::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

void EdgeSE3ProjectFlowDepth2::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAFlowDepth* vi = static_cast<VertexSBAFlowDepth*>(_vertices[0]);
  Vector2d obs(_measurement);
  Vector3d est = vi->estimate();
  Vector3d Xw;
  Xw << (obs(0)-cx)*est(2)/fx, (obs(1)-cy)*est(2)/fy, est(2);
  Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
  Vector3d xyz_trans = T.map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;

  Matrix<double,2,3> K;
  K(0,0) = fx; K(0,1) = 0;  K(0,2) = cx;
  K(1,0) = 0;  K(1,1) = fy; K(1,2) = cy;

  Matrix<double,3,4> T_mat;
  T_mat.block(0,0,3,3) = T.rotation().toRotationMatrix();
  T_mat.col(3) = T.translation();

  Matrix<double,2,4> A;
  A = K*T_mat*Twl;

  _jacobianOplusXi(0,0) = 1.0;
  _jacobianOplusXi(0,1) = 0.0;
  _jacobianOplusXi(0,2) = -1.0*( A(0,0)*(obs(0)-cx)*invfx + A(0,1)*(obs(1)-cy)*invfy + A(0,2) );

  _jacobianOplusXi(1,0) = 0.0;
  _jacobianOplusXi(1,1) = 1.0;
  _jacobianOplusXi(1,2) = -1.0*( A(1,0)*(obs(0)-cx)*invfx + A(1,1)*(obs(1)-cy)*invfy + A(1,2) );

}

// ************************************************************************************************

bool EdgeSE3ProjectDepth::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectDepth::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectDepth::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

void EdgeSE3ProjectDepth::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBADepth* vi = static_cast<VertexSBADepth*>(_vertices[0]);
  Vector2d obs(_measurement);
  Matrix<double, 1, 1> est = vi->estimate();
  Vector3d Xw;
  Xw << (obs(0)-cx)*est(0)/fx, (obs(1)-cy)*est(0)/fy, est(0);
  Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
  Vector3d xyz_trans = T.map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;
  double invfx = 1.0/fx;
  double invfy = 1.0/fy;

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;

  Matrix<double,2,3> K;
  K(0,0) = fx; K(0,1) = 0;  K(0,2) = cx;
  K(1,0) = 0;  K(1,1) = fy; K(1,2) = cy;

  Matrix<double,3,4> T_mat;
  T_mat.block(0,0,3,3) = T.rotation().toRotationMatrix();
  T_mat.col(3) = T.translation();

  Matrix<double,2,4> A;
  A = K*T_mat*Twl;

  _jacobianOplusXi(0,0) = -1.0*( A(0,0)*(obs(0)-cx)*invfx + A(0,1)*(obs(1)-cy)*invfy + A(0,2) );
  _jacobianOplusXi(1,0) = -1.0*( A(1,0)*(obs(0)-cx)*invfx + A(1,1)*(obs(1)-cy)*invfy + A(1,2) );

}

// ************************************************************************************************

bool EdgeDepthPrior::read(std::istream& is){
  for (int i=0; i<1; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<1; i++)
    for (int j=i; j<1; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeDepthPrior::write(std::ostream& os) const {

  for (int i=0; i<1; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<1; i++)
    for (int j=i; j<1; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeDepthPrior::linearizeOplus(){
    Matrix<double, 1, 1> jac(-1.0);
    _jacobianOplusXi = jac;
}

// ************************************************************************************************

EdgeSE3ProjectFlowDepth3::EdgeSE3ProjectFlowDepth3() : BaseMultiEdge<2, Vector2d>()
{
  resize(3);
  _jacobianOplus[0].resize(2,2);
  _jacobianOplus[1].resize(2,6);
  _jacobianOplus[2].resize(2,1);
}

bool EdgeSE3ProjectFlowDepth3::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectFlowDepth3::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectFlowDepth3::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

// void EdgeSE3ProjectFlowDepth3::linearizeOplus() {
//   VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
//   SE3Quat T(vj->estimate());
//   VertexSBAFlow* vi = static_cast<VertexSBAFlow*>(_vertices[0]);
//   VertexSBADepth* vk = static_cast<VertexSBADepth*>(_vertices[2]);
//   Vector2d obs(_measurement);
//   Vector2d flow = vi->estimate();
//   Matrix<double, 1, 1> depth = vk->estimate();
//   Vector3d Xw;
//   Xw << (obs(0)-cx)*depth(0)/fx, (obs(1)-cy)*depth(0)/fy, depth(0);
//   Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
//   Vector3d xyz_trans = T.map(Xw);

//   double x = xyz_trans[0];
//   double y = xyz_trans[1];
//   double z = xyz_trans[2];
//   double z_2 = z*z;
//   double invfx = 1.0/fx;
//   double invfy = 1.0/fy;

//   Matrix<double,2,6> J_1;

//   J_1(0,0) =  x*y/z_2 *fx;
//   J_1(0,1) = -(1+(x*x/z_2)) *fx;
//   J_1(0,2) = y/z *fx;
//   J_1(0,3) = -1./z *fx;
//   J_1(0,4) = 0;
//   J_1(0,5) = x/z_2 *fx;

//   J_1(1,0) = (1+y*y/z_2) *fy;
//   J_1(1,1) = -x*y/z_2 *fy;
//   J_1(1,2) = -x/z *fy;
//   J_1(1,3) = 0;
//   J_1(1,4) = -1./z *fy;
//   J_1(1,5) = y/z_2 *fy;

//   _jacobianOplus[1] = J_1;

//   Matrix<double,2,3> K;
//   K(0,0) = fx; K(0,1) = 0;  K(0,2) = cx;
//   K(1,0) = 0;  K(1,1) = fy; K(1,2) = cy;

//   Matrix<double,3,4> T_mat;
//   T_mat.block(0,0,3,3) = T.rotation().toRotationMatrix();
//   T_mat.col(3) = T.translation();

//   Matrix<double,2,4> A;
//   A = K*T_mat*Twl;

//   Matrix<double,2,2> J_0;

//   J_0(0,0) = 1.0;
//   J_0(0,1) = 0.0;

//   J_0(1,0) = 0.0;
//   J_0(1,1) = 1.0;

//   _jacobianOplus[0] = J_0;

//   Matrix<double,2,1> J_2;

//   J_2(0,0) = -1.0*( A(0,0)*(obs(0)-cx)*invfx + A(0,1)*(obs(1)-cy)*invfy + A(0,2) );
//   J_2(1,0) = -1.0*( A(1,0)*(obs(0)-cx)*invfx + A(1,1)*(obs(1)-cy)*invfy + A(1,2) );

//   _jacobianOplus[2] = J_2;

// }

// ************************************************************************************************


// ************************************************************************************************

} // end namespace
