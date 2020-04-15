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

// Modified by Raúl Mur Artal (2014)
// Added EdgeSE3ProjectXYZ (project using focal_length in x,y directions)
// Modified by Raúl Mur Artal (2016)
// Added EdgeStereoSE3ProjectXYZ (project using focal_length in x,y directions)
// Added EdgeSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)
// Added EdgeStereoSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)

// Modified by Jun Zhang (2019)
// Added EdgeSE3ProjectFlowDepth
// Added EdgeSE3ProjectDepth
// Added EdgeSE3ProjectFlow
// Added EdgeFlowDepthPrior
// Added EdgeDepthPrior

#ifndef G2O_SIX_DOF_TYPES_EXPMAP
#define G2O_SIX_DOF_TYPES_EXPMAP

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "../core/base_multi_edge.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include <Eigen/Geometry>

namespace g2o {
namespace types_six_dof_expmap {
void init();
}

using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;


/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class  VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSE3Expmap();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {
    _estimate = SE3Quat();
  }

  virtual void oplusImpl(const double* update_)  {
    Eigen::Map<const Vector6d> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
  }
};


class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(v2->estimate()));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
};


class  EdgeStereoSE3ProjectXYZ: public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(v2->estimate()),bf);
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const;

  double fx, fy, cx, cy, bf;
};

class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  Vector3d Xw;
  double fx, fy, cx, cy;
};


class  EdgeStereoSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeStereoSE3ProjectXYZOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector3d obs(_measurement);
    _error = obs - cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector3d cam_project(const Vector3d & trans_xyz) const;

  Vector3d Xw;
  double fx, fy, cx, cy, bf;
};

// **************************************************************************************************

class  EdgeSE3ProjectXYZOnlyObjMotion: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyObjMotion(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  Vector3d Xw;

  // NEW: projection matrix
  Matrix<double, 3, 4> P;

};


class  EdgeXYZPrior2: public  BaseUnaryEdge<3, Vector3d, VertexSBAPointXYZ>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeXYZPrior2(){}

  bool read(std::istream& is);
  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d obs(_measurement);
    _error = obs-v1->estimate();
  }

  bool isDepthPositive() {
    const VertexSBAPointXYZ* v1 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return v1->estimate()(2)>0.0;
  }

  virtual void linearizeOplus();

};

class  EdgeSE3ProjectXYZOnlyPoseBack: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZOnlyPoseBack(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Vector2d obs(_measurement);
    // SE3Quat est_inv = v1->estimate().inverse();
    // _error = obs-cam_project(est_inv.map(Xw));
    _error = obs-cam_project(v1->estimate().map_2(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map_2(Xw))(2)>0.0;
  }

  // virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  Vector3d Xw;
  double fx, fy, cx, cy;
};

// **********************************************************************************************************

class  EdgeSE3ProjectFlowDepth: public  BaseBinaryEdge<2, Vector2d, VertexSBAFlowDepth, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectFlowDepth(){};

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlowDepth* v2 = static_cast<const VertexSBAFlowDepth*>(_vertices[0]);
    Vector2d obs(_measurement);
    Vector3d est = v2->estimate();
    Vector3d Xw;
    Xw << (meas(0)-est(0)-cx)*est(2)/fx, (meas(1)-est(1)-cy)*est(2)/fy, est(2);
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    _error = obs-cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlowDepth* v2 = static_cast<const VertexSBAFlowDepth*>(_vertices[0]);
    Vector3d est = v2->estimate();
    Vector3d Xw;
    Xw << (meas(0)-est(0)-cx)*est(2)/fx, (meas(1)-est(1)-cy)*est(2)/fy, est(2);
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
  Matrix<double,2,1> meas;
  Matrix<double,4,4> Twl;
};

class  EdgeFlowDepthPrior: public  BaseUnaryEdge<3, Vector3d, VertexSBAFlowDepth>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeFlowDepthPrior(){}

  bool read(std::istream& is);
  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSBAFlowDepth* v1 = static_cast<const VertexSBAFlowDepth*>(_vertices[0]);
    Vector3d obs(_measurement);
    _error = obs-v1->estimate();
  }

  bool isDepthPositive() {
    const VertexSBAFlowDepth* v1 = static_cast<const VertexSBAFlowDepth*>(_vertices[0]);
    return v1->estimate()(2)>0.0;
  }

  virtual void linearizeOplus();

};

// **********************************************************************************************************

class  EdgeSE3ProjectFlow: public  BaseBinaryEdge<2, Vector2d, VertexSBAFlow, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectFlow(){};

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlow* v2 = static_cast<const VertexSBAFlow*>(_vertices[0]);
    Vector2d obs(_measurement);
    Vector2d est = v2->estimate();
    Vector3d Xw;
    Xw << (meas(0)-est(0)-cx)*depth/fx, (meas(1)-est(1)-cy)*depth/fy, depth;
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    _error = obs-cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlow* v2 = static_cast<const VertexSBAFlow*>(_vertices[0]);
    Vector2d est = v2->estimate();
    Vector3d Xw;
    Xw << (meas(0)-est(0)-cx)*depth/fx, (meas(1)-est(1)-cy)*depth/fy, depth;
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
  double depth;
  Matrix<double,2,1> meas;
  Matrix<double,4,4> Twl;
};

class  EdgeFlowPrior: public  BaseUnaryEdge<2, Vector2d, VertexSBAFlow>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeFlowPrior(){}

  bool read(std::istream& is);
  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSBAFlow* v1 = static_cast<const VertexSBAFlow*>(_vertices[0]);
    Vector2d obs(_measurement);
    // _error = obs-v1->estimate();
    _error = v1->estimate()-obs;
  }

  virtual void linearizeOplus();

};

// **********************************************************************************************************

class  EdgeSE3ProjectFlow2: public  BaseBinaryEdge<2, Vector2d, VertexSBAFlow, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectFlow2(){};

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlow* v2 = static_cast<const VertexSBAFlow*>(_vertices[0]);
    Vector2d obs(_measurement);
    Vector2d est = v2->estimate();
    Vector3d Xw;
    Xw << (obs(0)-cx)*depth/fx, (obs(1)-cy)*depth/fy, depth;
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    _error = (obs+est) - cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlow* v2 = static_cast<const VertexSBAFlow*>(_vertices[0]);
    Vector2d obs(_measurement);
    Vector2d est = v2->estimate();
    Vector3d Xw;
    Xw << (obs(0)-cx)*depth/fx, (obs(1)-cy)*depth/fy, depth;
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
  double depth;
  Matrix<double,4,4> Twl;
};

// **********************************************************************************************************

class  EdgeSE3ProjectFlowDepth2: public  BaseBinaryEdge<2, Vector2d, VertexSBAFlowDepth, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectFlowDepth2(){};

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlowDepth* v2 = static_cast<const VertexSBAFlowDepth*>(_vertices[0]);
    Vector2d obs(_measurement);
    Vector3d est = v2->estimate();
    Vector3d Xw;
    Xw << (obs(0)-cx)*est(2)/fx, (obs(1)-cy)*est(2)/fy, est(2);
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    _error = (obs+est.head(2))-cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlowDepth* v2 = static_cast<const VertexSBAFlowDepth*>(_vertices[0]);
    Vector2d obs(_measurement);
    Vector3d est = v2->estimate();
    Vector3d Xw;
    Xw << (obs(0)-cx)*est(2)/fx, (obs(1)-cy)*est(2)/fy, est(2);
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
  Matrix<double,4,4> Twl;
};

// **********************************************************************************************************

class  EdgeSE3ProjectDepth: public  BaseBinaryEdge<2, Vector2d, VertexSBADepth, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectDepth(){};

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBADepth* v2 = static_cast<const VertexSBADepth*>(_vertices[0]);
    Vector2d obs(_measurement);
    Matrix<double, 1, 1> est = v2->estimate();
    Vector3d Xw;
    Xw << (obs(0)-cx)*est(0)/fx, (obs(1)-cy)*est(0)/fy, est(0);
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    _error = (obs+flow)-cam_project(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBADepth* v2 = static_cast<const VertexSBADepth*>(_vertices[0]);
    Vector2d obs(_measurement);
    Matrix<double, 1, 1> est = v2->estimate();
    Vector3d Xw;
    Xw << (obs(0)-cx)*est(0)/fx, (obs(1)-cy)*est(0)/fy, est(0);
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
  Vector2d flow;
  Matrix<double,4,4> Twl;
};

class  EdgeDepthPrior: public  BaseUnaryEdge<1, Matrix<double, 1, 1>, VertexSBADepth>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeDepthPrior(){};

  bool read(std::istream& is);
  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSBADepth* v1 = static_cast<const VertexSBADepth*>(_vertices[0]);
    Matrix<double, 1, 1> obs(_measurement);
    _error = obs-v1->estimate();
  }

  virtual void linearizeOplus();

};

// **********************************************************************************************************


class  EdgeSE3ProjectFlowDepth3: public  BaseMultiEdge<2, Vector2d>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectFlowDepth3();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAFlow* v2 = static_cast<const VertexSBAFlow*>(_vertices[0]);
    const VertexSBADepth* v3 = static_cast<const VertexSBADepth*>(_vertices[2]);
    Vector2d obs(_measurement);
    Vector2d flow = v2->estimate();
    Matrix<double, 1, 1> depth = v3->estimate();
    Vector3d Xw;
    Xw << (obs(0)-cx)*depth(0)/fx, (obs(1)-cy)*depth(0)/fy, depth(0);
    Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
    _error = (obs+flow)-cam_project(v1->estimate().map(Xw));
  }

  // bool isDepthPositive() {
  //   const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
  //   const VertexSBADepth* v3 = static_cast<const VertexSBADepth*>(_vertices[2]);
  //   Vector2d obs(_measurement);
  //   Matrix<double, 1, 1> depth = v3->estimate();
  //   Vector3d Xw;
  //   Xw << (obs(0)-cx)*depth(0)/fx, (obs(1)-cy)*depth(0)/fy, depth(0);
  //   Xw = Twl.block(0,0,3,3)*Xw + Twl.col(3).head(3);
  //   return (v1->estimate().map(Xw))(2)>0.0;
  // }

  // virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
  Matrix<double,4,4> Twl;
};

// **********************************************************************************************************


// **********************************************************************************************************

} // end namespace

#endif
