// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#include "types_sba.h"
#include <iostream>

namespace g2o {

  using namespace std;


  VertexSBAPointXYZ::VertexSBAPointXYZ() : BaseVertex<3, Vector3d>()
  {
  }

  bool VertexSBAPointXYZ::read(std::istream& is)
  {
    Vector3d lv;
    for (int i=0; i<3; i++)
      is >> _estimate[i];
    return true;
  }

  bool VertexSBAPointXYZ::write(std::ostream& os) const
  {
    Vector3d lv=estimate();
    for (int i=0; i<3; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }

  VertexSBADepth::VertexSBADepth() : BaseVertex<1, Matrix<double, 1, 1> >()
  {
  }

  bool VertexSBADepth::read(std::istream& is)
  {
    Matrix<double, 1, 1> lv;
    for (int i=0; i<1; i++)
      is >> _estimate[i];
    return true;
  }

  bool VertexSBADepth::write(std::ostream& os) const
  {
    Matrix<double, 1, 1> lv=estimate();
    for (int i=0; i<1; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }

  VertexSBAFlow::VertexSBAFlow() : BaseVertex<2, Vector2d>()
  {
  }

  bool VertexSBAFlow::read(std::istream& is)
  {
    Vector2d lv;
    for (int i=0; i<2; i++)
      is >> _estimate[i];
    return true;
  }

  bool VertexSBAFlow::write(std::ostream& os) const
  {
    Vector2d lv=estimate();
    for (int i=0; i<2; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }

  VertexSBAFlowDepth::VertexSBAFlowDepth() : BaseVertex<3, Vector3d>()
  {
  }

  bool VertexSBAFlowDepth::read(std::istream& is)
  {
    Vector3d lv;
    for (int i=0; i<3; i++)
      is >> _estimate[i];
    return true;
  }

  bool VertexSBAFlowDepth::write(std::ostream& os) const
  {
    Vector3d lv=estimate();
    for (int i=0; i<3; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }

} // end namespace
