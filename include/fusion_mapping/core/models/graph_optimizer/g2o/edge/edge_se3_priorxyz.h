//
// Created by linsin on 13/03/2020.
//

#ifndef FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3PRIORXYZ_H_
#define FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3PRIORXYZ_H_
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PriorXYZ()
      :g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {
  }

  void computeError() override {
    const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

    Eigen::Vector3d estimate = v1->estimate().translation();
    _error = estimate - _measurement;
  }

  void setMeasurement(const Eigen::Vector3d& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);

    setMeasurement(Eigen::Vector3d(v));

    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  virtual bool write(std::ostream& os) const override {
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";
    for (int i = 0; i < information().rows(); ++i)
      for (int j = i; j < information().cols(); ++j)
        os << " " << information()(i, j);
    return os.good();
  }
};
}

#endif //FUSION_MAPPING_INCLUDE_FUSION_MAPPING_CORE_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3PRIORXYZ_H_
