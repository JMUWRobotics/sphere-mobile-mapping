#pragma once

#include "../lio_node.hpp"

class Scan;
typedef std::vector<Scan*> ScanVector;

class Scan {
public:
  static std::vector<Scan*> allScans;
  std::vector<Eigen::Vector3d> data;
  inline const Eigen::Vector3d get_rPos() const{
    return rPos;
  }
  inline const Eigen::Vector3d get_rPosTheta() const{
    return rPosTheta;
  }
  inline const Eigen::Quaterniond get_rPosQuat() const{
    return rQuat;
  }
  //! Pose matrix after initial and match transformations (org+dalign)
  inline const Sophus::SE3d get_transMat() const{
    return transMat;
  }
  //! Original pose matrix after initial transform
  inline const Sophus::SE3d get_transMatOrg() const{
    return transMatOrg;
  }
  //! Accumulated delta transformation matrix
  inline const Sophus::SE3d getDAlign() const{
    return dalignxf;
  }

  // Scan matching functions
  static void getPtPairs(std::vector<Eigen::Vector2i> &correspondences,
                         Scan* Source,
                         Scan* Target,
                         double max_dist_match2);
  void transform(Sophus::SE3d &pose);
  Scan();
  Scan(std::vector<Eigen::Vector3d> &points, Sophus::SE3d &pose);

protected:
  /**
   * The pose of the scan
   * Note: rPos/rPosTheta and transMat _should_
   *       always represent the same pose!!!
   */
  Eigen::Vector3d rPos;    //!< 3D position
  Eigen::Vector3d rPosTheta;    //!< 3D rotation in Euler representation
  Eigen::Quaterniond rQuat;        //!< 3D rotation in Quaternion representation
  Sophus::SE3d transMat;    //!< (4x4) transformation matrix
  Sophus::SE3d transMatOrg; //!< The original pose of the scan, e.g., from odometry

  /**
   * The dalignxf transformation represents the delta transformation
   * virtually applied to the tree and is used to compute are actual
   * corresponding points.
   */
  Sophus::SE3d dalignxf;
  kd_tree_t* tree;
};
