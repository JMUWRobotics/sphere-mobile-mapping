#include "scan.hpp"
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/parallel_for.h>

std::vector<Scan*> Scan::allScans;

Scan::Scan(std::vector<Eigen::Vector3d> &points, Sophus::SE3d &pose){
    rPos = Eigen::Vector3d(0,0,0);//pose.translation();//
    rQuat = Eigen::Quaterniond(1,0,0,0);//pose.unit_quaternion();//
    transMatOrg= Sophus::SE3d();//pose;//
    transMat= Sophus::SE3d();//pose;//
    dalignxf = Sophus::SE3d();
    data = points;
    tree = new kd_tree_t(3, points, 10);
}
Scan::Scan()
{
  // pose and transformations
  rPos = Eigen::Vector3d(0,0,0);
  rPosTheta = Eigen::Vector3d(0,0,0);
  rQuat = Eigen::Quaterniond(1,0,0,0);
  transMat = Sophus::SE3d();
  transMatOrg = Sophus::SE3d();
  dalignxf = Sophus::SE3d();
}
/**
 * Calculates a set of corresponding point pairs and returns them.
 * The function uses the k-d trees stored the the scan class, thus
 * the function createTrees and deletTrees have to be called before
 * resp. afterwards.
 * Here we implement the so called "fast corresponding points"; k-d
 * trees are not recomputed, instead the apply the inverse transformation
 * to to the given point set.
 *
 * @param pairs The resulting point pairs (vector will be filled)
 * @param Source The scan whose points are matched to Targets' points
 * @param Target The scan to whiche the points are matched
 * @param thread_num number of the thread (for parallelization)
 * @param rnd randomized point selection
 * @param max_dist_match2 maximal allowed distance for matching
 * @return a set of corresponding point pairs
 */
void Scan::getPtPairs(std::vector<Eigen::Vector2i> &correspondences,
                      Scan* Source, Scan* Target, double max_dist_match2)
{
  // std::vector<size_t> nn(1);
  // std::vector<double> dist2(1);
  // Eigen::Vector3d point_tf_test = Source->dalignxf.inverse() * Target->getDAlign()*Target->data[0];
  // const double query_test[3] = {point_tf_test.x(), point_tf_test.y(), point_tf_test.z()};
  // size_t found_test = Source->tree->index->knnSearch(query_test, 1, nn.data(), dist2.data());//Source->tree->index->knnSearch(query_test, 1, nn.data(), dist2.data());
  //for (auto pt : Target->data){
    std::vector<int> match_index(Target->data.size(), -1);

    size_t N = Target->data.size();
    tbb::parallel_for(tbb::blocked_range<size_t>(0, N),
        [&](const tbb::blocked_range<size_t> &range){
            std::vector<size_t> nn_index(1);
            std::vector<double> nn_dist2(1);
            for (size_t i = range.begin(); i < range.end(); ++i){
              Eigen::Vector3d point_tf = Source->getDAlign().inverse() * Target->getDAlign() * Target->data[i];
                const double query[3] = {point_tf.x(), point_tf.y(), point_tf.z()};
                size_t found = Source->tree->index->knnSearch(query, 1, nn_index.data(), nn_dist2.data());
                if (found > 0 && nn_dist2[0] <= max_dist_match2){
                    match_index[i] = static_cast<int>(nn_index[0]);
                }
            }
        });
        // Build correspondences vector and accumulate error
        correspondences.clear();
        for (size_t i = 0; i < Target->data.size(); ++i){
            if (match_index[i] >= 0){
                correspondences.emplace_back(static_cast<int>(i), match_index[i]);
            }
        }
    //}
}
void Scan::transform(Sophus::SE3d &pose){
  transMat=pose;
  dalignxf=pose;
  rPos = transMat.translation();
  rQuat = transMat.unit_quaternion();
  //cout << "Translation: " << rPos << endl;
  //cout << "Rotation: " << rQuat.w() <<", " << rQuat.x()<<", "<<rQuat.y()<<", "<<rQuat.z() << endl;
}
