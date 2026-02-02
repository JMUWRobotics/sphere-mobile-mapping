// convert the incoming std::vector<Eigen::Vector3d> into graph data structure
#include "graph_slam.hpp"

void performGraphSLAM(std::vector<std::vector<Eigen::Vector3d>> &scans, std::vector<Sophus::SE3d> &poses){
  ROS_INFO_STREAM("input: " << scans.size());
  Scan::allScans.reserve(scans.size());
  for(size_t i = 0; i < scans.size(); ++i){
    Scan* scan = new Scan(scans[i], poses[i]);
    Scan::allScans.push_back(scan);
  }
  ROS_INFO_STREAM("input: " << Scan::allScans.size());
  int clpairs = 10;
  int nrIt = 10;
  double epsilon = 0.01;
  double max_dist_match2 = 1;
  Graph *graph = computeGraph6Dautomatic(Scan::allScans, clpairs, max_dist_match2); //TODO: How to initialize clpairs?
  ROS_INFO_STREAM("Graph built successfully!");
  doGraphSlam6D(*graph, Scan::allScans, nrIt, epsilon, max_dist_match2); //TODO: How to initializr nrIt?
  ROS_INFO_STREAM("Graph SLAM done successfully!");
  for (size_t i=0; i < poses.size(); ++i){
    poses[i] = Scan::allScans[i]->getDAlign()*poses[i];
  }
}


Graph* computeGraph6Dautomatic(std::vector <Scan *> &allScans, int clpairs, double max_dist_match2)
{
    Sophus::SE3d id = Sophus::SE3d();

    int i = 0;
    i++;
    Graph *gr = new Graph(0, false);
    int j, maxj = (int)allScans.size();
  // #ifdef _OPENMP
  //   omp_set_num_threads(OPENMP_NUM_THREADS);
  // #pragma omp parallel for schedule(dynamic)
  // #endif
    for (j = 0; j <  maxj; j++) {
  // #ifdef _OPENMP
  //     int thread_num = omp_get_thread_num();
  // #else
  //     int thread_num = 0;
  // #endif
      for (int k = 0; k < (int)allScans.size(); k++) {
        if (j == k) continue;
        Scan * FirstScan  = allScans[j];
        Scan * SecondScan = allScans[k];
        std::vector<Eigen::Vector2i> correspondences;

        Scan::getPtPairs(correspondences, FirstScan, SecondScan, max_dist_match2);//TODO: How to initialize max_dist?
        ROS_INFO_STREAM("Found correspondences: " << j << " and " << k << ", " << correspondences.size());
        if (correspondences.size() > clpairs) {
  // #ifdef _OPENMP
  // #pragma omp critical
  // #endif
          gr->addLink(j, k);
        }
      }
    }

    return gr;
}

    /**
 * This function is used to match a set of laser scans with any minimally
 * connected Graph, using the globally consistent LUM-algorithm in 3D.
 *
 * @param gr Some Graph with no real subgraphs except for itself
 * @param allScans Contains all laser scans
 * @param nrIt The number of iterations the LUM-algorithm will run
 * @return Euclidian distance of all pose shifts
 */
double doGraphSlam6D(Graph gr, std::vector <Scan *> &allScans, int nrIt, double epsilon, double max_dist_match2)
{
  Eigen::Matrix4d id = Eigen::Matrix4d::Identity();
  double ret = DBL_MAX;

  for(int iteration = 0; iteration < nrIt && ret > epsilon; iteration++) {

    if (nrIt > 1) cout << "Iteration " << iteration << endl;


    // * Calculate X and CX from all Dij and Cij
    int n = (gr.getNrScans() - 1);

    // Construct the linear equation system..
    Eigen::MatrixXd G=Eigen::MatrixXd::Zero(7*n, 7*n);
    Eigen::VectorXd B=Eigen::VectorXd::Zero(7*n);
    // ...fill G and B...
    FillGB3D(&gr, &G, &B, allScans, max_dist_match2);
    //cout << "filled G: " << G << ", B: " << B << endl;
    // ...and solve it
    Eigen::VectorXd X(7*n);
    X = G.llt().solve(B);
    //cout << "solved: " << X << endl;
    //ColumnVector X =  solveSparseCholesky(G, B);

    //cout << "X done!" << endl;

    double sum_position_diff = 0.0;

    // Start with second Scan
    int loop_end = gr.getNrScans();
    cout << "size: " << loop_end << endl;
// #ifdef _OPENMP
// #pragma omp parallel for reduction(+:sum_position_diff)
// #endif
    for(int i = 1; i < loop_end; i++){

      // Now update the Poses
      Eigen::MatrixXd Ha = Eigen::MatrixXd::Identity(7,7);

      double xa = allScans[i]->get_rPos().x();
      double ya = allScans[i]->get_rPos().y();
      double za = allScans[i]->get_rPos().z();

      //TODO: Änderd
      double p = allScans[i]->get_rPosQuat().w();
      double q = allScans[i]->get_rPosQuat().x();
      double r = allScans[i]->get_rPosQuat().y();
      double s = allScans[i]->get_rPosQuat().z();

      double px = p * xa;
      double py = p * ya;
      double pz = p * za;

      double qx = q * xa;
      double qy = q * ya;
      double qz = q * za;

      double rx = r * xa;
      double ry = r * ya;
      double rz = r * za;

      double sx = s * xa;
      double sy = s * ya;
      double sz = s * za;

      // Fill Ha
      Ha(3,3) = 2 * p;
      Ha(4,3) = 2 * q;
      Ha(5,3) = 2 * r;
      Ha(6,3) = 2 * s;

      Ha(3,4) = 2 * q;
      Ha(4,4) = -2 * p;
      Ha(5,4) = -2 * s;
      Ha(6,4) = 2 * r;

      Ha(3,5) = 2 * r;
      Ha(4,5) = 2 * s;
      Ha(5,5) = -2 * p;
      Ha(6,5) = -2 * q;

      Ha(3,6) = 2 * s;
      Ha(4,6) = -2 * r;
      Ha(5,6) = 2 * q;
      Ha(6,6) = -2 * p;

      Ha(0,3) = -2 * (px + sy - rz);
      Ha(1,3) = -2 * (-sx + py + qz);
      Ha(2,3) = -2 * (rx - qy + pz);

      Ha(0,4) = -2 * (qx + ry + sz);
      Ha(1,4) = -2 * (-rx + qy - pz);
      Ha(2,4) = -2 * (-sx + py + qz);

      Ha(0,5) = -2 * (rx - qy + pz);
      Ha(1,5) = -2 * (qx + ry + sz);
      Ha(2,5) = -2 * (-px - sy + rz);

      Ha(0,6) = -2 * (sx - py - qz);
      Ha(1,6) = -2 * (px + sy - rz);
      Ha(2,6) = -2 * (qx + ry + sz);

      // Invert it
      Ha = Ha.inverse();

      // Get pose estimate
      Eigen::VectorXd Xtmp = X.segment((i-1)*7, 7);

      // Correct pose estimate
      Eigen::VectorXd result = Ha * Xtmp;

      Eigen::Vector3d rPos;
      Eigen::Quaterniond rPosQuat;

      // calculate the updated Pose
      rPos = allScans[i]->get_rPos() - result.segment(0, 3);

      double qtmp[4];
      qtmp[0] = result(3);
      qtmp[1] = result(4);
      qtmp[2] = result(5);
      qtmp[3] = result(6);

      rPosQuat.w() = allScans[i]->get_rPosQuat().w() - qtmp[0];
      rPosQuat.x() = allScans[i]->get_rPosQuat().x() - qtmp[1];
      rPosQuat.y() = allScans[i]->get_rPosQuat().y() - qtmp[2];
      rPosQuat.z() = allScans[i]->get_rPosQuat().z() - qtmp[3];

     rPosQuat.normalize();

      // Update the Pose
      Sophus::SE3d pose(rPosQuat, rPos);
      allScans[i]->transform(pose);

      Eigen::Vector3d x;
      x = result.segment(0,3);
      sum_position_diff += x.norm();
    }
    ret = (sum_position_diff / (double)gr.getNrScans());
  }

  return ret;
}
/**
 * A function to fill the linear system G X = B.
 *
 * @param gr the Graph is used to map the given covariances C and CD
 *           matrices to the correct link
 * @param CD A vector containing all covariances C multiplied with
 *           their respective estimations D
 * @param C A vector containing all covariances C of the pose
 *          difference estimations D
 * @param G The matrix G specifying the linear equation
 * @param B The vector B
 */
void FillGB3D(Graph *gr,
              Eigen::MatrixXd* G,
              Eigen::VectorXd* B,
              std::vector<Scan *> allScans,
              double max_dist_match2 )
{
// #ifdef _OPENMP
// #pragma omp parallel for schedule(dynamic)
// #endif
  for(int i = 0; i < gr->getNrLinks(); i++){
    int a = gr->getLink(i,0) - 1;
    int b = gr->getLink(i,1) - 1;
    Scan *FirstScan  = allScans[gr->getLink(i,0)];
    Scan *SecondScan = allScans[gr->getLink(i,1)];

    Eigen::MatrixXd Cab;
    Eigen::VectorXd CDab;
    covarianceQuat(FirstScan, SecondScan, max_dist_match2, Cab, CDab);

    if(a >= 0){
      B->segment<7>(a*7) += CDab;
      G->block<7,7>(a*7,a*7) += Cab;
    }
    if(b >= 0){
      B->segment<7>(b*7) -= CDab;
      G->block<7,7>(b*7, b*7) += Cab;
    }
    if(a >= 0 && b >= 0) {
      G->block<7,7>(a*7, b*7) = -Cab;
      G->block<7,7>(b*7, a*7) = -Cab;
    }
  }
}

/**
 * This function calculates the inverse covariances Cij and
 * the Vector Cij*Dij for two scans by finding pointpairs.
 *
 * @param first pointer to the first scan of the link
 * @param second pointer to the second scan of the link
 * @param nns_method Specifies which NNS method is used
 * @param rnd shall we use randomization for computing the point pairs?
 * @param max_dist_match2 maximal distance allowed for point pairs
 * @param C pointer to the inverse of the covariance matrix Cij
 * @param CD pointer to the vector Cij*Dij
 */
void covarianceQuat(Scan *first, Scan *second, double max_dist_match2,
                    Eigen::MatrixXd &C, Eigen::VectorXd &CD)
{
  // x,y,z       denote the coordinates of uk (Here averaged over ak and bk)
  // sx,sy,sz    are the sums of their respective coordinates of uk over
  //             each paint pair
  // xpy,xpz,ypz are the sums over x*x + y*y ,x*x + z*z and y*y + z*z
  //             respectively over each point pair
  // xy,yz,xz    are the sums over each respective multiplication
  // dx,dy,dz    are the deltas in each coordinate of a point pair
  // ss          is the estimation of the covariance of sensing error
  double x, y, z, sx, sy, sz, xy, yz, xz, ypz, xpz, xpy, dx, dy, dz, ss, xpypz;

  // D is needed to calculate the estimation of the covariance s
  Eigen::VectorXd D(7);
  // Almost Cij*Dij
  Eigen::VectorXd MZ=Eigen::VectorXd::Zero(7);
  // Almost the covarianve
  Eigen::MatrixXd MM=Eigen::MatrixXd::Zero(7,7);
  // A set of point pairs
  std::vector<Eigen::Vector2i> correspondences;
  // A point pair
  Eigen::Vector3d ak, bk;
  // number of pairs in a set
  int m;

// #ifdef _OPENMP
//   int thread_num = omp_get_thread_num();
// #else
//   int thread_num = 0;
// #endif

  Scan::getPtPairs(correspondences, first, second, max_dist_match2);

  m = correspondences.size();
  cout << "number of correspondences: " << m << endl;

  sx = sy = sz = xy = yz = xz = ypz = xpz = xpy = xpypz = ss = 0.0;

  if (m > 2) {
    // for each point pair
    for(int j = 0; j < m; j++){
      ak = first->getDAlign()*first->data[correspondences[j][1]];
      bk = second->getDAlign()*second->data[correspondences[j][0]];

      // Some temporary values
      x = (ak.x() + bk.x())/2.0;
      y = (ak.y() + bk.y())/2.0;
      z = (ak.z() + bk.z())/2.0;
      dx = ak.x() - bk.x();
      dy = ak.y() - bk.y();
      dz = ak.z() - bk.z();

      // Sum up all necessary values to construct MM
      sx += x;
      sy += y;
      sz += z;

      xpy += x*x + y*y;
      xpz += x*x + z*z;
      ypz += y*y + z*z;
      xpypz += x*x + y*y + z*z;

      xy += x*y;
      xz += x*z;
      yz += y*z;

      // Sum up each part of MZ
      MZ[0] += dx;
      MZ[1] += dy;
      MZ[2] += dz;
      MZ[3] += x * dx + y * dy + z * dz;
      MZ[4] += z * dy - y * dz;
      MZ[5] += x * dz - z * dx;
      MZ[6] += y * dx - x * dy;
    }
    // Now construct the symmetrical matrix MM
    MM(0,0) = MM(1,1) = MM(2,2) = m;

    MM(3,3) = xpypz;
    MM(4,4) = ypz;
    MM(5,5) = xpz;
    MM(6,6) = xpy;

    MM(0,3) = MM(3,0) = sx;
    MM(0,5) = MM(5,0) = -sz;
    MM(0,6) = MM(6,0) = sy;

    MM(1,3) = MM(3,1) = sy;
    MM(1,4) = MM(4,1) = sz;
    MM(1,6) = MM(6,1) = -sx;

    MM(2,3) = MM(3,2) = sz;
    MM(2,4) = MM(4,2) = -sy;
    MM(2,5) = MM(5,2) = sx;

    MM(4,5) = MM(5,4) = -xy;
    MM(4,6) = MM(6,4) = -xz;
    MM(5,6) = MM(6,5) = -yz;

    // Calculate the pose difference estimation
    D = MM.inverse() * MZ;

    // Again going through all point pairs to faster calculate s.
    // This cannot be done earlier as we need D, and therefore
    // MM and MZ to do this
    for(int j = 0; j < m; j++){
      ak = first->getDAlign()*first->data[correspondences[j][1]];
      bk = second->getDAlign()*second->data[correspondences[j][0]];

      x = (ak.x() + bk.x()) / 2.0;
      y = (ak.y() + bk.y()) / 2.0;
      z = (ak.z() + bk.z()) / 2.0;

      ss += square(ak.x() - bk.x() - (D(0) + x * D(3) - z * D(5) + y * D(6)))
        + square(ak.y() - bk.y() - (D(1) + y * D(3) + z * D(4) - x * D(6)))
        + square(ak.z() - bk.z() - (D(2) + z * D(3) - y * D(4) + x * D(5)));
    }

    ss =  ss / (2*m - 3);
    ss = 1.0 / ss;

    CD = MZ * ss;
    C = MM * ss;

  } else {

    // This case should not occur
    ss = 0.0;
    MM(0,0) = MM(0,1) = MM(0,2) = 0.0;
    MM(1,0) = MM(1,1) = MM(1,2) = 0.0;
    MM(2,0) = MM(2,1) = MM(2,2) = 0.0;
    MZ(5) = MZ(0) = MZ(1) = MZ(6) = 0.0;
    MZ(2) = MZ(3) = MZ(4) = 0.0;
    C = Eigen::MatrixXd::Zero(7,7);
    CD = Eigen::VectorXd::Zero(7);
    cerr << "Error calculating covariance matrix" << endl;

  }
}