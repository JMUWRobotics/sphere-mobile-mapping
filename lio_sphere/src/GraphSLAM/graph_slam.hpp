#pragma once

#include "graph.hpp"
#include "scan.hpp"

void performGraphSLAM(std::vector<std::vector<Eigen::Vector3d>> &scans, std::vector<Sophus::SE3d> &poses);
Graph* computeGraph6Dautomatic(std::vector <Scan *> &allScans, int clpairs, double max_dist_match2);
double doGraphSlam6D(Graph gr, std::vector <Scan *> &allScans, int nrIt, double epsilon, double max_dist_match2);
void FillGB3D(Graph *gr,
              Eigen::MatrixXd* G,
              Eigen::VectorXd* B,
              std::vector<Scan *> allScans,
              double max_dist_match2 );
void covarianceQuat(Scan *first, Scan *second, double max_dist_match2,
                    Eigen::MatrixXd &C, Eigen::VectorXd &CD);

