#include "hough.h"

/* ------------------------------- Accumulator ------------------------------- */

Accumulator::Accumulator(int rhoNum, int phiNum, int thetaNum, int rhoMax, int accumulatorMax){
    // Set variables
    this->rhoNum = rhoNum;
    this->phiNum = phiNum;
    this->rhoMax = rhoMax;
    this->accumulatorMax = accumulatorMax;

    double MAX_A = 2 * M_PI * 1.0;
    double step = 180.0 / phiNum;

    double r = sin(deg2rad(0.0));
    double z = cos(deg2rad(0.0));
    double r_next = 0.0;
    double z_next = 0.0;

    for(double phi = 0; phi < 180.0; phi += step){
        r_next = sin(deg2rad(phi + step));
        z_next = cos(deg2rad(phi + step));
        // Umfang des aktuellen Kreises
        double a = 2 * M_PI * (r + r_next) / 2.0;
        // Berechnung der Schrittgroesse in Grad
        double c = ((360.0 * (MAX_A / a)) / (thetaNum - 1));
        int size = static_cast<int>(1.0 + 360.0 / c);
        ballNr.push_back(size);
        r = r_next;
        z = z_next;
    }

    // Resize accumulator array accoridngly
    accumulator.resize(rhoNum, std::vector<std::vector<int>>(phiNum));
    for(int i = 0; i < rhoNum; i++){
        for(int j = 0; j < phiNum; j++){
            accumulator[i][j].resize(ballNr[j]);
            for(int k = 0; k < ballNr[j]; k++){
                accumulator[i][j][k] = 0;
            }
        }
    }
}

bool Accumulator::accumulate(double theta, double phi, double rho){
    int rhoindex = accumulator.size() - 1;
    if(rho < rhoMax){
        rhoindex = static_cast<int>((rho * static_cast<double>(accumulator.size()) / rhoMax));
    }

    int phiindex = static_cast<int>(phi * (((static_cast<double>(accumulator[0].size()) * 0.9999999) / M_PI)));
    int bNr = ballNr[phiindex];
    int thetaindex = static_cast<int>(theta * (((static_cast<double>(bNr) * 0.9999999999) / (2 * M_PI))));
    if(thetaindex >= bNr){
        std::cout << "duet";
    }

    accumulator[rhoindex][phiindex][thetaindex]++;
    return (accumulator[rhoindex][phiindex][thetaindex] >= accumulatorMax);
}

void Accumulator::resetAccumulator(){
    // Reset all bins
    for(unsigned int i = 0; i < accumulator.size(); i++){
        for(unsigned int j = 0; j < accumulator[i].size(); j++){
            int bNr = ballNr[j];
            for(int k = 0; k < bNr; k++){
                accumulator[i][j][k] = 0;
            }
        }
    }
}


/* ------------------------------- HOUGH ------------------------------- */

Hough::Hough(pcl::PointCloud<PointType>::Ptr &scan, Accumulator* acc, int minDist, int maxDist){
    this->scan = scan;
    this->acc = acc;
    this->minDist = minDist;
    this->maxDist = maxDist;
}

bool Hough::distanceOK(PointType p1, PointType p2, PointType p3){
    std::vector<double> d = {0.0, 0.0, 0.0};
    double d_sqr = 0.0;

    // p1 - p2
    subtract_points(p1, p2, d);
    d_sqr = pow(d[0], 2) + pow(d[1], 2) + pow(d[2], 2);
    if(d_sqr < pow(minDist, 2)) return false;
    if(d_sqr > pow(maxDist, 2)) return false;

    // p2 - p3
    subtract_points(p2, p3, d);
    d_sqr = pow(d[0], 2) + pow(d[1], 2) + pow(d[2], 2);
    if(d_sqr < pow(minDist, 2)) return false;
    if(d_sqr > pow(maxDist, 2)) return false;

    // p1 - p3
    subtract_points(p1, p3, d);
    d_sqr = pow(d[0], 2) + pow(d[1], 2) + pow(d[2], 2);
    if(d_sqr < pow(minDist, 2)) return false;
    if(d_sqr > pow(maxDist, 2)) return false;

    return true;
}

bool Hough::calculatePlane(PointType p1, PointType p2, PointType p3, double &theta, double &phi, double &rho){
    std::vector<double> v1 = {0.0, 0.0, 0.0}, v2 = {0.0, 0.0, 0.0}, n = {0.0, 0.0, 0.0};
    subtract_points(p3, p2, v1); // v1 = p3 - p2
    subtract_points(p1, p2, v2); // v2 = p1 - p2
    cross_product(v1, v2, n);    // n  = v1 x v2

    if((n[0]*n[0] + n[1]*n[1] + n[2]*n[2]) < pow(1e-9, 2)) return false;
    normalize_vector(n);

    rho = p1.x * n[0] + p1.y * n[1] + p1.z * n[2];
    if(rho < 0){
        rho = -rho;
        for(int i = 0; i < 3; i++){
            n[i] = -n[i];
        }
    }
    std::vector<double> polar = convert2polar(n);
    phi = polar[0];
    theta = polar[1];

    if((fabs(theta) < 0.001) && (fabs(phi) < 0.001) ) return false;
    return true;
}

void Hough::computeNormalVector(std::vector<double> &n, double rho, double theta, double phi){
    n[0] = cos(theta) * sin(phi);
    n[1] = sin(theta) * sin(phi);
    n[2] = cos(phi);
}

double Hough::RHT(std::vector<double> &n){
    PointType p1(0.0,0.0,0.0), p2(0.0,0.0,0.0), p3(0.0,0.0,0.0);
    double theta, phi, rho;

    int stop = 10000;
    int count = 0;
    while(count < stop){
        count++;
        // Caluculate 3 random points
        int p_i = static_cast<int>((scan->points.size()) * (rand() / (RAND_MAX + 1.0)));
        p1 = scan->points[p_i];
        p_i = static_cast<int>((scan->points.size()) * (rand() / (RAND_MAX + 1.0)));
        p2 = scan->points[p_i];
        p_i = static_cast<int>((scan->points.size()) * (rand() / (RAND_MAX + 1.0)));
        p3 = scan->points[p_i];

        // Check if ditsnace between points is okay
        if(!distanceOK(p1, p2, p3)) continue;

        // Calculate Plane from those 3 points
        if(calculatePlane(p1, p2, p3, theta, phi, rho)){
            // increment accumulator cell and if accumulate > accumulatorMax = found dominant plane
            if(acc->accumulate(theta, phi, rho)){
                computeNormalVector(n, rho, theta, phi);
                acc->resetAccumulator();
                return rho;
            }
        }
    }
    // printf("[hough.cpp]: RHT ran out of iterations!\n");
    return -1;
}
