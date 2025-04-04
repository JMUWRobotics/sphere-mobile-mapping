#include "math_helpers.h"

double deg2rad(const double deg){
    return deg * M_PI / 180.0;
}

void subtract_points(const PointType &p1, const PointType &p2, std::vector<double> &d){
    d[0] = p1.x - p2.x;
    d[1] = p1.y - p2.y;
    d[2] = p1.z - p2.z;
}

double dot_product(const std::vector<double> &v1, const std::vector<double> &v2){
    double product = 0.0;
    for(int i=0; i<v1.size(); i++)
        product += v1[i] * v2[i];
    return product;
}

void cross_product(const std::vector<double> &v1, const std::vector<double> &v2, std::vector<double> &res){
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void normalize_vector(std::vector<double> &v){
    double norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
}

std::vector<double> convert2polar(std::vector<double> cartesian_vector){
    double phi, theta, theta0;
    std::vector<double> polar = {0.0, 0.0, 0.0};

    normalize_vector(cartesian_vector);
    phi = acos(cartesian_vector[2]);

    if(fabs(phi) < 0.0001){
        theta = 0.0;
    } else if(fabs(M_PI - phi) < 0.0001){
        theta = 0.0;
    } else {
        if(fabs(cartesian_vector[0]/sin(phi)) > 1.0){
            if(cartesian_vector[0]/sin(phi) < 0){
                theta0 = M_PI;
            } else {
                theta0 = 0.0;
            }
        } else {
            theta0 = acos(cartesian_vector[0]/sin(phi));
        }

        double sintheta = cartesian_vector[1]/sin(phi);
        double EPS = 0.0001;

        if(fabs(sin(theta0) - sintheta) < EPS){
            theta = theta0;
        } else if(fabs(sin( 2*M_PI - theta0) - sintheta) < EPS){
            theta = 2*M_PI - theta0;
        } else {
            theta = 0;
            printf("ERROR! Error converting cartesian coordinates into polar\n");
        }
    }

    polar[0] = phi;
    polar[1] = theta;
    polar[2] = -1.0;
    return polar;
}

// void calculate_mean(const std::vector<std::vector<double>> &list_vectors, std::vector<double> &mean){
//     mean = {0.0, 0.0, 0.0};
//     for(int i = 0; i < list_vectors.size(); i++){
//         mean[0] += list_vectors[i][0];
//         mean[1] += list_vectors[i][1];
//         mean[2] += list_vectors[i][2];
//     }
//     mean[0] = mean[0] / list_vectors.size();
//     mean[1] = mean[1] / list_vectors.size();
//     mean[2] = mean[2] / list_vectors.size();
// }
