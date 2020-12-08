#ifndef OOQP_INTER_H
#define OOQP_INTER_H
#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

// Useful customized headers
#include "trajectory_generator_waypoint.h"
#include "ooqp_test_node.h"

using namespace std;
using namespace Eigen;
class ooqp_inter
{
public:
    ooqp_inter();
    VectorXd optimization(int var_num,int Qnnz,std::vector<int> Qrow,std::vector<int> Qcol,std::vector<double> Qvalue,int eq_con_num,int Annz, std::vector<int> Arow,
    std::vector<int> Acol,std::vector<double> Avalue,std::vector<double> bvalue);

};

#endif // OOQP_INTER_H
