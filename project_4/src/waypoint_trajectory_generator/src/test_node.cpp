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

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int    _dev_order, _min_order;

// for planning
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
Vector3d _startPos = Vector3d::Zero();
Vector3d _startVel = Vector3d::Zero();
int Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::MatrixXd PolyOOQPGeneration(
        const int d_order,                    // the order of derivative
        const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
        const Eigen::MatrixXd &Vel,           // boundary velocity
        const Eigen::MatrixXd &Acc,           // boundary acceleration
        const Eigen::VectorXd &Time);          // time allocation in each segment
void optimization(int var_num,int Qnnz,std::vector<int> Qrow,std::vector<int> Qcol,std::vector<double> Qvalue,int eq_con_num,int Annz, std::vector<int> Arow,
std::vector<int> Acol,std::vector<double> Avalue,std::vector<double> bvalue);
int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel",   _Vel,   1.0 );
    nh.param("planning/acc",   _Acc,   1.0 );
    nh.param("planning/dev_order", _dev_order,  3 );
    nh.param("planning/min_order", _min_order,  3 );
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    //_poly_numID is the maximum order of polynomial
    _poly_num1D = 2 * _dev_order;

    //state of start point
    _startPos(0)  = 0;
    _startPos(1)  = 0;
    _startPos(2)  = 0;

    _startVel(0)  = 0;
    _startVel(1)  = 0;
    _startVel(2)  = 0;

    TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;


    vector<Vector3d> wp_list;
    wp_list.clear();
    for (int k = 1; k < 4; k++)
    {
        Vector3d pt(k,k,k);
        wp_list.push_back(pt);
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) = _startPos;
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];
    MatrixXd vel = MatrixXd::Zero(2, 3);
    MatrixXd acc = MatrixXd::Zero(2, 3);
    //acceleration of start and end point are 0, and the target vel is 0
    vel.row(0) = _startVel;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.

    VectorXd time(waypoints.rows() - 1);
    for(int i =0;i<waypoints.rows()-1;i++)
        time(i)=1;
    _polyTime=time;

    _polyCoeff = PolyOOQPGeneration(3, waypoints, vel, acc, _polyTime);
}

/// solver the QP problem with ooqp library
Eigen::MatrixXd PolyOOQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    const int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    /// x, y and z;
    for(int i=0;i<3;i++)
    {
        VectorXd WP(m+1);
        for(int j=0;j<Path.rows();j++)
        {
            WP(j)=Path(j,i);
        }
        double vel_start=Vel(0,i);
        double acc_start=Acc(0,i);
        double vel_end  =Vel(1,i);
        double acc_end  =Acc(1,i);
        VectorXd start_state,end_state;
        start_state.resize(d_order);
        end_state.resize(d_order);
        start_state(0)=WP(0);
        start_state(1)=vel_start;
        start_state(2)=acc_start;
        end_state(0)=WP(m);
        end_state(1)=vel_end;
        end_state(2)=acc_end;

        /** number of variable **/
        int var_num=p_num1d*m;
        int equ_con_num=0;
        /**   Produce Matrix Q, J = p'*Q*p **/
        std::vector<int> Qrow;
        std::vector<int> Qcol;
        std::vector<double> Qvalue;

        std::vector<int> Arow;
        std::vector<int> Acol;
        std::vector<double> Avalue;
        std::vector<double> bvalue;
        /// compute Q matrix as a sparse matrix
        int num_of_Q=0;
        for(int k=0;k<m;k++)
        {
            double t=Time(k);
            for(int nr=0;nr<p_num1d;nr++)
                for(int nl=0;nl<=nr;nl++)
                {
                    if(nr<d_order||nl<d_order)
                        continue;
                    else
                    {
                        Qrow.push_back(k*p_num1d+nr);
                        Qcol.push_back(k*p_num1d+nl);
                        Qvalue.push_back(std::pow(t,nr+nl-2*d_order+1)*Factorial(nr)*Factorial(nl)/Factorial(nl-d_order)/Factorial(nr-d_order)/(nr+nl-2*d_order+1));
                        num_of_Q++;
                    }
                }
        }
        if(num_of_Q!=Qrow.size())
            ROS_FATAL("error in Q computation");
        /***equality constraints***/
        ///start pos vel acc and jerk
        for(int d=0;d<d_order;d++)
        {
            Arow.push_back(d);
            Acol.push_back(d);
            Avalue.push_back(Factorial(d));
            bvalue.push_back(start_state(d));
            equ_con_num++;
        }

        ///end pos vel acc and jerk
        for(int d=0;d<d_order;d++)
        {
            for(int j=d;j<p_num1d;j++)
            {
                Arow.push_back(equ_con_num);
                Acol.push_back(var_num-p_num1d+j);
                Avalue.push_back(std::pow(Time(m-1),(j-d))*Factorial(j)/Factorial(j-d));
            }
            bvalue.push_back(end_state(d));
            equ_con_num++;
        }

        ///waypoint constraints, the start pos of every segment
        for(int k=1;k<m;k++)
        {
            Arow.push_back(equ_con_num);
            Acol.push_back(k*p_num1d);
            Avalue.push_back(1.0);
            bvalue.push_back(WP(k));
            equ_con_num++;
        }

        ///dev continuity constrain between each 2 segments
        for(int k=0;k<m-1;k++)
        {
            int tmp_num=k*p_num1d;
            double ts=Time(k);

            for(int d=0;d<d_order;d++)
            {
                for(int j=d;j<p_num1d;j++)
                {
                    Arow.push_back(equ_con_num);
                    Acol.push_back(tmp_num+j);
                    Avalue.push_back(std::pow(ts,(j-d))*Factorial(j)/Factorial(j-d));
                }
                Arow.push_back(equ_con_num);
                Acol.push_back(tmp_num+p_num1d+d);
                Avalue.push_back(-Factorial(d));
                bvalue.push_back(0.0);
                equ_con_num++;
            }
        }
        optimization(var_num,Qrow.size(),Qrow,Qcol,Qvalue,equ_con_num,Arow.size(),Arow,
        Acol,Avalue,bvalue);
    }
    for(int k=0;k<m;k++)
    {
        VectorXd tmp(3*p_num1d),tx(p_num1d),ty(p_num1d),tz(p_num1d);
        tx=Px.segment(k*p_num1d,p_num1d);
        ty=Py.segment(k*p_num1d,p_num1d);
        tz=Pz.segment(k*p_num1d,p_num1d);
        tmp<<tx,ty,tz;
        PolyCoeff.row(k)=tmp;
    }
    return PolyCoeff;
}

void optimization(int var_num,int Qnnz,std::vector<int> Qrow,std::vector<int> Qcol,std::vector<double> Qvalue,int eq_con_num,int Annz, std::vector<int> Arow,
std::vector<int> Acol,std::vector<double> Avalue,std::vector<double> bvalue)
{

    int nx=var_num;
    VectorXd cv;
    cv=VectorXd::Zero( var_num );
    double  *c;
    c=cv.data();
    double xupp[var_num],xlow[var_num];
    char ixupp[var_num],ixlow[var_num];
    for(int i=0;i<var_num;i++)
    {
        xupp[i]=0;
        xlow[i]=0;
        ixupp[i]=0;
        ixlow[i]=0;
    }

    ///Q matrix
    int nnzQ=Qnnz;
    int *irowQ,*jcolQ;
    double *dQ;
    irowQ=Qrow.data();
    jcolQ=Qcol.data();
    dQ=Qvalue.data();

    /////equality contraintsint
    int my         = eq_con_num;
    int nnzA       = Annz;

    int *irowA=new int[Arow.size()];
    int *jcolA=new int[Acol.size()];
    double *dA=new double[Avalue.size()];
    double *b = new double[bvalue.size()];
    irowA=&Arow[0];
    jcolA=&Acol[0];
    dA=&Avalue[0];
    b =&bvalue[0];
    for(int j=0;j<Avalue.size();j++)
        std::cout<<dA[j]<<std::endl;

    ///inequality contraints
    const int mz   = 0;
    double clow[]  = {};
    char  iclow[]  = {};

    double cupp[]  = {};
    char  icupp[]  = {};

    const int nnzC = 0;
    int   irowC[]  = {};
    int   jcolC[]  = {};
    double   dC[]  = {};

    QpGenSparseMa27 * qp
            = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );

    QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
                c,      irowQ,  nnzQ,   jcolQ,  dQ,
                xlow,   ixlow,  xupp,   ixupp,
                irowA,  nnzA,   jcolA,  dA,     b,
                irowC,  nnzC,   jcolC,  dC,
                clow,   iclow,  cupp,   icupp );

    QpGenVars      * vars
            = (QpGenVars *) qp->makeVariables( prob );
    QpGenResiduals * resid
            = (QpGenResiduals *) qp->makeResiduals( prob );

    GondzioSolver  * s     = new GondzioSolver( qp, prob );

    int ierr = s->solve(prob,vars, resid);
    if( ierr == 0 ) {
        cout.precision(4);
//        cout << "Solution: \n";

//        vars->x->writefToStream( cout, "x[%{index}] = %{value}" );
    } else {
        cout << "Could not solve the problem.\n";
    }
    double variables[vars->x->length()];
//    std::cout<<vars->x->length()<<std::endl;
    vars->x->copyIntoArray(&variables[0]);
    for(int i=0;i<vars->x->length();i++)
    {
        std::cout<<" "<<variables[i]<<std::endl;
    }


}
