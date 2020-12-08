#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();                                 // the number of segments
  MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d); // position(x,y,z), so we need (3 * p_num1d) coefficients
  VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

  /* Produce Mapping Matrix M to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */

  //--------------------------------------------
  //Step1: Construct Hessian Q for objective function J

  MatrixXd Q = MatrixXd::Zero(m * (p_order + 1), m * (p_order + 1));
  double co_i;
  double co_l;
  double T;
  int indicator = 0;

  //cout << Time << endl;
  for (int k = 0; k <= m - 1; k++)
  {
    // STEP 1.1: calculate Q_k of the k-th segment
    MatrixXd Q_k = MatrixXd::Zero(p_order + 1, p_order + 1);

    for (double a = 0; a <= p_order; a++) //number of rows in Q_k depends on derivative of f(t) which is a polynomial segment
    {
      for (double b = 0; b <= p_order; b++)
      { // eg. 7+1=8 number of columns in Q_k essentially
        // only depends on derivative and order of f(t)
        // but in order to achieve symbolic consistency for the
        // coming QR solver, Q_k should be a square matrix with the
        // dimension of the parameter vector pj,which is p_order + 1

        if (a < d_order && b < d_order)
        {
          co_i = Factorial(p_order - a) / Factorial(p_order - a - d_order);
          co_l = Factorial(p_order - b) / Factorial(p_order - b - d_order);
          T = pow(Time(k), (p_order - a + p_order - b - p_order));
          Q_k(a, b) = co_i * co_l * T / (p_order - a + p_order - b - p_order);
        }
        else
        {
          Q_k(a, b) = 0;
        }
      }
    }
    //cout << Q_k << endl;

    //for (int i = 0;i < 8*m;i = i + 8) //对角排列Q_k：Q = blkdiag(Q, Q_k);
    //{
    Q.block(indicator, indicator, 8, 8) = Q_k;
    indicator = indicator + 8;
    //cout << Q << endl;
    //}
  }

  //----------------------------------------------

  //--------------------------
  //Steo2: Construct mapping matrix M

  MatrixXd M = MatrixXd::Zero(m * 8, m * 8);
  MatrixXd Co = MatrixXd::Zero(d_order, p_num1d);
  int order;
  int nom, dom;
  for (int col = 0; col <= p_order; col++) //Construct coefficient matrix Co
  {
    order = p_order - col; //p_order == 7
    Co(0, col) = 1;
    nom = Factorial(order);

    for (int row = 1; row < d_order; row++)
    {
      if (order > -1)
      {
        dom = Factorial(order - 1);
        Co(row, col) = nom / dom;
      }
      else
      {
        Co(row, col) = 0;
      }
      order = order - 1;
    }
  }

  //co_p = [1 1 1 1 1 1 1 1];
  //co_v = [7 6 5 4 3 2 1 0];
  //co_a = [42 30 20 12 6 2 0 0];
  //co_j = [210 120 60 24 6 0 0 0];
  //--------------------------

  indicator = 0;
  int t;
  int pre_order_index;
  int order_index;
  VectorXd co = VectorXd::Zero(p_num1d);
  VectorXd T_ = VectorXd::Zero(8);

  for (int k = 0; k <= m - 1; k++) //m:the number of segments; construct k=m M_k
  {
    MatrixXd M_k = MatrixXd::Zero(p_num1d, p_num1d);
    //#####################################################
    //STEP 1.1: calculate M_k of the k-th segment

    for (int row = 0; row <= d_order * 2 - 1; row++) // number of rows in M_k depends on the number of derivative of f(t) which we want to constrain, here: Minimum Snap pvaj;Minimum Jerk pva.
    {
      if (row <= d_order - 1) //at start point in easch segment: t=0
      {
        t = 0;
        co = Co.row(row);
      }
      else
      {
        t = Time(k);
        co = Co.row(row - d_order);
      }

      if (row <= d_order - 1) //useful parameter for T: to discern if it "passes" the start point of a segment
      {
        pre_order_index = d_order * 2 - (row + 1);
      }
      else
      {
        pre_order_index = d_order * 2 - (row + 1) + d_order;
      }

      for (int i = 0; i <= p_order; i++) // create T to reserve parameter T^n, eg. for d_order == 4: [T^7 T^6 ...T 1;T^6 T^5 ... 1 0;T^5 T^4...0 0;T^4 T^3 ... 0 0;]
      {
        order_index = pre_order_index - (i + 1) + 1;

        if (order_index >= 0)
        {
          T_(i) = pow(t, order_index);
        }
        else
        {
          T_(i) = 0;
        }
      }

      for (int col = 0; col <= p_order; col++) // number of columns in M_k only depends on p_order
      {
        M_k(row, col) = co(col) * T_(col);
      }
    }
    //cout << Time << endl;
    //cout << M_k << endl;
    //cout << M << endl;
    //for (int i = 0; i < 8*m;i = i + 8) //M_k：M = blkdiag(M, M_k);
    //{
    M.block(indicator, indicator, 8, 8) = M_k;
    indicator = indicator + 8;
    //}
  }
  //cout << M << endl;

  //Construct selection matrix Ct

  MatrixXd Ct = MatrixXd::Zero(8 * m, 4 * m + 4);
  int free_num = m - 1;

  for (int col = 0; col < 4; col++) //derivative constraints for start condition
  {
    for (int row = 0; row <= 8 * m - 1; row++)
    {
      if (col == row)
      {
        Ct(row, col) = 1;
        continue;
      }
      else
      {
        Ct(row, col) = 0;
        continue;
      }
    }
  }

  int index = 4;
  int count = 0;
  for (int col = 4; col <= 3 + free_num; col++) //derivative and continuity constraints for middle points
  {
    count = 0;

    for (int row = 0; row <= 8 * m - 1; row++)
    {
      if (row == index && count < 2)
      {
        Ct(row, col) = 1;
        count = count + 1;
        index = index + 4;
      }
      else
      {
        Ct(row, col) = 0;
      }
    }
  }

  int col_ = 0;
  for (int col = 4 + free_num; col <= 4 + free_num + 3; col++) //derivative constraints for end conditions
  {
    col_ = col_ + 1;
    for (int row = 0; row <= 8 * m - 1; row++)
    {
      if (row >= 8 * m - 1 - 3)
      {
        int row_ = row + 1 - (8 * m - 1) + 3;
        if (row_ == col_)
          Ct(row, col) = 1;
        else
          Ct(row, col) = 0;
      }
      else
      {
        Ct(row, col) = 0;
      }
    }
  }

  index = 5;
  int index_init = 5;
  int seg_count = 0;
  for (int col = 4 + free_num + 4; col <= 4 * m + 3; col++) //continuity constraints for middle points
  {
    count = 1;

    for (int row = 1; row < 8 * m - 1; row++)
    {
      if (row == index && count < 4)
      {
        Ct(row, col) = 1;
        index = index + 4;
        count = count + 1;
        seg_count = seg_count + 1;
      }
      else
      {
        Ct(row, col) = 0;
      }
      if (count == 3)
      {
        index_init = index_init + 1;
        index = index_init;
        count = count + 1;
      }

      if (seg_count == 6)
      {
        index_init = index_init + 5;
        index = index_init;
        seg_count = 0;
      }
    }
  }
  //cout << Ct << endl;

  /*   Produce the dereivatives in X, Y and Z axis directly.  */

  MatrixXd dF = MatrixXd::Zero(8 + m - 1, 3);
  MatrixXd start_cond = MatrixXd::Zero(d_order, 3);
  start_cond.row(0) = Path.row(0);
  start_cond.row(1) = Vel.row(0);
  start_cond.row(2) = Acc.row(0);
  start_cond.row(3) = Vector3d::Zero(); //j = 0
  //cout << start_cond << endl;
  MatrixXd end_cond = MatrixXd::Zero(d_order, 3);
  end_cond.row(0) = Path.row(Path.rows() - 1);
  end_cond.row(1) = Vel.row(1);
  end_cond.row(2) = Acc.row(1);
  end_cond.row(3) = Vector3d::Zero(); //j = 0
  //cout << end_cond << endl;

  for (int count = 0; count <= 2; count++)
  {
    for (int i = 0; i <= 3; i++)
    {
      dF(i, count) = start_cond(i, count);
    }

    int j = 1;
    for (int i = 4; i <= 8 + m - 1 - 5; i++)
    {
      dF(i, count) = Path(j, count);
      //cout << dF(i, count) << endl;
      j++;
      if (j == m)
        break;
    }

    j = 0;
    for (int i = 8 + m - 1 - 4; i <= 8 + m - 2; i++)
    {
      dF(i, count) = end_cond(j, count);
      j++;
      if (j == 4)
        break;
    }
  }
  //cout << dF << endl;
  //for start condition
  //for i = 1:4
  //  dF(i) = start_cond(i);

  //for middle points(only pos constraints)
  //j = 1;
  //for i( int = 5:8+m )-1-4
  //    dF(i) = waypoints(j+1);
  //        j = j+1;
  //        i( intf j == m )
  //            break

  //for end_cond
  //j = 1;
  //for( int i = 8+m )( int-1-3:8+m )-1
  //    dF(i) = end_cond(j);
  //   j = j + 1;
  //   if j == 4
  //       break

  /*   Produce the Minimum Snap cost function, the Hessian Matrix   */

  MatrixXd C = MatrixXd::Zero(4 * m + 4, 8 * m);
  C = Ct.transpose();
  MatrixXd R = MatrixXd::Zero(4 * m + 4, 4 * m + 4);
  R = C * (M.inverse()).transpose() * Q * M.inverse() * Ct;
  MatrixXd R_pp = MatrixXd::Zero(3 * (m - 1), 3 * (m - 1));
  R_pp = R.bottomRightCorner(3 * (m - 1), 3 * (m - 1)); //RowDim(dp)=3*(m-1)
  MatrixXd R_fp = MatrixXd::Zero(8 + m - 1, 3 * (m - 1));
  R_fp = R.topRightCorner(8 + m - 1, 3 * (m - 1)); //RowDim(dF)=8+m-1
  MatrixXd dp_op = MatrixXd::Zero(3 * (m - 1), 3);
  for (int i = 0; i < 3; i++)
  {
    dp_op.col(i) = -R_pp.inverse() * R_fp.transpose() * dF.col(i); //eg. m == 2: Dim(dp) == 3 x 3
  }
  //cout << dp_op << endl;

  // construct derivative vector d
  MatrixXd d = MatrixXd::Zero(4 * m + 4, 3); //Dim(d) == (4*m+4) x 3
  for (int j = 0; j < 3; j++)
  {
    d.block(0, j, 8 + m - 1, 1) = dF.col(j);
    d.block(8 + m - 1, j, 3 * (m - 1), 1) = dp_op.col(j);
  }
  //cout << d << endl;
  //cout << Path << endl;

  //construct parameter vector p with d
  MatrixXd p = MatrixXd::Zero(M.rows(), 3);
  //cout << M.rows() << endl;
  //cout << d.rows() << endl;
  p = M.inverse() * Ct * d;
  //cout << p << endl;

  int j = 0;
  for (int i = 0; i < m; i++)
  {
    PolyCoeff.block(i, 0, 1, 8) = p.block(j, 0, 8, 1).transpose().reverse();
    PolyCoeff.block(i, 8, 1, 8) = p.block(j, 1, 8, 1).transpose().reverse();
    PolyCoeff.block(i, 16, 1, 8) = p.block(j, 2, 8, 1).transpose().reverse();
    j = j + 8;
  }
  //MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);
  //cout << PolyCoeff << endl;
  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {

    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;

}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}