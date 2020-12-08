#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id)
{
  gl_xl = global_xyz_l(0);
  gl_yl = global_xyz_l(1);
  gl_zl = global_xyz_l(2);

  gl_xu = global_xyz_u(0);
  gl_yu = global_xyz_u(1);
  gl_zu = global_xyz_u(2);

  GLX_SIZE = max_x_id;
  GLY_SIZE = max_y_id;
  GLZ_SIZE = max_z_id;
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

  resolution = _resolution;
  inv_resolution = 1.0 / _resolution;

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++)
  {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++)
    {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++)
      {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z)
{
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else
  {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }
}

void AstarPathFinder::getVisitedNodes()
{
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
      {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);


      }
  //ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  //return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index)
{
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt)
{
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord)
{
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const
{
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const
{
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const
{
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const
{
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets, int _Heudirector)
{
  neighborPtrSets.clear();
  edgeCostSets.clear();
  for (int i = 1; (i >= -1 || i == 0); i--)
  {
    for (int j = 1; (j >= -1 || j == 0); j--)
    {
      for (int k = 1; (k >= -1 || k == 0); k--)
      {
        if (!((i == 0) && (j == 0) && (k == 0)))
        {
          int _x = currentPtr->index(0) + i;
          int _y = currentPtr->index(1) + j;
          int _z = currentPtr->index(2) + k;

          if ((_x >= 0 && _x < GLX_SIZE) && (_y >= 0 && _y < GLY_SIZE) && (_z >= 0 && _z < GLZ_SIZE))
          {
            GridNodePtr _neighborPtr = GridNodeMap[_x][_y][_z];

            if ((isFree(_neighborPtr->index)) && (_neighborPtr->id != -1)) // Check if the node is obs or already in closed set
            {
              neighborPtrSets.push_back(_neighborPtr);
              double edgeCost = getHeu(currentPtr, _neighborPtr, _Heudirector);
              edgeCostSets.push_back(edgeCost);
            }
          }
        }
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2, int Heudirector)
{
  // using digonal distance and one type of tie_breaker.
  /*Note:
    * director = 0 : Euclidean distance
    * director = 1 : Manhattan distance
    * director = 2 : Diagonal distance
    */
  //initialize Tie breaker as a random number
  srand((unsigned)time(NULL));
  int Tie_breaker = 1 + rand() / 100000;

  if (Heudirector == 0)
  {

    double dx = node1->coord(0) - node2->coord(0);
    double dy = node1->coord(1) - node2->coord(1);
    double dz = node1->coord(2) - node2->coord(2);
    double xd = pow(dx, 2);
    double yd = pow(dy, 2);
    double zd = pow(dz, 2);
    return (sqrt(xd + yd + zd) * Tie_breaker);
    //return (sqrt(xd + yd + zd));
  }

  if (Heudirector == 1)
  {
    double dx = abs(node1->coord(0) - node2->coord(0));
    double dy = abs(node1->coord(1) - node2->coord(1));
    double dz = abs(node1->coord(2) - node2->coord(2));
    return ((dx + dy + dz) * Tie_breaker);
    //return (dx + dy + dz);
  }

  if (Heudirector == 2)
  {
    double dx = abs(node1->coord(0) - node2->coord(0));
    double dy = abs(node1->coord(1) - node2->coord(1));
    double dz = abs(node1->coord(2) - node2->coord(2));
    double dmin = min({dx, dy, dz});
    double dmax = max({dx, dy, dz});
    double dmid = dx + dy + dz - dmin - dmax;
    return (((sqrt(3) - sqrt(2)) * dmin + (sqrt(2) - 1) * dmid + dmax) * Tie_breaker);
    //return ((sqrt(3) - sqrt(2)) * dmin + (sqrt(2) - 1) * dmid + dmax);
  }
  return 0;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt, int Heudirector)
{
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);
  StartPtr = startPtr; //prepare for getPath()

  ROS_INFO("Start Node position:(%f, %f,%f)", startPtr->coord(0), startPtr->coord(1), startPtr->coord(2));
  ROS_INFO("Goal Node position:(%f,%f,%f)", endPtr->coord(0), endPtr->coord(1), endPtr->coord(2));

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->fScore = getHeu(startPtr, endPtr, Heudirector);

  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));
  //startPtr->nodeMapIt = openSet.insert(make_pair(startPtr->fScore, startPtr));

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/

  //double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  while (!openSet.empty())
  {

    openSet.begin()->second->id = -1; //Mark a node with lowest f as expanded == put it in closed set
    currentPtr = openSet.begin()->second;
    //openSet.erase( openSet.begin()->second->nodeMapIt );  //Remove the node from the open set
    openSet.erase(openSet.begin());

    // if the current node is the goal
    if (currentPtr->index == goalIdx)
    {
      ros::Time time_2 = ros::Time::now();
      terminatePtr = currentPtr;
      ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution);
      return;
    }
    //get the succetion
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets, Heudirector); //STEP 4: finish AstarPathFinder::AstarGetSucc yourself
    /*
        *
        *
        * 
        * 
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */
    for (int i = 0; i < (int)neighborPtrSets.size(); i++)
    {
      /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
      neighborPtr = neighborPtrSets[i];
      if (neighborPtr->id == 0)
      { //discover a new node, which is not in the closed set and open set
        /*
                *
                *
                STEP 6:  As for a new node, do what you need to do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
        neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr, Heudirector);
        neighborPtr->cameFrom = currentPtr;
        neighborPtr->id = 1;
        openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
        //neighborPtr->nodeMapIt = openSet.insert( make_pair( neighborPtr->fScore , neighborPtr ) );

        continue;
      }
      else if (neighborPtr->id == 1)
      { //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
        /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset and then put neighbor in open set and record it
                please write your code below
                *        
                */
        if (neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i])
        {
          //openSet.erase(neighborPtr->nodeMapIt);
          neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i]; //update gscore
          neighborPtr->cameFrom = currentPtr;
          neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr, Heudirector);
          //neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
        }

        continue;
      }
      else
      { //this node is in closed set
        /*
                *
                please write your code below
                *        
                */
        continue;
      }
    }
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath()
{
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  // gridPath.push_back(terminatePtr);
  // Vector3d start_pt;
  // start_pt << StartPtr->coord(0), StartPtr->coord(1), StartPtr->coord(2);
  // Vector3i start_index = coord2gridIndex(start_pt);

  // cout<<terminatePtr->index<<endl;

  // for (int i = 0; gridPath[i]->index != start_index; i++)
  // {
  //   gridPath.push_back(gridPath[i]->cameFrom);
  // }
  auto ptr = terminatePtr;
  while (ptr->cameFrom != NULL)
  {
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
  }
  resetUsedGrids();
  ROS_WARN("gridPath: Grids have been reset.");
  for (auto ptr : gridPath)
    path.push_back(ptr->coord);

  std::reverse(path.begin(), path.end());
  return path;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> path,
                                               double path_resolution)
{
  vector<Vector3d> subPath;
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *
   * **/
  double epsilon = path_resolution;

  //cout<< path[path.size()-1] <<endl;

  subPath = RDP(path, epsilon);

  //cout<< subPath[path.size()-1] <<endl;

  return subPath;
}

vector<Vector3d> AstarPathFinder::RDP(const vector<Vector3d> &path, double epsilon)
{
  double dmax = 0;
  int index = 0;
  int end = path.size() - 1;
  auto a = path[0];
  auto b = path[end];
  Vector3d ab = b - a;
  double ab_norm = sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));

  // cout<<"path's size"<<endl;
  // cout<< path.size()<<endl;

  //pick the furtherest between point
  for (int i = 1; i != end; i++)
  {
    //cal the dist from point c to line ab
    auto c = path[i];
    //! sometimes here occur segmentation fault, 越界？
    Vector3d ac = c - a;
    auto ac_norm = sqrt(pow(a[0] - c[0], 2) + pow(a[1] - c[1], 2) + pow(a[2] - c[2], 2));
    auto proj_ac_t_ab = ac.dot(ab) / ab_norm;
    auto dist = sqrt(pow(ac_norm, 2) - pow(proj_ac_t_ab, 2));

    if (dist > dmax)
    {
      index = i;
      dmax = dist;
    }
  }

  vector<Vector3d> result;
  vector<Vector3d> path_f;
  vector<Vector3d> path_b;

  // if(index==1){
  //   cout<<"that's Fault coming"<<endl;
  // }

  if (dmax > epsilon)
  {
    auto path_ = path;
    vector<Vector3d>::iterator iter = path_.begin() + index + 1;

    for (auto index_ = index; index_ < int(path.size()); index_++)
    {

      path_b.push_back(path[index_]);

      if (index_ != int(path.size()) - 1)
      {
        path_.erase(iter);
        iter++;
      }
    }

    path_f = path_;

    // if(path_f.size()==1){
    //  cout<<"that's Fault"<<endl;
    //  cout<< "dmax is" <<endl;
    //  cout<< dmax<<endl;
    //  cout<< "index is"<<endl;
    //  cout<< index <<endl;
    // }

    //recursive call
    auto recResults1 = RDP(path_f, epsilon);
    auto recResults2 = RDP(path_b, epsilon);

    //reconstruct the result list: put two std::vector together
    result.insert(result.end(), recResults1.begin(), recResults1.end() - 1);
    result.insert(result.end(), recResults2.begin(), recResults2.end());
  }
  else
  {
    //reconstruct the path as result
    //with erasion of the furtherest between point path[index]
    result.push_back(path[0]);
    result.push_back(path[end]);
  }

  //cout<<"result size:"<<endl;
  //cout<< result.size() <<endl;
  //for(int i=0; i!= int(result.size());i++)
  //{
  //  cout<< result[i] <<endl;
  //}

  return result;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t)
{
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++)
  {
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

VectorXd AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time, Vector3d map_upper, Vector3d map_lower)
{
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
                           /**
   *
   * STEP 3.3:  finish the safeCheck()
   *
   * **/
  Vector3i idx;
  Vector3d pos;
  VectorXd time_check = Eigen::VectorXd::Zero(time.size()+1);
  for (int i = 0; i < time.size(); i++) 
  {
    for (double t = 0.0; t < time(i); t += 0.01)
    {
      pos = getPosPoly(polyCoeff, i, t);
      idx = coord2gridIndex(pos);
      if( isOccupied(idx) )
      {
        if( time_check[0] != -1)
        {
          time_check[0] = -1;
        }

        if( time_check(i+1) != 1)
        {
          time_check(i+1) = 1;
        }
        else
        {
          continue;
        }
        
      }
      
    }
  }
  return time_check;
}