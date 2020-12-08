#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0); //8-10行给35行的函数用
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0); //12-14行给setObs用
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE; //50*50*25=62500
    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
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
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::_isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::_isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets, int _Heudirector)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    for (int i = 1; (i >= -1 || i == 0); i--) 
    {
        for (int j = 1; (j >= -1 || j == 0); j--)
        {
            for (int k = 1; (k >= -1 || k == 0); k--)
            {
                if ( !( (i == 0) && (j == 0) && (k == 0) ) )
                {
                    int _x = currentPtr->index(0) + i;
                    int _y = currentPtr->index(1) + j;
                    int _z = currentPtr->index(2) + k;

                    if ( ( _x >= 0 && _x < GLX_SIZE ) && ( _y >= 0 && _y < GLY_SIZE ) && ( _z >= 0 && _z < GLZ_SIZE ) )
                    {
                        GridNodePtr _neighborPtr = GridNodeMap[_x][_y][_z];

                        if ( (_isFree(_neighborPtr->index)) && (_neighborPtr->id != -1) ) // Check if the node is obs or already in closed set
                        {
                            neighborPtrSets.push_back(_neighborPtr);
                            double edgeCost = getHeu(currentPtr, _neighborPtr,_Heudirector);
                            edgeCostSets.push_back(edgeCost);
                        }
                    }
                }
            }
        }
    }

    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    *
    */
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2, int Heudirector)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
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
        return ( sqrt( xd + yd + zd)*Tie_breaker );
        //return (sqrt(xd + yd + zd));
    }

    if (Heudirector == 1)
    {
        double dx = abs(node1->coord(0) - node2->coord(0));
        double dy = abs(node1->coord(1) - node2->coord(1));
        double dz = abs(node1->coord(2) - node2->coord(2));
        return( ( dx + dy +dz ) * Tie_breaker );
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
        return( ( ( sqrt(3) - sqrt(2)) * dmin + ( sqrt(2) - 1) * dmid + dmax ) * Tie_breaker );
        //return ((sqrt(3) - sqrt(2)) * dmin + (sqrt(2) - 1) * dmid + dmax);
    }
    return 0;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt, int _Heudirector)
{   
    ROS_INFO( "Chosen heuristic:(%d)",_Heudirector);
    ROS_INFO( "Map Bound:(%f, %f,%f)",gl_xl,gl_yl,gl_zl);
    ros::Time time_1 = ros::Time::now();    
    
    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    if( _isOccupied(end_idx) )
    {
        ROS_WARN("The goal is occupied, Please choose a new goal");
    }

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    ROS_INFO( "Start Node opsition:(%f, %f,%f)",startPtr->coord(0),startPtr->coord(1),startPtr->coord(2));
    ROS_INFO( "Goal Node opsition:(%f,%f,%f)",endPtr->coord(0),endPtr->coord(1),endPtr->coord(2));

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear(); //清空openSet
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr,_Heudirector);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt; 
    startPtr->nodeMapIt = openSet.insert( make_pair( startPtr -> fScore, startPtr ) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;


    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be found in Homework description
        *
        *
        */

        openSet.begin()->second->id = -1; //Mark a node with lowest f as expanded == put it in closed set
        currentPtr = openSet.begin()->second ;
        openSet.erase( openSet.begin()->second->nodeMapIt );  //Remove the node from the open set 
        
        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets,_Heudirector);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     
        /*
        *
        *
        * 
        * 
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
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
            neighborPtr = neighborPtrSets[i] ; 
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need to do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */ 
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i] ;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr,_Heudirector);
                neighborPtr->cameFrom = currentPtr ;
                neighborPtr->id = 1;
                neighborPtr->nodeMapIt = openSet.insert( make_pair( neighborPtr->fScore , neighborPtr ) );

                continue;
            }
            else if( neighborPtr->id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset and then put neighbor in open set and record it
                please write your code below
                *        
                */
                if (neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i])
                {
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i]; //update gscore
                    neighborPtr->cameFrom = currentPtr;
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr, _Heudirector);
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));  
                }

                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                continue;
            }

        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    gridPath.push_back( terminatePtr );
    Vector3d start_pt;
    start_pt << 0.0, 0.0, 1.0;
    Vector3i start_index = coord2gridIndex( start_pt );
    for ( int i = 0 ; gridPath[i]->index != start_index ; i++ )
    {
      gridPath.push_back( gridPath[i]->cameFrom );
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);   
        
    std::reverse(path.begin(),path.end());
    return path;
}

