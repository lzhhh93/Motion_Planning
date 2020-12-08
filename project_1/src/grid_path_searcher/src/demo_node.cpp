#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Astar_searcher.h"
#include "JPS_searcher.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;
 
namespace backward {
backward::SignalHandling sh;
}

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;

AstarPathFinder * _astar_path_finder     = new AstarPathFinder();
JPSPathFinder   * _jps_path_finder       = new JPSPathFinder();

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);

void visGridPath( vector<Vector3d> nodes, bool is_use_jps );
void visVisitedNode( vector<Vector3d> nodes );
void pathFinding(const Vector3d start_pt, const Vector3d target_pt, int _Heudirector);

void rcvWaypointsCallback(const nav_msgs::Path & wp) //获得waypoint后进行路径搜索
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    //获取交互式界面给出的终点坐标
    target_pt << wp.poses[0].pose.position.x, //<<把wp里的xyz坐标给到target_pt
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    //choose heuristic function: 0 = Euclidian, 1 = Manhatten, 2 = Diagonal)
    int Heudirector = 2;
    pathFinding(_start_pt, target_pt, Heudirector); 
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        // set obstalces into grid map(栅栏化地图) for path planning
        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        _jps_path_finder->setObs(pt.x, pt.y, pt.z);

        // for visualize only（可视化地图部分）
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);//把用于可视化的地图发布

    _has_map = true;
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt, int _Heudirector)
{
    //Call A* to search for a path
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt, _Heudirector); //把两个点给到AstarGraphSearch

    //Retrieve the path
    auto grid_path     = _astar_path_finder->getPath(); //读出getPath的输出赋给grid_path
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //Visualize the result
    visGridPath (grid_path, false);
    visVisitedNode(visited_nodes);

    //Reset map for next call
    _astar_path_finder->resetUsedGrids();

    //_use_jps = 0 -> Do not use JPS
    //_use_jps = 1 -> Use JPS
    //you just need to change the #define value of _use_jps
#define _use_jps 1
#if _use_jps
    {
        //Call JPS to search for a path
        _jps_path_finder -> JPSGraphSearch(start_pt, target_pt, _Heudirector);

        //Retrieve the path
        auto grid_path     = _jps_path_finder->getPath();
        auto visited_nodes = _jps_path_finder->getVisitedNodes();

        //Visualize the result
        visGridPath   (grid_path, _use_jps);
        visVisitedNode(visited_nodes);

        //Reset map for next call
        _jps_path_finder->resetUsedGrids();
    }
#endif
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");
    //订阅到地图信息的回调函数
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    //订阅到终点信息的回调函数
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    //声明要发布的topic
    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);

    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2); //0.2
    
    nh.param("map/x_size",        _x_size, 50.0); //_x_size = 10
    nh.param("map/y_size",        _y_size, 50.0); //_y_size = 10
    nh.param("map/z_size",        _z_size, 5.0 ); //_z_size = 5
    
    nh.param("planning/start_x",  _start_pt(0),  0.0); // {0,0,1}
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0; // {-5,-5,0}
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;//{5,5,5}
    
    _inv_resolution = 1.0 / _resolution; //5
    
    _max_x_id = (int)(_x_size * _inv_resolution); //50
    _max_y_id = (int)(_y_size * _inv_resolution); //50
    _max_z_id = (int)(_z_size * _inv_resolution); //25
    
    //定义了结构体 AstarPathFinder 变量_astar_path_finder,该结构体存储、实现了 Astar 路径规划
    //所需的所有信息和功能
    _astar_path_finder  = new AstarPathFinder();
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    //定义了结构体 JPSPathFinder 变量_jps_path_finder,该结构体存储、实现了 JPS 路径规划所需
    //的所有信息和功能
    _jps_path_finder    = new JPSPathFinder();
    _jps_path_finder    -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _astar_path_finder;
    delete _jps_path_finder;
    return 0;
}

void visGridPath( vector<Vector3d> nodes, bool is_use_jps )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    if(is_use_jps)
        node_vis.ns = "demo_node/jps_path";
    else
        node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    if(is_use_jps){
        node_vis.color.a = 1.0;
        node_vis.color.r = 1.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 0.0;
    }
    else{
        node_vis.color.a = 1.0;
        node_vis.color.r = 0.0;
        node_vis.color.g = 1.0;
        node_vis.color.b = 0.0;
    }


    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}