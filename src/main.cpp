#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <math.h>
#include <chrono>
#include "oamap_core.h"
#include "oa_planner.h"

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
/*
#include "visgraph.h"
#include "astar.h"*/

using namespace global_planner;
int main( int argc, char** argv )
{
	ros::init(argc, argv, "grid_map_nav");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::Publisher  pub = n.advertise<grid_map_msgs::GridMap>("/grid_map_nav/grid_map", 1, true);
	
	std::vector<std::vector<OaMapPoint>> polygons;
	
	std::vector<OaMapPoint> polygon1;
	polygon1.push_back(OaMapPoint(5,0,0,0));
	polygon1.push_back(OaMapPoint(15,6,0,0));
	polygon1.push_back(OaMapPoint(15,10,0,0));
	polygon1.push_back(OaMapPoint(18,13,0,0));
	polygon1.push_back(OaMapPoint(13,14,0,0));
	polygon1.push_back(OaMapPoint(8,16,0,0));
	polygon1.push_back(OaMapPoint(5,7,0,0));
	polygons.push_back(polygon1);
	
	std::vector<OaMapPoint> polygon2;
	polygon2.push_back(OaMapPoint(22,16,0,1));
	polygon2.push_back(OaMapPoint(26,11,0,1));
	polygon2.push_back(OaMapPoint(32,12,0,1));
	polygon2.push_back(OaMapPoint(30,15,0,1));
	polygon2.push_back(OaMapPoint(27,14,0,1));
	polygon2.push_back(OaMapPoint(27,19,0,1));
	polygon2.push_back(OaMapPoint(23,19.0,1));
	polygons.push_back(polygon2);
	
	std::vector<OaMapPoint> polygon3;
	polygon3.push_back(OaMapPoint(19,7,0,2));
	polygon3.push_back(OaMapPoint(25,7,0,2));
	polygon3.push_back(OaMapPoint(22,12,0,2));
	polygons.push_back(polygon3);
// 	std::vector<OaMapPoint> polygon4;
// 	polygon4.push_back(OaMapPoint(-5,-5,0,-1));
// 	polygon4.push_back(OaMapPoint(50,-5,0,-1));
// 	polygon4.push_back(OaMapPoint(50,40,0,-1));
// 	polygon4.push_back(OaMapPoint(30,40,0,-1));
// 	polygon4.push_back(OaMapPoint(30,20,0,-1));
// 	polygon4.push_back(OaMapPoint(15,20,0,-1));
// 	polygon4.push_back(OaMapPoint(15,40,0,-1));
// 	polygon4.push_back(OaMapPoint(-5,40,0,-1));
// 	polygons.push_back(polygon4);

	//要先进行起始点与末端点位置判断
	OaMapPoint start(0,0);
	OaMapPoint goal(35,15);
	std::shared_ptr<GlobalMap> global_map_ = std::make_shared<GlobalMap>(700,700,1);
	std::shared_ptr<BaseGlobalPlan> oa_planner = std::make_shared<OaPlanner>();
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	global_map_->BuildWithPolygons(polygons,2);
	oa_planner->Initialize(global_map_);
	std::shared_ptr<uint8_t> map_char = global_map_->GetCharMap();
	std::vector<OaMapPoint> plan;
	oa_planner->MakePlan(start,goal,plan);
	
	std::cout << "The size of the plan is: " << plan.size() << std::endl;
	
// 	global_map_.Inflation(2);

	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
	std::cout << "solve time cost = "<<time_used.count()<<" seconds. "<<std::endl;
	std::cout << "The path is: " << std::endl;
	for(int i=0; i<plan.size(); i++)
	{
		std::cout << plan[i].x() << ',' << plan[i].y() << std::endl;
	}
	ros::Rate r(30);
	ros::Time time;
	float f = 0.0;
	while(ros::ok())
	{
		time = ros::Time::now();
		visualization_msgs::Marker line_list, line_string;
		line_string.header.frame_id = line_list.header.frame_id = "map";
		line_string.header.stamp = line_list.header.stamp = ros::Time::now();
		line_string.ns = line_list.ns = "grid_map_nav";
		line_string.action = line_list.action = visualization_msgs::Marker::ADD;
		line_string.pose.orientation.w = line_list.pose.orientation.w = 1.0;

		line_list.id = 1;
		line_string.id = 2;

		line_list.type    = visualization_msgs::Marker::LINE_LIST;
		line_string.type  = visualization_msgs::Marker::LINE_STRIP;

		line_list.scale.x  = 0.1;
		line_string.scale.x = 0.1;
		
		line_list.color.r = 1.0;
		line_list.color.a = 1.0;
		
		line_string.color.g = 1.0;
		line_string.color.a = 1.0;
		
		for(auto& _polygon : polygons)
		{
			for(int i=1; i<_polygon.size(); i++)
			{
				geometry_msgs::Point p;
				p.x = _polygon[i-1].x();
				p.y = _polygon[i-1].y();
				p.z = 0;
				line_list.points.push_back(p);
				p.x = _polygon[i].x();
				p.y = _polygon[i].y();
				p.z = 0;
				line_list.points.push_back(p);
			}
			geometry_msgs::Point p;
			p.x = _polygon[_polygon.size()-1].x();
			p.y = _polygon[_polygon.size()-1].y();
			p.z = 0;
			line_list.points.push_back(p);
			p.x = _polygon[0].x();
			p.y = _polygon[0].y();
			p.z = 0;
			line_list.points.push_back(p);
		}

		for(int i=0; i<plan.size(); i++)
		{
			geometry_msgs::Point point;
			point.x = plan[i].x();
			point.y = plan[i].y();
			point.z = 0;

			line_string.points.push_back(point);
		}
		
		marker_pub.publish(line_list);
		marker_pub.publish(line_string);
		grid_map_msgs::GridMap message;
		grid_map::Position goal_point(goal.x(),goal.y());
		grid_map::GridMap mmap = global_map_->GetGridMap();
		grid_map::Index goal_index;  
		mmap.getIndex(goal_point, goal_index);
		mmap.getPosition(goal_index, goal_point);
		mmap.getIndex(goal_point, goal_index);
		mmap.getPosition(goal_index, goal_point);
		grid_map::GridMapRosConverter::toMessage(mmap, message);
		pub.publish(message);
		r.sleep();
	}
}