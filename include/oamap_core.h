#ifndef __OAMAP_CORE_H__
#define __OAMAP_CORE_H__

#include <vector>
#include <memory>
#include <Eigen/Core>

#include "grid_map_core/grid_map_core.hpp"

#define POT_HIGH 1.0e10        // unassigned cell potential

namespace global_planner 
{
static const unsigned char NO_INFORMATION = 255;
static const unsigned char LETHAL_OBSTACLE = 254;
static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static const unsigned char FREE_SPACE = 0;
static const unsigned char INFLATION_DAMPING = 20;
static const unsigned char INTERSECT_THRESHOLD = 200;

struct InflaPoint
{
public:
	InflaPoint(grid_map::Index _p, float _s=0):
		p_(_p),
		s_(_s)
	{}
	//用于存放坐标
	grid_map::Index p_;
	//用于存放当前的膨胀距离
	float      s_;
};

inline bool operator==(const InflaPoint& _linfla, const InflaPoint& _rinfla)
{
	return ((_linfla.p_[0] == _rinfla.p_[0])&&(_linfla.p_[1] == _rinfla.p_[1]));
}

class OaMapPoint
{
public:
	OaMapPoint(float _x,float _y,float _z=0,int _polygon_id = 0):
		vec_(_x,_y,_z),
		polygon_id_(_polygon_id)
	{}
	virtual ~OaMapPoint(){}
	float& x() {return vec_[0];}
	float& y() {return vec_[1];}
	float& z() {return vec_[2];}
	
	Eigen::Vector3f vec_;
	int polygon_id_;
};

class OaMapCore
{
public:
	OaMapCore(uint32_t _xs, uint32_t _ys, uint32_t _zs = 1):xs_(_xs),ys_(_ys),zs_(_zs),ns_(_xs*_ys*_zs)
	{}
	virtual ~OaMapCore(){}
	virtual bool BuildWithPolygons(std::vector<std::vector<OaMapPoint>>& _polygons, float size) = 0;
	virtual bool Inflation(float size) = 0;
	virtual std::shared_ptr<uint8_t> GetCharMap(void) = 0;
	virtual uint32_t GetSizeInCellsX(void)
	{
		return xs_;
	}
	virtual uint32_t GetSizeInCellsY(void)
	{
		return ys_;
	}
	virtual uint32_t GetSizeInCellsZ(void)
	{
		return zs_;
	}
	
	virtual bool WorldToMap(const double wx, const double wy, double& mx, double& my) = 0;
	virtual bool MapToWorld(const double mx, const double my, double& wx, double& wy) = 0;
	virtual bool Intersect(OaMapPoint& start, OaMapPoint& end) = 0;
protected:
	const uint32_t xs_;
	const uint32_t ys_;
	const uint32_t zs_;
	const uint32_t ns_;
};

class GlobalMap : public OaMapCore
{
public:
	GlobalMap(uint32_t _xs, uint32_t _ys, float _resolution):
		OaMapCore(_xs,_ys),
		initialize_(false),
		map_name_("global_map"),
		map_resolution_(_resolution),		
		map_({map_name_})
	{
		map_.setFrameId("map");
		map_.setGeometry(grid_map::Length(xs_,ys_),map_resolution_);
		char_map_ = std::shared_ptr<uint8_t>(new uint8_t[ys_*xs_], [](uint8_t* p){delete[] p;});
		map_["global_map"].setZero();
		memset(char_map_.get(), 0, sizeof(uint8_t)*xs_*ys_);
	}
	virtual ~GlobalMap()
	{}
	virtual bool BuildWithPolygons(std::vector<std::vector<OaMapPoint>>& _polygons, float size);
	virtual bool Inflation(float size){};
	virtual std::shared_ptr<uint8_t> GetCharMap(void);
	
	virtual bool WorldToMap(const double wx, const double wy, double& mx, double& my);
	virtual bool MapToWorld(const double mx, const double my, double& wx, double& wy);
	
	virtual uint32_t GetSizeInCellsX(void)
	{
		return map_.getSize()[0];
	}
	virtual uint32_t GetSizeInCellsY(void)
	{
		return map_.getSize()[1];
	}
	
	//only for debug
	grid_map::GridMap GetGridMap(void)
	{
		return map_;
	}
	
	virtual bool Intersect(OaMapPoint& start, OaMapPoint& end);
protected:
// 	std::vector<std::vector<OaMapPoint>> polygons_;
	bool initialize_;
	std::string       map_name_;
	float 			  map_resolution_;
	grid_map::GridMap map_;
	//用于存放map
	std::shared_ptr<uint8_t> char_map_;


};
}
#endif