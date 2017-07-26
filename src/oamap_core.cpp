#include "oamap_core.h"
#include <iostream>
#include <deque>

namespace global_planner
{
bool GlobalMap::BuildWithPolygons(std::vector<std::vector<OaMapPoint>>& _polygons, float size)
{
	if (_polygons.empty())
	{
		std::cout << "There is no map information!" << std::endl;
		return false;
	}
	std::deque<InflaPoint> inflation_vec;
	inflation_vec.clear();
	auto _char_map = char_map_.get();
	for(auto& _polygon : _polygons)
	{
		if(_polygon[0].polygon_id_ != -1)
		{
			grid_map::Position start_p(0,0);
			grid_map::Position end_p(0,0);
			for(std::size_t index = 1; index < _polygon.size(); ++index)
			{
				start_p = grid_map::Position(_polygon[index-1].x(), _polygon[index-1].y());
				end_p   = grid_map::Position(_polygon[index].x(), _polygon[index].y());
				for (grid_map::LineIterator iterator(map_, start_p, end_p);
					!iterator.isPastEnd(); ++iterator) 
				{
					map_.at(map_name_,*iterator) = LETHAL_OBSTACLE;
					_char_map[(*iterator)[0]+xs_*((*iterator)[1])] = LETHAL_OBSTACLE;
					InflaPoint infla_point(*iterator,0);
					inflation_vec.push_back(infla_point);
				}
			}
			start_p = grid_map::Position(_polygon[_polygon.size()-1].x(), _polygon[_polygon.size()-1].y());
			end_p   = grid_map::Position(_polygon[0].x(), _polygon[0].y());
			for (grid_map::LineIterator iterator(map_, start_p, end_p);
				!iterator.isPastEnd(); ++iterator) 
			{
				map_.at(map_name_,*iterator) = LETHAL_OBSTACLE;
				_char_map[(*iterator)[0]+xs_*((*iterator)[1])] = LETHAL_OBSTACLE;
				InflaPoint infla_point(*iterator,0);
				inflation_vec.push_back(infla_point);
			}
		}
	}
	if((LETHAL_OBSTACLE-INFLATION_DAMPING) <= INFLATION_DAMPING*size || size < 0.0f)
	{
		std::cout << "The inflation size is illegal!" << std::endl; 
		return false;
	}
// 	if(size == 0.0f)
// 	{
// 		return true;
// 	}
	//广度优先算法(用pop_back就为深度优先，用pop_front就为广度优先)
	while(!inflation_vec.empty())
	{
		InflaPoint infla_point = inflation_vec.front();
		inflation_vec.pop_front();
		float& value1 = map_.at(map_name_,infla_point.p_);

		for(int i=-1; i<=1; i++)
		{
			for(int j=-1; j<=1; j++)
			{
				if(i!=0 || j!=0)
				{
					grid_map::Index infla_index2;
					infla_index2[0] = infla_point.p_[0]+i;
					infla_index2[1] = infla_point.p_[1]+j;

					float& value2 = map_.at(map_name_,infla_index2);
					float  _value = value1 - INFLATION_DAMPING*map_resolution_;
					InflaPoint infla_point2(infla_index2, 0);

					if((value2 <= _value) && (_value >= LETHAL_OBSTACLE - INFLATION_DAMPING*size))
					{
						value2 = _value;
						_char_map[infla_index2[0]+xs_*infla_index2[1]] = _value;
						inflation_vec.push_back(infla_point2);
					}
				}
			}
		}
			
	}
	initialize_ = true;
	return true;
}

//膨胀
// bool GlobalMap::Inflation(float size)
// {
// 	std::deque<InflaPoint> inflation_vec;
// 	inflation_vec.clear();
// 	if(200 <= 20*size*map_resolution_ || size < 0.0f)
// 	{
// 		std::cout << "The inflation size is illegal!" << std::endl; 
// 		return false;
// 	}
// 	
// 	//把障碍物边界点推入栈中
// 	for(auto& _polygon : polygons_)
// 	{
// 		if(_polygon[0].polygon_id_ != -1)
// 		{
// 			grid_map::Position start_p(0,0);
// 			grid_map::Position end_p(0,0);
// 			for(std::size_t index = 1; index < _polygon.size(); ++index)
// 			{
// 				start_p = grid_map::Position(_polygon[index-1].x(), _polygon[index-1].y());
// 				end_p   = grid_map::Position(_polygon[index].x(), _polygon[index].y());
// 				for (grid_map::LineIterator iterator(map_, start_p, end_p);
// 					!iterator.isPastEnd(); ++iterator) 
// 				{
// 					InflaPoint infla_point(*iterator,0);
// 					inflation_vec.push_back(infla_point);
// 				}
// 			}
// 			start_p = grid_map::Position(_polygon[_polygon.size()-1].x(), _polygon[_polygon.size()-1].y());
// 			end_p   = grid_map::Position(_polygon[0].x(), _polygon[0].y());
// 			for (grid_map::LineIterator iterator(map_, start_p, end_p);
// 				!iterator.isPastEnd(); ++iterator) 
// 			{
// 				InflaPoint infla_point(*iterator,0);
// 				inflation_vec.push_back(infla_point);
// 			}
// 		}
// 	}
// 	
// // 	//广度优先算法(用pop_back就为深度优先，用pop_front就为广度优先)
// 	while(!inflation_vec.empty())
// 	{
// 		InflaPoint infla_point = inflation_vec.front();
// 		inflation_vec.pop_front();
// 		float& value1 = map_.at(map_name_,infla_point.p_);
// 
// 		for(int i=-1; i<=1; i++)
// 		{
// 			for(int j=-1; j<=1; j++)
// 			{
// 				if(i!=0 || j!=0)
// 				{
// 					grid_map::Index infla_index2;
// 					infla_index2[0] = infla_point.p_[0]+i;
// 					infla_index2[1] = infla_point.p_[1]+j;
// 
// 					float& value2 = map_.at(map_name_,infla_index2);
// 					float  _value = value1 - 20*map_resolution_;
// 					InflaPoint infla_point2(infla_index2, 0);
// 
// 					if(value2 <= _value)
// 					{
// 						value2 = _value;
// 						if(value2 > 255 - 20*size*map_resolution_)
// 						{
// 							inflation_vec.push_back(infla_point2);
// 						}
// 					}
// 				}
// 			}
// 		}
// 			
// 	}
// }
/*
bool GlobalMap::InflationWithBound(float size)
{
	
}*/

//返回值是按列存储
std::shared_ptr<uint8_t> GlobalMap::GetCharMap(void)
{
	if(!initialize_)
	{
		std::cout << "The map has not been initialized!" << std::endl;
		return std::shared_ptr<uint8_t>(nullptr);
	}
	
// 	return char_map_.get();
// 	float *data = new float[map_.getSize()[0]*map_.getSize()[1]];
// 	memset((void*)data, 0, map_.getSize()[0]*map_.getSize()[1]*sizeof(float));
// 	
// 	//将Eigen的数据输出到数组，
// 	Eigen::Map<Eigen::MatrixXf>(data, map_.getSize()[0],map_.getSize()[1]) = map_[map_name_];
// 	std::shared_ptr<uint8_t> data_char = std::shared_ptr<uint8_t>(new uint8_t[map_.getSize()[0]*map_.getSize()[1]], 
// 															[](uint8_t* p){delete[] p;});
// 	std::copy(data, &data[map_.getSize()[0]*map_.getSize()[1]], data_char.get());
// 	//释放data
// 	delete[] data;
	
	return char_map_;
}

bool GlobalMap::MapToWorld(const double mx, const double my, double& wx, double& wy)
{
	grid_map::Index m_index(mx, my);
	grid_map::Position w_position;
	if(!map_.getPosition(m_index, w_position))
	{
		return false;
	}
	wx = w_position[0];
	wy = w_position[1];
	return true;
}

bool GlobalMap::WorldToMap(const double wx, const double wy, double& mx, double& my)
{
	grid_map::Index m_index;
	grid_map::Position w_position(wx, wy);
	if(!map_.getIndex(w_position, m_index))
	{
		return false;
	}
	mx = m_index[0];
	my = m_index[1];
	return true;
}

bool GlobalMap::Intersect(OaMapPoint& start, OaMapPoint& end)
{
	grid_map::Position start_p(start.x(),start.y());
	grid_map::Position end_p(end.x(),end.y());

	for (grid_map::LineIterator iterator(map_, start_p, end_p);
		!iterator.isPastEnd(); ++iterator)
	{
		if(map_.at(map_name_,*iterator) >= INTERSECT_THRESHOLD)
		{
			return true;
		}
	}
	return false;
}
}
