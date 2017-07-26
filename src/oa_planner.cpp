#include "oa_planner.h"
#include <iostream>

namespace global_planner 
{
void OaPlanner::Initialize(std::shared_ptr< OaMapCore > _oa_map)
{
	if(!initialize_)
	{
		oa_map_ = _oa_map;
		unsigned int cx = oa_map_->GetSizeInCellsX(), cy = oa_map_->GetSizeInCellsY();
		
		p_calc_ = std::make_shared<QuadraticCalculator>(cx, cy);
		planner_ = std::make_shared<AStarExpansion>(p_calc_.get(), cx, cy);
		grid_path_ = std::make_shared<GridPath>(p_calc_.get());
		initialize_ = true;
	}
	else
	{
		std::cout << "The oa planner has been initialized!" << std::endl;
	}
}

bool OaPlanner::MakePlan(OaMapPoint& start, OaMapPoint& goal, std::vector<OaMapPoint>& plan)
{
	double start_x = 0;
	double start_y = 0;
	double goal_x  = 0;
	double goal_y  = 0;
	
	if(!oa_map_->WorldToMap(start.x(),start.y(),start_x,start_y)
	|| !oa_map_->WorldToMap(goal.x(), goal.y(), goal_x, goal_y))
	{
		std::cout << "The start or the goal is out of the map!" << std::endl;
		return false;
	}
	
	int nx = oa_map_->GetSizeInCellsX();
	int ny = oa_map_->GetSizeInCellsY();
	
	p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    grid_path_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];
	std::shared_ptr<uint8_t> char_map = oa_map_->GetCharMap();
	
	OutlineMap(char_map.get(), nx, ny, LETHAL_OBSTACLE);
	
	bool found_legal = planner_->calculatePotentials(char_map.get(), start_x, start_y, goal_x, goal_y,
                                                     nx * ny * 2, potential_array_);
	if(!found_legal)
	{
		std::cout << "A star plan failed!" << std::endl;
		return false;
	}
	
	plan.clear();
    std::vector<std::pair<float, float> > path;
	
	if (!grid_path_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path))
	{
        std::cout << "There is no path!" << std::endl;
        return false;
    }
	
	//将path转换为plan
	for(auto index = path.rbegin(); index != path.rend(); index++)
	{
		std::pair<float, float> point = *index;
		double path_x, path_y;
		oa_map_->MapToWorld(point.first, point.second, path_x, path_y);
		OaMapPoint plan_point(path_x, path_y);
		plan.push_back(plan_point);
	}
	PrunePlan(plan);
	delete[] potential_array_;
	return true;
}

bool OaPlanner::OutlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) 
{
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

bool OaPlanner::PrunePlan(std::vector<OaMapPoint>& _plan)
{
	if(_plan.empty())
	{
		std::cout << "There is no path to prune!" << std::endl;
		return false;
	}
	std::size_t plan_size = _plan.size();
	if(plan_size == 2 || plan_size == 1)
	{
		return true;
	}
	std::vector<OaMapPoint> _pplan;
	OaMapPoint start = _plan.at(0);
	_pplan.push_back(start);
	int prune_index = prune_size_ - 1;
	
	bool shrink_flag = false;
	
	//大步前进，小步后退
	while(1)
	{
		if(!shrink_flag)
		{
			if(prune_index >= plan_size-1)
			{
				if(!oa_map_->Intersect(start, _plan.at(plan_size-1)))
				{
					_pplan.push_back(_plan.at(plan_size-1));
					break;
				}
				else
				{
					shrink_flag = true;
					prune_index = plan_size - 2;
				}
			}
			else
			{
				if(!oa_map_->Intersect(start, _plan.at(prune_index)))
				{
					prune_index += prune_size_;
					shrink_flag =  false;
				}
				else
				{
					prune_index -= 1;
					shrink_flag =  true;
				}
			}
		}
		else
		{
			if(!oa_map_->Intersect(start, _plan.at(prune_index)))
			{
				_pplan.push_back(_plan.at(prune_index));
				start = _plan.at(prune_index);
				prune_index += prune_size_;
				shrink_flag =  false;
			}
			else
			{
				prune_index -= 1;
				shrink_flag =  true;
			}
		}
	}

	_plan.clear();
	_plan = _pplan;
	return false;
}

}
