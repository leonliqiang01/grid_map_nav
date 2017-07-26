#ifndef __OA_PLANNER_H__
#define __OA_PLANNER_H__

#include <memory>
#include "oamap_core.h"
#include "astar.h"
#include "grid_path.h"
#include "potential_calculator.h"
#include "quadratic_calculator.h"

namespace global_planner
{
class BaseGlobalPlan
{
public:
	BaseGlobalPlan(){}
	virtual ~BaseGlobalPlan(){}
	
	virtual bool MakePlan(OaMapPoint& start, OaMapPoint& goal, std::vector<OaMapPoint>& plan) = 0;
	virtual void Initialize(std::shared_ptr<OaMapCore> _oa_map) = 0;
};

class OaPlanner : public BaseGlobalPlan
{
public:
	OaPlanner():
		initialize_(false),
		prune_size_(10)
	{}
	virtual ~OaPlanner()
	{}
	virtual bool MakePlan(OaMapPoint& start, OaMapPoint& goal, std::vector<OaMapPoint>& plan);
	virtual void Initialize(std::shared_ptr<OaMapCore> _oa_map);
protected:
	bool OutlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
	bool PrunePlan(std::vector<OaMapPoint>& _plan);
	
	bool   initialize_;
	float* potential_array_;
	int    prune_size_;
	std::shared_ptr<OaMapCore>           oa_map_;
	std::shared_ptr<GridPath>            grid_path_;
	std::shared_ptr<Expander>            planner_;
	std::shared_ptr<PotentialCalculator> p_calc_;

};

}

#endif