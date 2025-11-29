#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <memory>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector2i index;

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	std::shared_ptr<GridMap2D> grid_map_;


	inline void coord2gridIndexFast(const double x, const double y, int &id_x, int &id_y);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector2d start_pt, const Eigen::Vector2d end_pt, Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx);

	inline Eigen::Vector2d Index2Coord(const Eigen::Vector2i &index) const;
	inline bool Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline bool checkOccupancy(const Eigen::Vector2d &pos) 
	{ 
		// std::cout << "checkOccupancy11" << std::endl;
		if (grid_map_ == nullptr) {  // 检查grid_map_是否为空
			std::cerr << "Error: grid_map_ is null!" << std::endl;
			return true;  // 或抛出异常
   		 }
		// std::cout << "checkOccupancy22" << std::endl;
		return (bool)grid_map_->getInflateOccupancy(pos);
	 }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector2d center_;
	Eigen::Vector2i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr **GridNodeMap_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(std::shared_ptr<GridMap2D> occ_map, const Eigen::Vector2i pool_size);

	bool AstarSearch(const double step_size, Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);

	std::vector<Eigen::Vector2d> getPath();
	std::vector<Eigen::Vector3d> get3DPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector2d AStar::Index2Coord(const Eigen::Vector2i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const
{
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector2d(0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1))
	{
		printf("Ran out of pool, index=%d %d \n", idx(0), idx(1));
		return false;
	}
	return true;
};

#endif
