/*
 * @function：Grid Map for Ego Planner
 * @author:juchunyu@qq.com
 */
#ifndef GRID_MAP_2D_H
#define GRID_MAP_2D_H

#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <iostream>

// 二维栅格地图类（与A*算法配套）
class GridMap2D {
private:
    std::vector<std::vector<bool>> grid_;           // 栅格数据：true=障碍物，false=自由空间
    std::vector<std::vector<bool>> original_grid_;  // 原始障碍物网格（true=原始障碍物）
    double resolution_;                             // 栅格分辨率（米/格）
    Eigen::Vector2i map_size_;                      // 栅格尺寸（行×列，对应y×x方向）
    Eigen::Vector2d origin_;                        // 地图原点（世界坐标，栅格(0,0)的中心）
    double inflate_radius_;
    Eigen::Vector2i world_size_;
    int grid_rows_;
    int grid_cols_;
    


public:
    /**
     * @brief 构造函数
     * @param resolution 栅格分辨率（米/格）
     * @param map_size 栅格尺寸（行、列，Eigen::Vector2i(row, col)）
     * @param origin 原点世界坐标（Eigen::Vector2d(x, y)）
     */
    GridMap2D(double resolution, const Eigen::Vector2i& map_size);


    void setCurPose(double x,double y);

    // void resetMap()
    // {
    //     original_grid_.resize(grid_rows_, std::vector<bool>(grid_cols_, false));  // 存储原始障碍物
    //     grid_.resize(grid_rows_, std::vector<bool>(grid_cols_, false));          // 存储原始+膨胀障碍物
    // }
    void resetMap()
    {
        // 只有在地图已初始化的情况下才重置
        if (grid_rows_ > 0 && grid_cols_ > 0) {
            for (auto& row : original_grid_) {
                std::fill(row.begin(), row.end(), false);
            }
            for (auto& row : grid_) {
                std::fill(row.begin(), row.end(), false);
            }
            std::cout << "[resetMap] 所有栅格已重置为自由空间。" << std::endl;
        } else {
            std::cout << "[resetMap] 警告：地图尚未初始化，无法重置。" << std::endl;
        }
    }

    /**
     * @brief 设置栅格障碍物状态
     * @param index 栅格索引（Eigen::Vector2i(row, col)）
     * @param is_obstacle 是否为障碍物
     */
    void setObstacle(const Eigen::Vector2i& index, bool is_obstacle = true);

    void inflateObstacles(double radius);

    bool getInflateOccupancy(const Eigen::Vector2d& world_pos) const;

    void inflate();

    void setInflateRadius(double radius);

    double getResolution();

    void resetGrids();
    /**
     * @brief 检查栅格是否为障碍物（A*核心接口）
     * @param index 栅格索引
     * @return 是否为障碍物（超出地图范围视为障碍物）
     */
    bool isObstacle(const Eigen::Vector2d& pos) const;

    /**
     * @brief 世界坐标转栅格索引（A*核心接口）
     * @param coord 世界坐标（Eigen::Vector2d(x, y)）
     * @return 栅格索引（Eigen::Vector2i(col, row)，对应x→col，y→row）
     */
    Eigen::Vector2i worldToGrid(const Eigen::Vector2d& coord) const;

    /**
     * @brief 栅格索引转世界坐标（A*核心接口）
     * @param index 栅格索引
     * @return 栅格中心的世界坐标
     */
    Eigen::Vector2d gridToWorld(const Eigen::Vector2i& index) const;

    /**
     * @brief 检查索引是否在地图范围内
     */
    bool isIndexValid(const Eigen::Vector2i& index) const;


    /**
     * @brief 获取障碍物点云（世界坐标系）
     * @param return_inflated_map true: 返回膨胀后的障碍物; false: 仅返回原始障碍物
     * @return std::vector<Eigen::Vector2d> 包含所有障碍物中心点的世界坐标列表
     */
    std::vector<Eigen::Vector2d> getObstaclePointCloud(bool return_inflated_map = true) const;

    // Getter
    double resolution() const;
    const Eigen::Vector2i& mapSize() const;
    const Eigen::Vector2d& origin() const;
};

using GridMap2DPtr = std::shared_ptr<GridMap2D>;  // 智能指针类型（与A*的grid_map_匹配）

#endif  // GRID_MAP_2D_H
