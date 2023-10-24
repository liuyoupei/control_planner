#ifndef COSTMAP_2D_INFLATER_H_
#define COSTMAP_2D_INFLATER_H_

#include "thrid_party/glog/logging.h"

#include <map>
#include <queue>

#include "cell_data.h"
#include "cost_values.h"
#include "costmap_2d.h"
DECLARE_string(config_path);
namespace costmap_2d {
struct InflationLayerOptions {
  bool   inflate_unknown  = false;
  double weight           = 15.0;
};

class Inflater {
public:
  Inflater();
  ~Inflater();

  void inflate(std::shared_ptr<costmap_2d::Costmap2D> costmap);

protected:
  double resolution_;
  double inscribed_radius_;

private:
  unsigned int seen_size_;
  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;

  double inflation_radius_;

  bool*                                   seen_;
  std::map<double, std::vector<CellData>> inflation_cells_;

  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double DistanceLookup(int mx, int my, int src_x, int src_y) {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }
  inline unsigned char CostLookup(int mx, int my, int src_x, int src_y) {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_costs_[dx][dy];
  }
  inline void Enqueue(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y);
  virtual inline unsigned char ComputeCost(double distance) const {
    unsigned char cost = 0;
    if (distance == 0)
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    else {
      double euclidean_distance = distance * resolution_;
      double factor             = exp(-1.0 * options_.weight * (euclidean_distance - inscribed_radius_));
      cost                      = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }
  void                  ComputeCaches();
  void                  DeleteKernels();
  double**              cached_distances_;
  unsigned char**       cached_costs_;
  InflationLayerOptions options_;
};

}  // namespace costmap_2d

#endif  // INFLATER_H_
