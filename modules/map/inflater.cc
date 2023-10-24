#include "inflater.h"
#include <yaml-cpp/yaml.h>
namespace costmap_2d {

Inflater::Inflater()
  : seen_(NULL), seen_size_(0), cached_distances_(NULL), inflation_radius_(0.0), cell_inflation_radius_(0),
    cached_cell_inflation_radius_(0) {
  YAML::Node config        = YAML::LoadFile(FLAGS_config_path);
  resolution_              = config["costmap_resolution"].as<float>();
  inscribed_radius_        = config["costmap_inscribed_radius"].as<float>();
  inflation_radius_        = config["costmap_inflation_radius"].as<float>();
  options_.weight          = config["costmap_weight"].as<int>();
  options_.inflate_unknown = config["costmap_inflate_unknown"].as<bool>();
  cell_inflation_radius_   = (unsigned int)(inflation_radius_ / resolution_);
}

Inflater::~Inflater() {
  DeleteKernels();
  if (seen_ != NULL) {
    delete[] seen_;
    seen_ = NULL;
  }
}

void Inflater::inflate(std::shared_ptr<costmap_2d::Costmap2D> costmap) {
  unsigned int size_x = costmap->GetSizeInCellsX(), size_y = costmap->GetSizeInCellsY();
  if (seen_ == NULL) {
    seen_size_ = size_x * size_y;
    seen_      = new bool[seen_size_];
  } else if (seen_size_ != size_x * size_y) {
    delete[] seen_;
    seen_      = NULL;
    seen_size_ = size_x * size_y;
    seen_      = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));

  ComputeCaches();

  std::vector<CellData>& obs_bin = inflation_cells_[0.0];
  for (int j = 0; j < size_y; j++) {
    for (int i = 0; i < size_x; i++) {
      int           index = costmap->GetIndex(i, j);
      unsigned char cost  = costmap->GetCost(index);
      if (cost == costmap_2d::LETHAL_OBSTACLE) {
        obs_bin.push_back(CellData(index, i, j, i, j));
      }
    }
  }

  std::map<double, std::vector<CellData>>::iterator bin;
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin) {
    for (int i = 0; i < bin->second.size(); ++i) {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i];

      unsigned int index = cell.index_;

      // ignore if already visited
      if (seen_[index]) {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char old_cost = costmap->GetCost(mx, my);
      //      costmap->SetCost(mx, my, new_cost);
      unsigned char cost = CostLookup(mx, my, sx, sy);
      if (old_cost == NO_INFORMATION
          && (options_.inflate_unknown ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        costmap->SetCost(mx, my, cost);
      else
        costmap->SetCost(mx, my, std::max(old_cost, cost));

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0) {
        Enqueue(index - 1, mx - 1, my, sx, sy);
      }
      if (my > 0) {
        Enqueue(index - size_x, mx, my - 1, sx, sy);
      }
      if (mx < size_x - 1) {
        Enqueue(index + 1, mx + 1, my, sx, sy);
      }
      if (my < size_y - 1) {
        Enqueue(index + size_x, mx, my + 1, sx, sy);
      }
    }
  }

  std::map<double, std::vector<CellData>>().swap(inflation_cells_);
}

inline void Inflater::Enqueue(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x,
                              unsigned int src_y) {
  if (!seen_[index]) {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = DistanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_)
      return;

    // push the cell data onto the inflation list and mark
    unsigned int dis = static_cast<int>(distance * 1000);
    inflation_cells_[dis].push_back(CellData(index, mx, my, src_x, src_y));
  }
}

void Inflater::ComputeCaches() {
  if (cell_inflation_radius_ == 0)
    return;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    DeleteKernels();

    cached_costs_     = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
      cached_costs_[i]     = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
        cached_distances_[i][j] = hypot(i, j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i) {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j) {
      cached_costs_[i][j] = ComputeCost(cached_distances_[i][j]);
    }
  }
}

void Inflater::DeleteKernels() {
  if (cached_distances_ != NULL) {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i) {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }
}

}  // namespace costmap_2d
