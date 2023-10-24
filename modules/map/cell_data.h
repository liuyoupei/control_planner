#ifndef COSTMAP_2D_CELL_DATA_H_
#define COSTMAP_2D_CELL_DATA_H_

namespace costmap_2d {

/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData {
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy)
    : index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy) {}
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

}  // namespace costmap_2d

#endif