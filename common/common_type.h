#ifndef CLEAN_ROBOT_COMMON_TYPE_H
#define CLEAN_ROBOT_COMMON_TYPE_H
#include "enum_type.h"
#include <iostream>
#include <math.h>
#include <vector>
#define LOG_INFO LOG(INFO) << __FILE__ << ":" << __func__ << "():" << __LINE__ << ":"
namespace common {
typedef struct Point {
  Point() {
    x_ = 0;
    y_ = 0;
  }
  Point(const Point& c) {
    x_ = c.x();
    y_ = c.y();
  }
  Point(int x, int y) {
    this->x_ = x;
    this->y_ = y;
  }
  friend std::ostream& operator<<(std::ostream& os, const Point& c) {
    os << "x = " << c.x() << " y = " << c.x();
    return os;
  }
  bool operator==(const Point& point) const {
    if (this->x_ == point.x_ && this->y_ == point.y_) {
      return true;
    } else {
      return false;
    }
  }

  bool operator!=(const Point& point) const {
    if (this->x_ != point.x_ || this->y_ != point.y_) {
      return true;
    } else {
      return false;
    }
  }

  inline int& x() /*!< Get ref to X coord. */ {
    return x_;
  }

  inline int& y() /*!< Get ref to Y coord. */ {
    return y_;
  }

  int x() const {
    return x_;
  }

  int y() const {
    return y_;
  }

  bool operator<(const Point& mapkey) const {
    if (mapkey.y_ > y_) {
      return true;
    } else if (mapkey.y_ == y_) {
      return (mapkey.x_ > x_);
    }
    return false;
  }

  Point operator-(const Point& point) const {
    return Point(x_ - point.x_, y_ - point.y_);
  }

  int x_;
  int y_;
} Point;
struct Twist {
  Twist() {
    linear_x  = 0;
    angular_z = 0;
  }
  void reset(const Twist& twist) {
    this->linear_x  = twist.linear_x;
    this->angular_z = twist.angular_z;
  }
  double linear_x;
  double angular_z;
};
typedef struct Pose {

  Pose() {
    x_   = 0;
    y_   = 0;
    z_   = 0;
    phi_ = 0;
  }

  Pose(const Pose& p) {
    x_   = p.x();
    y_   = p.y();
    z_   = p.z();
    phi_ = p.phi();
  }

  void reset() {
    x_   = 0;
    y_   = 0;
    z_   = 0;
    phi_ = 0;
  }

  Pose(double x, double y) {
    this->x_ = x;
    this->y_ = y;
  }

  Pose(double x, double y, double z) {
    this->x_ = x;
    this->y_ = y;
    this->z_ = z;
  }
  friend std::ostream& operator<<(std::ostream& os, const Pose& c) {
    os << "x = " << c.x() << " y = " << c.x() << " phi = " << c.phi();
    return os;
  }
  bool operator==(const Pose& pose) const {
    if (fabs(this->x_ - pose.x_) <= 0.1 && fabs(this->y_ - pose.y_) <= 0.1) {
      return true;
    } else {
      return false;
    }
  }

  bool operator!=(const Pose& pose) const {
    if (fabs(this->x_ - pose.x_) >= 0.1 || fabs(this->y_ - pose.y_) >= 0.1) {
      return true;
    } else {
      return false;
    }
  }

  inline double& x() /*!< Get ref to X coord. */ {
    return x_;
  }

  inline double& y() /*!< Get ref to Y coord. */ {
    return y_;
  }

  inline double& phi() /*!< Get ref to Y coord. */ {
    return phi_;
  }

  inline double& z() /*!< Get ref to Y coord. */ {
    return phi_;
  }

  double x() const {
    return x_;
  }

  double y() const {
    return y_;
  }

  double z() const {
    return z_;
  }

  double phi() const {
    return phi_;
  }

  double distanceTo(const Pose& pose) const {
    double dx       = pose.x_ - this->x_;
    double dy       = pose.y_ - this->y_;
    double distance = fabs(std::sqrt(dx * dx + dy * dy));
    return distance;
  }
  float fpwrappi(float angle) const {
    while (angle > M_PI)
      angle -= (2 * M_PI);
    while (angle <= -M_PI)
      angle += (2 * M_PI);
    return angle;
  }
  double distancePhiTo(const Pose& pose) const {
    double distance = fabs(fpwrappi(pose.phi() - this->phi()));
    return distance;
  }

  bool operator<(const Point& mapkey) const {
    Point point;
    if (mapkey.y_ > y_) {
      return true;
    } else if (mapkey.y_ == y_) {
      return (mapkey.x_ > x_);
    }
    return false;
  }

  Point operator-(const Point& point) const {
    return Point(x_ - point.x_, y_ - point.y_);
  }

  double x_;
  double y_;
  double z_;
  double phi_;
  Twist  twist;
} Pose;
struct Vector2D {
  float x;
  float y;

  Vector2D() {
    this->x = 0.0;
    this->y = 0.0;
  }

  Vector2D(float x, float y) {
    this->x = x;
    this->y = y;
  }

  Vector2D(const Vector2D& vec) {
    x = vec.x;
    y = vec.y;
  }

  Vector2D(double angle) {
    this->x = cos(angle);
    this->y = sin(angle);
  }

  Vector2D rotationVector(float angle) {
    Vector2D vector_2d;
    float    R  = getLength();
    vector_2d.x = x * cos(angle) - y * sin(angle);
    vector_2d.y = x * sin(angle) + y * cos(angle);
    return vector_2d;
  }

  Vector2D operator+(const Vector2D& vect) const {
    return Vector2D(x + vect.x, y + vect.y);
  }

  Vector2D operator-(const Vector2D& vect) const {
    return Vector2D(x - vect.x, y - vect.y);
  }

  Vector2D& operator=(const Vector2D& vec) {
    // return Vector2D(vec.x, vec.y);
    this->x = vec.x;
    this->y = vec.y;
    return *this;
  }

  friend float operator*(Vector2D a, Vector2D b) {
    float p = a.x * b.x + a.y * b.y;
    return p;
  };

  friend Vector2D operator*(Vector2D a, float value) {
    Vector2D p;
    p.x = a.x * value;
    p.y = a.y * value;
    return p;
  };

  float dot(const Vector2D& vect) const {
    return (x * vect.x + y * vect.y);
  }

  friend float xlJi(Vector2D a, Vector2D b) {
    float p = ((a.x) * (b.y)) - ((a.y) * (b.x));
    return p;
  };

  /**
   * 归一化,单位向量
   */
  void unitVector() {
    float length = getLength();
    x            = x / length;
    y            = y / length;
  }

  double RAD2DEG(const double x) const {
    return x * 180.0 / M_PI;
  }

  float getAngle(const Vector2D& vect) const {
    Vector2D vector1 = vect;
    Vector2D vector2(x, y);
    float    temp  = vector1.dot(vector2) / (vector1.getLength() * vector2.getLength());
    float    theta = acos(temp);
    if (fabs(temp - 1.0) < 1E-6) {
      theta = acos(1.0);
    }
    // float theta = acos(vector1.dot(vector2)/(vector1.getLength() * vector2.getLength()))/* * 180.0/M_PI*/ ;
    return RAD2DEG(theta);
  }

  /**
   * 向量的模
   * @return
   */
  float getLength() const {
    return sqrt(pow(x, 2) + pow(y, 2));
  }

  float crossedProduct(const Vector2D& vect) const {
    return x * vect.y - y * vect.x;
  }
};

template <typename T> struct Rect {
  Rect() {
    min_x = 1e6;
    min_y = 1e6;
    max_x = -1e6;
    max_y = -1e6;
  }
  void reset() {
    min_x = 1e6;
    min_y = 1e6;
    max_x = -1e6;
    max_y = -1e6;
  }
  void update(T x, T y) {
    if (min_x > x) {
      min_x = x;
    }
    if (max_x < x) {
      max_x = x;
    }
    if (min_y > y) {
      min_y = y;
    }
    if (max_y < y) {
      max_y = y;
    }
  }
  void update(const std::vector<common::Pose>& poses) {
    for (auto data : poses) {
      update(data.x(), data.y());
    }
    length = max_y - min_y;
    width  = max_x - min_x;
  }
  T getLength() {
    length = max_y - min_y;
    return length;
  }
  T getWidth() {
    width = max_x - min_x;
    return width;
  }
  void expand(T distance) {
    min_x -= distance;
    min_y -= distance;
    max_x += distance;
    max_y += distance;
    length = max_y - min_y;
    width  = max_x - min_x;
  }

  float getArea() {
    T length = getLength();
    T width  = getWidth();
    T area   = length * width;
    return area;
  }

  bool inRect(T x, T y) {
    if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
      return true;
    } else {
      return false;
    }
  }

  std::vector<Pose> getVertices() {
    std::vector<Pose> points;
    Pose              pose;
    pose.x() = min_x;
    pose.y() = min_y;
    points.push_back(pose);
    pose.x() = min_x;
    pose.y() = max_y;
    points.push_back(pose);
    pose.x() = max_x;
    pose.y() = max_y;
    points.push_back(pose);
    pose.x() = max_x;
    pose.y() = min_y;
    points.push_back(pose);
    return points;
  }

  std::vector<Pose> getAntiClockwiseVertices() {
    std::vector<Pose> points;
    Pose              pose;
    pose.x() = min_x;
    pose.y() = min_y;
    points.push_back(pose);
    pose.x() = max_x;
    pose.y() = min_y;
    points.push_back(pose);
    pose.x() = max_x;
    pose.y() = max_y;
    points.push_back(pose);
    pose.x() = min_x;
    pose.y() = max_y;
    points.push_back(pose);
    return points;
  }

  std::vector<Pose> getClockwiseVertices() {
    std::vector<Pose> points;
    Pose              pose;
    pose.x() = min_x;
    pose.y() = min_y;
    points.push_back(pose);
    pose.x() = min_x;
    pose.y() = max_y;
    points.push_back(pose);
    pose.x() = max_x;
    pose.y() = max_y;
    points.push_back(pose);
    pose.x() = max_x;
    pose.y() = min_y;
    points.push_back(pose);
    return points;
  }

  T min_x;
  T min_y;
  T max_x;
  T max_y;
  T length;
  T width;
};

struct LaserScan {
  float                     angle_min;
  float                     angle_max;
  float                     angle_increment;
  float                     time_increment;
  float                     scan_time;
  float                     range_min;
  float                     range_max;
  std::vector<float>        ranges;
  std::vector<float>        angles;
  std::vector<common::Pose> poses;
};
struct Header {
  std::string frame_id;
  int64_t     us;  // microsechod, 10e-6 seconds.
};
struct Quaternion {
  double w;
  double x;
  double y;
  double z;
};
struct Map_info {
public:
  float_t  resolution;
  uint32_t width;
  uint32_t height;
  Pose     origin;
};
struct Map {
  Header           header;
  Map_info         info;
  std::vector<int> data;
  int              getIndex(int x, int y) const {
    int index = y * info.width + x;
    return index;
  }
  int getData(int x, int y) const {
    int index = getIndex(x, y);
    int cost  = data[index];
    return cost;
  }
  int getData(int index) const {
    int cost = data[index];
    return cost;
  }
};

struct Bumper {
  Bumper() {
    right_bumper = false;
    left_bumper  = false;
  }
  bool right_bumper;
  bool left_bumper;
};

struct Edge {
  void reset() {
    start_pose.reset();
    end_pose.reset();
    edge_type = BlockEdgeType::NULL_EDGE;
  }
  bool operator!=(const Edge& edge) const {
    if (this->start_pose != edge.start_pose || this->end_pose != edge.end_pose) {
      return true;
    } else {
      return false;
    }
  }
  void update(float start_x, float start_y, float end_x, float end_y, BlockEdgeType edge_type) {
    start_pose.x()  = start_x;
    start_pose.y()  = start_y;
    end_pose.x()    = end_x;
    end_pose.y()    = end_y;
    this->edge_type = edge_type;
  }
  Pose getMiddlePose() {
    Pose temp_pose;
    temp_pose.x() = (start_pose.x() + end_pose.x()) / 2.0;
    temp_pose.y() = (start_pose.y() + end_pose.y()) / 2.0;
    return temp_pose;
  }
  Pose          start_pose;
  Pose          end_pose;
  Pose          middle_pose;
  BlockEdgeType edge_type;
  RoomType      room_type;
  Pose          foot_pose;
};

struct RoomData {
  std::vector<Edge>         edges;
  std::vector<common::Pose> active_segment;
  std::vector<common::Pose> outline;
  int                       times;
  Point                     block_coor;
  int                       id;
  std::vector<Pose>         getVertices() {
    std::vector<Pose> points;
    for (Edge e : edges) {
      points.emplace_back(e.start_pose);
    }
    return points;
  }
  void updateUpEdge() {
    edges[EdgeNum::UP_NUM].start_pose = edges[EdgeNum::RIGHT_NUM].end_pose;
    edges[EdgeNum::UP_NUM].end_pose   = edges[EdgeNum::LEFT_NUM].start_pose;
  }

  void updateDownEdge() {
    edges[EdgeNum::DOWN_NUM].start_pose = edges[EdgeNum::LEFT_NUM].end_pose;
    edges[EdgeNum::DOWN_NUM].end_pose   = edges[EdgeNum::RIGHT_NUM].start_pose;
  }

  void updateLeftEdge() {
    edges[EdgeNum::LEFT_NUM].start_pose = edges[EdgeNum::UP_NUM].end_pose;
    edges[EdgeNum::LEFT_NUM].end_pose   = edges[EdgeNum::DOWN_NUM].start_pose;
  }

  void updateRightEdge() {
    edges[EdgeNum::RIGHT_NUM].start_pose = edges[EdgeNum::DOWN_NUM].end_pose;
    edges[EdgeNum::RIGHT_NUM].end_pose   = edges[EdgeNum::UP_NUM].start_pose;
  }
  void knowDownLeftUpdateRightDiagonal() {
    edges[EdgeNum::RIGHT_NUM].start_pose   = edges[EdgeNum::DOWN_NUM].end_pose;
    edges[EdgeNum::RIGHT_NUM].end_pose.x() = edges[EdgeNum::LEFT_NUM].start_pose.x();
    edges[EdgeNum::RIGHT_NUM].end_pose.y() = edges[EdgeNum::DOWN_NUM].end_pose.y();
  }

  void knowUpLeftUpdateRightDiagonal() {
    edges[EdgeNum::RIGHT_NUM].start_pose.x() = edges[EdgeNum::LEFT_NUM].end_pose.x();
    edges[EdgeNum::RIGHT_NUM].start_pose.y() = edges[EdgeNum::UP_NUM].start_pose.y();
    edges[EdgeNum::RIGHT_NUM].end_pose       = edges[EdgeNum::UP_NUM].start_pose;
  }
  void knowRightDownUpdateUpDiagonal() {
    edges[EdgeNum::UP_NUM].start_pose   = edges[EdgeNum::RIGHT_NUM].end_pose;
    edges[EdgeNum::UP_NUM].end_pose.x() = edges[EdgeNum::RIGHT_NUM].end_pose.x();
    edges[EdgeNum::UP_NUM].end_pose.y() = edges[EdgeNum::DOWN_NUM].start_pose.y();
  }
  void knowUpRightUpdateLeftDiagonal() {
    edges[EdgeNum::LEFT_NUM].start_pose   = edges[EdgeNum::UP_NUM].end_pose;
    edges[EdgeNum::LEFT_NUM].end_pose.x() = edges[EdgeNum::RIGHT_NUM].start_pose.x();
    edges[EdgeNum::LEFT_NUM].end_pose.y() = edges[EdgeNum::UP_NUM].end_pose.y();
  }
};

struct GoBackData {
  GoBackData() {}
  void setDistanceData(common::ControlDir go_type, bool use_distance, float distance) {
    this->go_type      = go_type;
    this->use_distance = use_distance;
    this->distance     = distance;
  }
  void setTimeData(common::ControlDir go_type, bool use_distance, float time) {
    this->go_type      = go_type;
    this->use_distance = use_distance;
    this->distance     = distance;
  }
  common::ControlDir go_type;
  bool               use_distance;
  float              distance;
  float              time;
};

struct ArcData {
  int   arc_type;
  bool  use_angle;
  float angle;
  float time;
};

struct Lp2pData {
  Lp2pData(const Pose& goal_pose, const float& distance) {
    this->goal_pose = goal_pose;
    this->distance  = distance;
  }
  Pose  goal_pose;
  float distance;
};

struct Ap2pData {
  Ap2pData(const Pose& goal_pose, const Pose& next_pose) {
    this->goal_pose = goal_pose;
    this->next_pose = next_pose;
  }
  Pose goal_pose;
  Pose next_pose;
};

struct Np2pData {
  Np2pData() {
    reach_to_goal_angle = true;
  }
  Np2pData(const std::vector<common::Pose>& path, bool reach_to_goal_angle) {
    this->reach_to_goal_angle = reach_to_goal_angle;
    this->path                = path;
  }
  std::vector<common::Pose> path;
  bool                      reach_to_goal_angle;
};

struct FindPathData {
  FindPathData(const Pose& start_pose, const Pose& goal_pose, bool find_replace_goal) {
    this->start_pose        = start_pose;
    this->goal_pose         = goal_pose;
    this->find_replace_goal = find_replace_goal;
  }
  FindPathData() {}
  Pose start_pose;
  Pose goal_pose;
  bool find_replace_goal;
};
typedef enum CoverPointType { LP2P = 0, AP2P, NP2P, VIRTUAL } CoverPointType;
typedef struct CoverPoint {
  CoverPoint() {
    pose.x()            = 0;
    pose.y()            = 0;
    type                = LP2P;
    near_goal_distance_ = 0.04;
  }

  void reset() {
    pose.x()            = 0;
    pose.y()            = 0;
    type                = LP2P;
    near_goal_distance_ = 0.04;
  }

  CoverPoint(const common::Pose& pose, const CoverPointType& type) {
    this->pose          = pose;
    this->type          = type;
    near_goal_distance_ = 0.04;
  }

  CoverPoint(const common::Pose& pose) {
    this->pose          = pose;
    this->type          = LP2P;
    near_goal_distance_ = 0.04;
  }

  common::Pose   pose;
  CoverPointType type;
  float          near_goal_distance_;
} CoverPoint;

typedef enum CoverDirType { X_DIR = 0, Y_DIR } CoverDirType;

typedef struct PeripheralControl {
  int fan_motor;
  int brush_motor;
  int rolling_motor;
} PeripheralControl;
typedef struct SelectRoomData {
  int               room_id;
  int               times;
  PeripheralControl peripheral_control;
} SelectRoomData;
typedef struct RegionCleanData {
  common::Pose      centre_pose;
  int               times;
  PeripheralControl peripheral_control;
} RegionCleanData;

typedef struct GlobalCleanData {
  PeripheralControl peripheral_control;
} GlobalCleanData;
}  // namespace common
#endif  // CLEAN_ROBOT_COMMON_TYPE_H
