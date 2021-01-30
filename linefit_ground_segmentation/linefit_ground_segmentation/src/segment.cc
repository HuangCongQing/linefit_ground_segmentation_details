#include "ground_segmentation/segment.h"

Segment::Segment(const unsigned int& n_bins,
                 const double& max_slope,
                 const double& max_error,
                 const double& long_threshold,
                 const double& max_long_height,
                 const double& max_start_height,
                 const double& sensor_height) :
                 bins_(n_bins),
                 max_slope_(max_slope),
                 max_error_(max_error),
                 long_threshold_(long_threshold),
                 max_long_height_(max_long_height),
                 max_start_height_(max_start_height),
                 sensor_height_(sensor_height){}

/*分割线拟合*/
void Segment::fitSegmentLines() {
  // Find first point.
  auto line_start = bins_.begin();
  /*在整个的bins中找第一个点*/
  while (!line_start->hasPoint()) {
    ++line_start;
    // Stop if we reached last point.
    if (line_start == bins_.end()) return;
  }
  // Fill lines.
  bool is_long_line = false;
  double cur_ground_height = -sensor_height_;
  /*将线的第一个点的信息传递到*/
  std::list<Bin::MinZPoint> current_line_points(1, line_start->getMinZPoint());
  LocalLine cur_line = std::make_pair(0,0);
  /*从第一个点开始对于bins上的每一个点都进行遍历操作*/
  for (auto line_iter = line_start+1; line_iter != bins_.end(); ++line_iter) {
    /*如果我们设定的线上有点，则进行后面的操作*/
    if (line_iter->hasPoint()) {
      Bin::MinZPoint cur_point = line_iter->getMinZPoint();
      /*如果两个的线的长度大于我们设定的阈值，则我们认为这两个点之间是长线*/
      if (cur_point.d - current_line_points.back().d > long_threshold_) is_long_line = true;
      /*针对于后面几次遍历而言的，至少有三个点的情况下*/
      if (current_line_points.size() >= 2) {

        // Get expected z value to possibly reject far away points.
        /*获取远离点的z值*/
        double expected_z = std::numeric_limits<double>::max();
        /*如果是长线段且当前线段的点数大于2，则我们获取到期待的z，这个在第一次迭代的时候不会被执行*/
        if (is_long_line && current_line_points.size() > 2) {
          expected_z = cur_line.first * cur_point.d + cur_line.second;
        }
        /*将当前点插入到current_line之中*/
        current_line_points.push_back(cur_point);
        /*对于当前线点传入到本地线拟合中，得到最后的结果*/
        cur_line = fitLocalLine(current_line_points);
        /*将我们经过本地线拟合之后的*/
        const double error = getMaxError(current_line_points, cur_line);
        // Check if not a good line.
        /*将算出来的误差和最大误差进行比较，判断是否是一个合格的线*/
        if (error > max_error_ ||
            std::fabs(cur_line.first) > max_slope_ ||
            is_long_line && std::fabs(expected_z - cur_point.z) > max_long_height_) {
          // Add line until previous point as ground.
          /*添加线直到浅一点是地面点*/
          current_line_points.pop_back();
          // Don't let lines with 2 base points through.
          /*不要让有两个基点的线穿过*/
          if (current_line_points.size() >= 3) {
            /*对于当前线点进行本地拟合，得到一条新的线*/
            const LocalLine new_line = fitLocalLine(current_line_points);
            /*将进行处理后的点放入到线中*/
            lines_.push_back(localLineToLine(new_line, current_line_points));
            /*计算出当前地面点的高度*/
            cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
          }
          // Start new line.
          is_long_line = false;
          /*erase在删除的过程中还是减少vector的size*/
          current_line_points.erase(current_line_points.begin(), --current_line_points.end());
          --line_iter;
        }
        // Good line, continue.
        else { }
      }
    /*在有1到2个点的情况下的处理*/
      else {
        // Not enough points.
        /*判断是否满足添加条件，添加这些点*/
        if (cur_point.d - current_line_points.back().d < long_threshold_ &&
            std::fabs(current_line_points.back().z - cur_ground_height) < max_start_height_) {
          // Add point if valid.
          current_line_points.push_back(cur_point);
        }
        /*开始一条新的线*/
        else {
          // Start new line.
          current_line_points.clear();
          current_line_points.push_back(cur_point);
        }
      }
    }
  }
  // Add last line.
  /*添加最后一条线*/
  if (current_line_points.size() > 2) {
    const LocalLine new_line = fitLocalLine(current_line_points);
    lines_.push_back(localLineToLine(new_line, current_line_points));
  }
}

/*本地线到线，得到两个点，构建出一条直线*/
Segment::Line Segment::localLineToLine(const LocalLine& local_line,
                                       const std::list<Bin::MinZPoint>& line_points) {
  Line line;
  const double first_d = line_points.front().d;
  const double second_d = line_points.back().d;
  /*跟去前面的值来进行locl_line的处理*/
  const double first_z = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;
  line.first.z = first_z;
  line.first.d = first_d;
  line.second.z = second_z;
  line.second.d = second_d;
  return line;
}

/*到线的垂直距离*/
double Segment::verticalDistanceToLine(const double &d, const double &z) {
  static const double kMargin = 0.1;
  double distance = -1;
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    /*这里设定了论文中要求的距离*/
    /*针对于d点，按照设定的余量在前后范围内找到两个点*/
    if (it->first.d - kMargin < d && it->second.d + kMargin > d) {
      /*这里是先算出斜率，将传入的两个点传入到直线中，算出一个最近的额z值差，也就是垂直的距离*/
      /*算出找到的两个点之间的斜率*/
      const double delta_z = it->second.z - it->first.z;
      const double delta_d = it->second.d - it->first.d;
      const double expected_z = (d - it->first.d)/delta_d *delta_z + it->first.z;//(delta_z/delta_d)是一个斜率
      //算出最终的距离
      distance = std::fabs(z - expected_z);
    }
  }
  return distance;
}

double Segment::getMeanError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double error_sum = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    error_sum += residual * residual;
  }
  return error_sum/points.size();
}

double Segment::getMaxError(const std::list<Bin::MinZPoint> &points, const LocalLine &line) {
  double max_error = 0;
  for (auto it = points.begin(); it != points.end(); ++it) {
    const double residual = (line.first * it->d + line.second) - it->z;
    const double error = residual * residual;
    if (error > max_error) max_error = error;
  }
  return max_error;
}

/*本地线拟合*/
Segment::LocalLine Segment::fitLocalLine(const std::list<Bin::MinZPoint> &points) {
  const unsigned int n_points = points.size();
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  unsigned int counter = 0;
  for (auto iter = points.begin(); iter != points.end(); ++iter) {
    X(counter, 0) = iter->d;
    X(counter, 1) = 1;
    Y(counter) = iter->z;
    ++counter;
  }
  Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
  LocalLine line_result;
  line_result.first = result(0);
  line_result.second = result(1);
  return line_result;
}

bool Segment::getLines(std::list<Line> *lines) {
  if (lines_.empty()) {
    return false;
  }
  else {
    *lines = lines_;
    return true;
  }
}
