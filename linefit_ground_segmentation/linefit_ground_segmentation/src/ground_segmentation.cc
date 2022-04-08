#include "ground_segmentation/ground_segmentation.h"

#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <thread>
#include <boost/thread/thread.hpp>
/*可视化点云*/
void GroundSegmentation::visualizePointCloud(const PointCloud::ConstPtr& cloud,
                                             const std::string& id) {
  viewer_->addPointCloud(cloud, id, 0);  //   /*将点云和id添加到可视化显示器之中*/
}
/*显示地面线*/
void GroundSegmentation::visualizeLines(const std::list<PointLine>& lines) {
  size_t counter = 0;
  /*遍历显示地面线*/
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    viewer_->addLine<pcl::PointXYZ>(it->first, it->second, std::to_string(counter++));
  }
}

/*
  可视化:
  在这里，我们将pcl_viewer中需要设定的内容都设定好
*/
void GroundSegmentation::visualize(const std::list<PointLine>& lines,
                                   const PointCloud::ConstPtr& min_cloud,
                                   const PointCloud::ConstPtr& ground_cloud,
                                   const PointCloud::ConstPtr& obstacle_cloud) {
  viewer_->setBackgroundColor (0, 0, 0);
  viewer_->addCoordinateSystem (1.0);
  viewer_->initCameraParameters ();
  viewer_->setCameraPosition(-2.0, 0, 2.0, 1.0, 0, 0);
  visualizePointCloud(min_cloud, "min_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             0.0f, 1.0f, 0.0f,
                                             "min_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             2.0f,
                                             "min_cloud");
  visualizePointCloud(ground_cloud, "ground_cloud");
  viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                             1.0f, 0.0f, 0.0f,
                                             "ground_cloud");
  visualizePointCloud(obstacle_cloud, "obstacle_cloud");
  visualizeLines(lines);
  while (!viewer_->wasStopped ()){
      viewer_->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

/*地面分割的构造函数*/
GroundSegmentation::GroundSegmentation(const GroundSegmentationParams& params) :
    params_(params),
    segments_(params.n_segments, Segment(params.n_bins,
                                         params.max_slope,
                                         params.max_error_square,
                                         params.long_threshold,
                                         params.max_long_height,
                                         params.max_start_height,
                                         params.sensor_height)) {
  if (params.visualize) viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
}

/*地面分割的分割函数*/
void GroundSegmentation::segment(const PointCloud& cloud, std::vector<int>* segmentation) {
  /*初始化一些比较基础的东西*/
  std::cout << "Segmenting cloud with " << cloud.size() << " points...\n";
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();   // 起始时间
  segmentation->clear();
  segmentation->resize(cloud.size(), 0);
  bin_index_.resize(cloud.size());
  segment_coordinates_.resize(cloud.size());

  /*插入点云数据*/
  insertPoints(cloud);
  /*定义地面点的线的参数*/
  std::list<PointLine> lines;
  /*根据我们的设定来决定是否进行可视化的展示*/
  if (params_.visualize) {
    getLines(&lines);
  }
  else {
    getLines(NULL);
  }
  /*对于传入的分割进行细分*/
  /*从这里可以看到对于点云属于障碍物还是属于地面点进行了标签的划分*/
  assignCluster(segmentation);

  /*如果是进行了可视化的操作，则进行一下的操作*/
  if (params_.visualize) {
    // Visualize.
    PointCloud::Ptr obstacle_cloud(new PointCloud());
    // Get cloud of ground points.
    PointCloud::Ptr ground_cloud(new PointCloud());
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (segmentation->at(i) == 1) ground_cloud->push_back(cloud[i]);
      else obstacle_cloud->push_back(cloud[i]);
    }
    PointCloud::Ptr min_cloud(new PointCloud());
    getMinZPointCloud(min_cloud.get());
    visualize(lines, min_cloud, ground_cloud, obstacle_cloud);
  }
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fp_ms = end - start;
  std::cout << "Done! Took " << fp_ms.count() << "ms\n";  // 统计时间
}

/*获取到线*/
void GroundSegmentation::getLines(std::list<PointLine> *lines) {
  std::mutex line_mutex;
  std::vector<std::thread> thread_vec(params_.n_threads);
  unsigned int i;
  for (i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = params_.n_segments / params_.n_threads * i;
    const unsigned int end_index = params_.n_segments / params_.n_threads * (i+1);
    thread_vec[i] = std::thread(&GroundSegmentation::lineFitThread, this,
                                start_index, end_index, lines, &line_mutex);
  }
  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
  }
}

/*这里是获取线的操作*/
void GroundSegmentation::lineFitThread(const unsigned int start_index,
                                       const unsigned int end_index,
                                       std::list<PointLine> *lines, std::mutex* lines_mutex) {
  const bool visualize = lines;
  const double seg_step = 2*M_PI / params_.n_segments;
  double angle = -M_PI + seg_step/2 + seg_step * start_index;
  for (unsigned int i = start_index; i < end_index; ++i) {
    segments_[i].fitSegmentLines();
    // Convert lines to 3d if we want to.
    /*这里也是可视化的一些操作*/
    if (visualize) {
      std::list<Segment::Line> segment_lines;
      segments_[i].getLines(&segment_lines);
      for (auto line_iter = segment_lines.begin(); line_iter != segment_lines.end(); ++line_iter) {
        const pcl::PointXYZ start = minZPointTo3d(line_iter->first, angle);
        const pcl::PointXYZ end = minZPointTo3d(line_iter->second, angle);
        lines_mutex->lock();
        lines->emplace_back(start, end);
        lines_mutex->unlock();
      }

      angle += seg_step;
    }
  }
}

void GroundSegmentation::getMinZPointCloud(PointCloud* cloud) {
  const double seg_step = 2*M_PI / params_.n_segments;
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      const pcl::PointXYZ min = minZPointTo3d(bin_iter->getMinZPoint(), angle);
      cloud->push_back(min);
    }

    angle += seg_step;
  }
}

/*根据传入的二维点，也可以通过算出的angle将二维点转化为三维点，主要是x-y平面内的变换*/
pcl::PointXYZ GroundSegmentation::minZPointTo3d(const Bin::MinZPoint &min_z_point,
                                                const double &angle) {
  pcl::PointXYZ point;
  point.x = cos(angle) * min_z_point.d;
  point.y = sin(angle) * min_z_point.d;
  point.z = min_z_point.z;
  return point;
}

/*分配集群，将传入的分割进行簇的划分*/
void GroundSegmentation::assignCluster(std::vector<int>* segmentation) {
  std::vector<std::thread> thread_vec(params_.n_threads);
  const size_t cloud_size = segmentation->size();
  for (unsigned int i = 0; i < params_.n_threads; ++i) {
    const unsigned int start_index = cloud_size / params_.n_threads * i;
    const unsigned int end_index = cloud_size / params_.n_threads * (i+1);
    thread_vec[i] = std::thread(&GroundSegmentation::assignClusterThread, this,
                                start_index, end_index, segmentation);
  }
  for (auto it = thread_vec.begin(); it != thread_vec.end(); ++it) {
    it->join();
  }
}

/*执行分配集群的线程操作*/
void GroundSegmentation::assignClusterThread(const unsigned int &start_index,
                                             const unsigned int &end_index,
                                             std::vector<int> *segmentation) {
  /*通过传入的segments的个数，得到划分segment的步长，也就是δ值*/
  const double segment_step = 2*M_PI/params_.n_segments;
  /*进行遍历操作*/
  /*对于每一个点进行处理，根据距离判断是否属于远离地面的点*/
  for (unsigned int i = start_index; i < end_index; ++i) {
    /*首先，得到每一个三维点的二维映射*/
    Bin::MinZPoint point_2d = segment_coordinates_[i];
    /*得到第i个bin的第一个值，作为分割的索引*/
    const int segment_index = bin_index_[i].first;
    /*判定分割的index是需要大于0的*/
    if (segment_index >= 0) {
      /*计算处两个点到线的垂直距离*/
      /*将点投影到直线上*/
      double dist = segments_[segment_index].verticalDistanceToLine(point_2d.d, point_2d.z);
      // Search neighboring segments.
      /*搜索相邻的分割*/
      int steps = 1;
      /*根据划分好的segment来不断的进行数据的处理*/
      /*在设定的步长的情况下在搜索框内能有几个步长*/
      while (dist == -1 && steps * segment_step < params_.line_search_angle) {
        // Fix indices that are out of bounds.
        /*修复超出范围的索引*/
        int index_1 = segment_index + steps;//进行正向步长的搜索
        while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
        int index_2 = segment_index - steps;//进行反向步长的索引
        while (index_2 < 0) index_2 += params_.n_segments;
        // Get distance to neighboring lines.
        /*算出根据正反双向最近搜索的点到线段投影的距离*/
        const double dist_1 = segments_[index_1].verticalDistanceToLine(point_2d.d, point_2d.z);
        const double dist_2 = segments_[index_2].verticalDistanceToLine(point_2d.d, point_2d.z);
        // Select larger distance if both segments return a valid distance.
        /*如果两个分割都返回有效距离，则选择更大的距离*/
        if (dist_1 > dist) {
          dist = dist_1;
        }
        if (dist_2 > dist) {
          dist = dist_2;
        }
        /*不断增加步长，一直持续下去，直到跳出循环，这样可以保证比较公平的遍历到里面的点*/
        ++steps;
      }
      /*这里是进行标签的设定*/
      if (dist < params_.max_dist_to_line && dist != -1) {
        /*这里是对于是否属于地面点的判定*/
        segmentation->at(i) = 1;
      }
    }
  }
}

/*获取最小z点的点云数据*/
void GroundSegmentation::getMinZPoints(PointCloud* out_cloud) {
  /*得到分割的步长，以及bins的步长*/
  const double seg_step = 2*M_PI / params_.n_segments;
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;
  /*得到最小的r*/
  const double r_min = sqrt(params_.r_min_square);
  double angle = -M_PI + seg_step/2;
  for (auto seg_iter = segments_.begin(); seg_iter != segments_.end(); ++seg_iter) {
    double dist = r_min + bin_step/2;
    for (auto bin_iter = seg_iter->begin(); bin_iter != seg_iter->end(); ++bin_iter) {
      pcl::PointXYZ point;
      if (bin_iter->hasPoint()) {
        /*对于bin_iter进行最小z点的提取*/
        Bin::MinZPoint min_z_point(bin_iter->getMinZPoint());
        point.x = cos(angle) * min_z_point.d;
        point.y = sin(angle) * min_z_point.d;
        point.z = min_z_point.z;
        /*将point放入到out_cloud之中*/
        out_cloud->push_back(point);
      }
      /*按照步长增加dist*/
      dist += bin_step;
    }
    /*按照划分的步长进行角度的增加*/
    angle += seg_step;
  }
}


/*插入点云*/
void GroundSegmentation::insertPoints(const PointCloud& cloud) {
  std::vector<std::thread> threads(params_.n_threads);
  const size_t points_per_thread = cloud.size() / params_.n_threads;
  // Launch threads.
  /*根据我们设定的数目来将整个的点云分为几个部分开始处理，利用多线程来处理*/
  for (unsigned int i = 0; i < params_.n_threads - 1; ++i) {
    const size_t start_index = i * points_per_thread;
    const size_t end_index = (i+1) * points_per_thread - 1;
    threads[i] = std::thread(&GroundSegmentation::insertionThread, this,
                             cloud, start_index, end_index);
  }
  // Launch last thread which might have more points than others.
  /*启动最后一个可能含有更多点云的线程*/
  const size_t start_index = (params_.n_threads - 1) * points_per_thread;
  const size_t end_index = cloud.size() - 1;
  threads[params_.n_threads - 1] =
      std::thread(&GroundSegmentation::insertionThread, this, cloud, start_index, end_index);
  // Wait for threads to finish.
  for (auto it = threads.begin(); it != threads.end(); ++it) {
    it->join();
  }
}

/*线程启动中会执行的函数*/
void GroundSegmentation::insertionThread(const PointCloud& cloud,
                                         const size_t start_index,
                                         const size_t end_index) {
  /*同样的先算步长，然后再进行其他的操作*/
  const double segment_step = 2*M_PI / params_.n_segments;
  const double bin_step = (sqrt(params_.r_max_square) - sqrt(params_.r_min_square))
      / params_.n_bins;
  const double r_min = sqrt(params_.r_min_square);
  /*对于起始索引和终止索引进行遍历*/
  for (unsigned int i = start_index; i < end_index; ++i) {
    pcl::PointXYZ point(cloud[i]);
    /*这里是算模长*/
    const double range_square = point.x * point.x + point.y * point.y;
    const double range = sqrt(range_square);
    /*判断模场是否属于最小值和最大值之间*/
    if (range_square < params_.r_max_square && range_square > params_.r_min_square) {
       /*得到角度*/
      const double angle = std::atan2(point.y, point.x);
      /*根据模场和角度来算出bin的索引以及划分的索引*/
      const unsigned int bin_index = (range - r_min) / bin_step;
      const unsigned int segment_index = (angle + M_PI) / segment_step;
      const unsigned int segment_index_clamped = segment_index == params_.n_segments ? 0 : segment_index;
      /*对于设定的属于bin和划分中的集合进行数据的添加*/
      segments_[segment_index_clamped][bin_index].addPoint(range, point.z);
      /*将后面的数据作为一个元组全部传递到bin_index之中*/
      bin_index_[i] = std::make_pair(segment_index_clamped, bin_index);
    }
    else {
      bin_index_[i] = std::make_pair<int, int>(-1, -1);
    }
    /*获取到划分坐标为最小z点的坐标和range*/
    segment_coordinates_[i] = Bin::MinZPoint(range, point.z);
  }
}
