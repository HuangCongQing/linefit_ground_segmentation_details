#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>

#include "ground_segmentation/ground_segmentation.h"

/*我们定义的分割节点的类*/
class SegmentationNode {

  ros::Publisher ground_pub_;      //地面点的发布者
  ros::Publisher obstacle_pub_;    //障碍物点的发布者
  GroundSegmentationParams params_;//地面分割的参数

public:
  SegmentationNode(ros::NodeHandle& nh,
                   const std::string& ground_topic,
                   const std::string& obstacle_topic,
                   const GroundSegmentationParams& params,
                   const bool& latch = false) : params_(params) {
    ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(ground_topic, 1, latch);
    obstacle_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(obstacle_topic, 1, latch);
  }

  /*回调函数*/
  void scanCallback(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    /*将我们设定的参数传递进去，这里就直接跳转到linefit_ground_segmentation之中*/
    GroundSegmentation segmenter(params_);
    /*定义一个是否属于地面点的标签，来进行区分*/
    std::vector<int> labels;

    /*这个是地面分割类中的主要函数，分割函数9*/
    segmenter.segment(cloud, &labels);
    /*定义了两种点云，一个是地面点，一个是障碍物点*/
    pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
    ground_cloud.header = cloud.header;
    obstacle_cloud.header = cloud.header;
    /*将整个的无序点云按照标签分配到不同的点云之中*/
    /*通过这里的标签对于是否属于地面点进行了划分*/
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (labels[i] == 1) ground_cloud.push_back(cloud[i]);
      else obstacle_cloud.push_back(cloud[i]);
    }
    /*将按照标签分类好的数据分配到不同的点云之中*/
    ground_pub_.publish(ground_cloud);
    obstacle_pub_.publish(obstacle_cloud);
  }
};

/*主函数，给出了ros的接口，跳转到linefir_ground_segmentation之中*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation");
  /*调用glog中的函数来初始化日志，为后面做准备*/
  google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");

  // Do parameter stuff.
  /*参数设定，可以读入我们在yaml中设定的参数*/
  GroundSegmentationParams params;
  nh.param("visualize", params.visualize, params.visualize);
  nh.param("n_bins", params.n_bins, params.n_bins);
  nh.param("n_segments", params.n_segments, params.n_segments);
  nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
  nh.param("max_slope", params.max_slope, params.max_slope);
  nh.param("long_threshold", params.long_threshold, params.long_threshold);
  nh.param("max_long_height", params.max_long_height, params.max_long_height);
  nh.param("max_start_height", params.max_start_height, params.max_start_height);
  nh.param("sensor_height", params.sensor_height, params.sensor_height);
  nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
  nh.param("n_threads", params.n_threads, params.n_threads);
  // Params that need to be squared.
  /*得到平方后的数据信息，分别是r_min,r_max和max_fit_error*/
  double r_min, r_max, max_fit_error;
  if (nh.getParam("r_min", r_min)) {
    params.r_min_square = r_min*r_min;
  }
  if (nh.getParam("r_max", r_max)) {
    params.r_max_square = r_max*r_max;
  }
  if (nh.getParam("max_fit_error", max_fit_error)) {
    params.max_error_square = max_fit_error * max_fit_error;
  }

  std::string ground_topic, obstacle_topic, input_topic;
  bool latch;
  /*input topic是需要我们自己设定的，根据自己雷达的数据而定*/
  nh.param<std::string>("input_topic", input_topic, "input_cloud");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param("latch", latch, false);

  // Start node.
  /*我们初始化类，从类中调到别的地方*/
  SegmentationNode node(nh, ground_topic, obstacle_topic, params, latch);
  ros::Subscriber cloud_sub;
  /*从这里的回调函数，开始进入主要的函数中*/
  cloud_sub = nh.subscribe(input_topic, 1, &SegmentationNode::scanCallback, &node);
  ros::spin();
}