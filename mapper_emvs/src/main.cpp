#include <mapper_emvs/data_loading.hpp>
#include <mapper_emvs/mapper_emvs.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <dvs_msgs/EventArray.h>

#include <sstream>
#include <iostream>
#include <string>

// Input parameters
DEFINE_string(bag_filename, "input.bag", "Path to the rosbag");
DEFINE_string(event_topic, "/dvs/events", "Name of the event topic (default: /dvs/events)");
DEFINE_string(pose_topic, "/optitrack/davis", "Name of the pose topic (default: /optitrack/davis)");
DEFINE_string(camera_info_topic, "/dvs/camera_info", "Name of the camera info topic (default: /dvs/camera_info)");
DEFINE_double(start_time_s, 0.0, "Start time in seconds (default: 0.0)");
DEFINE_double(stop_time_s, 1000.0, "Stop time in seconds (default: 1000.0)");

// Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 5.0, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");

ros::Publisher depth_mapp;
image_geometry::PinholeCameraModel cam;
EMVS::MapperEMVS mapper;
geometry_utils::Transformation T_rv_w;
LinearTrajectory trajectory;

bool got_cam = false;
bool got_initial_stamp = false;
ros::Time initial_timestamp;
ros::Time last_ts;
std::map<ros::Time, geometry_utils::Transformation> poses;

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    LOG(INFO)<<"pos";
    
    if(!got_initial_stamp)
    {
      initial_timestamp = msg->header.stamp;
      got_initial_stamp = true;
    }
    //last_ts = msg->header.stamp;
    //ros::Time curent = msg->header.stamp;

    const Eigen::Vector3d position(msg->pose.position.x,
                            msg->pose.position.y,
                            msg->pose.position.z);
    const Eigen::Quaterniond quat(msg->pose.orientation.w,
                                msg->pose.orientation.x,
                                msg->pose.orientation.y,
                                msg->pose.orientation.z);

    geometry_utils::Transformation T(position, quat);
    poses.insert(std::pair<ros::Time, geometry_utils::Transformation>(ros::Time(msg->header.stamp.toSec() - initial_timestamp.toSec()), T));
    //poses.insert(std::pair<ros::Time, geometry_utils::Transformation>(ros::Time(msg->header.stamp.toSec() - last_ts.toSec()), T));
    /*
    if(poses.size() < 2)
    {
        LOG(INFO)<<"pose size: "<<poses.size();
        return;
    }
    trajectory = LinearTrajectory(poses);

    // Set the position of the reference view in the middle of the trajectory
    geometry_utils::Transformation T0_, T1_;
    ros::Time t0_, t1_;
    trajectory.getFirstControlPose(&T0_, &t0_);
    trajectory.getLastControlPose(&T1_, &t1_);
    geometry_utils::Transformation T_w_rv;
    trajectory.getPoseAt(ros::Time(0.5 * (t0_.toSec() + t1_.toSec())), T_w_rv);
    T_rv_w = T_w_rv.inverse();

    LOG(INFO)<<"transform:"<<T_rv_w;
    //last_ts = curent;
    */
}

void event_callback(const dvs_msgs::EventArray::ConstPtr& msg)
{
    LOG(INFO)<<"event";

    if(poses.size() < 2)
    {
        LOG(INFO)<<"pose size: "<<poses.size();
        return;
    }
    LinearTrajectory trajectory = LinearTrajectory(poses);

    // Set the position of the reference view in the middle of the trajectory
    geometry_utils::Transformation T0_, T1_;
    ros::Time t0_, t1_;
    trajectory.getFirstControlPose(&T0_, &t0_);
    trajectory.getLastControlPose(&T1_, &t1_);
    geometry_utils::Transformation T_w_rv;
    trajectory.getPoseAt(ros::Time(0.5 * (t0_.toSec() + t1_.toSec())), T_w_rv);
    T_rv_w = T_w_rv.inverse();

    LOG(INFO)<<"pre evaluate DSI";
    //std::chrono::high_resolution_clock::time_point t_start_dsi = std::chrono::hight_resolution_clock::now();
    mapper.evaluateDSI(msg->events, trajectory, T_rv_w, initial_timestamp);

    mapper.dsi_.writeGridNpy("dsi.npy");
  // 2. Extract semi-dense depth map from DSI
  EMVS::OptionsDepthMap opts_depth_map;
  opts_depth_map.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
  opts_depth_map.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
  opts_depth_map.median_filter_size_ = FLAGS_median_filter_size;
  cv::Mat depth_map, confidence_map, semidense_mask;
  mapper.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);
  
  // Save depth map, confidence map and semi-dense mask

  // Save semi-dense mask as an image
    std::ostringstream name;
    name<< "semidense_mask" << msg->header.seq << ".png"; 
    cv::imwrite(name.str(), 255 * semidense_mask);
    //cv::imwrite("semidense_mask.png", 255 * semidense_mask);
    
    //LOG(INFO)<<"pre clear";
    poses.clear();
    //got_initial_stamp = false;
    //LOG(INFO)<<"post clear";
}

void cam_callback(const sensor_msgs::CameraInfo msg)
{
    cam.fromCameraInfo(msg);
    got_cam = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "run_emvs");
    ros::NodeHandle n;

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
    

    

    ros::Subscriber cam_sub = n.subscribe("/dvs/camera_info", 1, cam_callback);
    while(!got_cam)
    {
        ros::spinOnce();
    }
    cam_sub.shutdown();

    
    EMVS::ShapeDSI dsi_shape(FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ,
                           FLAGS_min_depth, FLAGS_max_depth,
                           FLAGS_fov_deg);

    
    mapper.MapperEMVS_init(cam, dsi_shape);
    //EMVS::MapperEMVS mapper(cam, dsi_shape);
    

    //depth_mapp = n.advertise<>(topic, buffer);
    ros::Subscriber pose_sub = n.subscribe("/optitrack/davis", 10, pose_callback);
    //ros::Subscriber pose_sub = n.subscribe<geometry_msgs/PoseStamped>("/optitrack/davis", 10, pose_callback);
    ros::Subscriber event_sub = n.subscribe("/dvs/events", 10, event_callback);


    while(ros::ok())
    {
        ros::spin();
    }
    return 0;
}
