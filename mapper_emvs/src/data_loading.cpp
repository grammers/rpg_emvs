#include <mapper_emvs/data_loading.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <glog/logging.h>

namespace data_loading {

void parse_rosbag(const std::string &rosbag,
                  std::vector<prophesee_event_msgs::Event>& events_,
                  std::map<ros::Time, geometry_utils::Transformation>& poses_,
                  sensor_msgs::CameraInfo& camera_info_msg,
                  const std::string& event_topic,
                  const std::string& camera_info_topic,
                  const std::string& pose_topic,
                  const double tmin,
                  const double tmax)
{
  std::vector<std::string> topics;
  topics.push_back(event_topic);
  topics.push_back(camera_info_topic);
  topics.push_back(pose_topic);
  topics.push_back("/dvs/events");
  topics.push_back("/dvs/camera_info");
  topics.push_back("/optitrack/davis");

  poses_.clear();
  events_.clear();

  rosbag::Bag  bag(rosbag, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  bool continue_looping_through_bag = true;
  bool got_initial_stamp = false;
  ros::Time initial_timestamp;

  //ROS_INFO("for eatc");
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
  //ROS_INFO("for eatc");
    if(!continue_looping_through_bag)
    {
      break;
    }

    const std::string& topic_name = m.getTopic();
    VLOG(2) << topic_name;

    // Events
    if (topic_name == topics[0] || topic_name == topics[3])
    {
      prophesee_event_msgs::EventArray::ConstPtr msg = m.instantiate<prophesee_event_msgs::EventArray>();
      if (msg != NULL)
      {
        if(msg->events.empty())
        {
          continue;
        }
        const ros::Time& stamp = msg->events[0].ts;

        if(!got_initial_stamp)
        {
          initial_timestamp = stamp;
          LOG(INFO) << "initial stamp: " << stamp;
          got_initial_stamp = true;
        }

        for (size_t i = 0; i < msg->events.size(); ++i)
        {
          const double rel_stamp = (msg->events[i].ts - initial_timestamp).toSec();
          if(rel_stamp < tmin)
          {
            continue;
          }
          if(rel_stamp > tmax)
          {
            continue_looping_through_bag = false;
          }

          prophesee_event_msgs::Event ev_modified(msg->events[i]);
          ev_modified.ts = ros::Time(ev_modified.ts.toSec() - initial_timestamp.toSec());
          events_.push_back(ev_modified);
        }
      }
    }

    // Camera Info
    if (topic_name == topics[1] || topic_name == topics[4])
    {
      camera_info_msg = *(m.instantiate<sensor_msgs::CameraInfo>());
    }

    // Pose
    //std::cout<<topic_name<<std::endl;
    if (topic_name == topics[2] || topic_name == topics[5])
    {
    //ROS_INFO("read pos");
      const geometry_msgs::TransformStamped pose_msg
          = *(m.instantiate<geometry_msgs::TransformStamped>());
      const ros::Time& stamp = pose_msg.header.stamp;
      if(!got_initial_stamp)
      {
        initial_timestamp = stamp;
        LOG(INFO) << "Initial stamp: " << stamp;
        got_initial_stamp = true;
      }
      const double rel_stamp = (stamp - initial_timestamp).toSec();
      if(rel_stamp < tmin)
      {
        continue;
      }
      if(rel_stamp > tmax)
      {
        continue_looping_through_bag = false;
      }

      const Eigen::Vector3d position(pose_msg.transform.translation.x,
                                     pose_msg.transform.translation.y,
                                     pose_msg.transform.translation.z);
      const Eigen::Quaterniond quat(pose_msg.transform.rotation.w,
                                    pose_msg.transform.rotation.x,
                                    pose_msg.transform.rotation.y,
                                    pose_msg.transform.rotation.z);
      geometry_utils::Transformation T(position, quat);
      poses_.insert(std::pair<ros::Time, geometry_utils::Transformation>(ros::Time(pose_msg.header.stamp.toSec() - initial_timestamp.toSec()), T));
    }
  }

  // Sort events by increasing timestamps
  std::sort(events_.begin(), events_.end(),
            [](const prophesee_event_msgs::Event& a, const prophesee_event_msgs::Event& b) -> bool
  {
    return a.ts < b.ts;
  });
}

} // namespace data_loading
