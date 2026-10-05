#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <topic_tools/shape_shifter.h>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros/publisher.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#ifdef BIEVR_WITH_LIVOX2
#include <livox_ros_driver2/CustomMsg.h>
#endif

int main(int argc, char** argv) {
  srand(1);
  // ros::init strips ROS-specific arguments (remappings, __name:=, etc.) from
  // argv in place, leaving our config-file flags for loadConfigFromArgs.
  ros::init(argc, argv, "bievr_lio_topic_node");
  ros::NodeHandle nh;

  bievr::Config config;
  if (!bievr::loadConfigFromArgs({argv, argv + argc}, config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  std::shared_ptr<bievr::Pipeline> pipeline =
      std::make_shared<bievr::Pipeline>(config.pipeline_config);
  bievr::Synchronizer synchronizer(pipeline);
  bievr::Publisher lio_pub(nh, pipeline, "bievr_lio");

  ros::Subscriber pointcloud_sub = nh.subscribe<topic_tools::ShapeShifter>(
      config.topic_config.pointcloud_topic, 100000,
      [&](const topic_tools::ShapeShifter::ConstPtr& msg) {
        // Detect the message type by name
        std::string msg_type = msg->getDataType();
        LOG_FIRST(I, 1, "First message type received: " << msg_type);

        if (msg_type == "sensor_msgs/PointCloud2") {
          // Deserialize into sensor_msgs::PointCloud2
          sensor_msgs::PointCloud2::ConstPtr pc_msg = msg->instantiate<sensor_msgs::PointCloud2>();
          if (pc_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*pc_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to PointCloud2");
          }
        }
#ifdef BIEVR_WITH_LIVOX
        else if (msg_type == "livox_ros_driver/CustomMsg") {
          // Deserialize into livox_ros_driver::CustomMsg
          livox_ros_driver::CustomMsg::ConstPtr livox_msg =
              msg->instantiate<livox_ros_driver::CustomMsg>();
          if (livox_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*livox_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to livox_ros_driver CustomMsg");
          }
        }
#endif
#ifdef BIEVR_WITH_LIVOX2
        else if (msg_type == "livox_ros_driver2/CustomMsg") {
          // Deserialize into livox_ros_driver2::CustomMsg
          livox_ros_driver2::CustomMsg::ConstPtr livox2_msg =
              msg->instantiate<livox_ros_driver2::CustomMsg>();
          if (livox2_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*livox2_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to livox_ros_driver2 CustomMsg");
          }
        }
#endif
        else {
          LOG(W, "Received unsupported message type");
        }
      });

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
      config.topic_config.imu_topic, 100000, [&](const sensor_msgs::Imu::ConstPtr& msg) {
        bievr::ImuMeasurement imu;
        bievr::msgToImuMeasurement(*msg, imu);
        synchronizer.addImu(imu);
      });

  ros::spin();

  pipeline->saveMap();

  return 0;
}