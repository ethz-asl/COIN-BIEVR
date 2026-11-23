#ifndef COIN_BIEVR_ROS_COMMON_PUBLISHER_BASE_H_
#define COIN_BIEVR_ROS_COMMON_PUBLISHER_BASE_H_

// ROS-version-agnostic publisher logic shared by the ROS1 and ROS2 wrappers.
// Everything that genuinely differs between the two (the node handle, the
// type-erased publisher mechanism, the concrete message types and the TF
// broadcaster) is supplied by a `Backend` policy; the dispatch, lazy
// advertising, namespace handling and pipeline registration live here once.
//
// The message conversions (headerToMsg / pointCloudToMsg / transformToMsg /
// vecToMsg) are the version-agnostic templates from conversions.h, included
// below so they resolve by ordinary lookup at this template's definition. They
// build PointCloud2 messages via sensor_msgs::PointCloud2Modifier, so the TU
// that instantiates a Publisher must have included the matching
// sensor_msgs/point_cloud2_iterator header (the wrappers' conversions.h, pulled
// in by publisher.h, does this).

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "coin_bievr/common.h"
#include "coin_bievr/log++.h"
#include "coin_bievr/pipeline.h"
#include "coin_bievr_ros_common/conversions.h"

namespace coin_bievr {

template <typename Backend>
class PublisherBase {
 public:
  using Handle = typename Backend::Handle;

  // If ns is non-empty, every topic published through this object is advertised
  // under that namespace (e.g. ns="coin_bievr" turns "odometry" into
  // "/coin_bievr/odometry"). Absolute topics (leading '/') are left untouched.
  PublisherBase(Handle handle, std::shared_ptr<Pipeline> pipeline, const std::string& ns = "")
      : backend_(std::move(handle)), ns_(ns) {
    registerTypes<Pointcloud, IntensityPointcloud, Odometry, V3, Eigen::MatrixXf, DegeneracyInfo>(
        pipeline);
#ifdef COIN_BIEVR_WITH_NVBLOX
    registerTypes<IntensityMesh>(pipeline);
#endif
  }
  virtual ~PublisherBase() = default;

  template <typename T>
  bool publish(const T& data, const Header& header, const std::string& topic,
               const std::string& child_frame = "") {
    return publishImpl(data, header, topic, child_frame);
  }

 private:
  // Pointcloud and IntensityPointcloud both go out as PointCloud2.
  bool publishImpl(const Pointcloud& cloud, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::PointCloud2 msg;
    if (!getOrAdvertise<typename Backend::PointCloud2>(topic)) return false;
    headerToMsg(header, msg.header);
    pointCloudToMsg(cloud, msg);
    publishers_[topic].publish(msg);
    return true;
  }

  bool publishImpl(const IntensityPointcloud& cloud, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::PointCloud2 msg;
    if (!getOrAdvertise<typename Backend::PointCloud2>(topic)) return false;
    headerToMsg(header, msg.header);
    pointCloudToMsg(cloud, msg);
    publishers_[topic].publish(msg);
    return true;
  }

  bool publishImpl(const Odometry& odometry, const Header& header, const std::string& topic,
                   const std::string& child_frame) {
    typename Backend::Odometry odom_msg;
    if (!getOrAdvertise<typename Backend::Odometry>(topic)) return false;
    headerToMsg(header, odom_msg.header);
    odom_msg.child_frame_id = child_frame;
    transformToMsg(odometry.pose, odom_msg.pose.pose);
    // Twist is expressed in the child (body) frame.
    vecToMsg(odometry.linear_velocity, odom_msg.twist.twist.linear);
    vecToMsg(odometry.angular_velocity, odom_msg.twist.twist.angular);
    publishers_[topic].publish(odom_msg);

    // Mirror the pose as a TF transform.
    typename Backend::TransformStamped transform_msg;
    transform_msg.header = odom_msg.header;
    transform_msg.child_frame_id = child_frame;
    transformToMsg(odometry.pose, transform_msg.transform);
    backend_.sendTransform(transform_msg);
    return true;
  }

  // The projector's debug images, published as mono8.
  bool publishImpl(const Eigen::MatrixXf& image, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    if (image.size() == 0) return false;
    typename Backend::Image msg;
    if (!getOrAdvertise<typename Backend::Image>(topic)) return false;
    headerToMsg(header, msg.header);
    imageToMsg(image, msg);
    publishers_[topic].publish(msg);
    return true;
  }

#ifdef COIN_BIEVR_WITH_NVBLOX
  // The intensity map visualization, as nvblox_msgs/Mesh for nvblox_rviz_plugin.
  bool publishImpl(const IntensityMesh& mesh, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::NvbloxMesh msg;
    if (!getOrAdvertise<typename Backend::NvbloxMesh>(topic)) return false;
    headerToMsg(header, msg.header);
    Backend::setBlockSize(msg, mesh.block_size);
    intensityMeshToMsg(mesh, msg);
    publishers_[topic].publish(msg);
    return true;
  }
#endif

  // The degenerate directions of the photometric sampling, as RViz arrows.
  bool publishImpl(const DegeneracyInfo& degeneracy, const Header& header,
                   const std::string& topic, const std::string& /*child_frame*/) {
    typename Backend::MarkerArray msg;
    if (!getOrAdvertise<typename Backend::MarkerArray>(topic)) return false;
    typename Backend::MarkerArray::_markers_type::value_type::_header_type header_msg;
    headerToMsg(header, header_msg);
    degeneracyToMsg(degeneracy, header_msg, msg);
    publishers_[topic].publish(msg);
    return true;
  }

  bool publishImpl(const V3& vec, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::Vector3Stamped msg;
    if (!getOrAdvertise<typename Backend::Vector3Stamped>(topic)) return false;
    headerToMsg(header, msg.header);
    vecToMsg(vec, msg.vector);
    publishers_[topic].publish(msg);
    return true;
  }

  // Look up (or lazily advertise) the publisher for `topic`, verifying that its
  // message type matches any previously advertised one. Returns false on a
  // type mismatch (the message is then dropped by the caller).
  template <typename MsgT>
  bool getOrAdvertise(const std::string& topic) {
    auto it = publishers_.find(topic);
    if (it == publishers_.end()) {
      backend_.template advertise<MsgT>(publishers_[topic], resolveTopic(topic));
    } else if (!it->second.isSameType(typeid(MsgT))) {
      LOG(W, "Publisher type mismatch for topic " << topic);
      return false;
    }
    return true;
  }

  template <typename T>
  void registerWithPipeline(std::shared_ptr<Pipeline> pipeline) {
    pipeline->registerPublisher<T>(std::bind_front(&PublisherBase::publish<T>, this));
  }

  template <typename... Ts>
  void registerTypes(std::shared_ptr<Pipeline> pipeline) {
    (registerWithPipeline<Ts>(pipeline), ...);  // fold expression
  }

  // Prepends the namespace to a relative topic. Absolute topics (leading '/')
  // and the empty-namespace case are returned unchanged.
  std::string resolveTopic(const std::string& topic) const {
    if (ns_.empty() || topic.empty() || topic.front() == '/') {
      return topic;
    }
    return "/" + ns_ + "/" + topic;
  }

  Backend backend_;
  std::string ns_;
  std::unordered_map<std::string, typename Backend::TypedPublisher> publishers_;
};

}  // namespace coin_bievr

#endif  // COIN_BIEVR_ROS_COMMON_PUBLISHER_BASE_H_
