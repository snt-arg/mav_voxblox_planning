#include "voxblox_skeleton/skeleton_server.h"

#include "voxblox_skeleton/ros/skeleton_vis.h"

#include <voxblox_ros/conversions.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <boost/lockfree/spsc_queue.hpp>

#include "voxblox_skeleton/ros/bim_vis.h"

namespace voxblox {

boost::lockfree::spsc_queue<std::shared_ptr<SkeletonGenerator>>
    g_skeleton_generator_queue(1);

boost::lockfree::spsc_queue<std::shared_ptr<Layer<EsdfVoxel>>,
                            boost::lockfree::capacity<1>>
    g_esdf_queue;

SkeletonServer::SkeletonServer(const ros::NodeHandle &nh,
                               const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), esdf_server_(nh, nh_private),
      skeleton_generator_(std::make_shared<SkeletonGenerator>()) {

  // params
  nh_private_.param("skeleton_publish_data", publish_data_, publish_data_);
  nh_private_.param("skeleton_visualize_data", vis_data_, vis_data_);
  nh_private_.param("skeleton_sparse_graph_filepath", sparse_graph_filepath_,
                    sparse_graph_filepath_);
  nh_private_.param("frame_id", frame_id_, frame_id_);
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("skeleton_generation_interval", generation_interval_,
                    generation_interval_);

  SkeletonGenerator skeleton_generator;
  min_separation_angle_ = skeleton_generator.getMinSeparationAngle();
  nh_private_.param("skeleton_min_separation_angle", min_separation_angle_,
                    min_separation_angle_);

  generate_by_layer_neighbors_ =
      skeleton_generator.getGenerateByLayerNeighbors();
  nh_private_.param("skeleton_generate_by_layer_neighbors",
                    generate_by_layer_neighbors_, generate_by_layer_neighbors_);

  num_neighbors_for_edge_ = skeleton_generator.getNumNeighborsForEdge();
  nh_private_.param("skeleton_num_neighbors_for_edge", num_neighbors_for_edge_,
                    num_neighbors_for_edge_);

  min_gvd_distance_ = skeleton_generator.getMinGvdDistance();
  nh_private_.param("skeleton_min_gvd_distance", min_gvd_distance_,
                    min_gvd_distance_);

  std::string inspection_marker_topic;
  nh_private_.param("inspection_marker_topic", inspection_marker_topic,
                    inspection_marker_topic);

  // bim params
  std::string bim_filename;
  nh_private_.param("bim_filename", bim_filename, bim_filename);

  // publishers
  sparse_graph_pub_ =
      nh_private_.advertise<voxblox_planning_msgs::SkeletonGraph>(
          "skeleton_sparse_graph_out", 1, false);

  skeleton_layer_pub_ = nh_private_.advertise<voxblox_msgs::Layer>(
      "skeleton_layer_out", 1, false);

  skeleton_pc_vis_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZ>>(
      "skeleton_vis", 1, false);

  sparse_graph_vis_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "skeleton_sparse_graph", 1, false);

  bim_intersection_layer_vis_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "bim_intersections_vis", 1, false);

  bim_freespace_layer_vis_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "bim_freespace_vis", 1, false);

  bim_vis_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "bim_vis", 1, true);

  // subscribers
  sparse_graph_sub_ =
      nh_private_.subscribe<voxblox_planning_msgs::SkeletonGraph>(
          "skeleton_sparse_graph_in", 1,
          [&](const voxblox_planning_msgs::SkeletonGraph::ConstPtr &msg) {
            ROS_INFO_ONCE("Got a sparse graph from ROS topic!");
            skeleton_generator_->getSparseGraph().setFromGraphMsg(msg);
          });

  skeleton_layer_sub_ = nh_private_.subscribe<voxblox_msgs::Layer>(
      "skeleton_layer_in", 1, [&](const voxblox_msgs::Layer::ConstPtr &msg) {
        // set esdf layer which also initializes the skeleton layer
        skeleton_generator_->setEsdfLayer(
            esdf_server_.getEsdfMapPtr()
                ->getEsdfLayerPtr()); // also initializes the skeleton layer

        ROS_INFO_ONCE("Got a skeleton layer from ROS topic!");

        // note: we could also just update the map here
        deserializeMsgToLayer(*msg, skeleton_generator_->getSkeletonLayer());
        skeleton_generator_->updateSkeletonFromLayer();
      });

  inspection_marker_sub_ =
      nh_private_.subscribe<visualization_msgs::InteractiveMarkerFeedback>(
          inspection_marker_topic, 1,
          [&](const visualization_msgs::InteractiveMarkerFeedback::ConstPtr
                  &msg) { inspectionMarkerFeedbackCallback(msg); });

  // skeleton generator
  skeleton_generator_->setEsdfLayer(
      esdf_server_.getEsdfMapPtr()
          ->getEsdfLayerPtr()); // also initializes the skeleton layer

  skeleton_generator_->setMinSeparationAngle(min_separation_angle_);
  skeleton_generator_->setGenerateByLayerNeighbors(
      generate_by_layer_neighbors_);
  skeleton_generator_->setNumNeighborsForEdge(num_neighbors_for_edge_);
  skeleton_generator_->setMinGvdDistance(min_gvd_distance_);

  // load bim
  if (!bim_filename.empty()) {
    ROS_INFO("Loading bim map '%s'", bim_filename.c_str());

    auto vs = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxel_size();
    bim_map_ = bim::parse_bim(bim_filename, vs);

    esdf_server_.clear();

    bim_layers_ = map_builder::generateTsdfLayer(
        bim_map_, *esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr());

    esdf_server_.updateEsdfBatch();
    esdf_server_.updateEsdfBatch();
    esdf_server_.updateEsdfBatch();
    esdf_server_.generateMesh();
  }

  // timers
  if (generation_interval_ > 0.0) {
    skeleton_generator_timer_ = nh_.createTimer(
        ros::Duration(generation_interval_ * 0.5),
        [&](const ros::TimerEvent &) {
          bool threaded_generation = true;
          if (threaded_generation) {
            if (!generating_skeleton_ && g_skeleton_generator_queue.empty()) {
              // copy the esdf layer
              auto esdf_layer = std::make_unique<Layer<EsdfVoxel>>(
                  esdf_server_.getEsdfMapPtr()->getEsdfLayer());

              ROS_INFO("Launching GVD threaded");
              generate_threaded(std::move(esdf_layer));
            } else {
              std::shared_ptr<SkeletonGenerator> skeleton_generator;
              if (g_skeleton_generator_queue.pop(skeleton_generator)) {
                generating_skeleton_ = false;
                skeleton_generator_ = skeleton_generator;
              }
            }
          } else {
            generate();
          }
        });
  } else {
    ROS_INFO("Skeleton generation disabled");
  }
}

void SkeletonServer::generate() {
  // start with a fresh skeleton
  ROS_INFO("About to generate skeleton graph");
  skeleton_generator_ = std::make_unique<SkeletonGenerator>();

  // apply user params
  skeleton_generator_->setMinSeparationAngle(min_separation_angle_);
  skeleton_generator_->setGenerateByLayerNeighbors(
      generate_by_layer_neighbors_);
  skeleton_generator_->setNumNeighborsForEdge(num_neighbors_for_edge_);
  skeleton_generator_->setMinGvdDistance(min_gvd_distance_);

  // set esdf layer
  if (!skeleton_generator_->getSkeletonLayer()) {
    skeleton_generator_->setEsdfLayer(
        esdf_server_.getEsdfMapPtr()
            ->getEsdfLayerPtr()); // also initializes the skeleton layer
  }
  // skeleton_generator_.setEsdfLayer(
  //     esdf_server_.getEsdfMapPtr()
  //         ->getEsdfLayerPtr()); // also initializes the skeleton layer

  // generate skeleton
  skeleton_generator_->generateSkeleton();

  // generate sparse graph
  skeleton_generator_->generateSparseGraph();
  // skeleton_generator_.reconnectSubgraphsAlongEsdf();

  // pub
  if (publish_data_) {
    publishData();
  }
  if (vis_data_) {
    publishVisuals();
  }
}

void SkeletonServer::generate_threaded(
    std::unique_ptr<Layer<EsdfVoxel>> esdf_layer) {
  auto skeleton_generator = std::make_shared<SkeletonGenerator>();

  // apply user params
  skeleton_generator->setMinSeparationAngle(min_separation_angle_);
  skeleton_generator->setGenerateByLayerNeighbors(generate_by_layer_neighbors_);
  skeleton_generator->setNumNeighborsForEdge(num_neighbors_for_edge_);
  skeleton_generator->setMinGvdDistance(min_gvd_distance_);

  generating_skeleton_ = true;

  auto thread = std::thread([skeleton_generator = std::move(skeleton_generator),
                             esdf_layer = std::move(esdf_layer)]() {
    // start with a fresh skeleton
    ROS_INFO("About to generate skeleton graph");

    // set esdf layer
    skeleton_generator->setEsdfLayer(esdf_layer.get());

    // generate skeleton
    skeleton_generator->generateSkeleton();

    // generate sparse graph
    skeleton_generator->generateSparseGraph();
    skeleton_generator->simplifyGraph();

    // skeleton_generator->reconnectSubgraphsAlongEsdf();

    // ship it
    g_skeleton_generator_queue.push(skeleton_generator);
  });
  thread.detach();

  // pub
  if (publish_data_) {
    publishData();
  }
  if (vis_data_) {
    publishVisuals();
  }
}

const SkeletonGenerator &SkeletonServer::getSkeletonGenerator() const {
  return *skeleton_generator_;
}

const EsdfServer &SkeletonServer::getEsdfServer() const { return esdf_server_; }

void SkeletonServer::publishData() const {

  sparse_graph_pub_.publish(skeleton_generator_->getSparseGraph().toGraphMsg());

  if (skeleton_generator_->getSkeletonLayer()) {
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<SkeletonVoxel>(*skeleton_generator_->getSkeletonLayer(),
                                       false, &layer_msg);
    skeleton_layer_pub_.publish(layer_msg);
  }
}

void SkeletonServer::publishVisuals() const {
  // skeleton pointcloud
  Pointcloud pointcloud;
  std::vector<float> distances;

  skeleton_generator_->getSkeleton().getEdgePointcloudWithDistances(&pointcloud,
                                                                    &distances);
  pcl::PointCloud<pcl::PointXYZI> ptcloud_pcl;
  pointcloudToPclXYZI(pointcloud, distances, &ptcloud_pcl);
  ptcloud_pcl.header.frame_id = world_frame_;
  skeleton_pc_vis_pub_.publish(ptcloud_pcl);

  // sparse graph markers
  const SparseSkeletonGraph &graph = skeleton_generator_->getSparseGraph();
  visualization_msgs::MarkerArray marker_array;
  visualizeSkeletonGraph(graph, world_frame_, &marker_array, false, false);
  sparse_graph_vis_pub_.publish(marker_array);

  // intersections
  if (bim_layers_.intersection_layer) {
    visualization_msgs::MarkerArray marker_array;
    visualizeIntersectionLayer(*bim_layers_.intersection_layer, &marker_array,
                               world_frame_);
    bim_intersection_layer_vis_pub_.publish(marker_array);
  }

  // freespace
  if (bim_layers_.intersection_layer) {
    visualization_msgs::MarkerArray marker_array;
    visualizeIntersectionLayer(*bim_layers_.freespace_layer, &marker_array,
                               world_frame_);
    bim_freespace_layer_vis_pub_.publish(marker_array);
  }

  // bim (walls, doors etc.)
  if (!bim_map_.empty()) {
    visualization_msgs::MarkerArray marker_array;
    visualizeBIM(bim_map_, &marker_array, world_frame_);
    bim_vis_pub_.publish(marker_array);
  }
}

const SparseSkeletonGraph &SkeletonServer::getSparseGraph() const {
  return skeleton_generator_->getSparseGraph();
}

void SkeletonServer::inspectionMarkerFeedbackCallback(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &feedback) {

  if (esdf_server_.getEsdfMapPtr()) {
    const auto &marker_pose = feedback->pose.position;
    const auto voxel_size = esdf_server_.getEsdfMapPtr()->voxel_size();
    auto z_offset = 2 * voxel_size;

    if (std::remainder(z_offset, voxel_size) < kFloatEpsilon) {
      z_offset += voxel_size / 2.0;
    }

    // set slice level
    esdf_server_.setSliceLevel(marker_pose.z - z_offset);

    // print esdf info
    auto esdf = esdf_server_.getEsdfMapPtr()->getEsdfLayerConstPtr();
    auto voxel = esdf->getVoxelPtrByCoordinates(
        {marker_pose.x, marker_pose.y, marker_pose.z});

    if (voxel) {
      voxblox::GlobalIndex index = (voxblox::Point{marker_pose.x, marker_pose.y,
                                                   marker_pose.z - z_offset} *
                                    esdf->voxel_size_inv())
                                       .cast<voxblox::LongIndexElement>();

      ROS_INFO("========================================");
      ROS_INFO("ESDF Voxel INFO [%i,%i,%i]:", index.x(), index.y(), index.z());
      ROS_INFO("distance: %f", voxel->distance);
      ROS_INFO("fixed: %i", voxel->fixed);
      ROS_INFO("observed: %i", voxel->observed);
      ROS_INFO("hallucinated: %i", voxel->hallucinated);
      ROS_INFO("parent: [%i,%i,%i]", voxel->parent.x(), voxel->parent.y(),
               voxel->parent.z());
      ROS_INFO("========================================");
    }
  }
}

} // namespace voxblox