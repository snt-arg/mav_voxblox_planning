#include "voxblox_skeleton/skeleton.h"

namespace voxblox {

Skeleton::Skeleton() {}

void Skeleton::getPointcloud(Pointcloud *pointcloud) const {
  CHECK_NOTNULL(pointcloud);
  pointcloud->clear();

  pointcloud->reserve(edges_.size());

  for (const SkeletonPoint &point : edges_) {
    pointcloud->push_back(point.point);
  }
}

void Skeleton::getPointcloudWithDistances(Pointcloud *pointcloud,
                                          std::vector<float> *distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(points_.size());
  distances->reserve(points_.size());

  for (const SkeletonPoint &point : points_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

void Skeleton::getEdgePointcloudWithDistances(
    Pointcloud *pointcloud, std::vector<float> *distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(edges_.size());
  distances->reserve(edges_.size());

  for (const SkeletonPoint &point : edges_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

void Skeleton::getVertexPointcloudWithDistances(
    Pointcloud *pointcloud, std::vector<float> *distances) const {
  CHECK_NOTNULL(pointcloud);
  CHECK_NOTNULL(distances);
  pointcloud->clear();
  distances->clear();

  pointcloud->reserve(vertices_.size());
  distances->reserve(vertices_.size());

  for (const SkeletonPoint &point : vertices_) {
    pointcloud->push_back(point.point);
    distances->push_back(point.distance);
  }
}

SparseSkeletonGraph::SparseSkeletonGraph()
    : next_vertex_id_(0), next_edge_id_(0) {}

int64_t SparseSkeletonGraph::addVertex(const SkeletonVertex &vertex) {
  int64_t vertex_id = next_vertex_id_++;
  // Returns an iterator...
  auto iter_pair = vertex_map_.emplace(std::make_pair(vertex_id, vertex));
  iter_pair.first->second.vertex_id = vertex_id;
  return vertex_id;
}

int64_t SparseSkeletonGraph::addEdge(const SkeletonEdge &edge) {
  int64_t edge_id = next_edge_id_++;
  // Returns an iterator...
  auto iter_pair = edge_map_.emplace(std::make_pair(edge_id, edge));
  iter_pair.first->second.edge_id = edge_id;

  // Now hook it up to the vertices.
  SkeletonVertex &start_vertex = vertex_map_[edge.start_vertex];
  SkeletonVertex &end_vertex = vertex_map_[edge.end_vertex];

  start_vertex.edge_list.push_back(edge_id);
  end_vertex.edge_list.push_back(edge_id);

  iter_pair.first->second.start_point = start_vertex.point;
  iter_pair.first->second.end_point = end_vertex.point;

  return edge_id;
}

bool SparseSkeletonGraph::hasVertex(int64_t id) const {
  if (vertex_map_.find(id) != vertex_map_.end()) {
    return true;
  }
  return false;
}

bool SparseSkeletonGraph::hasEdge(int64_t id) const {
  if (edge_map_.find(id) != edge_map_.end()) {
    return true;
  }
  return false;
}

const SkeletonVertex &SparseSkeletonGraph::getVertex(int64_t id) const {
  return vertex_map_.find(id)->second;
}

const SkeletonEdge &SparseSkeletonGraph::getEdge(int64_t id) const {
  return edge_map_.find(id)->second;
}

SkeletonVertex &SparseSkeletonGraph::getVertex(int64_t id) {
  return vertex_map_.find(id)->second;
}

SkeletonEdge &SparseSkeletonGraph::getEdge(int64_t id) {
  return edge_map_.find(id)->second;
}

void SparseSkeletonGraph::clear() {
  next_vertex_id_ = 0;
  next_edge_id_ = 0;

  vertex_map_.clear();
  edge_map_.clear();
}

void SparseSkeletonGraph::getAllVertexIds(
    std::vector<int64_t> *vertex_ids) const {
  vertex_ids->reserve(vertex_map_.size());
  for (const std::pair<int64_t, SkeletonVertex> &kv : vertex_map_) {
    vertex_ids->push_back(kv.first);
  }
}

void SparseSkeletonGraph::getAllEdgeIds(std::vector<int64_t> *edge_ids) const {
  edge_ids->reserve(edge_map_.size());
  for (const std::pair<int64_t, SkeletonEdge> &kv : edge_map_) {
    edge_ids->push_back(kv.first);
  }
}

void SparseSkeletonGraph::removeVertex(int64_t vertex_id) {
  // Find the vertex.
  std::map<int64_t, SkeletonVertex>::iterator iter =
      vertex_map_.find(vertex_id);
  if (iter == vertex_map_.end()) {
    return;
  }
  const SkeletonVertex &vertex = iter->second;

  // Remove all edges that are connected to it.
  for (int64_t edge_id : vertex.edge_list) {
    removeEdge(edge_id);
  }

  // Remove the vertex.
  vertex_map_.erase(iter);
}

void SparseSkeletonGraph::removeEdge(int64_t edge_id) {
  // Find the edge.
  std::map<int64_t, SkeletonEdge>::iterator iter = edge_map_.find(edge_id);
  if (iter == edge_map_.end()) {
    return;
  }
  const SkeletonEdge &edge = iter->second;

  // Remove this edge from both vertices.
  SkeletonVertex &vertex_1 = getVertex(edge.start_vertex);
  SkeletonVertex &vertex_2 = getVertex(edge.end_vertex);

  for (size_t i = 0; i < vertex_1.edge_list.size(); i++) {
    if (vertex_1.edge_list[i] == edge_id) {
      vertex_1.edge_list.erase(vertex_1.edge_list.begin() + i);
      break;
    }
  }
  for (size_t i = 0; i < vertex_2.edge_list.size(); i++) {
    if (vertex_2.edge_list[i] == edge_id) {
      vertex_2.edge_list.erase(vertex_2.edge_list.begin() + i);
      break;
    }
  }

  edge_map_.erase(iter);
}

bool SparseSkeletonGraph::areVerticesDirectlyConnected(
    int64_t vertex_id_1, int64_t vertex_id_2) const {
  const SkeletonVertex &vertex_1 = getVertex(vertex_id_1);
  // Iterate over all edges of vertex 1.
  for (int64_t edge_id : vertex_1.edge_list) {
    // Check if any connect to vertex 2.
    const SkeletonEdge &edge = getEdge(edge_id);
    if (edge.start_vertex == vertex_id_2 || edge.end_vertex == vertex_id_2) {
      return true;
    }
  }

  return false;
}

void SparseSkeletonGraph::addSerializedVertex(const SkeletonVertex &vertex) {
  vertex_map_[vertex.vertex_id] = vertex;
}
void SparseSkeletonGraph::addSerializedEdge(const SkeletonEdge &edge) {
  edge_map_[edge.edge_id] = edge;
}

voxblox_planning_msgs::SkeletonGraph SparseSkeletonGraph::toGraphMsg() const {
  voxblox_planning_msgs::SkeletonGraph graph_msg;
  graph_msg.edges.reserve(edge_map_.size());
  graph_msg.vertices.reserve(vertex_map_.size());

  for (const std::pair<int64_t, SkeletonEdge> &kv : edge_map_) {
    auto &edge = kv.second;

    voxblox_planning_msgs::SkeletonEdge edge_msg;
    edge_msg.edge_id = edge.edge_id;
    edge_msg.start_vertex = edge.start_vertex;
    edge_msg.end_vertex = edge.end_vertex;
    edge_msg.end_distance = edge.end_distance;
    edge_msg.end_point_x = edge.end_point.x();
    edge_msg.end_point_y = edge.end_point.y();
    edge_msg.end_point_z = edge.end_point.z();
    edge_msg.start_point_x = edge.start_point.x();
    edge_msg.start_point_y = edge.start_point.y();
    edge_msg.start_point_z = edge.start_point.z();
    edge_msg.start_distance = edge.start_distance;

    graph_msg.edges.push_back(edge_msg);
  }

  for (const std::pair<int64_t, SkeletonVertex> &kv : vertex_map_) {
    auto &vertex = kv.second;

    voxblox_planning_msgs::SkeletonVertex vertex_msg;
    vertex_msg.point_x = vertex.point.x();
    vertex_msg.point_y = vertex.point.y();
    vertex_msg.point_z = vertex.point.z();
    vertex_msg.subgraph_id = vertex.subgraph_id;
    vertex_msg.vertex_id = vertex.vertex_id;
    vertex_msg.edge_list = vertex.edge_list;
    vertex_msg.distance = vertex.distance;

    graph_msg.vertices.push_back(vertex_msg);
  }

  return graph_msg;
}

void SparseSkeletonGraph::setFromGraphMsg(
    const voxblox_planning_msgs::SkeletonGraphConstPtr &graph_msg) {

  // start with a clean graph
  // to which we add vertices and edges from the ros msg
  clear();

  for (auto &vertex_msg : graph_msg->vertices) {
    voxblox::SkeletonVertex vertex;
    vertex.vertex_id = vertex_msg.vertex_id;
    vertex.distance = vertex_msg.distance;
    vertex.point = {vertex_msg.point_x, vertex_msg.point_y, vertex_msg.point_z};
    vertex.subgraph_id = vertex_msg.subgraph_id;
    vertex.edge_list = vertex_msg.edge_list;

    addSerializedVertex(vertex);
  }

  for (auto &edge_msg : graph_msg->edges) {
    voxblox::SkeletonEdge edge;
    edge.edge_id = edge_msg.edge_id;
    edge.end_distance = edge_msg.end_distance;
    edge.end_point = {edge_msg.end_point_x, edge_msg.end_point_y,
                      edge_msg.end_point_z};
    edge.end_vertex = edge_msg.end_vertex;
    edge.start_distance = edge_msg.start_distance;
    edge.start_point = {edge_msg.start_point_x, edge_msg.start_point_y,
                        edge_msg.start_point_z};
    edge.start_vertex = edge_msg.start_vertex;

    addSerializedEdge(edge);
  }
}

} // namespace voxblox
