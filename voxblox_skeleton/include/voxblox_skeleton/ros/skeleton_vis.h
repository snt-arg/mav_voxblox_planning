#ifndef VOXBLOX_SKELETON_ROS_SKELETON_VIS_H_
#define VOXBLOX_SKELETON_ROS_SKELETON_VIS_H_

#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <voxblox/core/common.h>
#include <voxblox/core/color.h>
#include <voxblox_ros/conversions.h>

#include "voxblox_skeleton/skeleton.h"

namespace voxblox {

struct connected_vertices_struct { 
public:
  int64_t id;
  bool visited;
  SkeletonVertex vertex;
  int64_t subgraph_id;
};

inline void connected_components(int64_t subgraph_id, const SparseSkeletonGraph& graph, connected_vertices_struct& connected_vertex, std::vector<connected_vertices_struct>& connected_vertices_struct_vec, std::vector<int64_t> closed_space_edge_ids) {
  std::vector<int64_t> edge_list = connected_vertex.vertex.edge_list;
  //std::cout << "connected_vertex.vertex.vertex_id " << connected_vertex.vertex.vertex_id << std::endl;
  for(int64_t edge_id : edge_list) {
    auto it_edge = std::find(closed_space_edge_ids.begin(), closed_space_edge_ids.end(), edge_id);
    //edge exists 
    if(it_edge != closed_space_edge_ids.end()) {
      const SkeletonEdge& connected_edge = graph.getEdge(edge_id);

      if(connected_edge.start_vertex != connected_vertex.vertex.vertex_id) {
        //this is the vertex neighbour        
        auto start_vertex = std::find_if(connected_vertices_struct_vec.begin(), connected_vertices_struct_vec.end(), boost::bind(&connected_vertices_struct::id, _1) == connected_edge.start_vertex);
        if(start_vertex != connected_vertices_struct_vec.end() && (*start_vertex).visited == false) {
          (*start_vertex).visited = true;
          (*start_vertex).subgraph_id = subgraph_id;
          connected_components(subgraph_id, graph, (*start_vertex), connected_vertices_struct_vec, closed_space_edge_ids);
        }
        else 
         continue;  
      }  
      else if (connected_edge.end_vertex != connected_vertex.vertex.vertex_id) {  
        //this is the vertex neighbour
        auto end_vertex = std::find_if(connected_vertices_struct_vec.begin(), connected_vertices_struct_vec.end(), boost::bind(&connected_vertices_struct::id, _1) == connected_edge.end_vertex);
        if(end_vertex != connected_vertices_struct_vec.end() && (*end_vertex).visited == false) {
          (*end_vertex).visited = true;
          (*end_vertex).subgraph_id = subgraph_id;
          connected_components(subgraph_id, graph, (*end_vertex), connected_vertices_struct_vec, closed_space_edge_ids);
        }
        else 
          continue;  
      }
    }
  }
  return; 
}

inline void visualizeSkeletonGraph(
    const SparseSkeletonGraph& graph, const std::string& frame_id,
    visualization_msgs::MarkerArray* marker_array) {
  bool visualize_subgraphs = true;
  bool visualize_freespace = false;
  CHECK_NOTNULL(marker_array);
  // Get a list of all vertices and visualize them as spheres.
  std::vector<int64_t> vertex_ids;
  graph.getAllVertexIds(&vertex_ids);
  std::vector<int64_t> deleted_vertex_ids;
  std::vector<connected_vertices_struct> connected_vertices_struct_vec;

  visualization_msgs::Marker closed_space_marker;
  visualization_msgs::Marker vertex_marker;
  vertex_marker.points.reserve(vertex_ids.size());
  // Also create the free-space marker.
  visualization_msgs::Marker vertex_free_space_marker;

  vertex_marker.header.frame_id = frame_id;
  vertex_marker.ns = "vertices";
  closed_space_marker.header.frame_id = frame_id;
  closed_space_marker.ns = "closed_spaces";

  if (!visualize_subgraphs) {
    vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    closed_space_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  } else {
    vertex_marker.type = visualization_msgs::Marker::CUBE_LIST;
    closed_space_marker.type = visualization_msgs::Marker::CUBE_LIST;
  }

  vertex_marker.pose.orientation.w = 1.0;
  vertex_marker.scale.x = 0.2;
  vertex_marker.scale.y = vertex_marker.scale.x;
  vertex_marker.scale.z = vertex_marker.scale.x;
  vertex_marker.color.r = 1.0;
  vertex_marker.color.a = 1.0;

  closed_space_marker.pose.orientation.w = 1.0;
  closed_space_marker.scale.x = 0.2;
  closed_space_marker.scale.y = closed_space_marker.scale.x;
  closed_space_marker.scale.z = closed_space_marker.scale.x;
  closed_space_marker.color.r = 1.0;
  closed_space_marker.color.a = 1.0;

  vertex_free_space_marker.header.frame_id = frame_id;
  vertex_free_space_marker.type = visualization_msgs::Marker::SPHERE;
  vertex_free_space_marker.ns = "vertex_space";
  vertex_free_space_marker.color.a = 0.2;
  vertex_free_space_marker.color.r = 1.0;
  vertex_free_space_marker.color.g = 1.0;

  for (int64_t vertex_id : vertex_ids) {
    geometry_msgs::Point point_msg;
    const SkeletonVertex& vertex = graph.getVertex(vertex_id);
    tf::pointEigenToMsg(vertex.point.cast<double>(), point_msg);
    vertex_marker.points.push_back(point_msg);

    connected_vertices_struct connected_vertex;
    if (vertex.distance > 0.8) {
      
      closed_space_marker.points.push_back(point_msg);
      connected_vertex.id = vertex_id;
      connected_vertex.visited = false;
      connected_vertex.vertex = vertex;
      connected_vertices_struct_vec.push_back(connected_vertex);
    }

    if (visualize_subgraphs) {
      std_msgs::ColorRGBA color_msg;
      Color color = rainbowColorMap(vertex.subgraph_id % 100 / 10.0);
      colorVoxbloxToMsg(color, &color_msg);
      if(vertex.distance > 0.8) {
        vertex_marker.colors.push_back(color_msg);
        closed_space_marker.colors.push_back(color_msg);
      }
      else {
        color_msg.b = 0; color_msg.r = 0; color_msg.g = 0;
        vertex_marker.colors.push_back(color_msg);
        deleted_vertex_ids.push_back(vertex_id);
      }
    }

    if (visualize_freespace) {
      //std::cout << "vertex distance: " << vertex.distance << std::endl;
      if (vertex.distance > 0.8) {
        vertex_free_space_marker.pose.orientation.w = 1.0;
        vertex_free_space_marker.pose.position = point_msg;
        vertex_free_space_marker.scale.x = vertex.distance;
        vertex_free_space_marker.scale.y = vertex.distance;
        vertex_free_space_marker.scale.z = vertex.distance;
        marker_array->markers.push_back(vertex_free_space_marker);
        vertex_free_space_marker.id++;
      }
    }
  }
  marker_array->markers.push_back(closed_space_marker);
  marker_array->markers.push_back(vertex_marker);

  // Get all edges and visualize as lines.
  std::vector<int64_t> edge_ids;
  std::vector<int64_t> closed_space_edge_ids;
  graph.getAllEdgeIds(&edge_ids);

  visualization_msgs::Marker edge_marker;
  edge_marker.points.reserve(edge_ids.size() * 2);

  edge_marker.header.frame_id = frame_id;
  edge_marker.ns = "edges";
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.pose.orientation.w = 1.0;
  edge_marker.scale.x = 0.1;

  edge_marker.scale.y = edge_marker.scale.x;
  edge_marker.scale.z = edge_marker.scale.x;
  edge_marker.color.b = 1.0;
  edge_marker.color.a = 1.0;

  for (int64_t edge_id : edge_ids) {
    geometry_msgs::Point point_msg;
    const SkeletonEdge& edge = graph.getEdge(edge_id);
    int64_t start_vertex_id = edge.start_vertex;
    int64_t end_vertex_id = edge.end_vertex;
    
    auto it_start = std::find(deleted_vertex_ids.begin(), deleted_vertex_ids.end(), start_vertex_id);
    auto it_end = std::find(deleted_vertex_ids.begin(), deleted_vertex_ids.end(), end_vertex_id);

    if(it_start != deleted_vertex_ids.end() || it_end != deleted_vertex_ids.end()) 
      continue;

    closed_space_edge_ids.push_back(edge_id);
    tf::pointEigenToMsg(edge.start_point.cast<double>(), point_msg);
    edge_marker.points.push_back(point_msg);
    tf::pointEigenToMsg(edge.end_point.cast<double>(), point_msg);
    edge_marker.points.push_back(point_msg);
  }
  marker_array->markers.push_back(edge_marker);

  int64_t subgraph_id = 0;
  for(connected_vertices_struct connected_vertex : connected_vertices_struct_vec) {
    if(connected_vertex.visited == false) {
      connected_vertex.visited = true;
      connected_vertex.subgraph_id = subgraph_id;
      connected_components(subgraph_id, graph, connected_vertex, connected_vertices_struct_vec, closed_space_edge_ids);
      subgraph_id++;      
    }    
  }
  std::cout << "subgraph id: " << subgraph_id << std::endl; 
 }
}  // namespace voxblox

#endif  // VOXBLOX_SKELETON_ROS_SKELETON_VIS_H_
