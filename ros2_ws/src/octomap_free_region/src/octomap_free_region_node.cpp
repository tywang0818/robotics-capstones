// Post-process octomap_server's binary OctoMap output: for every cell
// inside a configured cylindrical region that is currently UNKNOWN
// (i.e. has no leaf in the tree), insert a FREE leaf. Republish the
// augmented tree on the output topic.
//
// The intent is to mark regions that are known-empty-by-design but
// physically unobservable by the sensor as FREE — so downstream
// consumers (motion planner, frontier viz) do not treat them as
// unknown / unsafe.
//
// Wire-up (in the launch file):
//   octomap_server publishes to /octomap_binary_raw
//     -> this node subscribes to /octomap_binary_raw, augments, publishes to /octomap_binary
//     -> everyone else keeps subscribing to /octomap_binary unchanged.
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <map>

#include "octomap/OcTree.h"
#include "octomap/octomap_types.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace {

// Height -> RGBA (octomap-style rainbow by z). h in [0,1].
inline std_msgs::msg::ColorRGBA height_color(double h)
{
  h = std::min(1.0, std::max(0.0, h));
  // classic octomap height color map (6-segment HSV-ish rainbow)
  double s = 1.0, v = 1.0;
  double i = std::floor(h * 6.0);
  double f = h * 6.0 - i;
  double m = v * (1.0 - s);
  double n = v * (1.0 - s * f);
  double k = v * (1.0 - s * (1.0 - f));
  double r = 0, g = 0, b = 0;
  switch (static_cast<int>(i) % 6) {
    case 0: r = v; g = k; b = m; break;
    case 1: r = n; g = v; b = m; break;
    case 2: r = m; g = v; b = k; break;
    case 3: r = m; g = n; b = v; break;
    case 4: r = k; g = m; b = v; break;
    case 5: r = v; g = m; b = n; break;
  }
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(r); c.g = static_cast<float>(g);
  c.b = static_cast<float>(b); c.a = 1.0f;
  return c;
}

// Add a FREE leaf at (x, y, z) if the cell there is currently UNKNOWN
// (i.e. no leaf covers it). If a leaf already exists (FREE or OCC),
// leave it alone -- real observations always win over the static mask.
inline void add_free_if_unknown(octomap::OcTree & tree, double x, double y, double z)
{
  octomap::point3d p(static_cast<float>(x),
                     static_cast<float>(y),
                     static_cast<float>(z));
  if (tree.search(p) == nullptr) {
    // updateNode(occupied=false) creates a FREE leaf at max depth.
    // lazy_eval=false so pruning happens immediately.
    tree.updateNode(p, false, false);
  }
}

// Add an OCCUPIED leaf at (x, y, z) if the cell is currently UNKNOWN.
// Used to seal the floor under the arm base: the ground directly beneath
// the base is occluded (never observed) -> unknown; if a FREE cylinder
// bottoms out on it, that unknown floor becomes a free-unknown frontier
// pointing DOWN into the ground (high unknown-density, forever
// unobservable). Filling it OCCUPIED makes the free column bottom on a
// known floor -> no below-floor frontier. Real observations still win.
inline void add_occ_if_unknown(octomap::OcTree & tree, double x, double y, double z)
{
  octomap::point3d p(static_cast<float>(x),
                     static_cast<float>(y),
                     static_cast<float>(z));
  if (tree.search(p) == nullptr) {
    tree.updateNode(p, true, false);   // occupied=true -> OCCUPIED leaf
  }
}

}  // namespace


class OctomapFreeRegionNode : public rclcpp::Node
{
public:
  OctomapFreeRegionNode()
  : Node("octomap_free_region")
  {
    declare_parameter<std::string>("input_topic",  "/octomap_binary_raw");
    declare_parameter<std::string>("output_topic", "/octomap_binary");
    // Multiple cylinder regions to inject FREE into. Parallel arrays
    // -- ROS 2 params don't support arrays of structs, but a set of
    // parallel double-arrays does the job. Each cylinder i is defined
    // by (cylinder_x[i], cylinder_y[i], cylinder_r[i],
    //     cylinder_z_min[i], cylinder_z_max[i]).
    declare_parameter<std::vector<double>>(
      "cylinder_x",     std::vector<double>{0.0});
    declare_parameter<std::vector<double>>(
      "cylinder_y",     std::vector<double>{0.0});
    declare_parameter<std::vector<double>>(
      "cylinder_r",     std::vector<double>{0.30});
    declare_parameter<std::vector<double>>(
      "cylinder_z_min", std::vector<double>{0.60});
    declare_parameter<std::vector<double>>(
      "cylinder_z_max", std::vector<double>{1.25});
    // Parallel arrays for OCCUPIED disks (floor seal). Same schema as the
    // free cylinders; empty by default (opt-in). Cells inside these regions
    // that are currently UNKNOWN become OCCUPIED.
    declare_parameter<std::vector<double>>("occ_cylinder_x",     std::vector<double>{});
    declare_parameter<std::vector<double>>("occ_cylinder_y",     std::vector<double>{});
    declare_parameter<std::vector<double>>("occ_cylinder_r",     std::vector<double>{});
    declare_parameter<std::vector<double>>("occ_cylinder_z_min", std::vector<double>{});
    declare_parameter<std::vector<double>>("occ_cylinder_z_max", std::vector<double>{});
    declare_parameter<double>("sample_spacing", 0.025);
    declare_parameter<bool>  ("enable",         true);

    input_topic_  = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    const auto xs   = get_parameter("cylinder_x").as_double_array();
    const auto ys   = get_parameter("cylinder_y").as_double_array();
    const auto rs   = get_parameter("cylinder_r").as_double_array();
    const auto zmns = get_parameter("cylinder_z_min").as_double_array();
    const auto zmxs = get_parameter("cylinder_z_max").as_double_array();
    spacing_      = get_parameter("sample_spacing").as_double();
    enabled_      = get_parameter("enable").as_bool();

    const size_t n = xs.size();
    if (n == 0 || ys.size() != n || rs.size() != n
        || zmns.size() != n || zmxs.size() != n)
    {
      RCLCPP_ERROR(get_logger(),
        "cylinder_{x,y,r,z_min,z_max} must be non-empty and equal length; "
        "got x=%zu y=%zu r=%zu zmin=%zu zmax=%zu",
        xs.size(), ys.size(), rs.size(), zmns.size(), zmxs.size());
      throw std::runtime_error("cylinder parameter arrays malformed");
    }
    cylinders_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      cylinders_.push_back({xs[i], ys[i], rs[i], zmns[i], zmxs[i]});
    }

    // OCCUPIED disks (optional).
    const auto oxs   = get_parameter("occ_cylinder_x").as_double_array();
    const auto oys   = get_parameter("occ_cylinder_y").as_double_array();
    const auto ors   = get_parameter("occ_cylinder_r").as_double_array();
    const auto ozmns = get_parameter("occ_cylinder_z_min").as_double_array();
    const auto ozmxs = get_parameter("occ_cylinder_z_max").as_double_array();
    const size_t m = oxs.size();
    if (oys.size() != m || ors.size() != m || ozmns.size() != m || ozmxs.size() != m) {
      RCLCPP_ERROR(get_logger(),
        "occ_cylinder_{x,y,r,z_min,z_max} must be equal length; "
        "got x=%zu y=%zu r=%zu zmin=%zu zmax=%zu",
        oxs.size(), oys.size(), ors.size(), ozmns.size(), ozmxs.size());
      throw std::runtime_error("occ_cylinder parameter arrays malformed");
    }
    occ_cylinders_.reserve(m);
    for (size_t i = 0; i < m; ++i) {
      occ_cylinders_.push_back({oxs[i], oys[i], ors[i], ozmns[i], ozmxs[i]});
    }

    declare_parameter<std::string>("occ_vis_topic", "/octomap_sealed_occupied");
    occ_vis_topic_ = get_parameter("occ_vis_topic").as_string();

    pub_ = create_publisher<octomap_msgs::msg::Octomap>(output_topic_, 10);
    occ_vis_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      occ_vis_topic_, 10);
    sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      input_topic_, 10,
      std::bind(&OctomapFreeRegionNode::on_octomap, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "octomap_free_region: %s -> %s | %zu cylinder(s) spacing=%.3f "
      "enable=%s", input_topic_.c_str(), output_topic_.c_str(),
      cylinders_.size(), spacing_, enabled_ ? "true" : "false");
    for (size_t i = 0; i < cylinders_.size(); ++i) {
      const auto & c = cylinders_[i];
      RCLCPP_INFO(get_logger(),
        "  FREE cylinder %zu: x=%.2f y=%.2f r=%.2f z=[%.2f, %.2f]",
        i, c.x, c.y, c.r, c.z_min, c.z_max);
    }
    for (size_t i = 0; i < occ_cylinders_.size(); ++i) {
      const auto & c = occ_cylinders_[i];
      RCLCPP_INFO(get_logger(),
        "  OCC(floor-seal) disk %zu: x=%.2f y=%.2f r=%.2f z=[%.2f, %.2f]",
        i, c.x, c.y, c.r, c.z_min, c.z_max);
    }
  }

private:
  void on_octomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    if (!enabled_) {
      pub_->publish(*msg);
      return;
    }

    // Deserialize the incoming binary tree.
    auto * abstract = octomap_msgs::binaryMsgToMap(*msg);
    if (abstract == nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "octomap_free_region: binaryMsgToMap returned null; passing through.");
      pub_->publish(*msg);
      return;
    }
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract);
    if (tree == nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "octomap_free_region: not an OcTree; passing through.");
      delete abstract;
      pub_->publish(*msg);
      return;
    }

    // Walk each cylinder in a regular grid; insert FREE for cells
    // that are currently unknown. Real observations (FREE/OCC leaves
    // already in the tree) are left alone.
    int n_added = 0;
    for (const auto & c : cylinders_) {
      const double r_sq = c.r * c.r;
      for (double z = c.z_min; z <= c.z_max + 1e-9; z += spacing_) {
        for (double y = c.y - c.r; y <= c.y + c.r + 1e-9; y += spacing_) {
          const double dy = y - c.y;
          for (double x = c.x - c.r; x <= c.x + c.r + 1e-9; x += spacing_) {
            const double dx = x - c.x;
            if (dx * dx + dy * dy > r_sq) {continue;}
            add_free_if_unknown(*tree, x, y, z);
            ++n_added;   // approximate; add_free_if_unknown no-ops on known
          }
        }
      }
    }
    // OCCUPIED floor-seal disks: fill unknown cells OCCUPIED. Run AFTER the
    // free pass so the free column above wins; the disk seals the unknown
    // floor beneath it -> no free-unknown frontier punched through the floor.
    int n_occ = 0;
    for (const auto & c : occ_cylinders_) {
      const double r_sq = c.r * c.r;
      for (double z = c.z_min; z <= c.z_max + 1e-9; z += spacing_) {
        for (double y = c.y - c.r; y <= c.y + c.r + 1e-9; y += spacing_) {
          const double dy = y - c.y;
          for (double x = c.x - c.r; x <= c.x + c.r + 1e-9; x += spacing_) {
            const double dx = x - c.x;
            if (dx * dx + dy * dy > r_sq) {continue;}
            octomap::point3d p(static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(z));
            if (tree->search(p) == nullptr) {
              add_occ_if_unknown(*tree, x, y, z);
              ++n_occ;
            }
          }
        }
      }
    }
    tree->updateInnerOccupancy();

    // Publish an OCCUPIED-cells MarkerArray built from THIS sealed tree, so
    // RViz shows exactly the octomap used for decisions (octomap_server's
    // /occupied_cells_vis_array is from the raw pre-seal tree).
    publish_occupied_markers(*tree, msg->header);

    // Serialize back and publish. Preserve the original header stamp
    // and frame so downstream consumers see a coherent stream.
    octomap_msgs::msg::Octomap out;
    if (!octomap_msgs::binaryMapToMsg(*tree, out)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "octomap_free_region: binaryMapToMsg failed; passing through.");
      delete tree;
      pub_->publish(*msg);
      return;
    }
    out.header = msg->header;
    pub_->publish(out);

    if ((n_added > 0 || n_occ > 0) && msg_count_ < 3) {
      RCLCPP_INFO(get_logger(),
        "injection: msg #%lu, FREE cells swept=%d, OCC floor-seal added=%d.",
        msg_count_, n_added, n_occ);
    }
    ++msg_count_;
    delete tree;
  }

  // Build & publish occupied cells of `tree` as a height-colored CUBE_LIST
  // MarkerArray (one marker per leaf-size bucket, since a pruned octree has
  // leaves at several depths). DELETEALL first so stale cubes never linger.
  void publish_occupied_markers(octomap::OcTree & tree,
                                const std_msgs::msg::Header & header)
  {
    std::map<double, std::vector<octomap::point3d>> by_size;
    double zmin = 1e9, zmax = -1e9;
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
      if (!tree.isNodeOccupied(*it)) {continue;}
      const double s = it.getSize();
      const octomap::point3d c = it.getCoordinate();
      by_size[s].push_back(c);
      if (c.z() < zmin) {zmin = c.z();}
      if (c.z() > zmax) {zmax = c.z();}
    }
    const double zr = std::max(1e-3, zmax - zmin);

    visualization_msgs::msg::MarkerArray arr;
    // Clear previous frame's cubes.
    visualization_msgs::msg::Marker del;
    del.header = header; del.ns = "sealed_occupied";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int id = 0;
    for (const auto & kv : by_size) {
      const double size = kv.first;
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns = "sealed_occupied";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = m.scale.y = m.scale.z = size;
      m.pose.orientation.w = 1.0;
      m.points.reserve(kv.second.size());
      m.colors.reserve(kv.second.size());
      for (const auto & p : kv.second) {
        geometry_msgs::msg::Point gp;
        gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
        m.points.push_back(gp);
        m.colors.push_back(height_color((p.z() - zmin) / zr));
      }
      arr.markers.push_back(m);
    }
    occ_vis_pub_->publish(arr);
  }

  struct Cylinder { double x, y, r, z_min, z_max; };

  std::string input_topic_;
  std::string output_topic_;
  std::string occ_vis_topic_;
  std::vector<Cylinder> cylinders_;
  std::vector<Cylinder> occ_cylinders_;
  double spacing_;
  bool enabled_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occ_vis_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  uint64_t msg_count_ {0};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapFreeRegionNode>());
  rclcpp::shutdown();
  return 0;
}
