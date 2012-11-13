#include <pluginlib/class_list_macros.h>
#include "axes_display.h"
#include "camera_display.h"
#include "depth_cloud_display.h"
#include "grid_cells_display.h"
#include "grid_display.h"
#include "image_display.h"
#include "interactive_marker_display.h"
#include "laser_scan_display.h"
#include "map_display.h"
#include "marker_array_display.h"
#include "marker_display.h"
#include "odometry_display.h"
#include "path_display.h"
#include "point_cloud_display.h"
#include "point_cloud2_display.h"
#include "point_cloud_transformers.h"
#include "polygon_display.h"
#include "pose_array_display.h"
#include "pose_display.h"
#include "range_display.h"
#include "robot_model_display.h"
#include "tf_display.h"

PLUGINLIB_DECLARE_CLASS( rviz, Axes, rviz::AxesDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Camera, rviz::CameraDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, DepthCloud, rviz::DepthCloudDisplay, rviz::Display)
PLUGINLIB_DECLARE_CLASS( rviz, GridCells, rviz::GridCellsDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Grid, rviz::GridDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Image, rviz::ImageDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, InteractiveMarkers, rviz::InteractiveMarkerDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, LaserScan, rviz::LaserScanDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Map, rviz::MapDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, MarkerArray, rviz::MarkerArrayDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Marker, rviz::MarkerDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Odometry, rviz::OdometryDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Path, rviz::PathDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, PointCloud, rviz::PointCloudDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, PointCloud2, rviz::PointCloud2Display, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, AxisColor, rviz::AxisColorPCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz, FlatColor, rviz::FlatColorPCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz, Intensity, rviz::IntensityPCTransformer, rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz, RGB8,      rviz::RGB8PCTransformer,      rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz, RGBF32,    rviz::RGBF32PCTransformer,    rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz, XYZ,       rviz::XYZPCTransformer,       rviz::PointCloudTransformer )
PLUGINLIB_DECLARE_CLASS( rviz, Polygon, rviz::PolygonDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, PoseArray, rviz::PoseArrayDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Pose, rviz::PoseDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, Range, rviz::RangeDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, RobotModel, rviz::RobotModelDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( rviz, TF, rviz::TFDisplay, rviz::Display )
