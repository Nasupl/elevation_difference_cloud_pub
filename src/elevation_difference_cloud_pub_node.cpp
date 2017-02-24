#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <sys/time.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

class AccumulateScanNode
{
    public:
        AccumulateScanNode();
        ~AccumulateScanNode();

    private:
        ros::Subscriber scan_sub_;
        ros::Publisher  accumulate_scan_cloud_pub_;

        std::string target_frame_id_, fixed_frame_id_;

        tf::TransformListener tf_;
        laser_geometry::LaserProjection projector_;

        std::vector<sensor_msgs::PointCloud2> cloud2_v_;
        int cloud2_v_size_;

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

        sensor_msgs::PointCloud2 getScanCloud(const sensor_msgs::LaserScan& scan);
};


AccumulateScanNode::AccumulateScanNode()
{
    ros::NodeHandle private_nh("~");
    private_nh.param("target_frame_id", target_frame_id_, std::string("/base_link"));
    private_nh.param("fixed_frame_id", fixed_frame_id_, std::string("/odom"));

    private_nh.param("cloud2_v_size", cloud2_v_size_, 50);

    ros::NodeHandle nh;
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &AccumulateScanNode::scanCallback, this);
    accumulate_scan_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/accumulate_scan_cloud", 100, false);
}

AccumulateScanNode::~AccumulateScanNode()
{
}

sensor_msgs::PointCloud2 AccumulateScanNode::getScanCloud(const sensor_msgs::LaserScan& scan)
{
    sensor_msgs::PointCloud2 scan_cloud2;
    projector_.projectLaser(scan, scan_cloud2);

    return scan_cloud2;
}

void AccumulateScanNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    static ros::Time last_cloud_pub(0,0);

    cloud2_v_.push_back(getScanCloud(*scan));

    sensor_msgs::PointCloud2 cloud2_msg;
    cloud2_msg.data.clear();
    for (int i=0; i < cloud2_v_.size(); i++) {
    tf::StampedTransform transform;
    try {
      tf_.waitForTransform(scan->header.frame_id,
                           scan->header.stamp,
                           cloud2_v_.at(i).header.frame_id,
                           cloud2_v_.at(i).header.stamp,
                           fixed_frame_id_,
                           ros::Duration(0.5));

      tf_.lookupTransform(target_frame_id_,
                          scan->header.stamp,
                          cloud2_v_.at(i).header.frame_id,
                          cloud2_v_.at(i).header.stamp,
                          fixed_frame_id_,
                          transform);
    } catch(tf::TransformException e) {
      last_cloud_pub = scan->header.stamp;
      cloud2_v_.clear();
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud2_v_.at(i), *pcl_cloud);
    pcl_ros::transformPointCloud(*pcl_cloud,
                                 *pcl_cloud,
                                 transform);
    sensor_msgs::PointCloud2 transformed_cloud2;
    toROSMsg (*pcl_cloud, transformed_cloud2);
    pcl::concatenatePointCloud(cloud2_msg, transformed_cloud2, cloud2_msg);
    }

    cloud2_msg.header.frame_id = target_frame_id_;
    cloud2_msg.header.stamp = scan->header.stamp;
    accumulate_scan_cloud_pub_.publish(cloud2_msg);

    if (cloud2_v_.size() > cloud2_v_size_) {
      cloud2_v_.erase(cloud2_v_.begin());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accumulate_scan_node");
    AccumulateScanNode asn;

    ros::spin();
    return 0;

}
