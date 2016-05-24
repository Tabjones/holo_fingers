/*
 * Software License Agreement (BSD License)
 *
 *   Copyright (c) 2016, Federico Spinelli (fspinelli@gmail.com)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _INCL_HOLO_FINGERS_H_
#define _INCL_HOLO_FINGERS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/console.h>
#include <ros/common.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/rate.h>
#include <holo_fingers/calibrate.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace holo_fingers
{
    class HoloFingers
    {
        public:
        HoloFingers()=delete;
        HoloFingers(const std::string name_space);

        ///Get NodeHandle
        inline ros::NodeHandle getNodeHandle() const
        {
            return *nh_;
        }
        ///Main spin method
        void spinOnce();

        private:
        ///Callback to get input point cloud
        void cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
        bool cbCalib(holo_fingers::calibrate::Request &req, holo_fingers::calibrate::Response &res);
        void publishMarkers();
        void segment();
        void measure();
        void createMarkers();
        boost::shared_ptr<ros::NodeHandle> nh_;
        boost::shared_ptr<visualization_msgs::MarkerArray> marks_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::ServiceServer srv_calib_;
        tf::TransformBroadcaster brcaster_;
        tf::TransformListener listener_;

        std::string topic_, frame_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, index_, thumb_;
        pcl::PointXYZRGB pt_index_, pt_thumb_;
        float dist;
        //Params
        float pass;
    };
}
#endif //_INCL_holo_fingers_H_
