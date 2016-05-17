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
 * * Neither the name of copyright holder(s) nor the names of its
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

#include <holo_fingers/holo_fingers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

namespace holo_fingers
{
HoloFingers::HoloFingers(const std::string name_space)
{
    nh_ = boost::make_shared<ros::NodeHandle>(name_space);
    /* nh_->param<double>("cluster_tolerance", clus_tol_, 0.005); */
    /* nh_->param<int>("cluster_min_size", min_size_, 1000); */
    /* nh_->param<int>("cluster_max_size", max_size_, 10000); */
    nh_->param<std::string>("input_topic", topic_, "/pacman_vision/processed_scene");
    /* nh_->param<std::string>("reference_frame", frame_, "/qb_delta_base"); */
    sub_ = nh_->subscribe(nh_->resolveName(topic_), 1, &HoloFingers::cbCloud, this);
    srv_calib_ = nh_->advertiseService("calibrate", &HoloFingers::cbCalib, this);
    pub_ = nh_->advertise<visualization_msgs::MarkerArray>("distance_markers", 1);
    /* nh_->param<std::vector<double> >("translation", trasl, {0, 0, 0}); */
    /* nh_->param<std::vector<double> >("rotation", rot, {0, 0, 0, 1}); */
    marks_ = boost::make_shared<visualization_msgs::MarkerArray>();
}

void HoloFingers::publishMarkers()
{
    if(marks_ && cloud_)
        if (pub_.getNumSubscribers()>0)
        {
            for(size_t i=0; i<marks_->markers.size();++i)
            {
                marks_->markers[i].header.stamp = ros::Time::now();
                marks_->markers[i].header.frame_id = cloud_->header.frame_id;
            }
            pub_.publish(*marks_);
        }
}
void HoloFingers::spinOnce()
{
    ros::spinOnce();
    measure();
    createMarkers();
    publishMarkers();
}

void HoloFingers::createMarkers()
{
    if (!index_ || !thumb_)
        return;
    marks_ = boost::make_shared<visualization_msgs::MarkerArray>();
    visualization_msgs::Marker zaxis;
    zaxis.ns="Zaxis";
    zaxis.id=0;
    zaxis.type=visualization_msgs::Marker::LINE_STRIP;
    zaxis.action=visualization_msgs::Marker::ADD;
    zaxis.scale.x = 0.0005;
    zaxis.color.b=1;
    zaxis.color.a=1;
    geometry_msgs::Point pt;
    pt.x=0;
    pt.y=0;
    pt.z=0;
    zaxis.points.push_back(pt);
    pt.x=0;
    pt.y=0;
    pt.z=2;
    zaxis.points.push_back(pt);
    zaxis.lifetime=ros::Duration(1.0);
    marks_->markers.push_back(zaxis);
    visualization_msgs::Marker index;
    index.ns="index";
    index.id=0;
    index.type=visualization_msgs::Marker::POINTS;
    index.action=visualization_msgs::Marker::ADD;
    index.scale.x = 0.005;
    index.scale.y = 0.005;
    for (size_t i=0; i<index_->size(); ++i)
    {
        pt.x = index_->points[i].x;
        pt.y = index_->points[i].y;
        pt.z = index_->points[i].z;
        std_msgs::ColorRGBA cl;
        cl.r=0;
        cl.g=1;
        cl.b=0;
        cl.a=1;
        index.points.push_back(pt);
        index.colors.push_back(cl);
    }
    index.lifetime=ros::Duration(1.0);
    marks_->markers.push_back(index);
    visualization_msgs::Marker thumb;
    thumb.ns="thumb";
    thumb.id=0;
    thumb.type=visualization_msgs::Marker::POINTS;
    thumb.action=visualization_msgs::Marker::ADD;
    thumb.scale.x = 0.005;
    thumb.scale.y = 0.005;
    for (size_t i=0; i<thumb_->size(); ++i)
    {
        pt.x = thumb_->points[i].x;
        pt.y = thumb_->points[i].y;
        pt.z = thumb_->points[i].z;
        std_msgs::ColorRGBA cl;
        cl.r=1;
        cl.a=1;
        cl.g=0;
        cl.b=0;
        thumb.points.push_back(pt);
        thumb.colors.push_back(cl);
    }
    thumb.lifetime=ros::Duration(1.0);
    marks_->markers.push_back(thumb);
}

void  HoloFingers::segment()
{
    if(!cloud_)
        return;
    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*cloud_,min,max);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fingers = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PassThrough<pcl::PointXYZRGB> pass(true);
    index_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    thumb_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    //segment two finger tips for now (index,thumb)
    for(size_t i=0; i<2; ++i)
    {
        pass.setFilterFieldName("y");
        pass.setFilterLimits(max[1]-0.025, max[1]);
        pass.setInputCloud(cloud_);
        pass.filter(*tmp);
        pcl::getMinMax3D(*tmp,min,max);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min[2], min[2]+0.02);
        pass.setInputCloud(tmp);
        pass.filter(*fingers);
        pcl::getMinMax3D(*fingers,min,max);
        pass.setFilterFieldName("x");
        if (i==0){
            pass.setFilterLimits(min[0], min[0]+0.025);
            pass.setInputCloud(fingers);
            pass.filter(*index_);
            pcl::getMinMax3D(*index_,min,max);
            pcl::CropBox<pcl::PointXYZRGB> cb;
            cb.setMin(min);
            cb.setMax(max);
            cb.setInputCloud(cloud_);
            cb.setNegative(true);
            cb.filter(*fingers);
            cloud_ = fingers;
            continue;
        }
        else if (i==1){
            pass.setFilterLimits(max[0]-0.025, max[0]);
            pass.setInputCloud(fingers);
            pass.filter(*thumb_);
        }
        else{
            //Error
        }
    }
}

void HoloFingers::measure()
{
    /* Eigen::Vector4f min, max; */
    /* pcl::getMinMax3D(*cloud_,min,max); */
    /* pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(); */
    /* pcl::PointCloud<pcl::PointXYZRGB>::Ptr fingers = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(); */
    /* pcl::PassThrough<pcl::PointXYZRGB> pass; */
    /* pass.setFilterFieldName("y"); */
    /* pass.setFilterLimits(max[1]-0.025, max[1]); */
    /* pass.setInputCloud(cloud_); */
    /* pass.filter(*tmp); */
    /* pcl::getMinMax3D(*tmp,min,max); */
    /* pass.setFilterFieldName("z"); */
    /* pass.setFilterLimits(min[2], min[2]+0.02); */
    /* pass.setInputCloud(tmp); */
    /* pass.filter(*fingers); */
    /* pcl::getMinMax3D(*fingers,min,max); */
    /* index_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(); */
    /* thumb_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(); */
    /* pass.setFilterFieldName("x"); */
    /* pass.setFilterLimits(min[0], min[0]+0.02); */
    /* pass.setInputCloud(fingers); */
    /* pass.filter(*index_); */
    /* pass.setFilterLimits(max[0]-0.02, max[0]); */
    /* pass.setInputCloud(fingers); */
    /* pass.filter(*thumb_); */
}

void HoloFingers::cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    try
    {
        cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg (*msg, *cloud_);
    }
    catch (...)
    {
        ROS_ERROR("[HoloFingers::%s]\tError getting cloud from subscribed topic: %s",__func__,topic_.c_str());
        return;
    }
}

bool HoloFingers::cbCalib(holo_fingers::calibrate::Request &req, holo_fingers::calibrate::Response &res)
{
    /* tf::StampedTransform mark_cam, mark_delta; */
    /* mark_delta.setOrigin(tf::Vector3(0,0,0.5075)); */
    /* mark_delta.setRotation(tf::Quaternion(1,0,0,0)); */
    /* try */
    /* { */
    /*     listener_.waitForTransform("/camera_rgb_optical_frame", "/ar_marker_50", ros::Time(0), ros::Duration(5)); */
    /*     listener_.lookupTransform("/camera_rgb_optical_frame", "/ar_marker_50", ros::Time(0), mark_cam); */
    /* } */
    /* catch ( tf::TransformException ex) */
    /* { */
    /*     return false; */
    /* } */
    /* delta_trans_ = mark_cam*mark_delta.inverse(); */
    /* //Save to yaml */
    /* std::string path = ros::package::getPath("holo_fingers"); */
    /* std::string file = path + "/config/calibration.yaml"; */
    /* std::ofstream f; */
    /* f.open(file.c_str()); */
    /* if (f.is_open()) */
    /* { */
    /*   f << "# Results are written in the form that they can be directly sent to a static_transform_publisher node" << std::endl; */
    /*   f << "# Recall its usage use static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period(milliseconds)" << std::endl; */
    /*   f << "translation: [" << delta_trans_.getOrigin()[0] << ", " << */
    /*                       delta_trans_.getOrigin()[1] << ", " << */
    /*                       delta_trans_.getOrigin()[2]<<"]" << std::endl; */
    /*   f << "rotation: [" << delta_trans_.getRotation().getX() << ", " << */
    /*                         delta_trans_.getRotation().getY() << ", " << */
    /*                         delta_trans_.getRotation().getZ() << ", " << */
    /*                         delta_trans_.getRotation().getW() << "]" << std::endl; */
    /*   f << "frame_id: " << "camera_rgb_optical_frame" << std::endl; */
    /*   f << "child_frame_id: " << "qb_delta_base" << std::endl; */
    /*   f.close(); */
    /* } */
/* return true; */
}

} //End namespace