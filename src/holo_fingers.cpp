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
#include <pcl/search/kdtree.h>
#include <std_msgs/Float64.h>

namespace holo_fingers
{
HoloFingers::HoloFingers(const std::string name_space):mean_dist(0.), exp_dist(0.)
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
    pub_dist = nh_->advertise<std_msgs::Float64>("distance", 10);
    pub_mean = nh_->advertise<std_msgs::Float64>("mean_distance", 10);
    pub_exp = nh_->advertise<std_msgs::Float64>("exp_smoothing_distance", 10);
    /* nh_->param<std::vector<double> >("translation", trasl, {0, 0, 0}); */
    /* nh_->param<std::vector<double> >("rotation", rot, {0, 0, 0, 1}); */
    marks_ = boost::make_shared<visualization_msgs::MarkerArray>();
    nh_->param<float>("pass", pass, 0.015);
    nh_->param<float>("alpha", alpha, 0.3);
    nh_->param<int>("window", window, 30);
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
    if (!dist.empty()){
        std_msgs::Float64 d;
        d.data = dist.back();
        pub_dist.publish(d);
    }
    if (mean_dist != 0.0){
        std_msgs::Float64 dm;
        dm.data = mean_dist;
        pub_mean.publish(dm);
        mean_dist = 0.0;
    }
    if (exp_dist != 0.){
        std_msgs::Float64 de;
        de.data = exp_dist;
        pub_exp.publish(de);
    }
    cloud_.reset();
}
void HoloFingers::spinOnce()
{
    ros::spinOnce();
    segment();
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
    visualization_msgs::Marker dist_line;
    dist_line.ns="LineDistance";
    dist_line.id=0;
    dist_line.type=visualization_msgs::Marker::LINE_STRIP;
    dist_line.action=visualization_msgs::Marker::ADD;
    dist_line.scale.x = 0.001;
    pt.x = pt_index_.x;
    pt.y = pt_index_.y;
    pt.z = pt_index_.z;
    std_msgs::ColorRGBA cl;
    cl.r=0;
    cl.a=1;
    cl.g=1;
    cl.b=1;
    dist_line.points.push_back(pt);
    dist_line.colors.push_back(cl);
    pt.x = pt_thumb_.x;
    pt.y = pt_thumb_.y;
    pt.z = pt_thumb_.z;
    dist_line.points.push_back(pt);
    dist_line.colors.push_back(cl);
    dist_line.color.a=1;
    dist_line.color.g=1;
    dist_line.color.b=1;
    dist_line.lifetime=ros::Duration(1.0);
    marks_->markers.push_back(dist_line);
    if (mean_dist > 0.){
        visualization_msgs::Marker dist_text;
        dist_text.ns="Distance";
        dist_text.id=0;
        dist_text.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
        dist_text.action=visualization_msgs::Marker::ADD;
        dist_text.scale.z = 0.008;
        dist_text.text = std::to_string(mean_dist);
        dist_text.color.a=1;
        dist_text.color.g=1;
        dist_text.color.b=1;
        dist_text.lifetime=ros::Duration(1.0);
        dist_text.pose.position.x = (pt_index_.x + pt_thumb_.x)*0.5;
        dist_text.pose.position.y = (pt_index_.y + pt_thumb_.y)*0.5 -0.01;
        dist_text.pose.position.z = (pt_index_.z + pt_thumb_.z)*0.5;
        marks_->markers.push_back(dist_text);
    }
}

bool
HoloFingers::isOnTopOf(pcl::PointCloud<pcl::PointXYZRGB>::Ptr under, pcl::PointCloud<pcl::PointXYZRGB>::Ptr over)
{
    //is over directly on top of under ?
    if (!under || !over)
        return false;
    Eigen::Vector4f umin, umax, omin, omax;
    pcl::getMinMax3D(*under,umin,umax);
    pcl::getMinMax3D(*over, omin,omax);
    if ((omin[0] >= umin[0]-0.005 && omin[0] <= umin[0]+0.005) ||
        (omax[0] >= umax[0]-0.005 && omax[0] <= umax[0]+0.005) )
            return true;
    return false;
}

void  HoloFingers::segment()
{
    if(!cloud_ || !nh_)
        return;
    nh_->param<float>("pass", pass, 0.025);
    Eigen::Vector4f min, max, wmin, wmax;
    pcl::getMinMax3D(*cloud_,min,max);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::CropBox<pcl::PointXYZRGB> cb(true);
    index_.reset();
    thumb_.reset();
    wmin = min;
    wmax = max;
    //segment two finger tips for now (index,thumb)
    for(float y=max[1]; y>=min[1]; y -= pass)
    {
        wmin[1] = y - pass;
        wmax[1] = y;
        cb.setMin(wmin);
        cb.setMax(wmax);
        cb.setInputCloud(cloud_);
        cb.filter(*tmp);
        pcl::IndicesConstPtr indices = cb.getRemovedIndices();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr leftover = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::copyPointCloud(*cloud_, *indices, *leftover);
        cloud_ = leftover;
        if(!tmp->empty() && (!index_ || !thumb_)){
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            ec.setInputCloud(tmp);
            ec.setClusterTolerance(0.005);
            ec.setMinClusterSize(10);
            std::vector<pcl::PointIndices> clusters;
            ec.extract(clusters);
            if (clusters.empty())
                continue;
            else if (clusters.size()==1){
                if(!index_){
                    //temporary save this finger in index
                    index_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    pcl::copyPointCloud(*tmp, clusters[0], *index_);
                }
                else{
                    thumb_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    pcl::copyPointCloud(*tmp, clusters[0], *thumb_);
                    if (isOnTopOf(index_, thumb_))
                        thumb_.reset();
                }
            }
            else if (clusters.size()==2){
                if (!thumb_){
                    thumb_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    if (!index_)
                        pcl::copyPointCloud(*tmp, clusters[1], *thumb_);
                    if (index_){
                        pcl::copyPointCloud(*tmp, clusters[0], *thumb_);
                        if (isOnTopOf(index_,thumb_)){
                            pcl::copyPointCloud(*tmp, clusters[1], *thumb_);
                            if (isOnTopOf(index_,thumb_))
                                thumb_.reset();
                        }
                    }
                }
                if(!index_){
                    index_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    pcl::copyPointCloud(*tmp, clusters[0], *index_);
                    if (isOnTopOf(index_,thumb_))
                        thumb_.reset();
                    if (isOnTopOf(thumb_,index_)){
                        index_ = thumb_;
                        thumb_.reset();
                    }
                }
            }
            else{
                ROS_WARN("[HoloFingers::%s]\tFound more then two fingers... Change hand configuration!",__func__);
                ROS_WARN("[HoloFingers::%s]\tKeeping the largest two",__func__);
                size_t f(0), t(0);
                if (clusters[0].indices.size() > clusters[1].indices.size()){
                    f = 0;
                    t = 1;
                }
                else{
                    f=1;
                    t=0;
                }
                for(size_t i=2; i<clusters.size(); ++i)
                {
                    if(clusters[i].indices.size() >= clusters[f].indices.size()){
                        t = f;
                        f = i;
                    }
                    else if(clusters[i].indices.size() > clusters[t].indices.size())
                        t = i;
                }
                if(!thumb_){
                    thumb_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    if (!index_)
                        pcl::copyPointCloud(*tmp, clusters[t], *thumb_);
                    if (index_)
                        pcl::copyPointCloud(*tmp, clusters[f], *thumb_);
                }
                if(!index_){
                    index_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    pcl::copyPointCloud(*tmp, clusters[f], *index_);
                }
            }
            if (index_ && thumb_){
                //check if fingers need to be swapped
                Eigen::Vector4f tmin, tmax;
                pcl::getMinMax3D(*index_,tmin,tmax);
                float index_x = tmin[0];
                pcl::getMinMax3D(*thumb_,tmin,tmax);
                if (index_x > tmin[0]){
                    //swap
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2 =
                        boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    tmp2 = index_;
                    index_ = thumb_;
                    thumb_ = tmp2;
                }
            }
        }
        if (thumb_ && index_)
            break;
    }
}

void HoloFingers::measure()
{
    if (!index_ || !thumb_)
        return;
    nh_->param<float>("alpha", alpha, 0.3);
    nh_->param<int>("window", window, 30);
    Eigen::Vector4f min, max;
    pcl::getMinMax3D(*index_,min,max);
    pcl::PointXYZRGB  q;
    q.x = max[0];
    q.y = (min[1]+max[1])*0.5;
    q.z = (min[2]+max[2])*0.5;
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud(index_);
    std::vector<int> ki;
    std::vector<float> kd;
    tree.nearestKSearch(q,1,ki,kd);
    pt_index_ = index_->points[ki[0]];
    pcl::getMinMax3D(*thumb_,min,max);
    q.x = min[0];
    q.y = (min[1]+max[1])*0.5;
    q.z = (min[2]+max[2])*0.5;
    tree.setInputCloud(thumb_);
    tree.nearestKSearch(q,1,ki,kd);
    pt_thumb_ = thumb_->points[ki[0]];
    dist.push_back(std::sqrt( std::pow(pt_index_.x - pt_thumb_.x, 2) +
                   std::pow(pt_index_.y - pt_thumb_.y, 2) +
                   std::pow(pt_index_.z - pt_thumb_.z, 2) ) );
    while(dist.size() > window)
        dist.pop_front();
    if (dist.size() ==  window){
        mean_dist = 0;
        for (std::deque<double>::iterator it = dist.begin(); it!= dist.end(); ++it)
            mean_dist += *it;
        mean_dist/=window;
    }
    exp_dist = alpha*dist.back() + (1-alpha)*exp_dist;
}

void HoloFingers::cbCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    if (cloud_)
        return;
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
