/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Aitor Aldoma (the original pcl OpenNI version of this), Jamie Diprose (ported it to work with ROS)
 * Email  : jamie.diprose@gmail.com
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <face_detection/rf_face_detector_trainer.h>
#include <boost/iostreams/filter/bzip2.hpp>
#include <rf_face_detector/FaceList.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

using namespace std;

double trans_max_variance, face_threshold;
int min_votes_size, stride_sw, icp_iterations;
string point_cloud_topic, forest_fn, model_path;
bool use_normals, pose_refinement;

face_detection::RFFaceDetectorTrainer fdrf;
typedef face_detection::RFTreeNode<face_detection::FeatureType> NodeType;
pcl_ml::DecisionForest<NodeType> forest;

ros::Subscriber sub_cloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

rf_face_detector::FaceList face_list_msg;
ros::Publisher pub_face_list;
ros::Publisher pub_heat_map;


void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    face_list_msg.faces.clear();

    pcl::PCLPointCloud2 cloud_2;
    PointCloud cloud;
    pcl_conversions::toPCL(*msg, cloud_2);
    pcl::fromPCLPointCloud2(cloud_2, cloud);
    PointCloud::Ptr cloud_ptr(new PointCloud());
    *cloud_ptr = cloud;

    fdrf.setInputCloud(cloud_ptr);
    fdrf.detectFaces();
    std::vector<Eigen::VectorXf> heads;
    fdrf.getDetectedFaces(heads);

    // Publish heat map
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    fdrf.getFaceHeatMap(intensity_cloud);

    pcl::PCLPointCloud2 intensity_cloud_2;
    pcl::toPCLPointCloud2(*intensity_cloud, intensity_cloud_2);


    sensor_msgs::PointCloud2 intensity_msg;
    intensity_msg.header.frame_id = msg->header.frame_id;
    intensity_msg.header.stamp = msg->header.stamp;
    pcl_conversions::fromPCL(intensity_cloud_2, intensity_msg);
    pub_heat_map.publish(intensity_msg);

    for (size_t i = 0; i < heads.size(); i++)
    {
        geometry_msgs::Point point;
        point.x = heads[i][0];
        point.y = heads[i][1];
        point.z = heads[i][2];

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = msg->header.frame_id;
        pose_stamped.header.stamp = msg->header.stamp;
        pose_stamped.pose.position = point;
        face_list_msg.faces.push_back(pose_stamped);
    }

    pub_face_list.publish(face_list_msg);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rf_face_detector");
	ros::NodeHandle nh;
	ros::NodeHandle nhr("~");
	bool success = true;

	if(!nhr.hasParam("forest_fn"))
	{
	    ROS_ERROR("Please specify forest_fn");
	    success = false;
	}

	if(!nhr.hasParam("model_path"))
	{
        ROS_ERROR("Please specify model_path");
	    success = false;
	}

    if(success)
    {
        ROS_INFO("Loading parameters");
        // Load parameters
        nhr.param<string>("point_cloud_topic", point_cloud_topic, "/softkinetic_camera/depth_registered/points");
        nhr.getParam("forest_fn", forest_fn); //TODO: if file ends in .bz2 then unzip it into a temp dir and feed that to frdf
        nhr.getParam("model_path", model_path);
        nhr.param<double>("max_variance", trans_max_variance, 1600.0);
        nhr.param<int>("min_votes_size", min_votes_size, 300);
        nhr.param<bool>("use_normals", use_normals, false);
        nhr.param<double>("face_threshold", face_threshold, 0.99);
        nhr.param<int>("stride_sw", stride_sw, 4);
        nhr.param<bool>("pose_refinement", pose_refinement, false);
        nhr.param<int>("icp_iterations", icp_iterations, 5);

        ROS_INFO("point_cloud_topic: %s", point_cloud_topic.c_str());
        ROS_INFO("forest_fn: %s", forest_fn.c_str());
        ROS_INFO("model_path: %s", model_path.c_str());
        ROS_INFO("max_variance: %f", trans_max_variance);
        ROS_INFO("min_votes_size: %d", min_votes_size);
        ROS_INFO("use_normals: %d", use_normals);
        ROS_INFO("face_threshold: %f", face_threshold);
        ROS_INFO("stride_sw: %d", stride_sw);
        ROS_INFO("pose_refinement: %d", pose_refinement);
        ROS_INFO("icp_iterations: %d", icp_iterations);

        // Initialize fdrf
        ROS_INFO("Initializing frdf");
        fdrf.setForestFilename(forest_fn);
        fdrf.setWSize(80);
        fdrf.setUseNormals (use_normals);
        fdrf.setWStride (stride_sw);
        fdrf.setLeavesFaceMaxVariance(trans_max_variance);
        fdrf.setLeavesFaceThreshold(face_threshold);
        fdrf.setFaceMinVotes(min_votes_size);

        if (pose_refinement)
        {

            fdrf.setPoseRefinement(true, icp_iterations);
            fdrf.setModelPath(model_path);
        }

        // Load forest and pass to the detector
        std::filebuf fb;
        fb.open(forest_fn.c_str(), std::ios::in);
        std::istream os(&fb);
        forest.deserialize(os);
        fb.close();
        fdrf.setForest(forest);

        sub_cloud = nh.subscribe(point_cloud_topic, 1, point_cloud_callback);
        pub_face_list = nh.advertise<rf_face_detector::FaceList>("rf_face_detector/detected_faces", 1);
        pub_heat_map = nh.advertise<sensor_msgs::PointCloud2>("rf_face_detector/heat_map", 1);
        ROS_INFO("Running");
        ros::spin();
    }
    else
    {
        return -1;
    }

    return 0;
}