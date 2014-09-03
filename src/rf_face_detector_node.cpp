/*Copyright (c) 2014, Jamie Diprose
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the {organization} nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/recognition/face_detection/rf_face_detector_trainer.h>

using namespace std;

float trans_max_variance, face_threshold;
int min_votes_size, stride_sw, pose_refinement, icp_iterations;
std::string forest_fn, model_path;
bool use_normals, pose_refinement;

pcl::RFFaceDetectorTrainer fdrf;
typedef pcl::face_detection::RFTreeNode<pcl::face_detection::FeatureType> NodeType;
pcl::DecisionForest<NodeType> forest;

ros::Subscriber sub_cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
    fdrf.setInputCloud(scene);
    fdrf.detectFaces();
    std::vector<Eigen::VectorXf> heads;
    fdrf.getDetectedFaces(heads);
    //TODO: output heads to msg
    //TODO: publish to tf
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rf_face_detector");
	ros::NodeHandle nh;
	ros::NodeHandle nhr("~");

    if(nh.hasParam("forest_fn") && nh.hasParam("model_path"))
    {
        // Load parameters
        nhr.param<string>("point_cloud_topic", point_cloud_topic, "");
        nhr.param<string>("forest_fn", forest_fn, "");
        nhr.param<string>("model_path", model_path, "");
        nhr.param<float>("max_variance", trans_max_variance, 1600.f);
        nhr.param<int>("min_votes_size", min_votes_size, 300);
        nhr.param<bool>("use_normals", use_normals, false);
        nhr.param<float>("face_threshold", face_threshold, 0.99f);
        nhr.param<int>("stride_sw", stride_sw, 4);
        nhr.param<bool>("pose_refinement", pose_refinement, false);
        nhr.param<int>("icp_iterations", icp_iterations, 5);

        ROS_INFO("point_cloud_topic: %s", point_cloud_topic.c_str());
        ROS_INFO("forest_fn: %s",  );
        ROS_INFO("model_path: %s", );
        ROS_INFO("max_variance: %f", );
        ROS_INFO("min_votes_size: %d", );
        ROS_INFO("use_normals: %d", );
        ROS_INFO("face_threshold: %f", );
        ROS_INFO("stride_sw: %d", );
        ROS_INFO("pose_refinement: %d", );
        ROS_INFO("icp_iterations: %d", );

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

        sub_pcl = nh.subscribe(point_cloud_topic, 1, point_cloud_callback);
        ROS_INFO("Running");
        ros::spin();
    }
    else
    {
        ROS_ERROR("Please specify forest_fn and model_path")
        return -1
    }

    return 0
}