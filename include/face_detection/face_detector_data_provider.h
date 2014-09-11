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
 * Author : Aitor Aldoma
 * Email  :
 *
 */

#ifndef FACE_DETECTOR_DATA_PROVIDER_H_
#define FACE_DETECTOR_DATA_PROVIDER_H_

#include <pcl/common/common.h>
#include <face_detection/face_common.h>
#include <pcl_ml/dt/decision_tree_data_provider.h>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>

namespace bf = boost::filesystem;


  namespace face_detection
  {
    template<class FeatureType, class DataSet, class LabelType, class ExampleIndex, class NodeType>
    class FaceDetectorDataProvider: public pcl_ml::DecisionTreeTrainerDataProvider<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>
    {
      private:
        int num_images_;
        std::vector<std::string> image_files_;
        bool USE_NORMALS_;
        int w_size_;
        int patches_per_image_;
        int min_images_per_bin_;

        void getFilesInDirectory(bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
        {
          bf::directory_iterator end_itr;
          for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
          {
            //check if its a directory, then get models in it
            if (bf::is_directory (*itr))
            {
#if BOOST_FILESYSTEM_VERSION == 3
              std::string so_far = rel_path_so_far + (itr->path ().filename ()).string () + "/";
#else
              std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

              bf::path curr_path = itr->path ();
              getFilesInDirectory (curr_path, so_far, relative_paths, ext);
            } else
            {
              //check that it is a ply file and then add, otherwise ignore..
              std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
              std::string file = (itr->path ().filename ()).string ();
#else
              std::string file = (itr->path ()).filename ();
#endif

              boost::split (strs, file, boost::is_any_of ("."));
              std::string extension = strs[strs.size () - 1];

              if (extension.compare (ext) == 0)
              {
#if BOOST_FILESYSTEM_VERSION == 3
                std::string path = rel_path_so_far + (itr->path ().filename ()).string ();
#else
                std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

                relative_paths.push_back (path);
              }
            }
          }
        }

        inline bool readMatrixFromFile(std::string file, Eigen::Matrix4f & matrix)
        {

          std::ifstream in;
          in.open (file.c_str (), std::ifstream::in);
          if (!in.is_open ())
          {
            return false;
          }

          char linebuf[1024];
          in.getline (linebuf, 1024);
          std::string line (linebuf);
          std::vector < std::string > strs_2;
          boost::split (strs_2, line, boost::is_any_of (" "));

          for (int i = 0; i < 16; i++)
          {
            matrix (i / 4, i % 4) = static_cast<float> (atof (strs_2[i].c_str ()));
          }

          return true;
        }

        bool check_inside(int col, int row, int min_col, int max_col, int min_row, int max_row)
        {
          if (col >= min_col && col <= max_col && row >= min_row && row <= max_row)
            return true;

          return false;
        }

        template<class PointInT>
        void cropCloud(int min_col, int max_col, int min_row, int max_row, pcl::PointCloud<PointInT> & cloud_in, pcl::PointCloud<PointInT> & cloud_out)
        {
          cloud_out.width = max_col - min_col + 1;
          cloud_out.height = max_row - min_row + 1;
          cloud_out.points.resize (cloud_out.width * cloud_out.height);
          for (unsigned int u = 0; u < cloud_out.width; u++)
          {
            for (unsigned int v = 0; v < cloud_out.height; v++)
            {
              cloud_out.at (u, v) = cloud_in.at (min_col + u, min_row + v);
            }
          }

          cloud_out.is_dense = cloud_in.is_dense;
        }

      public:

        FaceDetectorDataProvider()
        {
          w_size_ = 80;
          USE_NORMALS_ = false;
          num_images_ = 10;
          patches_per_image_ = 20;
          min_images_per_bin_ = -1;
        }

        virtual ~FaceDetectorDataProvider()
        {

        }

        void setPatchesPerImage(int n)
        {
          patches_per_image_ = n;
        }

        void setMinImagesPerBin(int n)
        {
          min_images_per_bin_ = n;
        }

        void setUseNormals(bool use)
        {
          USE_NORMALS_ = use;
        }

        void setWSize(int size)
        {
          w_size_ = size;
        }

        void setNumImages(int n)
        {
          num_images_ = n;
        }

        void initialize(std::string & data_dir);

        //shuffle file and get the first num_images_ as requested by a tree
        //extract positive and negative samples
        //create training examples and labels
        void getDatasetAndLabels(DataSet & data_set, std::vector<LabelType> & label_data, std::vector<ExampleIndex> & examples);
    };
  }


#endif /* FACE_DETECTOR_DATA_PROVIDER_H_ */
