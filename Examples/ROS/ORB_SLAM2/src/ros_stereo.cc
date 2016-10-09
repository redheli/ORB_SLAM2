/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;


class ImageGrabber
{
public:
  ImageGrabber(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh),nhp_(nhp)
  {
    /* rosparam */
    nhp_.param("do_rectify", do_rectify_, true);
    nhp_.param("do_view", do_view_, true);
    nhp_.param("do_save", do_save_, true);
    nhp_.param("camera/left/image", camera_left_image_, string("camera/left/image"));
    nhp_.param("camera/right/image", camera_right_image_, string("camera/right/image"));
    nhp_.param("vocabulary_file", vocabulary_file_, string(""));
    nhp_.param("stereo_calib_file", stereo_calib_file_, string(""));
    if(do_rectify_)
      {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(stereo_calib_file_, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
          {
            ROS_FATAL("Wrong path to stereo calib setting file");
            return;
          }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
           rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
          {
            ROS_FATAL("Calibration parameters to rectify stereo are missing!");
            return;
          }
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
      }

    /* Create SLAM system. It initializes all system threads and gets ready to process frames. */
    mp_slam_ = new ORB_SLAM2::System(vocabulary_file_, stereo_calib_file_, ORB_SLAM2::System::STEREO, do_view_);

    /* stereo camera subscriber */
    left_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_left_image_, 1);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_right_image_, 1);
    sync_ = new message_filters::Synchronizer<sync_pol>(sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&ImageGrabber::GrabStereo,this,_1,_2));

    camera_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera/pose", 10);
  }

  ~ImageGrabber()
  {
    // Stop all threads
    mp_slam_->Shutdown();

    // Save camera trajectory
    if(do_save_)
      {
        mp_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
        mp_slam_->SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
        mp_slam_->SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");
      }

    delete mp_slam_;
    delete left_sub_;
    delete right_sub_;
    delete sync_;
  }

  void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
  {
    /* Copy the ros image message to cv::Mat. */
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
      {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
      {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    /* cw: c(camera frame) -> w (world frame) */
    /* we should convert the frame */
    cv::Mat mTcw;

    if(do_rectify_)
      {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        mTcw = mp_slam_->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
      }
    else
      {
        mTcw = mp_slam_->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
      }

    if(mTcw.dims == 2)
      {
        cv::Mat Rcw = mTcw.rowRange(0,3).colRange(0,3);
        tf::Matrix3x3 r_cw(Rcw.at<float>(0, 0), Rcw.at<float>(0, 1), Rcw.at<float>(0, 2),
                           Rcw.at<float>(1, 0), Rcw.at<float>(1, 1), Rcw.at<float>(1, 2),
                           Rcw.at<float>(2, 0), Rcw.at<float>(2, 1), Rcw.at<float>(2, 2));
        tf::Matrix3x3 r_wc = r_cw.transpose();

        cv::Mat t_cw = mTcw.rowRange(0,3).col(3);
        cv::Mat t_wc = - Rcw.t() * t_cw;

        ////////////////////////////////////////////////////////////////////////////////////
        // double r = 0, p = 0, y = 0;                                                    //
        // r_cw.getRPY(r,p,y);                                                            //
        // std::cout << " r_cw: roll : r:" << r << " p:" << p << " y:" << y << std::endl; //
        // std::cout << " t_cw:" << t_cw  << std::endl;                                   //
        // r_wc.getRPY(r,p,y);                                                            //
        // std::cout << " r_wc: roll : r:" << r << " p:" << p << " y:" << y << std::endl; //
        // std::cout << " t_wc:" << t_wc  << std::endl;                                   //
        ////////////////////////////////////////////////////////////////////////////////////

        /* transform from t_wc(vision world frame) to t_w_robot(robot world frame) */
        tf::Matrix3x3 r_wr_wc;
        r_wr_wc.setRPY(-M_PI / 2, 0, -M_PI / 2);
        tf::Transform t_w_robot(r_wr_wc * r_wc * r_wr_wc.transpose(),
                                r_wr_wc * tf::Vector3(t_wc.at<float>(0,0),
                                                      t_wc.at<float>(1,0),
                                                      t_wc.at<float>(2,0)));

        /* ros publish */
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header.stamp = msgLeft->header.stamp;
        camera_pose.header.frame_id = std::string("world");
        tf::poseTFToMsg(t_w_robot, camera_pose.pose);

        camera_pose_pub_.publish(camera_pose);
      }
  }

private:

  /* ros */
  ros::NodeHandle nh_, nhp_;
  ros::Publisher camera_pose_pub_;
  message_filters::Subscriber<sensor_msgs::Image>* left_sub_;
  message_filters::Subscriber<sensor_msgs::Image>* right_sub_;
  message_filters::Synchronizer<sync_pol>* sync_;

  /* orb slam */
  ORB_SLAM2::System* mp_slam_;
  cv::Mat M1l,M2l,M1r,M2r;

  /* ros param */
  bool do_rectify_;
  bool do_view_;
  bool do_save_;

  string camera_left_image_, camera_right_image_;
  string vocabulary_file_;
  string stereo_calib_file_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orb_slam_stereo");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ImageGrabber* igb = new ImageGrabber(nh, nhp);
  ros::spin();
  delete igb;
  return 0;
}
