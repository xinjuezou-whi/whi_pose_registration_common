/******************************************************************
node to handle create pattern cloud pcd tool

Features:
- create pattern cloud pcd tool

Written by Yue Zhou, sevendull@163.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-06-05: Initial version
2024-06-20: add curpose orientation yaw angle
2024-06-21: featurepose getinitvertical and horizon Reverse order
******************************************************************/
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <condition_variable>
#include <pcl/visualization/pcl_visualizer.h>     //PCL可视化头文件
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <signal.h>
#include "whi_pose_registration_common/pcl_visualize.h"
#include "whi_pose_registration_common/pose_utilities.h"
#include "whi_pose_registration_common/pcl_utilities.h"
#include <fstream>

struct Feature
{
    std::string name;
    std::string model_cloud;
    std::vector<double> feature_pose;
    std::vector<double> target_rela_pose;
    std::vector<double> cur_pose;

};

enum State
{
    STA_GET_SCAN = 0,       // get target from scan
    STA_FROM_SEG,           // get patten from target segment
    STA_WAIT,
    STA_QUIT
    
};


std::string filepath;
std::string outconfigfile;
std::mutex mtx;
std::condition_variable cv;
bool getcloud = false;
pcl::PointCloud<pcl::PointXYZ> cloudscan;
std::vector<double> xyrange;
tf2_ros::Buffer buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener{ nullptr };
std::string mapframe;
std::string base_link_frame;
std::string pcd_file_prefix;
int file_index = 0;
State state_;
std::shared_ptr<std::thread> th_handler = nullptr;
std::atomic_bool terminating = { false };
std::mutex mtx_state_;
std::vector<double> laser_pose;
std::string laser_frame;

//param for segment
std::string model_cloud_file_;
pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
double pattern_met_radius_thresh_{ 0.05 };
std::vector<float> ndtsample_coeffs_;
int ndtmaxiter_;

int mincut_size_{ 300 };
double sigma_{ 0.25 };
double weight_{ 0.8 };
int cut_min_neighbour_{ 5 };
// segment don
double seg_scale1_;
double seg_scale2_;
double seg_threshold_;
double seg_radius_; 


geometry_msgs::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame,
    const ros::Time& Time)
{
    try
    {
        return buffer_.lookupTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0));
    }
    catch (tf2::TransformException &e)
    {
        ROS_ERROR("%s", e.what());
        return geometry_msgs::TransformStamped();
    }
}

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZ> cloud,cloudout;
    pcl::fromROSMsg(input, cloud);
    int outj = 0;
    for(size_t i=0; i<cloud.points.size (); i++)
    {
        if(cloud.points[i].x > 2 || cloud.points[i].x < -2 || cloud.points[i].y > 2 || cloud.points[i].y < -2 )
            continue;
        else
            outj ++;
        
    }

    cloudout.width = outj;
    cloudout.height = 1;
    cloudout.points.resize(outj);
    int j = 0;
 
    for(size_t i=0; i<cloud.points.size (); i++)
    {
        if(cloud.points[i].x > 2 || cloud.points[i].x < -2 || cloud.points[i].y > 2 || cloud.points[i].y < -2 )
            continue;
        else
        {
            cloudout.points[j].x = cloud.points[i].x;
            cloudout.points[j].y = cloud.points[i].y;
            cloudout.points[j].z = cloud.points[i].z;
            j++;
        }
    }


    pcl::io::savePCDFileASCII (filepath, cloudout);
}


static sensor_msgs::PointCloud2 msgLaserScanToMsgPointCloud2(const sensor_msgs::LaserScan& MsgLaserScan)
{
    std::unique_ptr<laser_geometry::LaserProjection> projector = std::make_unique<laser_geometry::LaserProjection>();
    sensor_msgs::PointCloud2 msgCloud;
    projector->projectLaser(MsgLaserScan, msgCloud);

    return msgCloud;
}

static  pcl::PointCloud<pcl::PointXYZ>::Ptr fromMsgPointCloud2(const sensor_msgs::PointCloud2& MsgPointCloud2)
{
    // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl::PCLPointCloud2 pclCloud2;
    pcl_conversions::toPCL(MsgPointCloud2, pclCloud2);
    // convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZRGB>
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(pclCloud2, *pclCloud);


    return pclCloud;
}

static  pcl::PointCloud<pcl::PointXYZ>::Ptr fromMsgLaserScan(const sensor_msgs::LaserScan& MsgLaserScan)
{
    return fromMsgPointCloud2(msgLaserScanToMsgPointCloud2(MsgLaserScan));
}

void cloudCBLaser(const sensor_msgs::LaserScan::ConstPtr& Laser)
{
    std::lock_guard<std::mutex> lock(mtx_state_);
    static PclVisualize<pcl::PointXYZ> viewer;
    if (state_ == STA_GET_SCAN)
    {
        //file_index ++;
        ROS_INFO("in cloudCBLaser state= sta_get_scan ");
        auto cloud = fromMsgLaserScan(*Laser);

        int outj = 0;
        for(size_t i=0; i<cloud->points.size (); i++)
        {
            if(cloud->points[i].x > xyrange[0] || cloud->points[i].x < xyrange[1] || cloud->points[i].y > xyrange[2] || cloud->points[i].y < xyrange[3] )
                continue;
            else
                outj ++;
            
        }

        pcl::PointCloud<pcl::PointXYZ> cloudout;

        cloudout.width = outj;
        cloudout.height = 1;
        cloudout.points.resize(outj);
        int j = 0;
    
        for(size_t i=0; i<cloud->points.size (); i++)
        {
            if(cloud->points[i].x > xyrange[0] || cloud->points[i].x < xyrange[1] || cloud->points[i].y > xyrange[2] || cloud->points[i].y < xyrange[3] )
                continue;
            else
            {
                cloudout.points[j].x = cloud->points[i].x;
                cloudout.points[j].y = cloud->points[i].y;
                cloudout.points[j].z = cloud->points[i].z;
                j++;
            }
        }


        pcl::copyPointCloud(cloudout, cloudscan);
        std::string old_feature = "oldfeature";
        std::string oldfilename = filepath + old_feature + ".pcd";
        pcl::io::savePCDFileASCII (oldfilename, cloudscan);

        geometry_msgs::TransformStamped lasertransform;
        lasertransform.header.frame_id = laser_frame;
        lasertransform.child_frame_id = laser_frame;
        lasertransform.header.stamp = ros::Time::now();
    
        //  
        lasertransform.transform.translation.x = laser_pose[0];
        lasertransform.transform.translation.y = laser_pose[1];
        lasertransform.transform.translation.z = laser_pose[2];
        tf2::Quaternion q;
        q.setRPY(laser_pose[3]*M_PI/180, laser_pose[4]*M_PI/180, laser_pose[5]*M_PI/180);  
        lasertransform.transform.rotation.x = q.x();
        lasertransform.transform.rotation.y = q.y();
        lasertransform.transform.rotation.z = q.z();
        lasertransform.transform.rotation.w = q.w();        

        for (int i= 0; i < cloudscan.points.size(); i++)
        {
            geometry_msgs::Pose pointpose,aftertrans_pose;
            pointpose.position.x = cloudscan.points[i].x;
            pointpose.position.y = cloudscan.points[i].y;
            pointpose.position.z = cloudscan.points[i].z;
            pointpose.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
            aftertrans_pose = PoseUtilities::applyTransform(pointpose, lasertransform);
            cloudscan.points[i].x = aftertrans_pose.position.x;
            cloudscan.points[i].y = aftertrans_pose.position.y;
            cloudscan.points[i].z = aftertrans_pose.position.z;

        }



        ROS_INFO("cloudscan.points.size() = %d ",cloudscan.points.size()) ;
        if(!(cloudscan.points.size() > 0))
        {
            ROS_INFO("get cloudscan fail ");
            state_ = STA_WAIT;
            return ;
        }

        // find cloud min x; 
        double minx = 2;
        minx = cloudscan.points[0].x;
        double getinithorizon = 0.0;
        for(int i = 0; i < cloudscan.points.size() ; i++)
        {
            auto onex = cloudscan.points[i].x;
            auto oney = cloudscan.points[i].y;
            if(onex < minx)
            {
                minx = onex;
                getinithorizon =oney;
            }
        }

        double getinitvertical = fabs(minx);

        getinitvertical = std::round(getinitvertical * 1000) / 1000.0;
        getinithorizon = std::round(getinithorizon * 1000) / 1000.0;
        ROS_INFO("getinitvertical = %f ,getinithorizon =%f ",getinitvertical ,getinithorizon);

        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe, base_link_frame, ros::Time(0));
        geometry_msgs::Pose curpose;
        curpose.position.x = transBaselinkMap.transform.translation.x;
        curpose.position.y = transBaselinkMap.transform.translation.y;
        curpose.position.z = transBaselinkMap.transform.translation.z;
        curpose.position.x = std::round(curpose.position.x * 1000) / 1000.0;
        curpose.position.y = std::round(curpose.position.y * 1000) / 1000.0;
        curpose.orientation = transBaselinkMap.transform.rotation;
        //curpose.position.z = std::round(curpose.position.z * 1000) / 1000.0;
        ROS_INFO("curpose x:%f, y:%f, z:%f ",curpose.position.x,curpose.position.y,curpose.position.z);

                
        //ROS_INFO("configfile is : %s",configfile.c_str());
        
        /*
        fstream fs;
        fs.open(outconfigfile, ios::in);
    
        if (!fs)
        {
            cout << "not exist" << endl;
            //创建文件
            ofstream fout(outconfigfile);
            if (fout)
            {
                // 执行完操作后关闭文件句柄
                fout.close();
            }
        }
        else
        {
            cout << "file exist " << endl;
        }
        
        YAML::Node config_node = YAML::LoadFile(outconfigfile.c_str());

        YAML::Node feature_node;
        std::string feature_name = pcd_file_prefix + "_" + std::to_string(file_index);
        feature_node["name"] = feature_name;
        feature_node["feature_pose"].push_back(getinitvertical);
        feature_node["feature_pose"].push_back(getinithorizon);
        feature_node["feature_pose"].push_back(0.0);
        feature_node["cur_pose"].push_back(curpose.position.x);
        feature_node["cur_pose"].push_back(curpose.position.y);
        feature_node["cur_pose"].push_back(PoseUtilities::toEuler(curpose.orientation)[2]);
        YAML::Node target_node;
        feature_node["target_relative_pose"].push_back(0.0);
        feature_node["using_inertial"].push_back(0);
        config_node.push_back(feature_node);

        std::ofstream fout(outconfigfile);
        fout << config_node;
        fout.close();
        */

        //std::string filename = filepath + feature_name + ".pcd";
        std::string filename = filepath + "target.pcd";
        pcl::io::savePCDFileASCII (filename, cloudscan);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloudscan));
        //static PclVisualize<pcl::PointXYZ> viewer;
        viewer.viewCloud(cloud_ptr, "target");

        ROS_INFO("get scan, save file ,state = sta_wait");
        state_ = STA_WAIT;
    }
    else if (state_ == STA_FROM_SEG)
    {
        file_index ++;
        ROS_INFO("in cloudCBLaser state= STA_FROM_SEG ");
        auto pclCloud = fromMsgLaserScan(*Laser);

        ROS_INFO("model_cloud_file_ is %s ",model_cloud_file_.c_str());
        target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(model_cloud_file_.c_str(), *target_cloud_) == -1)
        {
            state_ = STA_WAIT;
            PCL_ERROR("Can not find the file model_cloud ");
            return;
        }
        ROS_INFO("Loaded : %d from the model_cloud file", target_cloud_->size());

        /// transform the laser according to user specified frame
        // get user define frame
        geometry_msgs::TransformStamped lasertransform;
        lasertransform.header.frame_id = laser_frame;
        lasertransform.child_frame_id = laser_frame;
        lasertransform.header.stamp = ros::Time::now();
        lasertransform.transform.translation.x = laser_pose[0];
        lasertransform.transform.translation.y = laser_pose[1];
        lasertransform.transform.translation.z = laser_pose[2];
        tf2::Quaternion q;
        q.setRPY(angles::from_degrees(laser_pose[3]),
            angles::from_degrees(laser_pose[4]), angles::from_degrees(laser_pose[5]));  
        lasertransform.transform.rotation.x = q.x();
        lasertransform.transform.rotation.y = q.y();
        lasertransform.transform.rotation.z = q.z();
        lasertransform.transform.rotation.w = q.w(); 
        // transform
        for (int i= 0; i < pclCloud->points.size(); i++)
        {
            geometry_msgs::Pose pointpose,aftertrans_pose;
            pointpose.position.x = pclCloud->points[i].x;
            pointpose.position.y = pclCloud->points[i].y;
            pointpose.position.z = pclCloud->points[i].z;
            pointpose.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
            aftertrans_pose = PoseUtilities::applyTransform(pointpose, lasertransform);
            pclCloud->points[i].x = aftertrans_pose.position.x;
            pclCloud->points[i].y = aftertrans_pose.position.y;
            pclCloud->points[i].z = aftertrans_pose.position.z;
        }

        // get all scan pclcloud , start get segment from target 
        /// segment don
        pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outcloudvec;
        ROS_INFO("start segment_don");
        PclUtilities<pcl::PointXYZ>::segment_don(pclCloud, target_cloud_, 
            outcloudvec, seg_scale1_, seg_scale2_, seg_threshold_, seg_radius_, ndtsample_coeffs_, ndtmaxiter_);
        ROS_INFO("finishi segment_don");
        if (outcloudvec.size() == 0)
        {
            state_ = STA_WAIT;
            //std::string outfile;
            //outfile = packpath_ + "/pcd/testgetall.pcd";
            //pcl::io::savePCDFileASCII (outfile, *pclCloud);                 
            ROS_INFO(" segment_don , get none features , check config or condition");
            return;
        }

        //--------------------从几个可能的特征找出最近的-----------
        // filter the target out within multiple patterns
        // comparation in laser frame
        geometry_msgs::Pose originpose;
        originpose.position.x = 0;
        originpose.position.y = 0;
        double center_dist_min = std::numeric_limits<double>::max();
        for (auto oneiter = outcloudvec.begin(); oneiter != outcloudvec.end(); oneiter++)
        {
            std::vector<mypoint> pvec;
            for (int i = 0; i < (*oneiter)->points.size(); ++i)
            {
                mypoint onepoint;
                onepoint.x = (*oneiter)->points[i].x;
                onepoint.y = (*oneiter)->points[i].y;
                pvec.push_back(onepoint);
            }
            double minradius;
            mypoint center;
            min_circle_cover(pvec, minradius, center);
            geometry_msgs::Pose centerpoint;
            centerpoint.position.x = center.x;
            centerpoint.position.y = center.y;
            double center_dist = PoseUtilities::distance(originpose,centerpoint);
            if(center_dist < center_dist_min)
            {
                center_dist_min = center_dist;
                *outcloud = **oneiter ;
            }
        }
        
        //---------------------- 求最小包围圆 ----------------
        // flitered target
        std::vector<mypoint> pvec;
        for (int i = 0; i < outcloud->points.size(); ++i)
        {
            mypoint onepoint;
            onepoint.x = outcloud->points[i].x;
            onepoint.y = outcloud->points[i].y;
            pvec.push_back(onepoint);
        }
        double minradius;
        mypoint center;
        min_circle_cover(pvec, minradius, center);
        ROS_INFO("outcloud ,radius : %.10lf \n  center.x: %.10lf center.y: %.10lf ",minradius,center.x, center.y);
        minradius = minradius + 0.03; 

        /*
        // 求模板点云的最小包围圆半径，作比较
        std::vector<mypoint> tarpvec;
        for (int i = 0; i < target_cloud_->points.size(); ++i)
        {
            mypoint onepoint;
            onepoint.x = target_cloud_->points[i].x;
            onepoint.y = target_cloud_->points[i].y;
            tarpvec.push_back(onepoint);
        }
        double tarminradius;
        mypoint tarcenter;
        min_circle_cover(tarpvec, tarminradius, tarcenter);
        ROS_INFO("targetcloud ,radius : %.10lf \n  center.x: %.10lf center.y: %.10lf ",tarminradius,tarcenter.x, tarcenter.y);
        
        // check if the filtered target meet the template pattern
        double delta_radius = minradius - tarminradius;
        ROS_INFO("delta_radius is: %f ",fabs(delta_radius));
        if(fabs(delta_radius) > pattern_met_radius_thresh_)
        {
            ROS_INFO("outcloud radius is far from target radius, maybe wrong");
            state_ = STA_WAIT;
            return;
        }

        */

        //--------------------- 在原点云上 mincut ------
        ROS_INFO("start segmentMinCut");
        std::vector<pcl::PointIndices> minclusterIndices;
        pcl::PointXYZ minpointCenter;
        minpointCenter.x = center.x;
        minpointCenter.y = center.y;
        minpointCenter.z = 0;

        //minradius = minradius + 0.02;       // mark here by zhouyue

        minclusterIndices = PclUtilities<pcl::PointXYZ>::segmentMinCut(pclCloud, minpointCenter,
            minradius, cut_min_neighbour_, sigma_, weight_);
        ROS_INFO("total minclusters number from epic: %d", minclusterIndices.size()) ;
        for (int i = 0; i < minclusterIndices.size(); ++i)
        {
            ROS_INFO("mincluster %d has points %d" , i, minclusterIndices[i].indices.size());
        }
        int ci = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr minoutcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
        for (const auto& cluster : minclusterIndices)
        {
            ci++;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeature(new pcl::PointCloud<pcl::PointXYZ>());
            PclUtilities<pcl::PointXYZ>::extractTo(pclCloud, cluster, cloudFeature);
            ROS_INFO("extractTo cluster indices = %d , cloudFeature->size() =%d " , ci,cloudFeature->size());

            if (!cloudFeature->empty() && cloudFeature->size() < mincut_size_)
            {
                *minoutcloud = *cloudFeature;
            }
        }
        if (minoutcloud->points.empty())
        {
            state_ = STA_WAIT;
            ROS_INFO("segmentMinCut fail");
            return;
        }            
        ROS_INFO("finish segmentMinCut, get outcloud");

        // start out feature 
        // find cloud min x; 
        double minx = 2;
        minx = minoutcloud->points[0].x;
        double getinithorizon = 0.0;
        for(int i = 0; i < minoutcloud->points.size() ; i++)
        {
            auto onex = minoutcloud->points[i].x;
            auto oney = minoutcloud->points[i].y;
            if(onex < minx)
            {
                minx = onex;
                getinithorizon =oney;
            }
        }

        double getinitvertical = fabs(minx);

        getinitvertical = std::round(getinitvertical * 1000) / 1000.0;
        getinithorizon = std::round(getinithorizon * 1000) / 1000.0;
        ROS_INFO("getinitvertical = %f ,getinithorizon =%f ",getinitvertical ,getinithorizon);

        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe, base_link_frame, ros::Time(0));
        geometry_msgs::Pose curpose;
        curpose.position.x = transBaselinkMap.transform.translation.x;
        curpose.position.y = transBaselinkMap.transform.translation.y;
        curpose.position.z = transBaselinkMap.transform.translation.z;
        curpose.position.x = std::round(curpose.position.x * 1000) / 1000.0;
        curpose.position.y = std::round(curpose.position.y * 1000) / 1000.0;
        curpose.orientation = transBaselinkMap.transform.rotation;
        //curpose.position.z = std::round(curpose.position.z * 1000) / 1000.0;
        ROS_INFO("curpose x:%f, y:%f, z:%f ",curpose.position.x,curpose.position.y,curpose.position.z);

                
        //ROS_INFO("configfile is : %s",configfile.c_str());

        fstream fs;
        fs.open(outconfigfile, ios::in);
    
        if (!fs)
        {
            cout << "not exist" << endl;
            //创建文件
            ofstream fout(outconfigfile);
            if (fout)
            {
                // 执行完操作后关闭文件句柄
                fout.close();
            }
        }
        else
        {
            cout << "file exist " << endl;
        }
        
        YAML::Node config_node = YAML::LoadFile(outconfigfile.c_str());

        YAML::Node feature_node;
        std::string feature_name = pcd_file_prefix + "_" + std::to_string(file_index);
        feature_node["name"] = feature_name;
        feature_node["feature_pose"].push_back(getinitvertical);
        feature_node["feature_pose"].push_back(getinithorizon);
        feature_node["feature_pose"].push_back(0.0);
        feature_node["cur_pose"].push_back(curpose.position.x);
        feature_node["cur_pose"].push_back(curpose.position.y);
        feature_node["cur_pose"].push_back(PoseUtilities::toEuler(curpose.orientation)[2]);
        YAML::Node target_node;
        feature_node["target_relative_pose"].push_back(0.0);
        //feature_node["using_inertial"].push_back(0);
        config_node.push_back(feature_node);

        std::ofstream fout(outconfigfile);
        fout << config_node;
        fout.close();

        std::string filename = filepath + feature_name + ".pcd";
        pcl::io::savePCDFileASCII (filename, *minoutcloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(*minoutcloud));
        int PointSize = 1;
        std::array<double, 3> PointColor = {0.0, 0.0, 1.0};
        viewer.viewCloud(cloud_ptr, feature_name, PointSize, PointColor);

        ROS_INFO("get scan, save file ,state = sta_wait");
        state_ = STA_WAIT;


    }
    else if (state_ == STA_QUIT)
    {
        ros::shutdown();
    }


}

void sigintHandler(int sig)
{
	// Do some custom action.
	// For example, publish a stop message to some other nodes.
	std::cout << "quiting......" << std::endl;

	terminating.store(true);
	th_handler->join();
 
	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

void userInput()
{
	while (!terminating.load())
	{
		int ch = getchar();
        std::lock_guard<std::mutex> lock(mtx_state_);
		switch (ch)
		{
		case 115: // s
			// start get 
			if (state_ == STA_WAIT)
			{
				state_ = STA_GET_SCAN;
                ROS_INFO("key input , start get scan ----");
			}
			break;
        case 116: //t
            if (state_ == STA_WAIT)
            {
                state_ = STA_FROM_SEG;
                ROS_INFO("t input, get pattern from target segment");

            }
            break;
		case 113: // q
			// quit 
			//ros::shutdown();
            state_ = STA_QUIT;
			break;            


        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



// type key s to get scan cloud 
 
int main (int argc, char **argv)
{

    std::cout << "\nWHI tool create pcd pattern VERSION 00.01.3" << std::endl;
	std::cout << "Copyright © 2024-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    std::cout << "input key 's' to get scan cloud " << std::endl;

    const std::string nodeName("whi_tool_create_pcd_pattern");
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh(nodeName);

    signal(SIGINT, sigintHandler);

    nh.getParam("feature_range",xyrange);
    ROS_INFO("feature_range , x max: %f ,x min: %f ,y max: %f, y min: %f",xyrange[0],xyrange[1],xyrange[2],xyrange[3]);
    std::string scan_topic;
    nh.getParam("map_frame",mapframe);
    nh.getParam("base_link_frame",base_link_frame);
    nh.getParam("pcd_file_path", filepath);
    nh.getParam("feature_file", outconfigfile);
    nh.getParam("pcd_file_prefix", pcd_file_prefix);
    nh.getParam("scan_topic", scan_topic);
    nh.getParam("laser_pose", laser_pose);
    nh.getParam("laser_frame", laser_frame);
    ROS_INFO("laser_pose , x : %f ,y : %f ,z : %f, roll : %f, pitch : %f, yaw : %f ",laser_pose[0],laser_pose[1],laser_pose[2],laser_pose[3],laser_pose[4],laser_pose[5]);

    nh.param("mincut_size", mincut_size_, 300);
    nh.param("sigma", sigma_, 0.25);
    nh.param("weight", weight_, 0.8);
    nh.param("cut_min_neighbour", cut_min_neighbour_, 5);

    nh.param("seg_normal_min_scale", seg_scale1_, 1.0);
    nh.param("seg_normal_max_scale", seg_scale2_, 1.0);
    nh.param("seg_threshold", seg_threshold_, 0.2);
    nh.param("seg_radius", seg_radius_, 5.0);

    nh.param("ndt_maxiter", ndtmaxiter_, 5000);
    nh.param("pattern_met_radius_thresh",pattern_met_radius_thresh_,0.4);
    if (!nh.getParam("pose_registration/LocatePose/ndt_sample_coeffs", ndtsample_coeffs_))
    {
        for (int i = 0; i < 3; ++i)
        {
            ndtsample_coeffs_.push_back(0.005);
        }
    }  
    nh.getParam("model_cloud_file",model_cloud_file_);

    tf_listener = std::make_shared<tf2_ros::TransformListener>(buffer_);
    state_ = STA_WAIT;

    th_handler = std::make_shared<std::thread>(userInput);

    ros::Subscriber bat_sub = nh.subscribe(scan_topic, 10, cloudCBLaser);
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();	

    return 0;
}