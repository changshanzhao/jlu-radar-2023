#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
//
#include "sensor_mags/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "pcl_ros/transforms.h"
#include "image_geometry/pinhole_camera_model.h"
#include "opencv2/highgui/highgui.hpp"
//
#include "vector"
#include "queue"
#include "math.h"
#include "algorithm"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "utils/ikd_Tree.h"

#include "livox_ros_driver/CustomMsg.h"
#include "livox_ros_driver/CustomPoint.h"

#define layers_per_map  5
#define queue_limit_    10
#define intensity_thresh_1 120.0
#define intensity_thresh_2 175.0

const float_t FOV                   = 70.4;         // 角度制
const uint8_t cloud_num_per_map[]   = {1, 4, 8, 16, 32};    // 每层分的格数
const uint8_t min_index_submap[]    = {0, 1, 5, 13, 29};   // 每层的最小索引
float boundry_tan_yz_x[layers_per_map - 1] = {0};   // 数量为 层数-1, 初始化时对这个数据进行计算
// std::vector<std::vector<float>> tan_y_z;
float boundry_atan_y_z[layers_per_map];

// EVALUATE

struct cloudFrame{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    size_t cloud_size;
};

float degree2rad(float d){
    return d * M_PI/ 180.0f;
}

float rad2dgree(float r){
    return r * 180.0f / M_PI;
}

class mapMaintainer{
    public:
        mapMaintainer(std::string topic_name) : nh_("/"), cloud_topic_name_(topic_name){
            init_ros();
            init_params();
            init_pcl();
            record_file_ = fopen("record.csv", "wb+");
            record_file_t_ = fopen("record_t.csv", "wb+");
            // timer = nh_.createTimer ( ros::Duration ( 0.1), &mapMaintainer::check_cloud, this);
        };

        ~mapMaintainer(){
            fclose(record_file_);
            fclose(record_file_t_);
        }
        
        void init_pcl(){
            vox_filter_ = pcl::VoxelGrid<pcl::PointXYZI>::Ptr(new pcl::VoxelGrid<pcl::PointXYZI>);
            map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            
            vox_filter_->setLeafSize(0.05f, 0.05f, 0.05f);
            vox_filter_->setInputCloud(map_);
            
            cloud_num_total = 0;
            for(int num : cloud_num_per_map){
                cloud_num_total += num;
            }
            map_list_.resize(cloud_num_total);
            // KD_TREE初始化
            map_list_kd_.resize(cloud_num_total);
            sub_map_buffs_.resize(cloud_num_total);
            for(int i=0; i < cloud_num_total; i++){
                map_list_[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
                map_list_kd_[i] = KD_TREE<pcl::PointXYZI>::Ptr(new KD_TREE<pcl::PointXYZI>(0.3, 0.6, 0.2));
                sub_map_buffs_[i].clear();
            }
        }

        void init_ros(){
            sub_livox_cloud_ = nh_.subscribe(cloud_topic_name_, 5, &mapMaintainer::cloudLivxoCB, this);         
            pub_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("scan", 5);
            pub_map_visual_split_ = nh_.advertise<sensor_msgs::PointCloud2>("map_s", 5);
            // map_timer = nh_.createTimer(ros::Duration ( 0.1), &mapMaintainer::process, this);
            visual_timer = nh_.createTimer(ros::Duration (1), &mapMaintainer::visualThread, this);
        }

        void init_params(){
            float degree_per_layer = FOV / layers_per_map;
            for(size_t k=0; k < layers_per_map - 1; k++){
                boundry_tan_yz_x[k] = tan(degree2rad(degree_per_layer / 2.0f) * (k + 1));
                boundry_atan_y_z[k] = M_PI * 2.0f / cloud_num_per_map[k];
            }
            boundry_atan_y_z[layers_per_map - 1] = M_PI * 2.0f / cloud_num_per_map[layers_per_map - 1];
            
        }

        void time_count(std::string filename, void (*func)(void)){
            clock_t t = clock();
            func();
            double time = (double)(clock() - t) / CLOCKS_PER_SEC;
            std::cout << "time cost of one process is : " << "\t" << time << std::endl;
        }

        void check_cloud(const ros::TimerEvent& event){
            for(auto p : map_list_kd_){
               fprintf(record_file_, "%d,", p->validnum());
            }
            fprintf(record_file_, "\b\n");
            
        }

        void check_cloud(float time_cost){
            for(auto p : map_list_kd_){
               fprintf(record_file_, "%d,", p->validnum());
            }
            fprintf(record_file_, "%0.3f", time_cost);
            fprintf(record_file_, "\b\n");
    
        }

        
        // 定时任务，处理点云队列
        // TODO 取出点云中的离群点，参考LOAM
        void process(const ros::TimerEvent& event){
            if(cloud_queue_.size() > 1){
                ROS_INFO("process once\n");
                pcl::PointCloud<pcl::PointXYZI>::Ptr cl(new pcl::PointCloud<pcl::PointXYZI>);
                *cl = *(cloud_queue_.front().cloud);
                cloud_queue_.pop();
                *cl += *(cloud_queue_.front().cloud);
                cloud_queue_.pop();
                // auto start = chrono::high_resolution_clock::now();
                // auto start = ros::Time::now();
                processCloud2(cl);
                // auto end      = chrono::high_resolution_clock::now();
                // float time = float(chrono::duration_cast<chrono::microseconds>(end - start).count());
                // check_cloud(time);
                // ROS_INFO(" process time : %lf\n", time);
            }
        }

        void cloudLivxoCB(sensor_msgs::PointCloud2::ConstPtr msg_cloud){
            // ROS_INFO("*********************************");
            // ROS_INFO("time 1 : %lf", ros::Time::now().toSec());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*msg_cloud, *pcl_cloud);

            cloudFrame frame;
            frame.cloud = pcl_cloud;
            frame.cloud_size = pcl_cloud->size();
            cloud_queue_.push(frame);
            // ROS_INFO("queue size : %ld ", cloud_queue_.size());
            if(cloud_queue_.size() >= 10) {
                cloud_queue_.pop();
            }
            // ROS_INFO("time 2 : %lf", ros::Time::now().toSec());
            // clock_t t = clock();0
            sensor_msgs::PointCloud2 msg_map(*msg_cloud);
            msg_map.header = std_msgs::Header();
            msg_map.header.frame_id = "livox_frame";
            pub_scan_.publish(msg_map);
            processCloud(pcl_cloud);
            //利用点云图绘制深度图函数
            drawDepthImg(pcl_cloud);

            // double time = (double)(clock() - t) / CLOCKS_PER_SEC;
            // fprintf(record_file_t_, "%lf\n", time);
        }

        void processCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud){
            pcl::PointCloud<pcl::PointXYZI> cl;
            vox_filter_->setInputCloud(in_cloud);
            vox_filter_->filter(cl);

            size_t s = cl.points.size();
            for(size_t i=0; i < s; i++){
                auto p = cl.points[i];
                size_t ind = subMapInd(p.x, p.y, p.z);
                // ROS_INFO("time 3 : %lf, ind : %ld", ros::Time::now().toSec(), ind);
                // std::cout << p.intensity << std::endl;
                if((p.intensity < intensity_thresh_1) || 8p.intensity > intensity_thresh_2)
                this->map_list_[ind]->push_back(p);
                // ROS_INFO("time 4 : %lf", ros::Time::now().toSec());
            }
            // pcl::PointCloud<pcl::PointXYZI> visual_map;
            // visual_map.clear();
            for(size_t i=0; i < cloud_num_total; i++){
                vox_filter_->setInputCloud(map_list_[i]);
                vox_filter_->filter(*map_list_[i]);
                // visual_map += *map_list_[i];
            }
            // // ROS_INFO("map size %ld", visual_map.size());
            // sensor_msgs::PointCloud2 msg_map;
            // msg_map.header = std_msgs::Header();
            // msg_map.header.frame_id = "map_s";
            // pub_scan_.publish(msg_map);
        }


        // 队列处理函数
        void processCloud2(pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud){
            size_t s = in_cloud->points.size();
            
            for(size_t i=0; i < s; i++){
                auto p = in_cloud->points[i];
                size_t ind = subMapInd(p.x, p.y, p.z);
                if((p.intensity < intensity_thresh_1) || p.intensity > intensity_thresh_2){
                    this->sub_map_buffs_[ind].push_back(p);
                }
            }
            
            for(size_t i = 0; i < cloud_num_total; i++){
                if(map_list_kd_[i]->size() > 0 && this->sub_map_buffs_[i].size() > 0){
                    map_list_kd_[i]->Add_Points(this->sub_map_buffs_[i], true);
                } else if (map_list_kd_[i]->size() <= 0 && this->sub_map_buffs_[i].size() > 0){
                    map_list_kd_[i]->Build(this->sub_map_buffs_[i]);
                }
            }
        }

        //发布相机参数的话题，distortion_coefficients是D，camera_matrix是K，projection_matrix是P，rectification_matrix是R
        sensor_msgs::CameraInfo getCameraInfo(void){        
        sensor_msgs::CameraInfo cam;

        vector<double> D{-0.057268, 0.195162, -0.001675, -0.001939, 0.000000};
        boost::array<double, 9> K = {
            3514.32845,    0.     ,  954.57526,
               0.     , 3514.22922,  764.96044,
               0.     ,    0.     ,    1.     
        };

         boost::array<double, 12> P = {
            3500.05005,    0.     ,  952.05403,    0.     ,
               0.     , 3504.17798,  763.58506,    0.     ,
               0.     ,    0.     ,    1.     ,    0.     
        };
        boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

        cam.height = 480;
        cam.width = 640;
        cam.distortion_model = "plumb_bob";
        cam.D = D;
        cam.K = K;
        cam.P = P;
        cam.R = r;
        cam.binning_x = 0;
        cam.binning_y = 0;
        cam.header.frame_id = "narrow_stereo";  //frame_id为camera，也就是相机名字
        cam.header.stamp = ros::Time::now();
        cam.header.stamp.nsec = 0;
        return cam;
        }



        //用于将点云坐标转换为图像坐标这个函数拆成两部分，每收一个话题分别调用一个函数
        /*
        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                   const sensor_msgs::ImageConstPtr& image_msg,
                   const sensor_msgs::CameraInfoConstPtr& camera_info_msg){   
            // 点云转换PCL格式(不需要)
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::fromROSMsg(*cloud_msg, *cloud);

            // 相机参数转换至针孔相机模型
            image_geometry::PinholeCameraModel cam_model;
            cam_model.fromCameraInfo(camera_info_msg);//相机

            // 将每个点投影到图像上
            cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
            for (const auto& point : cloud->points)
            {
              cv::Point2d pixel;
              cam_model.project3dToPixel(cv::Point3d(point.x, point.y, point.z), pixel);
              cv::circle(image, pixel, 2, cv::Scalar(0, 0, 255), -1);
              //计算每个点对应的深度值
              Eigen::Vector3f point_vector=point.getVector3fMap();
              float depth=point_vector.norm();
              //cout<<"Depth:  "<< depth <<endl;
            }

          // 展示转换结果
          cv::imshow("Projected Point Cloud", image);
          cv::waitKey(1);
        } */ 

        //针孔相机模型
        void pinholecamera(const sensor_msgs::CameraInfoConstPtr& camera_info_msg){
            //image_geometry::PinholeCameraModel cam_model;
            cam_model.fromCameraInfo(camera_info_msg);
        }

        //点云点投影到图像上
        //void pcl2image(const sensor_msgs::ImageConstPtr& image_msg){
        //     // 将每个点投影到图像上
        //    image= cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
        //}
        //绘制深度图的函数,维护一张新的
        void drawDepthImg(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud){
            image=Mat::zeros(Size(2048,1536),CV_8UC3);
             for (const auto& point : pcl_cloud->points)//cloud->points ?
            {
              cv::Point2d pixel0;
              cam_model.project3dToPixel(cv::Point3d(point.x, point.y, point.z), pixel0);
              //cv::circle(image, pixel0, 2, cv::Scalar(0, 0, 255), -1);
              //计算每个点对应的深度值
              Eigen::Vector3f point_vector=point.getVector3fMap();
              float depth=point_vector.norm();
              if(0<=pixel0.x<=2048||0<=pixel0.y<=1536)
              {
              image.at<float>(pixel0.x,pixel0.y).[0]=point.x;
              image.at<float>(pixel0.x,pixel0.y).[1]=point.y;
              image.at<float>(pixel0.x,pixel0.y).[2]=point.z;
              //cout<<"Depth:  "<< depth <<endl;
              }
            }        
            // 展示转换结果
            cv::imshow("Projected Point Cloud", image);
        }
        //传入深度图上点的坐标，返回点云图里的三维坐标
        void returnxyz(const cv::Point2d& pixel){
            // 根据传入像素点的坐标，返回点云点的三维坐标
            for (x=0;x<=image.rows;x++)
            {
                for(y=0;y<=image.rows;y++)
                {
                    if (x==pixel.x&&y==pixel.y)
                    {

                    // Calculate 3D coordinates of point in point cloud
                    //Eigen::Vector3f point_vector = point.getVector3fMap();
                    //std::cout << "Point cloud coordinate: (" << point_vector(0) << ", " << point_vector(1) << ", " << point_vector(2) << ")" << std::endl;
                    float a=image.at<float>(pixel0.x,pixel0.y).[0]=point.x;
                    float b=image.at<float>(pixel0.x,pixel0.y).[1]=point.y;
                    float c=image.at<float>(pixel0.x,pixel0.y).[2]=point.z;
                    return;
                    }
                }
            }
              std::cout << "No point in point cloud corresponds to pixel coordinate (" << pixel.x << ", " << pixel.y << ")" << std::endl;
              cv::waitKey(1);
        }

        void turn2imagepoint(pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud) {
            pcl::PointCloud<pcl::PointXYZI> cl;

            //vox_filter_->setInputCloud(in_cloud);
            //vox_filter_->filter(cl);
            size_t s = cl.points.size();
            for(size_t i=0; i < s; i++){
                auto p = cl.points[i];
                size_t ind = subMapInd(p.x, p.y, p.z);

                // ROS_INFO("time 3 : %lf, ind : %ld", ros::Time::now().toSec(), ind);
                // std::cout << p.intensity << std::endl;
                //if((p.intensity < intensity_thresh_1) || 8p.intensity > intensity_thresh_2)
                //this->map_list_[ind]->push_back(p);
                // ROS_INFO("time 4 : %lf", ros::Time::now().toSec());
            }
            // pcl::PointCloud<pcl::PointXYZI> visual_map;
            // visual_map.clear();
            for(size_t i=0; i < cloud_num_total; i++){
                vox_filter_->setInputCloud(map_list_[i]);
                vox_filter_->filter(*map_list_[i]);
                // visual_map += *map_list_[i];
            }

        }
        

        // 可视化线程
        void visualThread(const ros::TimerEvent& event){
            static long int map_points_size = 0;
            map_points_size = 0;
            long int last_size = map_points_size;
            // std::cout << map_list_kd_[0]->PCL_Storage.size() << std::endl;
            pcl::PointCloud<pcl::PointXYZI> visual_map;
            visual_map.clear();
            for(size_t i=0; i < cloud_num_total; i++){
                visual_map += *map_list_[i];
                map_points_size += map_list_[i]->size();
            }
            // ROS_INFO("map size %ld", visual_map.size());
            sensor_msgs::PointCloud2 msg_map;
            pcl::toROSMsg(visual_map, msg_map);
            msg_map.header = std_msgs::Header();
            msg_map.header.frame_id = "map_s";
            pub_map_visual_split_.publish(msg_map);
        }

        int subMapInd(float x, float y, float z){
            // x=y=0的情况特殊处理
            if(y*y < 0.01 && z*z < 0.01) return 0;
            // 根据新的分层策略修改索引方法
            float dis_y_z = sqrtf32(z*z + y*y);
            float tan_fov2 = dis_y_z / x;

            // if(tan_fov2 <= boundry_tan_yz_x[0]) return 0;

            // 求层号
            int fovInd = layers_per_map - 1;
            for(size_t k=0; k < layers_per_map-1; k++){
                if (tan_fov2 <= boundry_tan_yz_x[k]){
                    fovInd = k;
                    break;
                }
            }
            
            // 求层内子点云编号
            float d_yz = atan2(y, z);
            // ROS_INFO("d_yz : %f, boundry_atan_y_z[fovInd] : %f", d_yz, boundry_atan_y_z[fovInd]);
            int circleInd = (int)ceil((d_yz + M_PI) / boundry_atan_y_z[fovInd]);
            if(circleInd >= cloud_num_per_map[fovInd]) circleInd = cloud_num_per_map[fovInd];

            // ROS_INFO("fovInd : %d, circleInd : %d", fovInd, circleInd);
            // 上一层的最大索引加上这层的编号，即对应子点云的总编号
            return min_index_submap[fovInd] + circleInd - 1;
        }

    private:
        ros::NodeHandle nh_;
        ros::Timer timer;
        ros::Subscriber sub_livox_cloud_;
        ros::Publisher pub_scan_;
        ros::Publisher pub_map_visual_split_;
        std::string cloud_topic_name_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_;
        std::queue<cloudFrame> cloud_queue_;
        int cloud_num_total;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_list_;

        image_geometry::PinholeCameraModel cam_model;
        cv::Mat image;
        

        std::vector<KD_TREE<pcl::PointXYZI>::PointVector> sub_map_buffs_;
        std::vector<KD_TREE<pcl::PointXYZI>::Ptr> map_list_kd_;
        ros::Timer map_timer, visual_timer;

        pcl::VoxelGrid<pcl::PointXYZI>::Ptr vox_filter_;

        // EVALUATE
        FILE * record_file_;
        FILE * record_file_t_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "test_node");   //初始化了一个节点
    ROS_INFO("Start node");

    mapMaintainer m("livox/lidar");
    
    //ros::init(argc, argv, "camera_info");  
    ros::NodeHandle n;  //是否需要ros::NodeHandle nh;
    ros::Publisher pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1000);  
    sensor_msgs::CameraInfo camera_info_dyn;
    ros::Rate rate(10);  //点云更新频率

    // 订阅点云、图像话题
    //ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("point_cloud_topic", 1, boost::bind(cloudCallback, _1, image_msg, camera_info_msg));
    //ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("image_topic", 1, boost::bind(cloudCallback, cloud_msg, _1, camera_info_msg));
    //ros::Subscriber camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info_dyn", 1, boost::bind(cloudCallback, cloud_msg, image_msg, _1));
    //订阅处理好的相机参数话题，并调用针孔相机模型函数
    ros::Subscriber camerainfo_sub = nh_.subscribe("/camera/camera_info", 5, &mapMaintainer::pinholecamera, &m);
    //订阅视觉识别结果话题，同时调用返回三维坐标函数
    ros::Subscriber pixel_sub = nh_.subscribe(pixel_topic_name_, 5, &mapMaintainer::returnxyz, &m);



    while (ros::ok())
    {
        camera_info_dyn = m.getCameraInfo();
        pub.publish(camera_info_dyn); //发布出去
        rate.sleep();
        ros::spinOnce();
    }

    //ros::init(argc, argv, "point_cloud_projection");
   

    // ros::spin()

    std::cout << "#### Node Exited ####" << std::endl;
    return 0;
}