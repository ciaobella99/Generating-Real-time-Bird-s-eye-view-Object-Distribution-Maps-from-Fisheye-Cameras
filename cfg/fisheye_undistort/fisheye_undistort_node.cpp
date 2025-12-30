#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h> 
#include <ros/package.h>          // for ros::package::getPath

class FisheyeUndistortNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport it_;
  image_transport::ImageTransport private_it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher      info_pub_;
  sensor_msgs::CameraInfo cam_info_msg_;
  
  cv::Mat map1_, map2_;
  cv::Mat K_new_; 

public:
  FisheyeUndistortNode() : nh_(), private_nh_("~"), it_(nh_), private_it_(private_nh_)
  {
    // 打印 OpenCV 版本
    ROS_INFO("OpenCV version: %s", CV_VERSION);
    
    try {
      // 使用私有節點句柄讀取參數
      XmlRpc::XmlRpcValue K_list, D_list;
      int width, height;
      
      // 直接使用參數名稱，不帶 ~
      if (!private_nh_.getParam("K", K_list)) {
        ROS_ERROR("Failed to get parameter 'K'");
        throw std::runtime_error("Missing parameter 'K'");
      }
      
      if (!private_nh_.getParam("D", D_list)) {
        ROS_ERROR("Failed to get parameter 'D'");
        throw std::runtime_error("Missing parameter 'D'");
      }
      
      if (!private_nh_.getParam("width", width)) {
        ROS_ERROR("Failed to get parameter 'width'");
        throw std::runtime_error("Missing parameter 'width'");
      }
      
      if (!private_nh_.getParam("height", height)) {
        ROS_ERROR("Failed to get parameter 'height'");
        throw std::runtime_error("Missing parameter 'height'");
      }
      
      // 檢查參數類型
      if (K_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter 'K' is not an array");
        throw std::runtime_error("Invalid parameter type for 'K'");
      }
      
      if (D_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter 'D' is not an array");
        throw std::runtime_error("Invalid parameter type for 'D'");
      }
      
      // 輸出參數大小便於調試
      ROS_INFO("K array size: %d", K_list.size());
      ROS_INFO("D array size: %d", D_list.size());
      
      // 將 XML-RPC 值轉換為 OpenCV 矩陣
      cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          int idx = i*3 + j;
          if (idx < K_list.size()) {
            double val = 0.0;
            switch (K_list[idx].getType()) {
              case XmlRpc::XmlRpcValue::TypeInt:
                val = static_cast<double>(static_cast<int>(K_list[idx]));
                break;
              case XmlRpc::XmlRpcValue::TypeDouble:
                val = static_cast<double>(K_list[idx]);
                break;
              default:
                ROS_WARN("K[%d,%d] has unexpected type, using 0", i, j);
            }
            K.at<double>(i, j) = val;
          }
        }
      }
      
      // 處理扭曲係數
      cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
      for (int i = 0; i < std::min(4, (int)D_list.size()); i++) {
        double val = 0.0;
        switch (D_list[i].getType()) {
          case XmlRpc::XmlRpcValue::TypeInt:
            val = static_cast<double>(static_cast<int>(D_list[i]));
            break;
          case XmlRpc::XmlRpcValue::TypeDouble:
            val = static_cast<double>(D_list[i]);
            break;
          default:
            ROS_WARN("D[%d] has unexpected type, using 0", i);
        }
        D.at<double>(i) = val;
      }
      
      
      cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

    // 1) 計算新的相機矩陣 K_new
      cv::Mat K_new;
      // balance: [0,1]，0=裁到最小失真、1=保留最多視場；fov_scale 預設1.0
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
        K, D, cv::Size(width, height),
        R,       // rectification rotation
        K_new,   // output new intrinsic
        /*balance=*/1.0,
        cv::Size(width, height),
        /*fov_scale=*/1.0
      );

      // 2) 用 K_new 生成 undistort map
      cv::fisheye::initUndistortRectifyMap(
        K, D, R, K_new,
        cv::Size(width, height),
        CV_32FC1,
        map1_, map2_
      );


      // 儲存 K_new 到成員變量，方便日後輸出
      K_new_ = K_new.clone();

      std::string pkg_dir  = ros::package::getPath("fisheye_undistort");
      std::string yaml_file = pkg_dir + "/config/camera_intrinsics.yaml";

      // 在寫檔前
      ROS_INFO_STREAM("→ 要把新內參寫到: " << yaml_file);

      // 開檔
      cv::FileStorage fs(yaml_file, cv::FileStorage::WRITE);
      if (!fs.isOpened()) {
        ROS_ERROR_STREAM("✗ 無法開啟檔案做寫入: " << yaml_file);
      } else {
        ROS_INFO("✔ FileStorage opened successfully");
        fs << "fx" << K_new_.at<double>(0,0)
            << "fy" << K_new_.at<double>(1,1)
            << "cx" << K_new_.at<double>(0,2)
            << "cy" << K_new_.at<double>(1,2);
        fs.release();
        ROS_INFO("✔ 已經寫入 camera_intrinsics.yaml");
      }

      // 3) 設定並發布 CameraInfo
      info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info_rect", 1);
      cam_info_msg_.header.frame_id = "fisheye_front";
      cam_info_msg_.width  = width;
      cam_info_msg_.height = height;
      // 填入 K_new (row-major)
      // 填 K
      cam_info_msg_.K[0] = K_new_.at<double>(0,0);
      cam_info_msg_.K[1] = 0;
      cam_info_msg_.K[2] = K_new_.at<double>(0,2);
      cam_info_msg_.K[3] = 0;
      cam_info_msg_.K[4] = K_new_.at<double>(1,1);
      cam_info_msg_.K[5] = K_new_.at<double>(1,2);
      cam_info_msg_.K[6] = 0;
      cam_info_msg_.K[7] = 0;
      cam_info_msg_.K[8] = 1;

      // 填 P (3×4)
      cam_info_msg_.P[0]  = K_new_.at<double>(0,0);
      cam_info_msg_.P[1]  = 0;
      cam_info_msg_.P[2]  = K_new_.at<double>(0,2);
      cam_info_msg_.P[3]  = 0;
      cam_info_msg_.P[4]  = 0;
      cam_info_msg_.P[5]  = K_new_.at<double>(1,1);
      cam_info_msg_.P[6]  = K_new_.at<double>(1,2);
      cam_info_msg_.P[7]  = 0;
      cam_info_msg_.P[8]  = 0;
      cam_info_msg_.P[9]  = 0;
      cam_info_msg_.P[10] = 1;
      cam_info_msg_.P[11] = 0;


      
      // 日誌輸出
      ROS_INFO_STREAM("K matrix: " << K);
      ROS_INFO_STREAM("D coeffs: " << D);
      ROS_INFO_STREAM("Image dimensions: " << width << "x" << height);
      
      // try {
      //   // 生成校正映射
      //   cv::fisheye::initUndistortRectifyMap(
      //     K, D, R, P,
      //     cv::Size(width, height),
      //     CV_32FC1,
      //     map1_, map2_
      //   );
      //   ROS_INFO("Successfully generated undistortion maps");
      // } catch (const cv::Exception& e) {
      //   ROS_ERROR("OpenCV error in initUndistortRectifyMap: %s", e.what());
      //   throw;
      // }
      
      // 訂閱和發布 - 使用私有 ImageTransport
      image_sub_ = private_it_.subscribe("image_raw", 1, &FisheyeUndistortNode::imageCallback, this);
      image_pub_ = private_it_.advertise("image_rect", 1);
      
      // 顯示實際的完整主題名稱（用於調試）
      std::string input_topic = ros::names::resolve(private_nh_.getNamespace() + "/image_raw");
      std::string output_topic = ros::names::resolve(private_nh_.getNamespace() + "/image_rect");
      
      ROS_INFO_STREAM("Subscribing to: " << input_topic);
      ROS_INFO_STREAM("Publishing to: " << output_topic);
      
    } catch (const std::exception& e) {
      ROS_ERROR("Initialization error: %s", e.what());
      throw;
    }
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try {
      // 打印接收到的圖像信息
      ROS_INFO("Received image: %dx%d, encoding: %s", 
               msg->width, msg->height, msg->encoding.c_str());
      
      // 轉換 ROS 圖像為 OpenCV 格式
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      
      // 校正圖像
      cv::Mat undistorted;
      cv::remap(cv_ptr->image, undistorted, map1_, map2_, cv::INTER_LINEAR);
      
      // 轉換回 ROS 消息並發布
      sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(
        msg->header, sensor_msgs::image_encodings::BGR8, undistorted).toImageMsg();
      image_pub_.publish(out_msg);
      // 同步輸出 CameraInfo
      cam_info_msg_.header.stamp = msg->header.stamp;
      info_pub_.publish(cam_info_msg_);

      
      ROS_INFO("Published undistorted image");
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (cv::Exception& e) {
      ROS_ERROR("OpenCV exception: %s", e.what());
    } catch (const std::exception& e) {
      ROS_ERROR("Standard exception: %s", e.what());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fisheye_undistort");
  
  try {
    ROS_INFO("Initializing FisheyeUndistortNode...");
    FisheyeUndistortNode node;
    ROS_INFO("Node initialized successfully, entering spin...");
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("Node initialization error: %s", e.what());
    return 1;
  }
  
  return 0;
}