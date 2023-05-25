#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>
#include "livox_sdk.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "queue"
#include "condition_variable"
#include "mutex"
#include "pcl/filters/voxel_grid.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"

const uint32_t KEthPacketMaxLength = 1500;
const size_t packet_length_table[] = {1362};
const size_t point_per_packet_table[] = {96};
const int update_pack = 2;

std::mutex queue_mutex;
std::condition_variable cv_queue;

typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

typedef struct {
//   uint64_t time_rcv; /**< receive time when data arrive */
  uint32_t point_num;
  uint8_t raw_data[KEthPacketMaxLength];
} StoragePacket;
std::queue<StoragePacket*> data_q;
uint64_t point_nums(0);
// TODO 设计雷达数据队列，以及对应的条件量

// TODO 实现地图更新程序
void UpdateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr& map){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t k=0; k < update_pack; k++){
        StoragePacket* pack = data_q.front();
        LivoxEthPacket * ethpack = reinterpret_cast<LivoxEthPacket *>(pack->raw_data);
        LivoxPoint* points = reinterpret_cast<LivoxPoint *>(ethpack->data);
        for(size_t l=0; l < point_per_packet_table[0]; l++){
            pcl::PointXYZ p;
            p.x = points[l].x / 1000.0f;
            p.y = points[l].y / 1000.0f;
            p.z = points[l].z / 1000.0f;
            cloud_r->push_back(p);
        }
		data_q.pop();
    }
    *map += *cloud_r;
};

void vox_vis_map(pcl::PointCloud<pcl::PointXYZ>::Ptr& map, pcl::VoxelGrid<pcl::PointXYZ>& filter, ros::Publisher& pub_){
	filter.setInputCloud(map);
	filter.filter(*map);

	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*map, cloud_msg);
	cloud_msg.header.frame_id = "map";
	cloud_msg.header.stamp = ros::Time::now();
	pub_.publish(cloud_msg);
}

DeviceItem devices[kMaxLidarCount];
uint32_t data_recveive_count[kMaxLidarCount];

/** Connect all the broadcast device. */
int lidar_count = 0;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];

/** Connect the broadcast device in list, please input the broadcast code and modify the BROADCAST_CODE_LIST_SIZE. */
/*#define BROADCAST_CODE_LIST_SIZE  3
int lidar_count = BROADCAST_CODE_LIST_SIZE;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize] = {
  "000000000000002",
  "000000000000003",
  "000000000000004"
};*/

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  if (data) {
    data_recveive_count[handle] ++ ;
    if (data_recveive_count[handle] % 100 == 0) {
      /** Parsing the timestamp and the point cloud data. */
      uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
      if ( data ->data_type == kExtendCartesian) {
        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
        // TODO 将雷达原始数据压入队列，并根据数据量启动地图更新线程
        StoragePacket* packet = new StoragePacket;
        memcpy(packet->raw_data, data, packet_length_table[0]);
        packet->point_num = data_num;
        data_q.push(packet);
        point_nums += data_num;

        // 通知处理线程
      }else if ( data ->data_type == kExtendSpherical) {
        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kDualExtendCartesian) {
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
      }else if ( data ->data_type == kDualExtendSpherical) {
        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
      }
    }
  }
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
  printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      devices[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data) {
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
  if (status != kStatusSuccess) {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack) {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

void LidarConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  QueryDeviceInformation(handle, OnDeviceInformation, NULL);
  if (devices[handle].device_state == kDeviceStateDisconnect) {
    devices[handle].device_state = kDeviceStateConnect;
    devices[handle].info = *info;
  }
}

void LidarDisConnect(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info) {
  uint8_t handle = info->handle;
  devices[handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventConnect) {
    LidarConnect(info);
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    LidarDisConnect(info);
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    LidarStateChange(info);
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (devices[handle].device_state == kDeviceStateConnect) {
    printf("Device Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Device feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal) {
      LidarStartSampling(handle, OnSampleCallback, NULL);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL || info->dev_type == kDeviceTypeHub) {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  if (lidar_count > 0) {
    bool found = false;
    int i = 0;
    for (i = 0; i < lidar_count; ++i) {
      if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess) {
    /** Set the point cloud data for a specific Livox LiDAR. */
    SetDataCallback(handle, GetLidarData, NULL);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "livox_node");
	ros::NodeHandle nh_;
	ros::Publisher map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("livox_map", 5);
  	printf("Livox SDK initializing.\n");
	/** Initialize Livox-SDK. */
	if (!Init()) {
		return -1;
	}
	printf("Livox SDK has been initialized.\n");

	LivoxSdkVersion _sdkversion;
	GetLivoxSdkVersion(&_sdkversion);
	printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

	memset(devices, 0, sizeof(devices));
	memset(data_recveive_count, 0, sizeof(data_recveive_count));

	/** Set the callback function receiving broadcast message from Livox LiDAR. */
	SetBroadcastCallback(OnDeviceBroadcast);

	/** Set the callback function called when device state change,
	 * which means connection/disconnection and changing of LiDAR state.
	 */
	SetDeviceStateUpdateCallback(OnDeviceInfoChange);

	/** Start the device discovering routine. */
	if (!Start()) {
		Uninit();
		ROS_ERROR("Start Lidar Failed.");
		return -1;
	}
	printf("Start discovering device.\n");

	//   sleep(30);
    // ros::spin();
	ros::Rate loop_r(50);
	uint8_t count = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
	pcl::VoxelGrid<pcl::PointXYZ> vox_f_;
	vox_f_.setInputCloud(map);
	vox_f_.setLeafSize(0.1f, 0.1f, 0.1f);
	while(ros::ok()){
		if(data_q.size() >= 2){
			UpdateMap(map);
			count ++;
		}
		if(0==count%5){
			// filter
			// visualize
			ROS_INFO("Visual PUB");
			count = count % 5;
			vox_vis_map(map, vox_f_, map_pub_);
		}
		ros::spinOnce();
		// loop_r.sleep();
    }

	int i = 0;
	for (i = 0; i < kMaxLidarCount; ++i) {
		if (devices[i].device_state == kDeviceStateSampling) {
			/** Stop the sampling of Livox LiDAR. */
			LidarStopSampling(devices[i].handle, OnStopSampleCallback, NULL);
		}
	}
	std::cout << "UNINIT " << std::endl;
	/** Uninitialize Livox-SDK. */
	Uninit();
}
