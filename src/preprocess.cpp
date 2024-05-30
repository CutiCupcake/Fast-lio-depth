#include "preprocess.h"
#include <pcl/filters/bilateral.h>
#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.1), point_filter_num(1)
{
  inf_bound = 2;
  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disB = 0.0; // B?
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;
  
  case tof:
    tof_handler(msg);
    break;

  default:
    printf
    
    ("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;
  
  if (feature_enabled)
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

        bool is_new = false;
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    for(int j=0; j<N_SCANS; j++)
    {
      if(pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types);
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);//消息类型转成点云类型，记源记尾，才会理解和有印象。
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)//***scans可以设为点云的行***
    {
      pl_buff[i].clear();//pl_buff指的是扫描线数组，对pl_buff向量中的每一个
      pl_buff[i].reserve(plsize);//pl_buff[128] 声明了一个最多包含128个 PointCloudXYZI 对象的数组。这意味着这个数组可以存储来自最多128线激光雷达的点云数据，其中每一线对应数组中的一个 PointCloudXYZI 对象。
    }//对每个扫描线


    for (uint i = 0; i < plsize; i++)//遍历所有点云，plsize是此帧点云所有的点。
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;//去除盲点
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;//无用
      PointType added_pt;//xyznormal格式的点云点 add_pt
 
      added_pt.x = pl_orig.points[i].x;//每个点都赋予到xyznormal格式的点云点中，PointType 代表着typedef pcl::PointXYZINormal PointType;
      //**定义点云的语句：pcl::点云<每个点的格式> 名字
      //pcl：：pointcloud<pcl::pointXYZ> pl

      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;//分别设置它的xyz的法向量为0
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;//算y轴除以x轴，点与x轴的夹角，并且转化成角度。
      if (yaw_angle >= 180.0)//如果与x轴超过180度。也就是落在第三象限
        yaw_angle -= 360.0;//按逆时针计算坐标，这样就能落在-180到180之间。
      if (yaw_angle <= -180.0)//
        yaw_angle += 360.0;//按顺时针计算坐标，

      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;//计算xyznormal格式的点云点 add_pt的时间信息。这步可以删去。
      //***这步可以删去！***
      if(pl_orig.points[i].ring < N_SCANS && pl_orig.points[i].ring!= 0)//如果ring信息在扫描线数目内，则把这个点加入pl——buff对应的扫描线内，比如pl_buff[3]=add_pt
      {//***可以把点所处的行列信息加入到属性中，主要是行信息！***
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);//***pl_buff存储着扫描线信息，将这些个点分门别类到各个的扫描线，也就是点云行中。***
      }
    }//每个点云去除盲点操作，计算每个点云点的与x轴的夹角yaw，并且把其信息重新赋值给xyznormal格式的点云点add_pt后，加入到对应的扫描线点云pl_buff，分好类。
    //这一步处理了这一帧所有点云并且整理成xyznormal格式的点云点 add_pt ，放入了pl_buff里，并且计算出每个点云点的yaw角
    //***这一步把每个pl_buff数组都填满了，每一个pl_buff[n]都相当于第n行的所有点云存储在里面的向量。
    for (int j = 0; j < N_SCANS; j++)//对每个扫线 ***每一行进行操作
    {
      PointCloudXYZI &pl = pl_buff[j];//第J个扫描线的点云集合赋给pl，typedef pcl::PointCloud<PointType> PointCloudXYZI;
      //&符号用于声明一个引用。引用是一个已经存在的变量的别名。在您的代码示例 PointCloudXYZI &pl = pl_buff[j]; 中，pl是对pl_buff[j]的引用。这意味着pl和pl_buff[j]指向的是同一个对象，在pl上做的任何修改都会反映在pl_buff[j]上。
      //要是没有&，则相当于复制过去。
      int linesize = pl.size();//读取这个扫描线有多少个点云。
      vector<orgtype> &types = typess[j];//vector<orgtype> typess[128];也是个别名操作。表示typess是一个包含128个vector<orgtype>元素的数组，它的每一个元素都是一个vector而非orgtype
      //每个vector<orgtype>元素本身是一个动态大小的数组，可以容纳任意数量的orgtype类型的元素。
      //所以types可以阔列成点云点。types[i]
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)//对这个扫描线【j】内的所有点
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }//对于每个点 pl[i]，计算其 range（在水平面上的到**原点**的距离）和 dista（与在当前扫描线的**下一个点的**距离的平方）。这里使用了欧几里得距离的公式。
      //并且存入types【i】的range和dista里。这么看，types【i】应该是在第J条扫描线的i点的距离属性。
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);//扫描线上的最后一个点 pl[linesize] 的 range 被单独计算，但其 dista 未被设置（因为它没有下一个点）。
      give_feature(pl, types);//把这条扫描线（包含其所有的点云），和此线中的每一个点的属性放入give_feature中进行处理。
    }//对这帧点云的每条扫描线，计算出每个点的到原心的水平距离、到下一个点的欧式距离的平方、存入数组types中，types是一个动态大小的数组，可以容纳任意数量的orgtype类型（扫描线的点的信息）的元素
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}


void Preprocess::tof_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<tof_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);//ros转成数据结构xyz
  int plsize = pl_orig.size();//获得点云大小
  pl_corn.reserve(plsize);//重置角点
  pl_surf.reserve(plsize);//重置平面点

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)//***scans可以设为点云的行***
    {
      pl_buff[i].clear();//pl_buff指的是扫描线数组，对pl_buff向量中的每一个
      pl_buff[i].reserve(plsize);//pl_buff[128] 声明了一个最多包含128个 PointCloudXYZI 对象的数组。这意味着这个数组可以存储来自最多128线激光雷达的点云数据，其中每一线对应数组中的一个 PointCloudXYZI 对象。
    }//对每个扫描线


    for (uint i = 0; i < plsize; i++)//遍历所有点云，plsize是此帧点云所有的点。
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;//去除盲点
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;//无用
      //ROS_INFO_STREAM(" No: " << i << "  points: " << pl_orig.points[i].x<<pl_orig.points[i].y<<pl_orig.points[i].z<<"  rings:  "<<pl_orig.points[i].ring);
      PointType added_pt;//xyznormal格式的点云点 add_pt
      added_pt.x = pl_orig.points[i].x;//每个点都赋予到xyznormal格式的点云点中，PointType 代表着typedef pcl::PointXYZINormal PointType;
      //**定义点云的语句：pcl::点云<每个点的格式> 名字
      //pcl：：pointcloud<pcl::pointXYZ> pl

      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;//分别设置它的xyz的法向量为0
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;//算y轴除以x轴，点与x轴的夹角，并且转化成角度。
      if (yaw_angle >= 180.0)//如果与x轴超过180度。也就是落在第三象限
        yaw_angle -= 360.0;//按逆时针计算坐标，这样就能落在-180到180之间。
      if (yaw_angle <= -180.0)//
        yaw_angle += 360.0;//按顺时针计算坐标，

      added_pt.curvature = 0;//计算xyznormal格式的点云点 add_pt的时间信息。这步可以删去。
      //***这步可以删去！***
      if(pl_orig.points[i].ring < N_SCANS && pl_orig.points[i].ring!= 0)//如果ring信息在扫描线数目内，则把这个点加入pl——buff对应的扫描线内，比如pl_buff[3]=add_pt,//0行包含大量不确定值和NaN值
      {//***可以把点所处的行列信息加入到属性中，主要是行信息！***
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);//***pl_buff存储着扫描线信息，将这些个点分门别类到各个的扫描线，也就是点云行中。***
      //ROS_INFO("pl_buff:%i,%d,",i,pl_orig.points[i].ring);
      }
    }//每个点云去除盲点操作，计算每个点云点的与x轴的夹角yaw，并且把其信息重新赋值给xyznormal格式的点云点add_pt后，加入到对应的扫描线点云pl_buff，分好类。
    //这一步处理了这一帧所有点云并且整理成xyznormal格式的点云点 add_pt ，放入了pl_buff里，并且计算出每个点云点的yaw角
    //***这一步把每个pl_buff数组都填满了，每一个pl_buff[n]都相当于第n行的所有点云存储在里面的向量。
    for (int j =1; j < N_SCANS-2; j++)//对每个扫线 ***每一行进行操作，且第0条扫描线应该被跳过。
    {
     
      PointCloudXYZI &pl = pl_buff[j];//第J个扫描线的点云集合赋给pl，typedef pcl::PointCloud<PointType> PointCloudXYZI;
          if (pl.empty()) {
        continue;  // 跳过空扫描线，跳到 for 循环的下一个迭代，j 会自动增加
      }
      //ROS_INFO("pl:NO.%i,%i",j,pl.size());
      //&符号用于声明一个引用。引用是一个已经存在的变量的别名。在您的代码示例 PointCloudXYZI &pl = pl_buff[j]; 中，pl是对pl_buff[j]的引用。这意味着pl和pl_buff[j]指向的是同一个对象，在pl上做的任何修改都会反映在pl_buff[j]上。
      //要是没有&，则相当于复制过去。
      int linesize = pl.size();//读取这个扫描线有多少个点云。
      //ROS_INFO("pl:NO.%i,%i",j,linesize);
      vector<orgtype> &types = typess[j];//vector<orgtype> typess[128];也是个别名操作。表示typess是一个包含128个vector<orgtype>元素的数组，它的每一个元素都是一个vector而非orgtype
      //每个vector<orgtype>元素本身是一个动态大小的数组，可以容纳任意数量的orgtype类型的元素。
      //所以types可以阔列成点云点。types[i]
      types.clear();
      types.resize(linesize);
      linesize--;
      //ROS_INFO("hi,J:%i,linesize:%i",j,linesize);
      for (uint i = 0; i < linesize; i++)//对这个扫描线【j】内的所有点
      {
        //ROS_INFO("pl:NO.%f,%f",pl[i].x,pl[i + 1].x);
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
        //ROS_INFO("pl:NO.%i,range:%f,dista:%f",i,types[i].range,types[i].dista);
      }
      //ROS_INFO("hi,there,endddd");
      //对于每个点 pl[i]，计算其 range（在水平面上的到**原点**的距离）和 dista（与在当前扫描线的**下一个点的**距离的平方）。这里使用了欧几里得距离的公式。
      //并且存入types【i】的range和dista里。这么看，types【i】应该是在第J条扫描线的i点的距离属性。
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);//扫描线上的最后一个点 pl[linesize] 的 range 被单独计算，但其 dista 未被设置（因为它没有下一个点）。
              //ROS_INFO("hi,there:%i",j);
      give_feature(pl, types);//把这条扫描线（包含其所有的点云），和此线中的每一个点的属性放入give_feature中进行处理。
                    //ROS_INFO("hi,there,end");
    }//对这帧点云的每条扫描线，计算出每个点的到原心的水平距离、到下一个点的欧式距离的平方、存入数组types中，types是一个动态大小的数组，可以容纳任意数量的orgtype类型（扫描线的点的信息）的元素
  }
  else{
  
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      //if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      //added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);  
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS,true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
      
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer  = pl_orig.points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

        if (!given_offset_time)
        {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)//对每一行点云，和其点的距离属性types进行输入。
{
  int plsize = pl.size();//此条扫描线的点云数。
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }//跳过盲点

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;//groupsize=8.为什么？，如果扫描线点云数目大于8，则减去8，否则置0。***前后排除八个点。***

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());//Eigen::Vector3d vec(anotherVec);将创建一个新向量，其值从anotherVec复制过来
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;//i2也是没被赋值的变量。
  //i_nex=0,
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for(uint i=head; i<plsize2; i++)//第一个小于盲点的点的索引head赋予i，i<plsize2，是上面的与8做比较的一个数值。
  {
    if(types[i].range < blind)//再继续检查水平距离是否小于盲点
    {
      continue;
    }

    i2 = i;
    //ROS_INFO("i = %i",i);
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);//用于是否是平面，如果是，则返回1。

    //对每隔i+groupsize长度的线段进行判断。如果：1.任意点是盲点，整段线段丢弃。返回2和000方向向量，i_nex回归到当前的遍历点i（本来是到了i+groupsize）。

    //i_nex=0,curr_direct是三维0向量。types是当前点云的距离信息。pl是当前点云点集合。
    //ROS_INFO("After judge,i_nex = %i",i_nex);
    //ROS_INFO("plane_type = %i",plane_type);
    //ROS_WARN("    types.size = %i",types.size());

    if(plane_type == 1) 
    {
      for(uint j=i; j<=i_nex; j++)//j=195,j<=201,j++
      { 
        if(j!=i && j!=i_nex)//j!=
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;//
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        //ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
         //ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        //ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;//i_cur点到原点的距离，乘以A+B是在？
  group_dis = group_dis * group_dis;//平方
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;//定义一个数组disarr
  disarr.reserve(224);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)//attention i_cur+8是否越界？ 排除前八个点。i_nex=i_cur，i_nex从头开始，每次扫描完都更新到最新的i_cur位置
 
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }
   //对当前扫描线的每隔groupsize个点进行判断。若i_cur或者i_nex越界，即遍历到头，跳出循环；
  //只要某个点不是blind，将其的dista加入disarr（只要不进行return，disarr就有用。一旦return，disarr就释放。）
  //经过上述循环，此时i_nex = i_cur+group_size ；disarr至少含有8个数组。
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;//1.越界跳出，2.zero

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;//计算i_cur+group_size到i_cur的距离欧式距离。即这一段线段的首尾的欧式距离
    two_dis = vx*vx + vy*vy + vz*vz;//计算i_cur+group_size到i_cur的距离欧式距离。即这一段线段的首尾的欧式距离
    if(two_dis >= group_dis)
    {
      break;//如果首尾距离，大于，range的线型距离。则跳出。
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;//否则，首尾距离较小，此时i_nex > i_cur+group_size， i_nex++。并且将其存入disarr
    //至此，disarr里全部都是从icur 到i nex的所有dista
  }//此语句结束，i_nex要么是到扫描线最后一个，此时disarr存入太多；要么就是因为首尾距离太大跳出，这种情况disarr不会存入过多，所以不会报内存错误？
    //这行代码的目的是找出此扫描段的首尾距离最接近group_dis的片段。

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }
  //通过上述语句找出最接近groupdis的片段，icur——inex向量。
//leng_wid为固定底inex-icur后，最大的面积
//这一段的目的是找出此片段的最离群的点。
  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }
//求出这个点距离底边的高。如果高小于0.06m，则返回。（面不能太平整？要一点起伏？不要太平整的点？为何？可以考虑把p2l调大一点。）
  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }
//将disarr进行排序，也就是此扫描线的每个点到下一个点的欧式距离进行从大到小排序。
  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }
//如果排序后的disarr数组的倒数第二小的距离比10^-16次方还要小，则返回0。（可能发生吗？）
  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {//最大的距离除以倒数二小的距离，如果大于3.24倍。也不要。说明有边缘的跨变？
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  //经过上述的筛选。都没满足，则输出这个icur到inex的向量。这个向量长度是最接近于groupsize的。
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}
