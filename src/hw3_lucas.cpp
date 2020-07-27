# include "ros/ros.h"
# include "sensor_msgs/Imu.h"
# include "geometry_msgs/Vector3.h"
# include "geometry_msgs/Point.h"
# include "visualization_msgs/Marker.h"
# include "Eigen/Dense"




Eigen::Vector3d b_acc,b_gyr,m_acc,m_gyr,acc_global,gry_global;
Eigen::Vector3d gravity_global(0, 0, 9.81), velocity_global(0, 0, 0), s_global(0, 0, 0);
Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
double now=0, past=0, dt=0;
bool update=false;

void callback(const sensor_msgs::Imu::ConstPtr& msg){  //訂閱器呼叫函式幫忙讀topic內部資料
    now = msg -> header.stamp.sec + double(msg -> header.stamp.nsec) * 1e-9; //header 在終端機怎看 （header.stamp.tosec() ）
    Eigen::Vector3d gyr(msg -> angular_velocity.x,msg -> angular_velocity.y,msg -> angular_velocity.z);
    Eigen::Vector3d acc(msg -> linear_acceleration.x,msg -> linear_acceleration.y,msg -> linear_acceleration.z);
      if (past != 0){
        dt = now-past;
        m_gyr << (b_gyr.x() + gyr.x())/2, (b_gyr.y() + gyr.y())/2, (b_gyr.z() + gyr.z())/2;
        m_acc << (b_acc.x() + acc.x())/2, (b_acc.y() + acc.y())/2, (b_acc.z() + acc.z())/2;
      }
      else{
        dt=0;
      }
    b_gyr = gyr;
    b_acc = acc;
    past = now;
    update = true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "hw3_lucas");      //visualize_path_node檔名
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber sub = n.subscribe("imu/data", 1000, callback);
    ros::Rate rate(200);// 200HZ
    visualization_msgs::Marker line_strip;

    while (ros::ok()){
          line_strip.header.frame_id = "/map";
          line_strip.header.stamp = ros::Time::now();
          line_strip.ns = "points_and_lines";
          line_strip.action = visualization_msgs::Marker::ADD;
          line_strip.pose.orientation.w = 1.0;
          line_strip.id = 1;
          line_strip.type = visualization_msgs::Marker::LINE_STRIP;
          line_strip.scale.x = 0.1;
          line_strip.color.b = 1.0;
          line_strip.color.a = 1.0; //看教學

      if (dt != 0 && update == true){
          Eigen::Matrix3d B;
          B<<             0,-m_gyr.z()*dt, m_gyr.y()*dt,
               m_gyr.z()*dt,            0,-m_gyr.x()*dt,
              -m_gyr.y()*dt, m_gyr.x()*dt,            0;

          double sigma = (m_gyr * dt).norm(); //絕對值？ 算長度
          C = C * (I + (sin(sigma) / sigma) * B + ((1 - cos(sigma))/ pow(sigma, 2.0)) * B * B);
          acc_global = C * m_acc;
          velocity_global = velocity_global + dt * (acc_global - gravity_global);
          s_global = s_global + dt * velocity_global;

          geometry_msgs::Point p;
          p.x = s_global.x();
          p.y = s_global.y();
          p.z = s_global.z();

          line_strip.points.push_back(p);
          update = false;
          marker_pub.publish(line_strip);
      }
      ros::spinOnce(); //跟著rate的速度回傳值 spinonce 會繼續執行下面code spin則不會
      rate.sleep();    //跟著rate走 不會受rosbag影響
        }
        return 0;
    }
