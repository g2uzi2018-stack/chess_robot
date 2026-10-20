#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <iomanip>

int main(int argc,char**argv){
  try{
    if(argc!=2)throw std::runtime_error("usage: model_fk.csv; SYNTHETIC MODEL TEST ONLY");
    std::ifstream input(argv[1]);if(!input)throw std::runtime_error("cannot read model CSV");
    rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("nova5_synthetic_tf_check");
    auto pub=node->create_publisher<sensor_msgs::msg::JointState>("/validation/synthetic_joint_states",10);
    tf2_ros::Buffer buffer(node->get_clock());tf2_ros::TransformListener listener(buffer);
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if(pub->get_subscription_count()==0)throw std::runtime_error("robot_state_publisher not subscribed");
    std::cout<<"synthetic_sample,position_error_m,orientation_error_rad,tf_age_s\n"<<std::setprecision(17);
    std::string line;std::getline(input,line);int count=0;double maxp=0,maxr=0;
    while(std::getline(input,line)){
      std::stringstream ss(line);std::string tag,token;std::getline(ss,tag,',');std::vector<double> v;
      while(std::getline(ss,token,','))v.push_back(std::stod(token));
      if(v.size()!=17)throw std::runtime_error("unexpected model row");
      sensor_msgs::msg::JointState msg;for(int i=0;i<6;i++){msg.name.push_back("joint"+std::to_string(i+1));msg.position.push_back(v[i]*std::acos(-1)/180.);}
      const auto first_stamp=node->now();
      for(int repeat=0;repeat<12;repeat++){msg.header.stamp=node->now();pub->publish(msg);rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(50));}
      auto tf=buffer.lookupTransform("base_link","Link6",tf2::TimePointZero);
      if(rclcpp::Time(tf.header.stamp)<first_stamp)throw std::runtime_error("TF not refreshed for this sample");
      Eigen::Vector3d p(tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z),expected(v[6],v[7],v[8]);
      Eigen::Quaterniond q(tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z),eq(v[9],v[10],v[11],v[12]);
      double ep=(p-expected).norm(),er=Eigen::AngleAxisd(q.normalized().conjugate()*eq.normalized()).angle(),age=(node->now()-rclcpp::Time(tf.header.stamp)).seconds();
      std::cout<<tag<<','<<ep<<','<<er<<','<<age<<'\n';maxp=std::max(ep,maxp);maxr=std::max(er,maxr);count++;
      if(ep>1e-8 || er>1e-8 || age>0.5)throw std::runtime_error("offline TF consistency/freshness threshold exceeded");
    }
    rclcpp::shutdown();if(count!=13)throw std::runtime_error("expected 13 offline poses");
    std::cerr<<"PASS synthetic model TF only: samples="<<count<<" max_position_m="<<maxp<<" max_rotation_rad="<<maxr<<"; no hardware feedback claim\n";return 0;
  }catch(const std::exception&e){if(rclcpp::ok())rclcpp::shutdown();std::cerr<<e.what()<<'\n';return 2;}
}
