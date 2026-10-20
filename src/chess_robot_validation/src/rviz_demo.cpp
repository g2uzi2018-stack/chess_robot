// Synthetic JointState and URDF path demonstration. No sockets, no controller API.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/SVD>
#include <array>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>

int main(int argc,char**argv){
  try{
    if(argc!=3)throw std::runtime_error("usage: urdf synthetic_csv_output; SIMULATION ONLY");
    urdf::Model model;if(!model.initFile(argv[1]))throw std::runtime_error("URDF parse failed");
    KDL::Tree tree;KDL::Chain chain;
    if(!kdl_parser::treeFromUrdfModel(model,tree) || !tree.getChain("base_link","Link6",chain))throw std::runtime_error("chain failed");
    KDL::ChainFkSolverPos_recursive fk(chain);KDL::ChainJntToJacSolver jac(chain);
    std::ofstream out(argv[2]);if(!out)throw std::runtime_error("cannot write CSV");
    out<<"synthetic_time_s,record_ros_time_ns";
    for(auto prefix:{"q_deg","qd_deg_s"})for(int j=1;j<=6;j++)out<<','<<prefix<<j;
    out<<",flange_x_m,flange_y_m,flange_z_m,qw,qx,qy,qz,jacobian_min_sv_scaled,jacobian_condition_scaled\n"<<std::setprecision(17);
    rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("nova5_rviz_SYNTHETIC_ONLY");
    auto pub=node->create_publisher<sensor_msgs::msg::JointState>("/validation/synthetic_joint_states",10);
    auto paths=node->create_publisher<nav_msgs::msg::Path>("/validation/synthetic_flange_path",rclcpp::QoS(1).transient_local().reliable());
    nav_msgs::msg::Path path;path.header.frame_id="base_link";
    std::array<double,6> base{15,30,60,-30,60,15}, amplitude{15,8,10,8,10,15};
    std::array<int,6> harmonic{1,1,2,1,2,1};
    const double pi=std::acos(-1.0), duration=20.0;double minsv=1e9,maxcond=0;size_t count=0;
    const auto started=std::chrono::steady_clock::now();rclcpp::WallRate rate(50);
    while(rclcpp::ok()){
      double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
      // 5 s initial hold + 20 s closed smooth curve + 5 s final hold.
      if(elapsed>30.0)break;
      double t=std::clamp(elapsed-5.0,0.0,duration);
      sensor_msgs::msg::JointState msg;msg.header.stamp=node->now();
      KDL::JntArray q(6);std::array<double,6> degrees{},speeds{};
      for(size_t j=0;j<6;j++){
        double omega=2*pi*harmonic[j]/duration;
        degrees[j]=base[j]+amplitude[j]*0.5*(1-std::cos(omega*t));
        speeds[j]=(elapsed>=5 && elapsed<=25)?amplitude[j]*0.5*omega*std::sin(omega*t):0.0;
        q(j)=degrees[j]*pi/180.0;auto joint=model.getJoint("joint"+std::to_string(j+1));
        if(!joint || !joint->limits || q(j)<joint->limits->lower || q(j)>joint->limits->upper)throw std::runtime_error("synthetic model position limit exceeded");
        msg.name.push_back(joint->name);msg.position.push_back(q(j));msg.velocity.push_back(speeds[j]*pi/180.0);
      }
      KDL::Frame pose;fk.JntToCart(q,pose);double x,y,z,w;pose.M.GetQuaternion(x,y,z,w);
      KDL::Jacobian J(6);jac.JntToJac(q,J);Eigen::Matrix<double,6,6> scaled=J.data;scaled.topRows<3>()/=0.85;
      Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svd(scaled);auto s=svd.singularValues();
      double smallest=s(5),condition=s(0)/smallest;minsv=std::min(minsv,smallest);maxcond=std::max(maxcond,condition);
      if(smallest<0.05 || condition>80)throw std::runtime_error("synthetic Jacobian screening threshold exceeded");
      geometry_msgs::msg::PoseStamped p;p.header=path.header;p.header.stamp=msg.header.stamp;
      p.pose.position.x=pose.p.x();p.pose.position.y=pose.p.y();p.pose.position.z=pose.p.z();
      p.pose.orientation.x=x;p.pose.orientation.y=y;p.pose.orientation.z=z;p.pose.orientation.w=w;
      if(elapsed>=5 && elapsed<=25){path.poses.push_back(p);path.header.stamp=msg.header.stamp;}
      pub->publish(msg);if(count%5==0 || elapsed>25)paths->publish(path);
      out<<t<<','<<node->now().nanoseconds();for(double value:degrees)out<<','<<value;for(double value:speeds)out<<','<<value;
      out<<','<<pose.p.x()<<','<<pose.p.y()<<','<<pose.p.z()<<','<<w<<','<<x<<','<<y<<','<<z<<','<<smallest<<','<<condition<<'\n';
      count++;rclcpp::spin_some(node);rate.sleep();
    }
    rclcpp::shutdown();std::cout<<"SYNTHETIC ONLY frames="<<count<<" min_scaled_sv="<<minsv<<" max_scaled_condition="<<maxcond<<"; no dynamics, motion controller, hardware limits or actual TCP tracking tested\n";return count?0:2;
  }catch(const std::exception&e){if(rclcpp::ok())rclcpp::shutdown();std::cerr<<e.what()<<'\n';return 2;}
}
