#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

Eigen::Isometry3d origin(const urdf::Pose& p){
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  t.linear()=Eigen::Quaterniond(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z).normalized().toRotationMatrix();return t;
}
int main(int argc,char** argv){
  try{
    if(argc!=3) throw std::runtime_error("usage: nova5_model_check urdf output_prefix");
    urdf::Model model;if(!model.initFile(argv[1])) throw std::runtime_error("URDF parse failed");
    KDL::Tree tree;KDL::Chain chain;if(!kdl_parser::treeFromUrdfModel(model,tree) || !tree.getChain("base_link","Link6",chain) || chain.getNrOfJoints()!=6)throw std::runtime_error("chain invalid");
    KDL::ChainFkSolverPos_recursive fk(chain);KDL::ChainJntToJacSolver jac(chain);
    std::ofstream meta(std::string(argv[2])+"_joints.csv"),out(std::string(argv[2])+"_fk.csv");
    meta<<"joint,parent,child,axis_x,axis_y,axis_z,lower_rad,upper_rad,velocity_rad_s,effort_Nm\n"<<std::setprecision(17);
    for(int i=1;i<=6;i++){
      auto j=model.getJoint("joint"+std::to_string(i));if(!j || !j->limits || j->type!=urdf::Joint::REVOLUTE)throw std::runtime_error("joint naming/type mismatch");
      meta<<j->name<<','<<j->parent_link_name<<','<<j->child_link_name<<','<<j->axis.x<<','<<j->axis.y<<','<<j->axis.z<<','<<j->limits->lower<<','<<j->limits->upper<<','<<j->limits->velocity<<','<<j->limits->effort<<'\n';
    }
    out<<"sample,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,x_m,y_m,z_m,qw,qx,qy,qz,kdl_eigen_position_error_m,kdl_eigen_rotation_matrix_error,jacobian_min_sv_scaled,jacobian_condition_scaled\n"<<std::setprecision(17);
    std::mt19937 rng(6021);std::uniform_real_distribution<double> dist(-1,1);double maxp=0,maxr=0;int saved=0;
    for(int candidate=0;candidate<2000 && saved<13;candidate++){
      KDL::JntArray q(6);for(int i=0;i<6;i++)q(i)=candidate==0?0:dist(rng)*(i==1 || i==2?1.5:2.0);
      KDL::Jacobian J(6);jac.JntToJac(q,J);Eigen::Matrix<double,6,6> scaled=J.data;scaled.topRows<3>()/=0.85;
      Eigen::JacobiSVD<Eigen::Matrix<double,6,6>> svd(scaled);auto sv=svd.singularValues();
      if(candidate && (sv(5)<0.05 || sv(0)/sv(5)>80))continue;
      Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
      for(int i=1;i<=6;i++){auto j=model.getJoint("joint"+std::to_string(i));T=T*origin(j->parent_to_joint_origin_transform);Eigen::Vector3d a(j->axis.x,j->axis.y,j->axis.z);T.rotate(Eigen::AngleAxisd(q(i-1),a.normalized()));}
      KDL::Frame k;fk.JntToCart(q,k);Eigen::Vector3d kp(k.p.x(),k.p.y(),k.p.z());Eigen::Matrix3d kr;for(int r=0;r<3;r++)for(int c=0;c<3;c++)kr(r,c)=k.M(r,c);
      double ep=(kp-T.translation()).norm(),er=(kr-T.linear()).norm();maxp=std::max(maxp,ep);maxr=std::max(maxr,er);
      out<<(saved==0?"zero_diagnostic":"candidate_"+std::to_string(saved));for(int i=0;i<6;i++)out<<','<<q(i)*180/std::acos(-1);
      Eigen::Quaterniond quat(T.linear());out<<','<<T.translation().x()<<','<<T.translation().y()<<','<<T.translation().z()<<','<<quat.w()<<','<<quat.x()<<','<<quat.y()<<','<<quat.z()<<','<<ep<<','<<er<<','<<sv(5)<<','<<sv(0)/sv(5)<<'\n';saved++;
    }
    std::cout<<std::setprecision(17)<<"links="<<model.links_.size()<<" joints="<<model.joints_.size()<<" chain_dof="<<chain.getNrOfJoints()<<" samples="<<saved<<" max_position_error_m="<<maxp<<" max_rotation_matrix_error="<<maxr<<'\n';
    if(saved!=13 || maxp>1e-10 || maxr>1e-10)throw std::runtime_error("offline numeric consistency failed");
    std::cout<<"PASS offline KDL/Eigen consistency only. Zero pose may be singular. Candidate poses are NOT collision-cleared or approved for hardware. No controller FK/IK or absolute accuracy claim.\n";return 0;
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}
}
