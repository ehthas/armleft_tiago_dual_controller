// ROS
#include <cmath>
#include <eigen3/Eigen/Dense>


#include <boost/type_traits/is_same.hpp>
#include <boost/numeric/odeint/stepper/stepper_categories.hpp>
#include <boost/numeric/odeint/integrate/null_observer.hpp>
#include <boost/numeric/odeint/integrate/detail/integrate_adaptive.hpp>

#include <boost/numeric/odeint.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/skewness.hpp>


#include <XmlRpc.h>
#include <armleft_tiago_dual_controller/armleft_tiago_dual_controller.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;
using namespace boost::accumulators;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;


typedef std::vector<double> state_type;
typedef runge_kutta4<state_type> rk4;


namespace armleft_tiago_dual_controller   
{
bool MyEnergyShapingPositionController::initRequest(hardware_interface::RobotHW* robot_hw,
                                                 ros::NodeHandle& root_nh,
                                                 ros::NodeHandle& controller_nh,
                                                 ClaimedResources& claimed_resources)
{
    std::cout << "Controller in Init Request Function";
    // Check if construction finished cleanly
    if (state_ != CONSTRUCTED)
    {
       ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
       return false;
    }

    // Get a pointer to the joint position and not effort control interface
    hardware_interface::EffortJointInterface* effort_iface =
       robot_hw->get<hardware_interface::EffortJointInterface>();

    if (!effort_iface)
    {
      ROS_ERROR("This controller requires a hardware interface of type EffortJointInterface."
                " Make sure this is registered in the hardware_interface::RobotHW class.");
      return false;
    }

            // Get a pointer to the joint position control interface
            //hardware_interface::JointStateInterface* joint_state_iface =
            //    robot_hw->get<hardware_interface::JointStateInterface>();
            //if (!joint_state_iface)
            //{
            //   ROS_ERROR("This controller requires a hardware interface of type JointStateInterface."
            //            " Make sure this is registered in the hardware_interface::RobotHW class.");
            //   return false;
            //}

            // Clear resources associated at both interfaces
   effort_iface->clearClaims();
            //joint_state_iface->clearClaims();


   if (!init(effort_iface, root_nh, controller_nh))  //  joint_state_iface, root_nh,
   {
      ROS_ERROR("Failed to initialize the controller");
      return false;
   }

   // Saves the resources claimed by this controller
   claimed_resources.push_back(hardware_interface::InterfaceResources(
                hardware_interface::internal::demangledTypeName<hardware_interface::EffortJointInterface>(),
                effort_iface->getClaims()));
            effort_iface->clearClaims();

    // Changes state to INITIALIZED
    state_ = INITIALIZED;
            
    std::cout << "Controller exiting Init Request Function";
    //ROS_INFO_STREAM("Controller exiting Init Request Function");
    return true;
}
   
bool MyEnergyShapingPositionController::init(hardware_interface::EffortJointInterface* effort_iface, ros::NodeHandle& /*root_nh,*/, ros::NodeHandle& control_nh)
                 /*hardware_interface::JointStateInterface* joint_state_iface,
                 ros::NodeHandle& root_nh,*/ 
{
   ROS_INFO_STREAM("Loading Tiago_Dual_Energy_Shaping_Controller"); 
   //std::vector<std::string> joint_names_;
   //std::vector<std::string> joints_;
            //typedef Eigen::Quaternion<double> Quaterniond;


   //joints_.resize(joint_names_.size());
   //joints_.setZero();


            //ROS_INFO_STREAM("Loading public Energy Shaping Controller");

            // Check int the param server if subchains specified
   std::vector<std::string> tip_links;
   control_nh.getParam("robot_model_chains", tip_links);

            //effortjoint_handle-> getHandle(joint_state_iface);

   std::vector<double> joint_position_min;
   std::vector<double> joint_position_max;
   std::vector<double> joint_vel_min;
   std::vector<double> joint_vel_max;
   std::vector<double> joint_damping;
   std::vector<double> joint_friction;
   std::vector<double> joint_max_effort;

            //double motor_torque_constant;   // 0.0 removed from all 5 parameters
            //double reduction_ratio;
            //double viscous_friction;
            //double velocity_tolerance;
            //double static_friction;

   if (tip_links.size() > 0)
   {
               // Parse the robot if subchains specified
     RigidBodyDynamics::Addons::URDFReadFromParamServer(
         &rbdl_model_, RigidBodyDynamics::FloatingBaseType::FixedBase, tip_links,
         joint_names_, joint_position_min, joint_position_max, joint_vel_min,
         joint_vel_max, joint_damping, joint_friction, joint_max_effort);
   }
   else
   {
               // Parse the full robot if there is no subchain specified
     RigidBodyDynamics::Addons::URDFReadFromParamServer(
         &rbdl_model_, RigidBodyDynamics::FloatingBaseType::FixedBase, joint_names_,
         joint_position_min, joint_position_max, joint_vel_min, joint_vel_max,
         joint_damping, joint_friction, joint_max_effort);
    }

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
      
      try
      {  
      // Try to get an effort interface handle to command the joint in effort
        hardware_interface::JointHandle joint_handle =
            effort_iface->getHandle(joint_names_[i]);
      }
      catch (...)
      {
        ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Effort interface");
        return false;
      }
  }

/*    for (size_t i = 0; i < joint_names_.size(); i++)
    {
               // Checks joint type from param server
               //std::string control_type;
      if (!control_nh.getParam("joints", joints_[i]))
      {
         ROS_ERROR_STREAM("Could not find joint " << joints_[i]);
         return false;
      }
               
//               joints_ = robot_hw->getHandle(joint_names_[i]);
//            } 
               // Read the actuator parameters from param server
               //ActuatorParameters actuator_parameters;
               //if (!control_nh.getParam("joints/" + joint_names_[i] + "/motor_torque_constant", motor_torque_constant))
               //{
               //   ROS_ERROR_STREAM("Could not find motor torque constant for joint " << joint_names_[i]);
               //return false;
               //}
               //if (!control_nh.getParam("joints/" + joint_names_[i] + "/reduction_ratio", reduction_ratio))
               //{
               //   ROS_ERROR_STREAM("Could not find reduction ratio for joint " << joint_names_[i]);
               //return false;
               //}

               // Reads the optional gravity compensation parameters
               //GravityCompensationParameters friction_parameters;
               //if (!control_nh.getParam("viscous_friction", viscous_friction))
               //{ 
               //   ROS_WARN_STREAM("No viscous friction defined for joint "
               //         << joint_names_[i] << ". Setting it to 0.0");
               //}
               //if (!control_nh.getParam("velocity_tolerance", velocity_tolerance))
               //{
               //ROS_WARN_STREAM("No velocity tolerance defined for joint "
               //         << joint_names_[i] << ". Setting it to 0.0");
               //}
               //if (!control_nh.getParam("static_friction", static_friction))
               //{
               //ROS_WARN_STREAM("No static friction defined for joint " << joint_names_[i]
               //                                                 << ". Setting it to 0.0");
               //}
        try
        {
                  // Try to get an effort interface handle to command the joint in effort
           hardware_interface::JointHandle joint_handle=
             position_iface->getHandle(joints_[i]);
                      //joint_handle = joint_handle;
                   
        }
        catch (...)
        {
           ROS_ERROR_STREAM("Could not find joint " << joints_[i] << " with Effort interface");
           return false;
        }
               //else 
               //{
               //try
               //{
                  // Try to get a joint state handle which only allows us to read the current states
                  // of the joint
               //   hardware_interface::JointStateHandle joint_state_handle =
               //       joint_state_iface->getHandle(joint_names_[i]);
                      //joint_state_handle = joint_state_handle;
                  // Insert this handle in the map of static joints
                  //static_joints_.insert(std::make_pair(joint_names_[i], joint_state_handle));
               //}
               //catch (...)
               //{
               //   ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Position interface");
               //   return false;
               //}
               ///}
    }  
*/
            //assert(joint_types_.size() == joint_names_.size());


            //std::string link_name = "arm_left_tool_link";
            //unsigned int tip_id = model_.rbdl_model_.GetBodyId(link_name);
            //Eigen::MatrixXd jacobian;      // jacobian initialization
            //jacobian.resize(6, model_.joint_names_.size());
            //jacobian.setZero();

            //Eigen::MatrixXd M;             // Mass Matrix initialization
            //M.resize(7, model_.joint_names_.size());
            //M.setZero();
            //Eigen::MatrixXd Mass;    //inertia matrix
            //Mass.resize(model_.joint_names_.size(), model_.joint_names_.size());
            //Mass.setZero();
            

            //state_type s0(7);  // Initial condition vector of 7 elements 


            //int tt = 15;                                    // apply force(F) of -10 N at this time for one second 
            //int T = 4;

            
            //std::vector<std::string> joints;
            //if (!n.getParam("joints", joints)) 
            //{
            //   ROS_ERROR("Could not read joint names from param server");
            //   return false;
            //}

            //if (joints.size() != 7)
            //{
            //   ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got " << joints.size() << " instead of 7 names!");
            //   return false;
            //}
/*            for (int i = 0; i < joint_names_.size(); i++)
            {

                joint_handle_.resize(joint_names_.size());
            //for (size_t i=0; i<7; ++i)
            //{
                joint_handle_[i] = hw->getHandle(joints[i]); 
            //    command_[i] = joint_handles_[i].getPosition(); 
            
            //}
            }
*/
            // retrieve gains
            
            //if (!n.getParam("gains", gains_)) 
            //{
            //   ROS_ERROR("Could not read joint gains from param server");
            //   return false;
            //}    

            //std::array<double, 7> q_start{{0, 0, 0, 0, 0, 0, 0}};
            //for (size_t i=0 i < q_start.size() i++)
            //{
            //    if (std::abs(joint_handles_[i].getPosition() - q_start[i]) > 0.1)
            //       {
            //        ROS_ERROR_STREAM( "Robot is not in expected start state" )
            //       return false;
            //       }  

            //}


            //for (&joint : joints) 
            //{
            //   joint_handles_.push_back(hw->getHandle(joint));
            //}

            //for (&joint_handle : joint_handles_) 
            //{
            //   command_.push_back(joint_handle.getPosition());
            //}

            

            //sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &MyEnergyShapingPositionController::setCommandCallback, this);
   q_act_.resize(joint_names_.size());
   qdot_.resize(joint_names_.size());
   q_zero_.resize(joint_names_.size());
   tau_cmd_.resize(joint_names_.size());            

   q_act_.setZero();
   qdot_.setZero();
   q_zero_.setZero();
   tau_cmd_.setZero();

   H.resize(joint_names_.size());
   pc.resize(joint_names_.size());
   tauc.resize(joint_names_.size());
   u_vec.resize(joint_names_.size());

   H.setZero();
   pc.setZero();
   tauc.setZero();
   u_vec.setZero();
   
   tau.resize(joint_names_.size());
   tau.setZero();
   return true;
}

void MyEnergyShapingPositionController::update(const ros::Time& time, const ros::Duration& period) 
{
  typedef Eigen::Quaternion<double> Quaterniond;
  typedef std::vector<double> state_type;
  typedef runge_kutta4<state_type> rk4;

  // Integration parameters
  double t0 = 0.0;
  double t1 = 10.0;
  double d_t = 1.0;
            
   // initial values
   int ko = 100;             //rotational stiffness constant
   int kt = 1000;            //translational stiffness constant
   int b = 50;               //Damping coefficient
   int epsilon = 0.001;      //Minimum energy in tank
   int Emax = 1;             //Maximum allowed energy
   int Pmax = 2;             //Maximum allowed power
   
   MatrixXd I3(3,3);
   I3 << 1,0,0,0,1,0,0,0,1;
   MatrixXd I7(7,7);
   I7 << 1,0,0,0,0,0,0,
         0,1,0,0,0,0,0,
         0,0,1,0,0,0,0,
         0,0,0,1,0,0,0,
         0,0,0,0,1,0,0,
         0,0,0,0,0,1,0,
         0,0,0,0,0,0,1;
            
   // initial equations
   MatrixXd Bi = b * I7;               // I7 is equivalent to eye(7) where eye refers to identity matrix and 6 refers to size of matrix
   MatrixXd Ko = ko * I3;
   MatrixXd Kt = kt * I3; 
   MatrixXd Kc(3,3);
   Kc << 0,0,0,0,0,0,0,0,0;

   MatrixXd Goi = 0.5*Ko.trace()*I3 - Ko;
   MatrixXd Gti = 0.5*Kt.trace()*I3 - Kt;          // trace refers to tensor space operator
   MatrixXd Gci = 0.5*Kc.trace()*I3 - Kc;

   double gamma = sqrt(2*epsilon);                     // square root

   for (int i = 0; i < joint_names_.size(); ++i)
   {
     q_act_[i] = joint_handle.getPosition();
     //double actual_velocity = actuated_joint.joint_handle.getVelocity();
//     qdot_[i] = joint_handle.getVelocity();
   } 

     std::string link_name = "arm_left_tool_link";  //arm_left_tool_link
     unsigned int tip_id = rbdl_model_.GetBodyId(link_name.c_str());
     

    Eigen::MatrixXd R_t0(3,3);      // current config rotation matrix initialization
     //R_t0.resize(3, joint_names_.size());
    R_t0.setZero();
    RigidBodyDynamics::CalcBodyWorldOrientation(rbdl_model_, q_act_, tip_id, R_t0, false);

    MatrixXd R_0t = R_t0.transpose();

    Eigen::VectorXd p_0t(3,1);      // current config position vector initialization
    p_0t.setZero();
    RigidBodyDynamics::CalcBodyToBaseCoordinates(rbdl_model_, q_act_, tip_id, p_0t, false);

    Matrix4d H_0t = Matrix4d::Identity();
    
    H_0t.block(0,0,3,3) = R_0t;
    H_0t.block(0,3,3,1) = p_0t;


            //  later on the following section needs to take in pose and should be given to the following code for calculating homegenous matrix

            //Hot = [0 0 -1 -0.6; 0 1 0 0; 1 0 0 0.6; 0 0 0 1];   // current end effector config
            //MatrixXd Hot(4,4);
            //Hot << 0,0,-1,-0.6,0,1,0,0,1,0,0,0.6,0,0,0,1;

            //geometry_msgs::Pose current_pose = arm_right.getCurrentPose(arm_left_tool_link).pose;        // end_effector_link .pose
            //std::cout<<"now Robot position: [x,y,z]: ["<<current_pose.position.x<<","<<current_pose.position.y<<","<<current_pose.position.z<<"]"<<std::endl;
            //std::cout<<"now Robot orientation: [x,y,z,w]: ["<<current_pose.orientation.x<<","<<current_pose.orientation.y<<","<<current_pose.orientation.z
            //         <<","<<current_pose.orientation.w<<"]"<<std::endl;
            
            //MatrixXd q2r_c = Quaterniond(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z).toRotationMatrix();     // q2r_c refers to quatrernion to rotation for current end effector pose
            // 0.048,0.998,-0.024,0.015
            //VectorXd ee_current_trans_vec(3,1);
            //ee_current_trans_vec << current_pose.position.x,current_pose.position.y,current_pose.position.z;        //0.192, 0.229, 0.496;  
            //Matrix4d Hot = Matrix4d::Identity();
            //Hot.block(0,0,3,3) = q2r_c;
            //Hot.block(0,3,3,1) = ee_current_trans_vec;
             
    //MatrixXd q2r_c = Quaterniond(0.513,-0.509,-0.508,0.468).toRotationMatrix();    //0.513,-0.509,-0.508,0.468
    //VectorXd ee_current_trans_vec(3,1);
    //ee_current_trans_vec << 0.223, 0.227, 0.500; // 0.223, 0.227, 0.500 current_pose.position.x,current_pose.position.y,current_pose.position.z;        //0.192, 0.229, 0.496;  
    //Matrix4d Hot = Matrix4d::Identity();
    //Hot.block(0,0,3,3) = q2r_c;
    //Hot.block(0,3,3,1) = ee_current_trans_vec;


            //Hov = [0 0 -1 -0.6; 0 1 0 (0.3*sin(2*pi/T)*tt); 1 0 0 0.6; 0 0 0 1];     // desired end effector config
            //MatrixXd Hov(4,4);
            //Hov << 0,0,-1,-0.6,0,1,0,(0.3*sin(2*M_PI/T)*tt),1,0,0,0.6,0,0,0,1;

    MatrixXd R_0v = Quaterniond(0.525,-0.526,-0.529,0.410).toRotationMatrix();     // R_0v refers to quatrernion to rotation for desired end effector pose
    VectorXd p_0v(3,1);
    p_0v << 0.229, 0.372, 0.499;   // 0.229, 0.372, 0.499  
    
    Matrix4d H_v0 = Matrix4d::Identity();
    
    H_v0.block(0,0,3,3) = R_0v.transpose();
    H_v0.block(0,3,3,1) = -(R_0v.transpose()*p_0v);
    
    //MatrixXd Hvo=[transpose(q2r_d) rp;0 0 0 1];

    // Pre step to compute rotation and position components using relative configurations of ee to compute wrench 
   
    MatrixXd H_vt = H_v0*H_0t;             
    

    // extracting rotaional Rvt and translational pvt part from Hvt for further calculating wrench 
    MatrixXd R_vt = H_vt.block(0,0,3,3);

    MatrixXd p_vt = H_vt.block(0,3,3,1);

    MatrixXd R_tv = R_vt.transpose(); 

    // converting position vector to skew-symmetric matrix
    //MatrixXd tilde_p_vt = [0 -p_vt(3,1) p_vt(2,1);p_vt(3,1) 0 -p_vt(1,1);-p_vt(2,1) p_vt(1,1) 0];
    MatrixXd tilde_p_vt(3,3); 
    tilde_p_vt << 0,-p_vt(2,0),p_vt(1,0),p_vt(2,0),0,-p_vt(0,0),-p_vt(1,0),p_vt(0,0),0;

            // mass matrix tiago arm
            /* M =    */           // in matlab used m = 1 and then M = m * eye(6);
            
            //RigidBodyDynamics::getMassMatrix(model_.rbdl_model_, q_act_, M );  

            //Eigen::MatrixXd M;    //inertia matrix
            //inertia_mat.resize(model_.joint_names_.size(), model_.joint_names_.size());
            //inertia_mat.setZero();
            
            
     Eigen::MatrixXd Mass;    //inertia matrix
     Mass.resize(joint_names_.size(), joint_names_.size());
     Mass.setZero();
     RigidBodyDynamics::CompositeRigidBodyAlgorithm(rbdl_model_, q_act_, Mass, false);
            
            
            // Jacobian 
            /* Jacobian =    */           // in matlab used Jacobian = eye(6) here used the current ee position to calculate jacobian
     //std::string link_name = "arm_left_tool_link";  //arm_left_tool_link
     //unsigned int tip_id = rbdl_model_.GetBodyId(link_name.c_str());
     Eigen::MatrixXd jacobian;      // jacobian initialization
     jacobian.resize(6, joint_names_.size());
     jacobian.setZero();
     RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q_act_, tip_id,
                                             Eigen::Vector3d(0, 0, 0), jacobian, false);  

            
            // Safety layer (PE, KE and total Energy of the system)
            //Vti = -0.25*trace(skewness(pvt)*Gti*skewness(pvt) -0.25*trace(skewness(pvt)*Rvt*Gti*Rtv*skewness(pvt);
            //Voi = -(trace(Goi*Rvt));
            //Vci = trace(Gci*Rtv*skewness(pvt);

     int Vti = (-1/4*(tilde_p_vt*Gti*tilde_p_vt).trace())-(1/4*(tilde_p_vt*R_vt*Gti*R_tv*tilde_p_vt).trace());
     int Voi = -((Goi*R_vt).trace());
     int Vci = ((Gci*R_tv*tilde_p_vt).trace());


     int V_pi = Vti + Voi + Vci;         // initial potential energy

     int T_k = 1/2*qdot_.transpose() * Mass * qdot_;        // transpose of qdot x M x qdot

     int E_tot = T_k + V_pi;               // initial energy of the system
     
     int lamba_;

     if (E_tot > Emax)  
        lamba_ = (Emax - T_k)/ V_pi;
     else
        lamba_ = 1;
     return;
            // calculation of new co-stiffness matrices and corresponding potential energy

     MatrixXd Go = lamba_ * Goi;           // new co-stiffness matrices
     MatrixXd Gt = lamba_ * Gti;
     MatrixXd Gc = lamba_ * Gci;

            //Vt = -0.25*trace(skewness(pvt)*Gt*skewness(pvt) -0.25*trace(skewness(pvt)*Rvt*Gt*Rtv*skewness(pvt);
            //Vo = -(trace(Go*Rvt));
            //Vc = trace(Gc*Rtv*skewness(pvt);

     //int Vt = ((-0.25*pvt_s*Gt*pvt_s).trace())-((0.25*pvt_s*Rvt*Gt*Rtv*pvt_s).trace());
     //int Vo = -((Go*Rvt).trace());
     //int Vc = ((Gc*Rtv*pvt_s).trace());
     
     int Vt = (-1/4*(tilde_p_vt*Gt*tilde_p_vt).trace())-(1/4*(tilde_p_vt*R_vt*Gt*R_tv*tilde_p_vt).trace());
     int Vo = -((Go*R_vt).trace());
     int Vc = ((Gc*R_tv*tilde_p_vt).trace());


     int V_p = Vt + Vo + Vc;         // potential energy
     E_tot = T_k + V_p;            // total energy of the system
                
            // Wrench applied on manipulator end effector due to spring can now be calculated as follows
            // Wt = [mt ft]T

            // Wrench Acting on End-Effector

            // Rotational part of the wrench
            //tGo = Go * Rtd;
            //astGo = (tGo - transpose(tGo))./2;
            //tGt = Gt * Rdt * skewness(ptd) * skewness(ptd) * Rtd;
            //astGt = (tGt - transpose(tGt))./2;
            //tGc = Gc * skewness(ptd) * Rtd;
            //astGc = (tGc - transpose(tGc))./2;
            //tG = (-2*astGo) - (astGt) - (2*astGc);
            //t = [tG(3,2); tG(1,3); tG(2,1)];

     // Rotational part of wrench
     MatrixXd tilde_m_t = - 2* 1/2*(Go*R_vt-(Go*R_vt).transpose())-1/2*(Gt*R_tv*tilde_p_vt*tilde_p_vt*R_vt-    (Gt*R_tv*tilde_p_vt*tilde_p_vt*R_vt).transpose())-2*1/2*(Gc*tilde_p_vt*R_vt-(Gc*tilde_p_vt*R_vt).transpose());
     VectorXd m_t(3,1); 
     //m_t = [tilde_m_t(3,2); tilde_m_t(1,3); tilde_m_t(2,1)];   in matlab
     m_t << tilde_m_t(2,1),tilde_m_t(0,2),tilde_m_t(1,0);

     // Translational part of wrench
     MatrixXd tilde_f_t = -R_tv * 1/2*(Gt*tilde_p_vt- (Gt*tilde_p_vt).transpose())*R_vt-1/2*(Gt*R_tv*tilde_p_vt*R_vt-(Gt*R_tv*tilde_p_vt*R_vt).transpose())-2*1/2*(Gc*R_vt-(Gc*R_vt).transpose());
     VectorXd f_t(3,1);
     //f_t = [tilde_f_t(3,2); tilde_f_t(1,3); tilde_f_t(2,1)];   in matlab
     f_t << tilde_f_t(2,1),tilde_f_t(0,2),tilde_f_t(1,0);
            
     VectorXd Wt(6,1);           // wrench vector initialization
     Wt << 0,0,0,0,0,0; 
    
     Wt.block(0,0,3,1) = m_t;          // Wsn0(1:3,1) = t ... t represented with small m here;
     Wt.block(3,0,3,1) = f_t;          // Wsn0(4:6,1) = f; 

     MatrixXd tilde_p_0t(3,3);
     tilde_p_0t << 0,-p_0t(2,0),p_0t(1,0),p_0t(2,0),0,-p_0t(0,0),-p_0t(1,0),p_0t(0,0),0;

     MatrixXd AdjT_H_t0(6,6);
     AdjT_H_t0.fill(0);

     AdjT_H_t0.block(0,0,3,3) = R_0t;
     AdjT_H_t0.block(0,3,3,3) = tilde_p_0t*R_0t;
     AdjT_H_t0.block(3,3,3,3) = R_0t;

     VectorXd W0(6,1);           // wrench vector transformation of 
     W0 = AdjT_H_t0* Wt;     


     //Matrix6d AdjT_H_t0 = Matrix6d::Identity();
     //AdjT_H_t0 = [R_0t tilde_p_0t*R_0t; zeros(3) R_0t]; 
//R_0t =     
 

     //MatrixXd Htipbase(6,6);
     //Htipbase.fill(0);
     //Htipbase.block(0,0,3,3) = q2r;
     //Htipbase.block(3,3,3,3) = q2r;
     //VectorXd j(3,1);
     //j = ee_current_trans_vec;
     //MatrixXd sk_mat(3,3); 
     //sk_mat << 0,-(j(2)),j(1),j(2),0.0,-j(0),-j(1),j(0),0.0;
     //Htipbase.block(3,0,3,3) = sk_mat*(q2r); 
           
     //VectorXd W0 = (((Htipbase.inverse()).adjoint()).transpose()) * (Wsn0);      // wrench acting on end effector expressed in inertial frame

           
     // Power of the System 
     
     int pc = ((jacobian.transpose() * W0 - Bi * qdot_).transpose()) * qdot_ ;  // Initial power of the controller
     
     int beta;
     if (pc > Pmax) 
          int beta = (((((jacobian.transpose()) * W0).transpose())*qdot_) - Pmax)/ ((qdot_.transpose())*Bi*qdot_);
     else
          int beta = 1; 
     return;

     // New joint damping matrix using scaling parameter beta
     MatrixXd B = beta * Bi;

     // New power of the controller using new joint damping matrix
     tau_cmd_ = ((jacobian.transpose()) * W0) - B * qdot_;                    // Controller force

     pc = (tau_cmd_.transpose()) * qdot_ ;      // Power of the controller


     std::vector<double> s0;
     //std::vector<double> s_i;
     //s_i.resize(1, joint_names_.size());
     //s_i << 12,12,12,0,0,0,0;  
     //s0.resize(1, joint_names_.size());
     s0[0] = sqrt(2*12);
     s0[1] = sqrt(2*12);
     s0[2] = sqrt(2*12);
     s0[3] = sqrt(2*0);
     s0[4] = sqrt(2*0);
     s0[5] = sqrt(2*0);
     s0[6] = sqrt(2*0);

           // Run integrator with rk4 stepper
           //std::cout << "==========  rk4 - basic stepper  ====================" << std::endl;
           //auto r = make_pair( a.begin() , a.begin() + 3 );
           //const state_type x { 0.0, 0.0 };
           //const state_type cx { 0, 0 };
           //integrator_adaptive <rk4(), my_system, &s, t0, t1, dt> integrator;
           //template< class Stepper , class System , class State , class Time >
           
           //integrate_adaptive<rk4(), my_system, &s, t0, t1, dt, null_observer()>; // my_observer);     // can add another argument after dt preferable a function for observing the output
           //state_type integrate_adaptive< Stepper::stepper, System system, State &start_state, Time start_time , Time end_time , Time dt >
           //{
           //return 
     state_type integrate_adaptive();
           // stepper_type() , my_system , s , t0 , t1 , dt , null_observer() );
           //}

     int s1 = s0[0];
     int s2 = s0[1];     
     int s3 = s0[2];
     int s4 = s0[3];
     int s5 = s0[4];
     int s6 = s0[5];
     int s7 = s0[6];

     // Controller torques for each joint
     tauc[0] = tau_cmd_[0];
     tauc[1] = tau_cmd_[1];
     tauc[2] = tau_cmd_[2];
     tauc[3] = tau_cmd_[3];
     tauc[4] = tau_cmd_[4];
     tauc[5] = tau_cmd_[5];
     tauc[6] = tau_cmd_[6];
          

           //return detail::integrate_adaptive(stepper, system, start_state, start_time, end_time, dt)
           // Potential energy in each tank, an energy tank is modeled as a spring with const stiffness k = 1
           // connected to robot through a transmission unit 
           // so H = 0.5*k*s^2 ==> H = 0.5*s^2 
     H[0] = 0.5*s1*s1;
     H[1] = 0.5*s2*s2;
     H[2] = 0.5*s3*s3;
     H[3] = 0.5*s4*s4;
     H[4] = 0.5*s5*s5;
     H[5] = 0.5*s6*s6;
     H[6] = 0.5*s7*s7;           

     int Htot = H[0] + H[1] + H[2] + H[3] + H[4] + H[5] + H[6];     // Total energy in tanks

     // Power of the controller on each joint
     //pc[0] = tauc[0] * qdot_[0];
     //pc[1] = tauc[1] * qdot_[1];
     //pc[2] = tauc[2] * qdot_[2];
     //pc[3] = tauc[3] * qdot_[3];
     //pc[4] = tauc[4] * qdot_[4];
     //pc[5] = tauc[5] * qdot_[5];
     //pc[6] = tauc[6] * qdot_[6];
          
           
           // transmission unit allows power flow from controller to robot and it is regulated by ratio u
           // transmission variable
     if ((H[0] > epsilon))     // || (pc[0] < 0)

          u_vec[0]=-tauc[0]/s0[0];
     else

          u_vec[0]=(-tauc[0]/gamma*gamma)*s0[0];
     return;

     if ((H[1] > epsilon))    // || (pc[1] < 0)

          u_vec[1]=-tauc[1]/s0[1];
     else

          u_vec[1]=((-tauc[1])/gamma*gamma)*s0[1];
     return;

     if ((H[2] > epsilon))   // || (pc[2] < 0)

          u_vec[2]=-tauc[2]/s0[2];
     else

          u_vec[2]=(-tauc[2]/gamma*gamma)*s0[2];
     return;

     if ((H[3] > epsilon))   // || (pc[3] < 0)

          u_vec[3]=-tauc[3]/s0[3];
     else

          u_vec[3]=(-tauc[3]/gamma*gamma)*s0[3];
     return;

     if ((H[4] > epsilon))   // || (pc[4] < 0)

          u_vec[4]=-tauc[4]/s0[4];
     else

          u_vec[4]=(-tauc[4]/gamma*gamma)*s0[4]; 
     return;

     if ((H[5] > epsilon))  // || (pc[5] < 0)

          u_vec[5]=-tauc[5]/s0[5];
     else

          u_vec[5]=(-tauc[5]/gamma*gamma)*s0[5];
     return;

     if ((H[6] > epsilon))  // || (pc[6] < 0)

          u_vec[6]=-tauc[6]/s0[6];
     else

          u_vec[6]=(-tauc[6]/gamma*gamma)*s0[6];
     return;

     //VectorXd W0(6,1);
     u_vec << u_vec[0],u_vec[1],u_vec[2],u_vec[3],u_vec[4],u_vec[5],u_vec[6];
     H << H[0],H[1],u_vec[2],u_vec[3],u_vec[4],u_vec[5],u_vec[6]; 

           //u=[u1;u2;u3;u4;u5;u6];


     // For all the joints...
     for (size_t i = 0; i < joint_names_.size(); ++i)
     {
         tau[i] = - u_vec[i] * s0[i];
         double output_torque = tau[i];

               //output_torque += joint_names_[i].viscous_friction * qdot;
               //if (qdot > joints.friction_parameters.velocity_tolerance)
               //   output_torque += joint.friction_parameters.static_friction;
               //else
               //   output_torque -= joint.friction_parameters.static_friction;

               //double output_effort =
               //output_torque / (joint_names_[i].motor_torque_constant *
               //                 joint_names_[i].reduction_ratio);

         if (std::isnan(output_torque))  // If desired effort is not valid
         {
             ROS_ERROR_STREAM("Output torque is not valid for joint "
                                    << joint_names_[i] << " = " << output_torque);
         return;
         }
         else
         {
                   // Command an effort to the joint via ros_cotrol interface
             joint_handle.setCommand(output_torque);
         }
      }

           //return;   
            //for (size_t i = 0; i < joint_handles_.size(); i++) 
            //{
                //double error = command_.at(i) - joint_handles_.at(i).getPosition();
                //double commanded_effort = error * gains_.at(i);
                //std::cout << "Error\n";
                //ROS_ERROR_STREAM("Error b/w desired and actual joint positions " << commanded_effort );
                //joint_handles_.at(i).setCommand(commanded_effort);
            //}

}

       //void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) 
       //{ 
       //     command_ = msg->data; 
       //}

template< class Stepper , class System , class State , class Time > 
       //state_type integrate_adaptive(
       // Stepper rk4() , System my_system , State &s,
       // Time t0 , Time t1 , Time dt )
       //{
       //   return integrate_adaptive( rk4() , my_system , s , t0 , t1 , dt , null_observer() );
       //}
       
state_type integrate_adaptive( Stepper stepper_type , System my_system , State s0 , Time t0 , Time t1 , Time d_t , null_observer() )
{
  return integrate_adaptive( rk4() , my_system , s0 , t0 , t1 , d_t , null_observer() );
}

void my_system ( const state_type &s0 , state_type &dsdt , const double t) 
{
         //template< class Stepper , class System , class State , class Time >
         //integrate_adaptive(
         //Stepper stepper , System system , State &start_state ,
         //Time start_time , Time end_time , Time dt )
         //{
         //  return integrate_adaptive( stepper , system , start_state , start_time , end_time , dt , null_observer() );
         //}
            //state_type u_vec(6);
  //for (size_t i=0; i<6; ++i)
  //{
  //   u_vec[i]=0.0;
  //}
  //qdot_.resize(joint_names_.size());
  //u_vec.resize(joint_names_.size());  
  
  Eigen::VectorXd qdot_;
  Eigen::VectorXd u_vec; 
  std::vector<std::string> joint_names_; 
 
  qdot_.resize(joint_names_.size());
  u_vec.resize(joint_names_.size());

  for (size_t i=0; i<6; ++i)
  {
     dsdt[i] = qdot_[i]*u_vec[i];
  }  
            
        
}

       //void my_observer( const state_type &s, const double t )
       //{
       //  for (int i = 0; i < 6; i++)
	    //  {
		 //     //printf("\ns0\n", s0[i]); // use a for-loop to output the row
       //     std::cout << "\ns0 = \n" << s[i] << std::endl;
	    //  }
            //std::cout  << t << "   " << s[0] << "   " << s[1] << std::endl;
       //}

       //struct GravityCompensationParameters
       //{
       //  double static_friction = 0.0;
       //  double viscous_friction = 0.0;
       //  double velocity_tolerance = 0.0;
       //};

      // struct ActuatorParameters
      // {
      //  double motor_torque_constant = 0.0;
      //   double reduction_ratio = 0.0;
      // };

       //GravityCompensationParameters friction_parameters;
       


void MyEnergyShapingPositionController::starting(const ros::Time& time)  
{
}

void MyEnergyShapingPositionController::stopping(const ros::Time& time)  
{
}

}
//      private:

            //bool init(hardware_interface::EffortJointInterface* effort_iface,
            //          //hardware_interface::JointStateInterface* joint_state_iface,
            //          /*ros::NodeHandle& root_nh,*/ ros::NodeHandle& control_nh);

//            RigidBodyDynamics::Model rbdl_model_;  /*!< Robot model from RBDL */

            //std::vector<double> gains_;
            //std::vector<double> command_;
            //ros::Subscriber sub_coRigidBodyDynamics::Model rbdl_model_;  /*!< Robot model from RBDL */mmand_;
            

//            Eigen::VectorXd q_zero_;    /*!< Zero vector with joint_names size */
//            Eigen::VectorXd tau_cmd_;   /*!< Vector with the necessary torque to maintain gravity */
//            Eigen::VectorXd q_act_;     /*!< Vector with the current position of the joint states */
//            Eigen::VectorXd qdot_;
//            Eigen::VectorXd H;
//            Eigen::VectorXd pc;
//            Eigen::VectorXd tau;



            //std::vector<std::string> rbdl_model_;                  /*!< Vector with the joint names of all the joints, including the actuated and the static ones */

//            std::vector<std::string> joint_names_;
//            std::vector<std::string> joints_;

//            std::vector<double> u_vec;
//            std::vector<double> s0;
            //std::vector<double> s0;

            //double t0;
            //double t1;
            //double dt;
            //VectorXd q_zero_;
            //VectorXd q_act_;
            //VectorXd qdot_;
            //VectorXd tau_cmd_;
            
//            typedef runge_kutta4<state_type> rk4;
            //typedef runge_kutta_dopri5< double > stepper_type;


            //std::map<std::string, JointType> joint_types_;          /*!< Map to define which joint are actuated and which one are static */
            //std::map<std::string, ActuatedJoint> actuated_joints_;  /*!< Map with the actuated joints and his parameters + hardware interface to read and write commands*/
            //std::map<std::string, hardware_interface::JointStateHandle> static_joints_;   /*!< Map with the static joints and his hardware interface to read the current position */
//            hardware_interface::JointHandle joint_handle;
            //hardware_interface::JointStateHandle joint_state_handle_; 
             
//            ddynamic_reconfigure::DDynamicReconfigurePtr ddr_;     /*!< Dyanic reconfigure */



//       };
//   PLUGINLIB_EXPORT_CLASS(energy_controller_ns::MyEnergyShapingPositionController, controller_interface::ControllerBase); 
// }
