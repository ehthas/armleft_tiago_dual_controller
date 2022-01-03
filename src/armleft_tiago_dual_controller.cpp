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
   tau.resize(joint_names_.size());
   u_vec.resize(joint_names_.size());
   

   H.setZero();
   pc.setZero();
   tau.setZero();
   u_vec.setZero();
   
   return true;
}

void MyEnergyShapingPositionController::update(const ros::Time& time, const ros::Duration& period) 
{
  typedef Eigen::Quaternion<double> Quaterniond;
  typedef std::vector<double> state_type;
  typedef runge_kutta4<state_type> rk4;
            //typedef runge_kutta4<state_type> rk4;
            //VectorXd s0(6);
            //state_type s0(6);
            //state_type s(6);
            //VectorXd H(6);
            //VectorXd pc(6);
            //VectorXd tau(6);

            //H.resize(joint_names_.size());
            //pc.resize(joint_names_.size());
            //tau.resize(joint_names_.size());

            //VectorXd q_zero;
            //VectorXd q_act;
            //VectorXd qdot;
            //VectorXd tau_cmd;
            //VectorXd u_vec;
            //state_type u_vec;

            // Iinitializa q_act_, q_zero_, tau_cmd_
/*            q_act_.resize(joint_names_.size());
            qdot_.resize(joint_names_.size());
            q_zero_.resize(joint_names_.size());
            tau_cmd_.resize(joint_names_.size());

            q_act_.setZero();
            qdot_.setZero();
            q_zero_.setZero();
            tau_cmd_.setZero();
*/
            //VectorXd H(7);   // energy tanks 
            //int pc1,pc2,pc3,pc4,pc5,pc6,pc7;  // power calculated from before interating it with transmission ratio
            //int tau1,tau2,tau3,tau4,tau5,tau6,tau7; // torque cal from controller without energy tank and transmission ratio
/*            for (int i=0; i<6; ++i)
            {
               s0[i]=sqrt(3);
               //s[i]=0.0;
               H[i]=0.0;
               pc[i]=0.0;
               tau[i]=0.0;

            }
*/  
            //s0[0] = 0.0;
            //s0[1] = 1.0;

            // Integration parameters
  double t0 = 0.0;
  double t1 = 10.0;
  double d_t = 1.0;
            
            // may be required to initiate
            //double static_friction = 0.0;
            //double viscous_friction = 0.0;
            //double velocity_tolerance = 0.0;

            // initial values
   int ko = 250;      //Stiffness constant
   int kt = 250;      //Stiffness constant
   int b = 3;      //Damping Coefficient
   int epsilon = 0.001;      //Minimum energy in tank
   int Emax = 0.6;      //Maximum allowed energy
   int Pmax = 0.8;      //Maximum allowed power
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
   MatrixXd Kti = kt * I3;
   MatrixXd Koi = ko * I3; 
   MatrixXd Kci(3,3);
   Kci << 0,0,0,0,0,0,0,0,0;

            //MatrixXd Gti = 0.5*trace(Kti)*eye(3) - Kti;          // trace refers to tensor space operator
            //MatrixXd Goi = 0.5*trace(Koi)*eye(3) - Koi;
            //MatrixXd Gci = 0.5*trace(Kci)*eye(3) - Kci;
   MatrixXd Gti = 0.5*Kti.trace()*I3 - Kti;          // trace refers to tensor space operator
   MatrixXd Goi = 0.5*Koi.trace()*I3 - Koi;
   MatrixXd Gci = 0.5*Kci.trace()*I3 - Kci;


   
   double gamma = sqrt(2*epsilon);                     // square root

//   for (int i = 0; i < joint_names_.size(); ++i)
//   {
//     q_act_[i] = joint_handle.getPosition();
     //double actual_velocity = actuated_joint.joint_handle.getVelocity();
//     qdot_[i] = joint_handle.getVelocity();
//   } 

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
             
    MatrixXd q2r_c = Quaterniond(0.513,-0.509,-0.508,0.468).toRotationMatrix();    //0.513,-0.509,-0.508,0.468
    VectorXd ee_current_trans_vec(3,1);
    ee_current_trans_vec << 0.223, 0.227, 0.500; // 0.223, 0.227, 0.500 current_pose.position.x,current_pose.position.y,current_pose.position.z;        //0.192, 0.229, 0.496;  
    Matrix4d Hot = Matrix4d::Identity();
    Hot.block(0,0,3,3) = q2r_c;
    Hot.block(0,3,3,1) = ee_current_trans_vec;


            //Hov = [0 0 -1 -0.6; 0 1 0 (0.3*sin(2*pi/T)*tt); 1 0 0 0.6; 0 0 0 1];     // desired end effector config
            //MatrixXd Hov(4,4);
            //Hov << 0,0,-1,-0.6,0,1,0,(0.3*sin(2*M_PI/T)*tt),1,0,0,0.6,0,0,0,1;

    MatrixXd q2r_d = Quaterniond(0.525,-0.526,-0.529,0.410).toRotationMatrix();     // q2r_d refers to quatrernion to rotation for desired end effector pose
    VectorXd ee_desired_trans_vec(3,1);
    ee_desired_trans_vec << 0.229, 0.372, 0.499;   // 0.229, 0.372, 0.499  
    Matrix4d Hov = Matrix4d::Identity();
    Hov.block(0,0,3,3) = q2r_d;
    Hov.block(0,3,3,1) = ee_desired_trans_vec;

    MatrixXd Hvo=[transpose(q2r_d) -transpose(q2r_d)*ee_desired_trans_vec;0 0 0 1];

            // Pre step to compute rotation and position components using relative configurations of ee to compute wrench 
            //Hvt = inv(Hov)*Hot;
    MatrixXd Hvt = Hvo*Hot;             
    

            // extracting rotaional Rvt and translational pvt part from Hvt for further calculating wrench 
            //Rvt = Hvt((1:3),(1:3));        // this will extract first three rows and first three columns from Hvt
    MatrixXd Rvt = Hvt.block(0,0,3,3);

            //pvt = Hvt((1:3),4);            // this will extract first three row entries of 4th column from Hvt
    MatrixXd pvt = Hvt.block(0,3,3,1);

            //Rtv = inv(Rvt);
    MatrixXd Rtv = Rvt.inverse(); 

            //skewness of pvt
    //accumulator_set<double, stats<tag::skewness > > pvt_sk;
    //pvt_sk(pvt(0));
    //pvt_sk(pvt(1));
    //pvt_sk(pvt(2));

    //double pvt_s = skewness(pvt_sk); 
    double pvt_s = [0 -pvt(3,1) pvt(2,1);pvt(3,1) 0 -pvt(1,1);-pvt(2,1) pvt(1,1) 0];

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
     std::string link_name = "arm_left_tool_link";  //arm_left_tool_link
     unsigned int tip_id = rbdl_model_.GetBodyId(link_name.c_str());
     Eigen::MatrixXd jacobian;      // jacobian initialization
     jacobian.resize(6, joint_names_.size());
     jacobian.setZero();
     RigidBodyDynamics::CalcPointJacobian6D(rbdl_model_, q_act_, tip_id,
                                             Eigen::Vector3d(0, 0, 0), jacobian, false);  

            
            // Safety layer (PE, KE and total Energy of the system)
            //Vti = -0.25*trace(skewness(pvt)*Gti*skewness(pvt) -0.25*trace(skewness(pvt)*Rvt*Gti*Rtv*skewness(pvt);
            //Voi = -(trace(Goi*Rvt));
            //Vci = trace(Gci*Rtv*skewness(pvt);

     int Vti = ((-0.25*pvt_s*Gti*pvt_s).trace())-((0.25*pvt_s*Rvt*Gti*Rtv*pvt_s).trace());
     int Voi = -((Goi*Rvt).trace());
     int Vci = ((Gci*Rtv*pvt_s).trace());


     int Vi = Vti + Voi + Vci;         // initial potential energy

     int KE = qdot_.transpose() * Mass * qdot_;        // transpose of qdot x M x qdot

     int Etoti = KE + Vi;               // initial energy of the system
     int k_c;
     if (Etoti > Emax)  
        k_c = (Emax - KE)/ Vi;
     else
        k_c = 1;
     return;
            // calculation of new co-stiffness matrices and corresponding potential energy

     MatrixXd Gt = k_c * Gti;           // new co-stiffness matrices
     MatrixXd Go = k_c * Goi;
     MatrixXd Gc = k_c * Gci;

            //Vt = -0.25*trace(skewness(pvt)*Gt*skewness(pvt) -0.25*trace(skewness(pvt)*Rvt*Gt*Rtv*skewness(pvt);
            //Vo = -(trace(Go*Rvt));
            //Vc = trace(Gc*Rtv*skewness(pvt);

     int Vt = ((-0.25*pvt_s*Gt*pvt_s).trace())-((0.25*pvt_s*Rvt*Gt*Rtv*pvt_s).trace());
     int Vo = -((Go*Rvt).trace());
     int Vc = ((Gc*Rtv*pvt_s).trace());

     int V = Vt + Vo + Vc;         // potential energy
     int Etot = KE + V;            // total energy of the system
                
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

     MatrixXd mGo = Go * Rvt;                           // 3x3 torque components 
     MatrixXd asmGo = (mGo - mGo.transpose())/2;        // Skew/Anti symmetric part of torque components 
     MatrixXd mGt = Gt * Rtv * pvt_s * pvt_s * Rvt;     // 3x3 torque components 
     MatrixXd asmGt = (mGt - mGt.transpose())/2;        // Skew/Anti symmetric part of torque components
     MatrixXd mGc = Gc * pvt_s * Rvt;                   // 3x3 torque components 
     MatrixXd asmGc = (mGc - mGc.transpose())/2;        // Skew/Anti symmetric part of torque components
     MatrixXd mG = (-2*asmGo) - (asmGt) - (2*asmGc);    // 3x3 Torque (rotational part of wrench)
     VectorXd m(3,1); 
     m << mG(2,1), mG(0,2), mG(1,0);          //   tG(3,2), tG(1,3), tG(2,1);    3x1 Torque (rotational part of wrench)

            // Translational part of wrench
            //fGt = Gt * skewness(ptd);
            //asfGt = (fGt - transpose(fGt))./2;
            //fGt2 = Gt * Rdt * skewness(ptd) * Rtd;
            //asfGt2 = (fGt2 - transpose(fGt2))./2;
            //fGc = Gc * Rtd;
            //asfGc = (fGc - transpose(Gc))./2;
            //fG = (-Rdt * asfGt * Rtd) - (asfGt2) - (2 * asfGc);
            //f = [fG(3,2); fG(1,3); fG(2,1)];

     MatrixXd fGt = Gt * pvt_s;                                       // 3x3 force components
     MatrixXd asfGt = (fGt - fGt.transpose())/2;                      // Skew/Anti symmetric part of force components 
     MatrixXd fGt2 = Gt * Rtv * pvt_s * Rvt;                          // 3x3 force components
     MatrixXd asfGt2 = (fGt2 - fGt2.transpose())/2;                   // Skew/Anti symmetric part of force components
     MatrixXd fGc = Gc * Rvt;                                         // 3x3 force components  
     MatrixXd asfGc = (fGc - Gc.transpose())/2;                       // Skew/Anti symmetric part of force components
     MatrixXd fG = (-Rtv * asfGt * Rvt) - (asfGt2) - (2 * asfGc);     // 3x3 Torque (Translational part of wrench) 
     VectorXd f(3,1); 
     f << fG(2,1), fG(0,2), fG(1,0);                        //  fG(3,2), fG(1,3), fG(2,1);    3x1 Torque (Translational part of wrench)

            
     VectorXd Wsn0(6,1);           // wrench vector initialization
     Wsn0 << 0,0,0,0,0,0; 
    
     Wsn0.block(0,0,3,1) = m;          // Wsn0(1:3,1) = t ... t represented with small m here;
     Wsn0.block(3,0,3,1) = f;          // Wsn0(4:6,1) = f; 
            
            //MatrixXd Htip0(6,6);
            //Htip0 << 0,0,-1,0,0,-0.6,
            //         0,1,0,0,0,0,
            //       1,0,0,0,0,0.6,
            //      0.6,0,0,0,0,-1,
            //       0,0,0,0,1,0;
            //     -0.6,0,0,1,0,0;
     MatrixXd q2r = Quaterniond(0.048,0.998,-0.024,0.015).toRotationMatrix();
     VectorXd ee_runtime_trans_vec(3,1);
     ee_runtime_trans_vec << 0.192, 0.229, 0.496;  
     Matrix4d tee_current = Matrix4d::Identity();
     tee_current.block(0,0,3,3) = q2r;
     tee_current.block(0,3,3,1) = ee_runtime_trans_vec;
            //std::cout << "\nCurrent Pose EE = \n" << tee_current << std::endl;



     MatrixXd Htipbase(6,6);
     Htipbase.fill(0);
     Htipbase.block(0,0,3,3) = q2r;
     Htipbase.block(3,3,3,3) = q2r;
     VectorXd j(3,1);
     j = ee_current_trans_vec;
     MatrixXd sk_mat(3,3); 
     sk_mat << 0,-(j(2)),j(1),j(2),0.0,-j(0),-j(1),j(0),0.0;
     Htipbase.block(3,0,3,3) = sk_mat*(q2r); 
           
     VectorXd W0 = (((Htipbase.inverse()).adjoint()).transpose()) * (Wsn0);      // wrench acting on end effector expressed in inertial frame

           // Power of the System 
     VectorXd Fs = jacobian.transpose() * W0;        // Force due to spatial compliance
     VectorXd Fdi = Bi * qdot_;                      // Force due to joint damping

     VectorXd Fci = Fs - Fdi;                       // Initial controller force

     int Pci = Fci.transpose() * qdot_;          // Initial power of the controller
           
     int beta;
     if (Pci > Pmax) 
         beta = ((Fs.transpose()*qdot_) - Pmax)/ (Fdi.transpose()*qdot_);
     else
         beta = 1;
     return;

           // New joint damping matrix using scaling parameter beta
     MatrixXd B = beta * Bi;

           // New power of the controller using new joint damping matrix
     VectorXd Fd = B * qdot_;                   // Force due to joint damping
     VectorXd F_c = Fs - Fd;                    // Controller force

     VectorXd P_c = F_c.transpose() * qdot_;       // Power of the controller


           // Controller torques for each joint
     tau[0] = F_c[0];
     tau[1] = F_c[1];
     tau[2] = F_c[2];
     tau[3] = F_c[3];
     tau[4] = F_c[4];
     tau[5] = F_c[5];
     tau[6] = F_c[6];
          
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



           //return detail::integrate_adaptive(stepper, system, start_state, start_time, end_time, dt)
           // Potential energy in each tank, an energy tank is modeled as a spring with const stiffness k = 1
           // connected to robot through a transmission unit 
           // so H = 0.5*k*s^2 ==> H = 0.5*s^2 
     H[0] = 0.5*s0[0]*s0[0];
     H[1] = 0.5*s0[1]*s0[1];
     H[2] = 0.5*s0[2]*s0[2];
     H[3] = 0.5*s0[3]*s0[3];
     H[4] = 0.5*s0[4]*s0[4];
     H[5] = 0.5*s0[5]*s0[5];
     H[6] = 0.5*s0[6]*s0[6];           

     int Htot = H[0] + H[1] + H[2] + H[3] + H[4] + H[5] + H[6];     // Total energy in tanks

           // Power of the controller on each joint
     pc[0] = tau[0] * qdot_[0];
     pc[1] = tau[1] * qdot_[1];
     pc[2] = tau[2] * qdot_[2];
     pc[3] = tau[3] * qdot_[3];
     pc[4] = tau[4] * qdot_[4];
     pc[5] = tau[5] * qdot_[5];
     pc[6] = tau[6] * qdot_[6];
          
           
           // transmission unit allows power flow from controller to robot and it is regulated by ratio u
           // transmission variable
     if ((H[0] > epsilon))     // || (pc[0] < 0)

          u_vec[0]=-tau[0]/s0[0];
     else

          u_vec[0]=(-tau[0]/gamma*gamma)*s0[0];
     return;

     if ((H[1] > epsilon))    // || (pc[1] < 0)

          u_vec[1]=-tau[1]/s0[1];
     else

          u_vec[1]=((-tau[1])/gamma*gamma)*s0[1];
     return;

     if ((H[2] > epsilon))   // || (pc[2] < 0)

              u_vec[2]=-tau[2]/s0[2];
     else

              u_vec[2]=(-tau[2]/gamma*gamma)*s0[2];
     return;

     if ((H[3] > epsilon))   // || (pc[3] < 0)

              u_vec[3]=-tau[3]/s0[3];
     else

              u_vec[3]=(-tau[3]/gamma*gamma)*s0[3];
     return;

     if ((H[4] > epsilon))   // || (pc[4] < 0)

              u_vec[4]=-tau[4]/s0[4];
     else

              u_vec[4]=(-tau[4]/gamma*gamma)*s0[4]; 
     return;

     if ((H[5] > epsilon))  // || (pc[5] < 0)

              u_vec[5]=-tau[5]/s0[5];
     else

              u_vec[5]=(-tau[5]/gamma*gamma)*s0[5];
     return;

     if ((H[6] > epsilon))  // || (pc[6] < 0)

              u_vec[6]=-tau[6]/s0[6];
     else

              u_vec[6]=(-tau[6]/gamma*gamma)*s0[6];
     return;

           //u=[u1;u2;u3;u4;u5;u6];


           // For all the joints...
     for (size_t i = 0; i < joint_names_.size(); ++i)
     {
         tau_cmd_[i] = - u_vec[i] * s0[i];
         double output_torque = tau_cmd_[i];

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
