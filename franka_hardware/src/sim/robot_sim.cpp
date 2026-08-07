#include "franka_hardware/sim/robot_sim.hpp"
#include <cstring>
#include <Eigen/Dense>

namespace franka_hardware{ 

bool RobotSim::populateIndices(){
  const mjModel* m_ = franka_hardware_model_->getMjModel();
  // body index loop
  for(int n = 0; n < kNumberOfJoints+2; n++){
    std::string link_name = robot_name_ + "_link" + std::to_string(n);
    int body_id = mj_name2id(m_, mjOBJ_BODY, link_name.c_str());
    if(body_id != -1){
      link_indices_[n] = body_id;
    }
    else{
      std::cerr << "No mjOBJ_BODY called " << link_name << " found" << std::endl;
      return false;
    }
  }

  // joint site loop
  for(int n = 0; n < kNumberOfJoints+2; n++){
    std::string site_name;
    if(n < kNumberOfJoints){
      site_name = robot_name_ + "_joint" + std::to_string(n+1) + "_site";
    }
    else if (n == kNumberOfJoints){
      site_name = robot_name_ + "_flange_site";
    }
    else{
      site_name = robot_name_ + "_ee_site";
    }

    int site_id = mj_name2id(m_, mjOBJ_SITE, site_name.c_str());
    if(site_id != -1){
      joint_site_indices_[n] = site_id;
    }
    else{
      std::cerr << "No mjOBJ_SITE called " << site_name << " found" << std::endl;
      return false;
    }
  }

  // joint index loop
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string joint_name = robot_name_ + "_joint" + std::to_string(n+1);
    int joint_id = mj_name2id(m_, mjOBJ_JOINT, joint_name.c_str());
    if(joint_id != -1){
      joint_indices_[n] = joint_id;
      joint_qvel_indices_[n] = m_->jnt_dofadr[joint_id]; // for qvel (nv x 1) indexing
      joint_qpos_indices_[n] = m_->jnt_qposadr[joint_id]; // for qpos (nq x 1) indexing
    }
    else{
      std::cerr << "No mjOBJ_JOINT called " << joint_name << " found" << std::endl;
      return false;
    }
  }

  // Find the actuator indices for each of the robot's torque and velocity actuators
  for(int n = 0; n < kNumberOfJoints; n++){
    std::string joint_name = robot_name_ + "_joint" + std::to_string(n+1);
    std::string trq_name = robot_name_ + "_act_trq" + std::to_string(n+1);
    std::string vel_name = robot_name_ + "_act_vel" + std::to_string(n+1);
    std::string pos_name = robot_name_ + "_act_pos" + std::to_string(n+1);
    int trq_id = mj_name2id(m_, mjOBJ_ACTUATOR, trq_name.c_str());
    int vel_id = mj_name2id(m_, mjOBJ_ACTUATOR, vel_name.c_str());
    int pos_id = mj_name2id(m_, mjOBJ_ACTUATOR, pos_name.c_str());
    if(trq_id != -1){
      act_trq_indices_[n] = trq_id;
    }
    else{
      std::cerr << "No torque mjOBJ_ACTUATOR " << trq_name << " found for " << joint_name<< std::endl; 
      return false;
    }
    if(vel_id != -1){
      act_vel_indices_[n] = vel_id;
    }
    else{
      std::cerr << "No velocity mjOBJ_ACTUATOR " << vel_name << " found for " << joint_name << std::endl; 
      return false;
    }
    if(pos_id != -1){
      act_pos_indices_[n] = pos_id;
    }
    else{
      std::cerr << "No position mjOBJ_ACTUATOR " << pos_name << " found for " << joint_name << std::endl; 
      return false;
    }
  }

  // Gripper
  if(has_gripper_){
    // gripper joint loop
    for(int n = 0; n < 2; n++){
      std::string joint_name = robot_name_ + "_finger_joint" + std::to_string(n+1); //left, then right
      int joint_id = mj_name2id(m_, mjOBJ_JOINT, joint_name.c_str());
      if(joint_id != -1){
        gripper_joint_indices_[n] = joint_id;
        gripper_joint_qvel_indices_[n] = m_->jnt_dofadr[joint_id]; // for qvel (nv x 1) indexing
        gripper_joint_qpos_indices_[n] = m_->jnt_qposadr[joint_id]; // for qpos (nq x 1) indexing
      }
      else{
        std::cerr << "No joint found for " << joint_name << std::endl; 
        return false;
      }
    }
    // gripper act loop
    std::string act_name = robot_name_ + "_act_gripper";
    int act_id = mj_name2id(m_, mjOBJ_ACTUATOR, act_name.c_str());
    if(act_id != -1){
      gripper_act_idx_ = act_id;
    }
    else{
      std::cerr << "No mjOBJ_ACTUATOR " << act_name << " found for gripper" << std::endl; 
      return false;
    }
  }
  setModelIndices();
  return true;
}

franka::RobotState RobotSim::populateFrankaState(){
  /*
    Incomplete:
    std::array<double, 7UL> q_d_; // desired ...
    std::array<double, 7UL> dq_d_; // desired ...
    std::array<double, 7UL> ddq_d_; // desired ...
    std::array<double, 7UL> tau_J_d_; // desired ...
    std::array<double, 7UL> dtau_J_; // derivative joint torque
    
    franka::RobotState current_state_; // <- pack all of them into franka::RobotState
  */
  // mjModel* m = franka_hardware_model_->getMjModel();
  mjData* d = franka_hardware_model_->getMjData();
  double tau_ext_hat_filtered[7] = {0};
  for(int i=0; i<kNumberOfJoints; i++){
    current_state_.q[i] = d->qpos[joint_qpos_indices_[i]];
    current_state_.dq[i] = d->qvel[joint_qvel_indices_[i]];
    // the actual franka publishes non-zero values when in gravcomp mode, so add the qfrc_gravcomp to match that
    current_state_.tau_J[i] = d->actuator_force[act_trq_indices_[i]] + d->qfrc_gravcomp[joint_qvel_indices_[i]]; 
    tau_ext_hat_filtered[i] = d->qfrc_applied[joint_qvel_indices_[i]];
    current_state_.tau_ext_hat_filtered[i] = tau_ext_hat_filtered[i];
  }

  // Compute external wrench using a simplified approach
  // Since MuJoCo contact forces are handled by the constraint solver,
  // we use actuator forces as a proxy for external forces
  const mjModel* m = franka_hardware_model_->getMjModel();

  // Initialize to zero
  for(int i=0; i<6; i++){
    current_state_.K_F_ext_hat_K[i] = 0.0;
    current_state_.O_F_ext_hat_K[i] = 0.0;
  }

  // TEMPORARY: Apply simulated external force when contact is detected
  // Check if there are any contacts involving robot geoms
  int ee_body_id = link_indices_[8];  // link8 (end-effector body)

  bool has_contact = false;
  int contact_count = 0;

  // Debug: print contact info (only once per 1000 cycles to avoid spam)
  static int debug_counter = 0;
  debug_counter++;

  // Debug: Show EE body ID and its geoms
  if(debug_counter % 1000 == 0){
    std::cerr << "Robot " << robot_name_ << " EE body ID: " << ee_body_id << std::endl;
    std::cerr << "  Geoms attached to EE body: ";
    for(int j = 0; j < m->ngeom; j++){
      if(m->geom_bodyid[j] == ee_body_id){
        std::cerr << j << " ";
      }
    }
    std::cerr << std::endl;

    // Show what bodies geoms 0, 72, 73, 167 belong to
    int check_geoms[] = {0, 72, 73, 167};
    for(int geom_id : check_geoms){
      if(geom_id < m->ngeom){
        std::cerr << "  Geom " << geom_id << " belongs to body " << m->geom_bodyid[geom_id] << std::endl;
      }
    }
  }

  for(int i = 0; i < d->ncon; i++){
    const mjContact& contact = d->contact[i];

    // Check if contact involves our end-effector body's geoms
    for(int j = 0; j < m->ngeom; j++){
      if(m->geom_bodyid[j] == ee_body_id){
        if(contact.geom1 == j || contact.geom2 == j){
          has_contact = true;
          contact_count++;

          if(debug_counter % 1000 == 0 && has_contact){
            std::cerr << "Robot " << robot_name_ << " contact detected!" << std::endl;
            std::cerr << "  Contact geom1: " << contact.geom1 << ", geom2: " << contact.geom2 << std::endl;
            std::cerr << "  EE geom: " << j << ", EE body: " << ee_body_id << std::endl;
            std::cerr << "  Distance: " << contact.dist << std::endl;
          }

          // Simple force estimation based on penetration depth
          double penetration = -contact.dist;
          if(penetration > 0){
            // Use contact normal for force direction
            double force_mag = penetration * 100.0; // stiffness factor (N/m)

            // Apply force in normal direction (contact frame first column)
            // contact.frame is 3x3 row-major: frame[0-2] = normal, [3-5] = tangent1, [6-8] = tangent2
            double force_x = contact.frame[0] * force_mag;
            double force_y = contact.frame[1] * force_mag;
            double force_z = contact.frame[2] * force_mag;

            // Determine direction (force is applied to geom1, so if we're geom1, reverse)
            int ee_geom_idx = j;
            if(contact.geom1 == ee_geom_idx){
              current_state_.K_F_ext_hat_K[0] -= force_x;
              current_state_.K_F_ext_hat_K[1] -= force_y;
              current_state_.K_F_ext_hat_K[2] -= force_z;
            } else {
              current_state_.K_F_ext_hat_K[0] += force_x;
              current_state_.K_F_ext_hat_K[1] += force_y;
              current_state_.K_F_ext_hat_K[2] += force_z;
            }

            // Copy to O_F_ext_hat_K
            current_state_.O_F_ext_hat_K[0] = current_state_.K_F_ext_hat_K[0];
            current_state_.O_F_ext_hat_K[1] = current_state_.K_F_ext_hat_K[1];
            current_state_.O_F_ext_hat_K[2] = current_state_.K_F_ext_hat_K[2];
          }
          break;
        }
      }
      if(has_contact) break;
    }
  }

  // Debug: show total contacts periodically
  if(debug_counter % 1000 == 0){
    std::cerr << "Robot " << robot_name_ << " - Total contacts in scene: " << d->ncon << std::endl;
    if(d->ncon > 0){
      for(int i = 0; i < std::min(5, d->ncon); i++){
        std::cerr << "  Contact " << i << ": geom1=" << d->contact[i].geom1
                  << ", geom2=" << d->contact[i].geom2
                  << ", dist=" << d->contact[i].dist << std::endl;
      }
    }
  }

  // xpos is in W;
  // l7_W -> l7_B
  // T^B_W = l0_W
  // 
  // Doesn't really work; set up the inverses and stuff properly later.
  // o_t_ee_ = xpose[link0] * xpose[link7]
  int& s7 = joint_site_indices_[8];  // _ee_site on hand/gripper TCP, matching mj_dual.xml
  int& l0 = link_indices_[0];
  double eePosres[3] = {0};
  double eeQuatres[4] = {0};
  franka_hardware_model_->getBSiteInALinkframe(eePosres, eeQuatres, l0, s7);

  // transform data so that we can assign to O_T_EE
  double o_t_ee_rMVec[9] = {0};
  mju_quat2Mat(o_t_ee_rMVec, eeQuatres);
  double TMat[16] = {o_t_ee_rMVec[0], o_t_ee_rMVec[1], o_t_ee_rMVec[2], eePosres[0],
                     o_t_ee_rMVec[3], o_t_ee_rMVec[4], o_t_ee_rMVec[5], eePosres[1],
                     o_t_ee_rMVec[6], o_t_ee_rMVec[7], o_t_ee_rMVec[8], eePosres[2],
                     0,0,0,1};
  // finally, write to the state
  // convert to col major format
  // convertToColMajor
  int k = 0;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      current_state_.O_T_EE[k] = TMat[row * 4 + col];
      k++;
    }
  }

  return current_state_;
}

franka_hardware::ModelSim* RobotSim::getModel() {
      return franka_hardware_model_.get();
}

bool RobotSim::setModelIndices(){
  franka_hardware_model_->setIndices(link_indices_,
                                     joint_site_indices_,
                                     joint_indices_,
                                     joint_qpos_indices_,
                                     joint_qvel_indices_,
                                     act_trq_indices_,
                                     act_pos_indices_,
                                     act_vel_indices_);
  return true;
}

} // namespace franka_hardware