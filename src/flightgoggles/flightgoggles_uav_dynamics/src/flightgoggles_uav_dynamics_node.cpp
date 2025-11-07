/**
 * @file
 * @author Ezra Tal
 * @author Winter Guerra
 * @author Varun Murali
 * @brief Implementation of the UAV dynamics and imu simulation
 */

#include "flightgoggles_uav_dynamics_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> n = rclcpp::Node::make_shared("flightgoggles_uav_dynamics_node");

  // Init class
  Uav_Dynamics uav_dynamics_node(n);

  // Spin
  rclcpp::spin(n);

  return 0;
}


/**
 * @brief Constuctor
 * @param nh ROS node handle
 */
Uav_Dynamics::Uav_Dynamics(std::shared_ptr<rclcpp::Node> nh):
node_(nh),
tfPub_(
nh->get_node_parameters_interface(),
nh->get_node_topics_interface(),
rclcpp::QoS(rclcpp::KeepLast(100)),
rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>()
),
// Buffer needs a clock
tfBuffer_(nh->get_clock()),
// Listener needs the Buffer
tfListener_(tfBuffer_),
imu_(nh),
pid_(nh),
lpf_(nh),
initPose_(7,0.0)
{

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/vehicle_mass",   vehicleMass_ );
  vehicleMass_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/vehicle_mass").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/motor_time_constant",   motorTimeconstant_ );
  motorTimeconstant_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/motor_time_constant").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/moment_arm",   momentArm_ );
  momentArm_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/moment_arm").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/thrust_coefficient",   thrustCoeff_ );
  thrustCoeff_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/thrust_coefficient").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/torque_coefficient",   torqueCoeff_ );
  torqueCoeff_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/torque_coefficient").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/drag_coefficient",   dragCoeff_ );
  dragCoeff_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/drag_coefficient").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx",   vehicleInertia_[0] );
  vehicleInertia_[0] = nh->get_parameter("/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy",   vehicleInertia_[1] );
  vehicleInertia_[1] = nh->get_parameter("/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz",   vehicleInertia_[2] );
  vehicleInertia_[2] = nh->get_parameter("/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/max_prop_speed",   maxPropSpeed_ );
  maxPropSpeed_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/max_prop_speed").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/angular_process_noise",   angAccelProcessNoiseAutoCorrelation_ );
  angAccelProcessNoiseAutoCorrelation_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/angular_process_noise").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/linear_process_noise",   linAccelProcessNoiseAutoCorrelation_ );
  linAccelProcessNoiseAutoCorrelation_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/linear_process_noise").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/reset_timeout",   resetTimeout_ );
  resetTimeout_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/reset_timeout").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/min_arming_thrust",   minArmingThrust_ );
  minArmingThrust_ = nh->get_parameter("/uav/flightgoggles_uav_dynamics/min_arming_thrust").get_value<double>();

nh->declare_parameter("/uav/flightgoggles_uav_dynamics/clockscale",   clockScale );
  clockScale = nh->get_parameter("/uav/flightgoggles_uav_dynamics/clockscale").get_value<double>();

RCLCPP_INFO(nh->get_logger(), "Running clock at %0.2f scale", clockScale);

  initPose_[6] = 1.0; // force valid quaternion to start

  position_[0] = initPose_.at(0);
  position_[1] = initPose_.at(1);
  position_[2] = initPose_.at(2);

  attitude_[0] = initPose_.at(3);
  attitude_[1] = initPose_.at(4);
  attitude_[2] = initPose_.at(5);
  attitude_[3] = initPose_.at(6);

  for (size_t i = 0; i<4; i++){
    propSpeed_[i] = sqrt(vehicleMass_/4.*grav_/thrustCoeff_);
  }

  ////////////////// Init subscribers and publishers
  // Allow for up to 100ms sim time buffer of outgoing IMU messages.
  // This should improve IMU integration methods on slow client nodes (see issue #63).
  imuPub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 96);
  gpsPub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/uav/sensors/gps", 96);
  velPub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/uav/sensors/velocity", 96);
  armPub_ = node_->create_publisher<std_msgs::msg::Bool>("/uav/armed", 96);

  inputCommandSub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>("/uav/input/rateThrust", 1, std::bind(&Uav_Dynamics::inputCallback, this, _1));

  clockPub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock",1);
  rosgraph_msgs::msg::Clock msg;
  msg.clock = currentTime_;
  clockPub_->publish(msg);

  timeLastReset_ = currentTime_;

  // Init main simulation loop at 2x framerate.
  simulationLoopTimer_ = node_->create_wall_timer(std::chrono::nanoseconds(static_cast<int>((dt_secs / clockScale) * 1e9)), std::bind(&Uav_Dynamics::simulationLoopTimerCallback, this));
}

/**
 * Main Simulator loop
 * @param event Wall clock timer event
 */
void Uav_Dynamics::simulationLoopTimerCallback(){
  // Step the time forward
  int32_t seconds = static_cast<int32_t>(dt_secs);
  uint32_t nanoseconds = static_cast<uint32_t>((dt_secs - seconds) * 1e9);
  rclcpp::Duration duration(seconds, nanoseconds);
  currentTime_ += duration;
  rosgraph_msgs::msg::Clock msg;
  msg.clock = currentTime_;
  clockPub_->publish(msg);

  // Only propagate simulation after have received input message
  if (armed_) {

    lpf_.proceedState(imuMeasurement_.angular_velocity, dt_secs);

    pid_.controlUpdate(lastCommandMsg_->twist.angular, lpf_.filterState_,
                       lpf_.filterStateDer_, angAccCommand_, dt_secs);
    computeMotorSpeedCommand();
    proceedState();
    imu_.getMeasurement(imuMeasurement_, angVelocity_, specificForceBodyFrame_, currentTime_);
    imuPub_->publish(imuMeasurement_);
  }

  publishState();
  simulationLoopTimer_->cancel();
  simulationLoopTimer_ = node_->create_wall_timer(std::chrono::nanoseconds(static_cast<int>((dt_secs / clockScale) * 1e9)), std::bind(&Uav_Dynamics::simulationLoopTimerCallback, this));
}

/**
 * inputCallback to handle incoming rate thrust message
 * @param msg rateThrust message from keyboard/ joystick controller
 */
void Uav_Dynamics::inputCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg){
	lastCommandMsg_ = msg;
	if (!armed_ && ((currentTime_.seconds() - timeLastReset_.seconds()) > resetTimeout_)) {
		if (msg->twist.linear.z >= (minArmingThrust_))
			armed_ = true;
	}
}

/**
 * computeMotorSpeedCommand computes the current motor speed
 */
void Uav_Dynamics::computeMotorSpeedCommand(void){
  double momentThrust[4] = {
    vehicleInertia_[0]*angAccCommand_[0],
    vehicleInertia_[1]*angAccCommand_[1],
    vehicleInertia_[2]*angAccCommand_[2],
    lastCommandMsg_->twist.linear.z
  };

  double motorSpeedsSquared[4] = {
    momentThrust[0]/(4*momentArm_*thrustCoeff_)+ -momentThrust[1]/(4*momentArm_*thrustCoeff_)+ -momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
    momentThrust[0]/(4*momentArm_*thrustCoeff_)+  momentThrust[1]/(4*momentArm_*thrustCoeff_)+  momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
   -momentThrust[0]/(4*momentArm_*thrustCoeff_)+  momentThrust[1]/(4*momentArm_*thrustCoeff_)+ -momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_),
   -momentThrust[0]/(4*momentArm_*thrustCoeff_)+ -momentThrust[1]/(4*momentArm_*thrustCoeff_)+  momentThrust[2]/(4*torqueCoeff_)+ momentThrust[3]/(4*thrustCoeff_)
  };

  // Assuming rotor speeds >= 0
  for(size_t i = 0; i < 4; i++)
    propSpeedCommand_[i] = fmin(sqrt(fmax(0.,motorSpeedsSquared[i])),maxPropSpeed_);
}


/**
 * proceedState compute first order euler integration of the state given inputs
 */
void Uav_Dynamics::proceedState(void){

  // Explicit Euler integration

  for(size_t i = 0; i < 3; i++){
    position_[i] += dt_secs*velocity_[i];
    velocity_[i] += dt_secs*specificForce_[i];
  }
  velocity_[2] -= dt_secs*grav_;

  double attitudeDer[4];
  attitudeDer[0] = 0.5*(angVelocity_[0]*attitude_[3] + angVelocity_[2]*attitude_[1] - angVelocity_[1]*attitude_[2]);
  attitudeDer[1] = 0.5*(angVelocity_[1]*attitude_[3] - angVelocity_[2]*attitude_[0] + angVelocity_[0]*attitude_[2]);
  attitudeDer[2] = 0.5*(angVelocity_[2]*attitude_[3] + angVelocity_[1]*attitude_[0] - angVelocity_[0]*attitude_[1]);
  attitudeDer[3] = 0.5*(-angVelocity_[0]*attitude_[0] - angVelocity_[1]*attitude_[1] - angVelocity_[2]*attitude_[2]);

  for(size_t i = 0; i < 4; i++)
    attitude_[i] += dt_secs*attitudeDer[i];

  double attNorm = sqrt(pow(attitude_[0],2.) + pow(attitude_[1],2.) + pow(attitude_[2],2.) + pow(attitude_[3],2.));

  for(size_t i = 0; i < 4; i++)
    attitude_[i] /= attNorm;

  double angAccelProcessNoise[3] = {
    sqrt(angAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(angAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(angAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_)
  };

  double linAccelProcessNoise[3] = {
    sqrt(linAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(linAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
    sqrt(linAccelProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_)
  };

  double angAccel[3] = {
    momentArm_*thrustCoeff_/vehicleInertia_[0]*(pow(propSpeed_[0],2.) +pow(propSpeed_[1],2.) -pow(propSpeed_[2],2.) -pow(propSpeed_[3],2.)) + angAccelProcessNoise[0],
    momentArm_*thrustCoeff_/vehicleInertia_[1]*(-pow(propSpeed_[0],2.) +pow(propSpeed_[1],2.) +pow(propSpeed_[2],2.) -pow(propSpeed_[3],2.)) + angAccelProcessNoise[1],
    torqueCoeff_/vehicleInertia_[2]*(-pow(propSpeed_[0],2.) +pow(propSpeed_[1],2.) -pow(propSpeed_[2],2.) +pow(propSpeed_[3],2.)) + angAccelProcessNoise[2]
  };

  for(size_t i = 0; i < 3; i++)
    angVelocity_[i] += dt_secs*angAccel[i];

  for(size_t i = 0; i < 4; i++)
    propSpeed_[i] += fmin(dt_secs,motorTimeconstant_)*((propSpeedCommand_[i] - propSpeed_[i])/motorTimeconstant_);

  double specificThrust = thrustCoeff_*(pow(propSpeed_[0],2.) + pow(propSpeed_[1],2.) + pow(propSpeed_[2],2.) + pow(propSpeed_[3],2.))/vehicleMass_;

  double speed = sqrt(pow(velocity_[0],2.) + pow(velocity_[1],2.) + pow(velocity_[2],2.));
  if (includeDrag_ && (speed > 0.)){
    double drag = dragCoeff_*pow(speed,2.);

    double dragForce[3];
    dragForce[0] = drag*-velocity_[0]/speed + linAccelProcessNoise[0];
    dragForce[1] = drag*-velocity_[1]/speed + linAccelProcessNoise[1];
    dragForce[2] = drag*-velocity_[2]/speed + linAccelProcessNoise[2];

    double a = attitude_[3];
    double b = attitude_[0];
    double c = attitude_[1];
    double d = attitude_[2];

    specificForceBodyFrame_[0] = (pow(a,2.) + pow(b,2.) - pow(c,2.) - pow(d,2.))*dragForce[0] +
                                 (2*b*c +2*a*d)*dragForce[1] +
                                 (2*b*d - 2*a*c)*dragForce[2];
    specificForceBodyFrame_[1] = (2*b*c - 2*a*d)*dragForce[0] +
                                 (pow(a,2.) - pow(b,2.) + pow(c,2.) - pow(d,2.))*dragForce[1] +
                                 (2*c*d + 2*a*b)*dragForce[2];
    specificForceBodyFrame_[2] = (2*b*d + 2*a*c)*dragForce[0] +
                                 (2*c*d - 2*a*b)*dragForce[1] +
                                 (pow(a,2.) - pow(b,2.) - pow(c,2.) + pow(d,2.))*dragForce[2];

    specificForce_[0] = dragForce[0];
    specificForce_[1] = dragForce[1];
    specificForce_[2] = dragForce[2];
  }
  else{
    double a = attitude_[3];
    double b = attitude_[0];
    double c = attitude_[1];
    double d = attitude_[2];

    specificForceBodyFrame_[0] = (pow(a,2.) + pow(b,2.) - pow(c,2.) - pow(d,2.))*linAccelProcessNoise[0] +
                                 (2*b*c +2*a*d)*linAccelProcessNoise[1] +
                                 (2*b*d - 2*a*c)*linAccelProcessNoise[2];
    specificForceBodyFrame_[1] = (2*b*c - 2*a*d)*linAccelProcessNoise[0] +
                                 (pow(a,2.) - pow(b,2.) + pow(c,2.) - pow(d,2.))*linAccelProcessNoise[1] +
                                 (2*c*d + 2*a*b)*linAccelProcessNoise[2];
    specificForceBodyFrame_[2] = (2*b*d + 2*a*c)*linAccelProcessNoise[0] +
                                 (2*c*d - 2*a*b)*linAccelProcessNoise[1] +
                                 (pow(a,2.) - pow(b,2.) - pow(c,2.) + pow(d,2.))*linAccelProcessNoise[2];
    specificForce_[0] = linAccelProcessNoise[0];
    specificForce_[1] = linAccelProcessNoise[1];
    specificForce_[2] = linAccelProcessNoise[2];
  }

  specificForceBodyFrame_[2] += specificThrust;

  specificForce_[0] += (2.*attitude_[0]*attitude_[2] + 2.*attitude_[1]*attitude_[3])*specificThrust;
  specificForce_[1] += (2.*attitude_[1]*attitude_[2] - 2.*attitude_[0]*attitude_[3])*specificThrust;
  specificForce_[2] += (-pow(attitude_[0],2.) - pow(attitude_[1],2.) + pow(attitude_[2],2.) + pow(attitude_[3],2.))*specificThrust;
}

/**
 * resetState reset state to initial
 */
void Uav_Dynamics::resetState(void){
  position_[0] = initPose_.at(0);
  position_[1] = initPose_.at(1);
  position_[2] = initPose_.at(2);

  attitude_[0] = initPose_.at(3);
  attitude_[1] = initPose_.at(4);
  attitude_[2] = initPose_.at(5);
  attitude_[3] = initPose_.at(6);

  for (size_t i = 0; i<3; i++){
    angVelocity_[i] = 0.;
    velocity_[i] = 0.;
    specificForce_[i] = 0.;
    propSpeed_[i] = sqrt(vehicleMass_/4.*grav_/thrustCoeff_);
  }
  propSpeed_[3] = sqrt(vehicleMass_/4.*grav_/thrustCoeff_);

  specificForce_[2] = 9.81;
}

/**
 * publishState publishes state transform message
 */
void Uav_Dynamics::publishState(void){
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = currentTime_;
  transform.header.frame_id = "world";

  transform.transform.translation.x = position_[0];
  transform.transform.translation.y = position_[1];
  transform.transform.translation.z = position_[2];

  transform.transform.rotation.x = attitude_[0];
  transform.transform.rotation.y = attitude_[1];
  transform.transform.rotation.z = attitude_[2];
  transform.transform.rotation.w = attitude_[3];

  transform.child_frame_id = "uav/imu";

  tfPub_.sendTransform(transform);


  geometry_msgs::msg::PoseStamped GPSReading ;

  GPSReading.header.frame_id = "world";
  GPSReading.header.stamp = currentTime_;

  // double position_noise_lvl = 0.01;
  // double attitude_noise_lvl = 0.0001;
  // double velocity_noise_lvl = 0.01;

  // double position_noise = position_noise_lvl * (std::rand()%10);
  // double attitude_noise = attitude_noise_lvl * (std::rand()%10);
  // double velocity_noise = velocity_noise_lvl * (std::rand()%10);

  double position_noise_lvl = 0;
  double attitude_noise_lvl = 0;
  double velocity_noise_lvl = 0;

  double position_noise = position_noise_lvl * (std::rand()%10);
  double attitude_noise = attitude_noise_lvl * (std::rand()%10);
  double velocity_noise = velocity_noise_lvl * (std::rand()%10);

  GPSReading.pose.position.x = position_[0] + position_noise;
  GPSReading.pose.position.y = position_[1] + position_noise;
  GPSReading.pose.position.z = position_[2] + position_noise;

  GPSReading.pose.orientation.x = attitude_[0] + attitude_noise;
  GPSReading.pose.orientation.y = attitude_[1] + attitude_noise;
  GPSReading.pose.orientation.z = attitude_[2] + attitude_noise;
  GPSReading.pose.orientation.w = attitude_[3] + attitude_noise;

  gpsPub_->publish(GPSReading);

  std_msgs::msg::Bool armMsg;
  armMsg.data = armed_;
  armPub_->publish(armMsg);

  geometry_msgs::msg::TwistStamped VelocityReading;

  VelocityReading.header.frame_id = "world";
  VelocityReading.header.stamp = currentTime_;

  VelocityReading.twist.linear.x = velocity_[0] + velocity_noise;
  VelocityReading.twist.linear.y = velocity_[1] + velocity_noise;
  VelocityReading.twist.linear.z = velocity_[2] + velocity_noise;

  velPub_->publish(VelocityReading);
  // *

}

/**
 * Constructor for Uav_Imu
 */
Uav_Imu::Uav_Imu(std::shared_ptr<rclcpp::Node> node){


node->declare_parameter("/uav/flightgoggles_imu/accelerometer_variance",   accMeasNoiseVariance_ );
  accMeasNoiseVariance_ = node->get_parameter("/uav/flightgoggles_imu/accelerometer_variance").get_value<double>();

node->declare_parameter("/uav/flightgoggles_imu/gyroscope_variance",   gyroMeasNoiseVariance_ );
  gyroMeasNoiseVariance_ = node->get_parameter("/uav/flightgoggles_imu/gyroscope_variance").get_value<double>();

}

/**
 * getMeasurement helper function to get the simulated imu measurement
 * @param meas message to be filled up
 * @param angVel angular velocity buffer
 * @param accel acceleration buffer
 * @param currTime current simulation time
 */
void Uav_Imu::getMeasurement(sensor_msgs::msg::Imu & meas, double * angVel,
                        double * accel, rclcpp::Time currTime){
  meas.header.stamp = currTime;
  meas.orientation_covariance[0] = -1;

  meas.angular_velocity.x = angVel[0] + sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);
  meas.linear_acceleration.x = accel[0] + sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

  meas.angular_velocity.y = angVel[1] + sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);
  meas.linear_acceleration.y = accel[1] + sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

  meas.angular_velocity.z = angVel[2] + sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);
  meas.linear_acceleration.z = accel[2] + sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

  meas.angular_velocity_covariance[0] = gyroMeasNoiseVariance_;
  meas.linear_acceleration_covariance[0] = accMeasNoiseVariance_;
  for (size_t i = 1; i < 8; i++){
    if (i == 4){
      meas.angular_velocity_covariance[i] = gyroMeasNoiseVariance_;
      meas.linear_acceleration_covariance[i] = accMeasNoiseVariance_;
    }
    else{
      meas.angular_velocity_covariance[i] = 0.;
      meas.linear_acceleration_covariance[i] = 0.;
    }
    meas.angular_velocity_covariance[8] = gyroMeasNoiseVariance_;
    meas.linear_acceleration_covariance[8] = accMeasNoiseVariance_;
  }
}

/**
 * Constructor for Uav_LowPassFilter
 */
Uav_LowPassFilter::Uav_LowPassFilter(std::shared_ptr<rclcpp::Node> node){

node->declare_parameter("/uav/flightgoggles_lpf/gain_p",   gainP_ );
  gainP_ = node->get_parameter("/uav/flightgoggles_lpf/gain_p").get_value<double>();
node->declare_parameter("/uav/flightgoggles_lpf/gain_q",   gainQ_ );
  gainQ_ = node->get_parameter("/uav/flightgoggles_lpf/gain_q").get_value<double>();

}

/**
 * proceedState propagates the state
 * @param value
 * @param dt
 */
void Uav_LowPassFilter::proceedState(geometry_msgs::msg::Vector3 & value, double dt){
  double input[] = {value.x, value.y, value.z};

  double det = gainP_ * dt * dt + gainQ_ * dt + 1.;
  double stateDer;
  for (size_t ind = 0; ind < 3; ind++) {
    stateDer = (filterStateDer_[ind] + gainP_ * dt * input[ind]) / det -
               (dt * gainP_ * filterState_[ind]) / det;
    filterState_[ind] =
        (dt * (filterStateDer_[ind] + gainP_ * dt * input[ind])) / det +
        ((dt * gainQ_ + 1.) * filterState_[ind]) / det;
    filterStateDer_[ind] = stateDer;
  }
}

/**
 * resetState resets the state
 */
void Uav_LowPassFilter::resetState(void){
  for (size_t ind = 0; ind < 3; ind++) {
    filterState_[ind] = 0.;
    filterStateDer_[ind] = 0.;
  }
}

/**
 * Constructor for the Uav Pid
 */
Uav_Pid::Uav_Pid(std::shared_ptr<rclcpp::Node> node){

node->declare_parameter("/uav/flightgoggles_pid/gain_p_roll",  propGain_[0] );
 propGain_[0] = node->get_parameter("/uav/flightgoggles_pid/gain_p_roll").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_i_roll",   intGain_[0] );
  intGain_[0] = node->get_parameter("/uav/flightgoggles_pid/gain_i_roll").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_d_roll",   derGain_[0] );
  derGain_[0] = node->get_parameter("/uav/flightgoggles_pid/gain_d_roll").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_p_pitch",   propGain_[1] );
  propGain_[1] = node->get_parameter("/uav/flightgoggles_pid/gain_p_pitch").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_i_pitch",   intGain_[1] );
  intGain_[1] = node->get_parameter("/uav/flightgoggles_pid/gain_i_pitch").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_d_pitch",   derGain_[1] );
  derGain_[1] = node->get_parameter("/uav/flightgoggles_pid/gain_d_pitch").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_p_yaw",   propGain_[2] );
  propGain_[2] = node->get_parameter("/uav/flightgoggles_pid/gain_p_yaw").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_i_yaw",   intGain_[2] );
  intGain_[2] = node->get_parameter("/uav/flightgoggles_pid/gain_i_yaw").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/gain_d_yaw",   derGain_[2] );
  derGain_[2] = node->get_parameter("/uav/flightgoggles_pid/gain_d_yaw").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/int_bound_roll",   intBound_[0] );
  intBound_[0] = node->get_parameter("/uav/flightgoggles_pid/int_bound_roll").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/int_bound_pitch",   intBound_[1] );
  intBound_[1] = node->get_parameter("/uav/flightgoggles_pid/int_bound_pitch").get_value<double>();

node->declare_parameter("/uav/flightgoggles_pid/int_bound_yaw",   intBound_[2] );
  intBound_[2] = node->get_parameter("/uav/flightgoggles_pid/int_bound_yaw").get_value<double>();

}

/**
 * controlUpdate
 * @param command
 * @param curval
 * @param curder
 * @param out
 * @param dt
 */
void Uav_Pid::controlUpdate(geometry_msgs::msg::Vector3 & command, double * curval,
                      double * curder, double * out, double dt){

  double stateDev[] = {command.x-curval[0], command.y-curval[1], command.z-curval[2]};

  for (size_t ind = 0; ind < 3; ind++) {
    intState_[ind] += dt * stateDev[ind];
    intState_[ind] = fmin(fmax(-intBound_[ind],intState_[ind]),intBound_[ind]);
    out[ind] = propGain_[ind] * stateDev[ind] +
               intGain_[ind] * intState_[ind] + derGain_[ind] * -curder[ind];
  }
}

/**
 * resetState
 */
void Uav_Pid::resetState(void){
  for (size_t ind = 0; ind < 3; ind++) {
    intState_[ind] = 0.;
  }
}
