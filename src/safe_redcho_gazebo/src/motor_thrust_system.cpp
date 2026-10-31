#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <safe_redcho_gazebo/motor_math.hpp>

namespace safe_redcho_gazebo
{
class MotorThrustSystem : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate,
                          public gz::sim::ISystemPostUpdate
{
public:
  ~MotorThrustSystem() override
  {
    if (executor && node) executor->remove_node(node);
    subscription.reset();
    statePublisher.reset();
    motorPublisher.reset();
    node.reset();
    executor.reset();
    if (context && context->is_valid()) context->shutdown("Gazebo plugin unloaded");
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    const auto readDouble = [&sdf](const std::string &key, double fallback)
    { return sdf->HasElement(key) ? sdf->Get<double>(key) : fallback; };
    const auto linkName = sdf->HasElement("link_name") ? sdf->Get<std::string>("link_name") : "base_link";
    robotName = sdf->HasElement("robot_name") ? sdf->Get<std::string>("robot_name") : "cf1";
    if (!model.Valid(ecm)) return;
    link = gz::sim::Link(model.LinkByName(ecm, linkName));
    if (!link.Valid(ecm)) return;
    drag = readDouble("drag_coefficient", 0.012);
    maxMotorThrust = readDouble("max_motor_thrust", 0.16);
    activeLimit = maxMotorThrust;
    lever = readDouble("arm_length", 0.03973)/std::sqrt(2.0);
    torqueRatio = readDouble("torque_ratio", 7.9379e-12/3.1582e-10);
    commandTimeout = readDouble("command_timeout", 0.25);
    statePeriod = 1.0/readDouble("state_frequency", 100.0);
    if (!(maxMotorThrust>0.0 && statePeriod>0.0 && commandTimeout>0.0)) return;
    link.EnableVelocityChecks(ecm);

    // Private ROS context: no global rclcpp init/shutdown shared with other plugins.
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    options.use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>(robotName+"_motor_plugin", options);
    rclcpp::ExecutorOptions executorOptions;
    executorOptions.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
    executor->add_node(node);
    subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/"+robotName+"/motor_command", rclcpp::QoS(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        if (msg->data.size()!=5 ||
            !std::all_of(msg->data.begin(), msg->data.end(), [](double v){return std::isfinite(v);}) ||
            msg->data[4]<0.0)
        {
          RCLCPP_ERROR(node->get_logger(), "Expected finite [f1,f2,f3,f4,Fmax] in newtons; rejected command");
          return;
        }
        for (std::size_t i=0;i<4;++i) command[i]=msg->data[i];
        activeLimit=std::min(maxMotorThrust,msg->data[4]);
        lastCommandTime=currentSimTime;
        haveCommand=true;
      });
    statePublisher=node->create_publisher<nav_msgs::msg::Odometry>("/"+robotName+"/state",rclcpp::QoS(10));
    motorPublisher=node->create_publisher<std_msgs::msg::Float64MultiArray>("/"+robotName+"/motor_applied",rclcpp::QoS(10));
    ready=true;
    RCLCPP_INFO(node->get_logger(), "Motor plugin ready: /%s/motor_command; Newton thrust, body torque, quadratic drag",robotName.c_str());
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready) return;
    currentSimTime=std::chrono::duration<double>(info.simTime).count();
    if (currentSimTime<previousSimTime)
    {
      command.fill(0.0); haveCommand=false; lastStateTime=-1.0; activeLimit=maxMotorThrust;
    }
    previousSimTime=currentSimTime;
    // ROS callbacks and actuator calculations run on the Gazebo simulation thread.
    // No callback accesses ECM from a separate thread.
    executor->spin_some();
    if (info.paused) return;
    std::array<double,4> effective{};
    if (haveCommand && currentSimTime-lastCommandTime<=commandTimeout) effective=command;
    applied=MixMotors(effective,activeLimit,lever,torqueRatio);
    const auto pose=link.WorldPose(ecm);
    const auto velocity=link.WorldLinearVelocity(ecm);
    if (!pose || !velocity) return;
    auto force=pose->Rot().RotateVector(gz::math::Vector3d(0.0,0.0,applied.thrust));
    force-=drag*velocity->Length()*(*velocity);
    const auto torque=pose->Rot().RotateVector(gz::math::Vector3d(applied.torque[0],applied.torque[1],applied.torque[2]));
    link.AddWorldWrench(ecm,force,torque);
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (!ready || info.paused) return;
    const double t=std::chrono::duration<double>(info.simTime).count();
    if (lastStateTime>=0.0 && t-lastStateTime+1e-9<statePeriod) return;
    const auto pose=link.WorldPose(ecm);
    const auto velocity=link.WorldLinearVelocity(ecm);
    const auto angular=link.WorldAngularVelocity(ecm);
    if (!pose || !velocity || !angular) return;
    lastStateTime=t;
    nav_msgs::msg::Odometry state;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    state.header.stamp.sec=static_cast<int32_t>(ns/1000000000LL);
    state.header.stamp.nanosec=static_cast<uint32_t>(ns%1000000000LL);
    state.header.frame_id="world";
    state.child_frame_id=robotName+"/base_link";
    state.pose.pose.position.x=pose->Pos().X();
    state.pose.pose.position.y=pose->Pos().Y();
    state.pose.pose.position.z=pose->Pos().Z();
    state.pose.pose.orientation.x=pose->Rot().X();
    state.pose.pose.orientation.y=pose->Rot().Y();
    state.pose.pose.orientation.z=pose->Rot().Z();
    state.pose.pose.orientation.w=pose->Rot().W();
    // nav_msgs/Odometry twist is expressed in child_frame_id. Both vectors
    // below are BODY coordinates; pose is expressed in the world frame.
    const auto bodyVelocity=pose->Rot().RotateVectorReverse(*velocity);
    const auto bodyAngular=pose->Rot().RotateVectorReverse(*angular);
    state.twist.twist.linear.x=bodyVelocity.X();
    state.twist.twist.linear.y=bodyVelocity.Y();
    state.twist.twist.linear.z=bodyVelocity.Z();
    state.twist.twist.angular.x=bodyAngular.X();
    state.twist.twist.angular.y=bodyAngular.Y();
    state.twist.twist.angular.z=bodyAngular.Z();
    statePublisher->publish(state);
    std_msgs::msg::Float64MultiArray motors;
    motors.data={applied.motors[0],applied.motors[1],applied.motors[2],applied.motors[3],activeLimit};
    motorPublisher->publish(motors);
  }
private:
  gz::sim::Link link;
  bool ready{false},haveCommand{false};
  std::string robotName;
  std::array<double,4> command{};
  MotorWrench applied;
  double drag{0.012},maxMotorThrust{0.16},activeLimit{0.16},lever{0.0},torqueRatio{0.0};
  double statePeriod{0.01},commandTimeout{0.25},currentSimTime{0.0},previousSimTime{0.0};
  double lastCommandTime{0.0},lastStateTime{-1.0};
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr statePublisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motorPublisher;
};
}
GZ_ADD_PLUGIN(safe_redcho_gazebo::MotorThrustSystem,
              gz::sim::System,
              safe_redcho_gazebo::MotorThrustSystem::ISystemConfigure,
              safe_redcho_gazebo::MotorThrustSystem::ISystemPreUpdate,
              safe_redcho_gazebo::MotorThrustSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(safe_redcho_gazebo::MotorThrustSystem,"safe_redcho_gazebo::MotorThrustSystem")
