#include "SafeRedchoPropellers.hh"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <tinyxml2.h>

SafeRedchoPropellers::~SafeRedchoPropellers()
{
  if (executor && node) executor->remove_node(node);
  motorSub.reset();
  stateSub.reset();
  executor.reset();
  node.reset();
  if (context && context->is_valid()) context->shutdown("GUI closed");
}

void SafeRedchoPropellers::LoadConfig(const tinyxml2::XMLElement *xml)
{
  if (xml)
  {
    auto elem = xml->FirstChildElement("robot_name");
    if (elem && elem->GetText()) robot = elem->GetText();
  }
  context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions opts;
  opts.context(context);
  opts.use_global_arguments(false);
  node = std::make_shared<rclcpp::Node>(robot + "_propeller_view", opts);
  rclcpp::ExecutorOptions exOpts;
  exOpts.context = context;
  executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(exOpts);
  // Callbacks are spun exclusively in Render(), so their state is not shared
  // with a second callback thread. Scene operations also occur only there.
  stateSub = node->create_subscription<nav_msgs::msg::Odometry>(
    "/" + robot + "/state", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
      simTime = static_cast<double>(msg->header.stamp.sec)
        + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
      haveTime = true;
    });
  motorSub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/" + robot + "/motor_applied", rclcpp::QoS(1),
    [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
    {
      if (msg->data.size() != 5) return;
      for (auto f : msg->data) if (!std::isfinite(f)) return;
      for (unsigned int i = 0; i < 4; ++i)
        thrust[i] = std::max(0.0, msg->data[i]);
      motorTime = simTime;
    });
  executor->add_node(node);
  auto window = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  if (window) window->installEventFilter(this);
  std::cout << "[SafeRedchoPropellers] Waiting for rotor visuals and ROS feedback"
            << std::endl;
}

bool SafeRedchoPropellers::eventFilter(QObject *obj, QEvent *event)
{
  if (event->type() == gz::gui::events::Render::kType) Render();
  return QObject::eventFilter(obj, event);
}

gz::rendering::VisualPtr SafeRedchoPropellers::FindRotor(unsigned int index)
{
  const auto suffix = robot + "::base_link::rotor_" + std::to_string(index + 1);
  auto visual = scene->VisualByName(suffix);
  if (visual) return visual;
  // Some scene configurations include a world prefix in their scoped names.
  for (unsigned int j = 0; j < scene->VisualCount(); ++j)
  {
    auto candidate = scene->VisualByIndex(j);
    if (!candidate) continue;
    const auto &name = candidate->Name();
    if (name.size() > suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0 &&
        name[name.size() - suffix.size() - 1] == ':')
      return candidate;
  }
  return nullptr;
}

void SafeRedchoPropellers::Render()
{
  if (executor && context->is_valid()) executor->spin_some();
  if (!scene)
  {
    for (const auto &name : gz::rendering::loadedEngines())
    {
      auto engine = gz::rendering::engine(name);
      if (!engine) continue;
      for (unsigned int i = 0; i < engine->SceneCount(); ++i)
      {
        auto candidate = engine->SceneByIndex(i);
        if (candidate && candidate->IsInitialized() && candidate->RootVisual())
        {
          scene = candidate;
          break;
        }
      }
      if (scene) break;
    }
  }
  if (!scene || !haveTime) return;
  double dt = started ? simTime - lastTime : 0.0;
  if (dt < 0.0)
  {
    angle.fill(0.0);
    thrust.fill(0.0);
    motorTime = -1.0;
    dt = 0.0;
    announced = false;
  }
  started = true;
  lastTime = simTime;
  // Limit visual jumps after GUI stalls; this has no effect on physics.
  dt = std::clamp(dt, 0.0, 0.1);
  unsigned int found = 0;
  constexpr std::array<double, 4> direction{1.0, -1.0, 1.0, -1.0};
  constexpr double hoverThrust = 0.0299 * 9.81 / 4.0;
  constexpr double twoPi = 6.2831853071795864769;
  const bool fresh = motorTime >= 0.0 && simTime - motorTime <= 0.5;
  for (unsigned int i = 0; i < 4; ++i)
  {
    auto rotor = FindRotor(i);
    if (!rotor) continue;
    ++found;
    // Illustrative 2 rotations/s at hover. sqrt(thrust) mimics speed scaling
    // qualitatively, but is NOT a calibrated RPM estimate.
    const double omega = fresh ? twoPi * 2.0 *
      std::sqrt(thrust[i] / hoverThrust) : 0.0;
    angle[i] = std::remainder(angle[i] + direction[i] * omega * dt, twoPi);
    rotor->SetLocalRotation(gz::math::Quaterniond(0.0, 0.0, angle[i]));
  }
  if (found == 4 && !announced)
  {
    std::cout << "[SafeRedchoPropellers] Found 4 rotor visuals; animation active"
              << std::endl;
    announced = true;
  }
}

GZ_ADD_PLUGIN(SafeRedchoPropellers, gz::gui::Plugin)
