#ifndef SAFE_REDCHO_PROPELLERS_HH
#define SAFE_REDCHO_PROPELLERS_HH
#include <array>
#include <memory>
#include <string>
#include <gz/gui/Plugin.hh>
#include <gz/rendering/RenderTypes.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// Global class name deliberately matches the embedded QML resource.
class SafeRedchoPropellers : public gz::gui::Plugin
{
  Q_OBJECT
  public: SafeRedchoPropellers() = default;
  public: ~SafeRedchoPropellers() override;
  public: void LoadConfig(const tinyxml2::XMLElement *) override;
  protected: bool eventFilter(QObject *, QEvent *) override;
  private: void Render();
  private: gz::rendering::VisualPtr FindRotor(unsigned int);
  private: std::string robot{"cf1"};
  private: gz::rendering::ScenePtr scene;
  private: std::array<double, 4> thrust{};
  private: std::array<double, 4> angle{};
  private: double simTime{0.0}, lastTime{0.0}, motorTime{-1.0};
  private: bool haveTime{false}, started{false}, announced{false};
  private: rclcpp::Context::SharedPtr context;
  private: rclcpp::Node::SharedPtr node;
  private: std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  private: rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stateSub;
  private: rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motorSub;
};
#endif
