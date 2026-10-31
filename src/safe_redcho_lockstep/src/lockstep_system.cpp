#include <algorithm>
#include <atomic>
#include <functional>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Element.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <safe_redcho_interfaces/srv/step_swarm.hpp>
#include <safe_redcho_gazebo/motor_math.hpp>

namespace safe_redcho_lockstep
{
using Step = safe_redcho_interfaces::srv::StepSwarm;
constexpr std::size_t N = 6;
constexpr int64_t PHYSICS_NS = 1000000;
constexpr int64_t CONTROL_NS = 10000000;

class LockstepSystem : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemPostUpdate
{
public:
  ~LockstepSystem() override
  {
    {
      std::lock_guard<std::mutex> guard(mutex);
      stopping = true;
    }
    cv.notify_all();
    if (executor) executor->cancel();
    if (worker.joinable()) worker.join();
    service.reset();
    executor.reset();
    node.reset();
    if (context && context->is_valid()) context->shutdown("Lockstep unloaded");
  }

  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &config,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &manager) override
  {
    worldEntity = entity;
    if (config->HasElement("initial_velocities"))
    {
      std::istringstream stream(config->Get<std::string>("initial_velocities"));
      for (auto &velocity : initialVelocities)
      {
        double x,y,z;
        if (!(stream >> x >> y >> z) || !std::isfinite(x+y+z))
          throw std::runtime_error("Expected 18 finite initial velocity components");
        velocity.Set(x,y,z);
      }
      double extra;
      if (stream >> extra) throw std::runtime_error("Extra initial velocity components");
      bootstrapEnabled = true;
    }
    const auto name = _ecm.Component<gz::sim::components::Name>(entity);
    controlService = "/world/" + (name ? name->Data() : std::string("safe_redcho_lockstep")) + "/control";
    events = &manager;
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions opts;
    opts.context(context);
    opts.use_global_arguments(false);
    node = std::make_shared<rclcpp::Node>("safe_redcho_lockstep_backend", opts);
    rclcpp::ExecutorOptions ex;
    ex.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(ex);
    executor->add_node(node);
    service = node->create_service<Step>("/safe_redcho/step",
      [this](const Step::Request::SharedPtr request,
             Step::Response::SharedPtr response)
      {
        OnRequest(request, response);
      });
    for (std::size_t i = 0; i < N; ++i)
    {
      const auto prefix = "/cf" + std::to_string(i + 1);
      statePubs[i] = node->create_publisher<nav_msgs::msg::Odometry>(
        prefix + "/state", rclcpp::QoS(10));
      motorPubs[i] = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        prefix + "/motor_applied", rclcpp::QoS(10));
    }
    // The service thread queues plain data and waits; it never touches ECM.
    worker = std::thread([this]() { executor->spin(); });
    RCLCPP_INFO(node->get_logger(),
      "Native-step backend v3.1 (one-shot velocity bootstrap): 6 agents, 10 x 1 ms physics steps per /safe_redcho/step");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (!node) return;
    if (!modelsReady) Discover(ecm);
    if (bootstrapCleanupPending)
    {
      // Physics clears velocity commands to zero rather than making them
      // inert. Remove the one-shot command before any mission physics step.
      // This also runs while paused, after the bootstrap PostUpdate snapshot.
      for (auto &link : links)
      {
        ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(link.Entity());
        ecm.RemoveComponent<gz::sim::components::WorldLinearVelocityCmd>(link.Entity());
      }
      bootstrapCleanupPending = false;
      RCLCPP_INFO(node->get_logger(), "Bootstrap velocity commands removed; mission uses motor wrenches only");
    }
    {
      std::lock_guard<std::mutex> guard(mutex);
      if (cancel || stopping)
      {
        fault = true;
        active = false;
        queued.reset();
        events->Emit<gz::sim::events::Pause>(true);
      }
      if (queued && !active)
      {
        if (fault || !modelsReady || !info.paused ||
            ((queued->sequence == 0) ? (!bootstrapEnabled || bootstrapped) :
              (queued->sequence != lastSequence + 1 || (bootstrapEnabled && !bootstrapped))))
        {
          CompleteLocked(false, "Backend not ready/paused, faulted, or sequence invalid");
        }
        else
        {
          sequence = queued->sequence;
          bootstrapActive = sequence == 0;
          for (std::size_t i = 0; i < N; ++i)
          {
            std::array<double,4> f;
            if (bootstrapActive) { f.fill(0.0); limits[i] = 0.16; }
            else {
              std::copy_n(queued->forces.begin() + 4*i, 4, f.begin());
              limits[i] = queued->limits[i];
            }
            motors[i] = safe_redcho_gazebo::MixMotors(
              f, limits[i], 0.03973/std::sqrt(2.0), 7.9379e-12/3.1582e-10);
          }
          startNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
            info.simTime).count();
          ticks = 0;
          wrenchTicks = 0;
          active = true;
          queued.reset();
          // Queue the native world-control operation. Never change pause
          // state directly inside this update phase.
          controlResult = std::make_shared<std::atomic<int>>(0);
          const auto outcome = controlResult;
          std::function<void(const gz::msgs::Boolean &, const bool)> callback =
            [outcome](const gz::msgs::Boolean &reply, const bool ok)
            { outcome->store(ok && reply.data() ? 1 : -1); };
          gz::msgs::WorldControl request;
          request.set_pause(true);
          request.set_multi_step(bootstrapActive ? 1 : 10);
          if (!transport.Request<gz::msgs::WorldControl, gz::msgs::Boolean>(
                controlService, request, callback))
          {
            active = false;
            fault = true;
            CompleteLocked(false, "Native world-control request could not be queued");
          }
        }
      }
    }
    if (active && controlResult && controlResult->load() < 0)
    {
      Fail("Native world-control service rejected the request");
      return;
    }
    if (info.paused) return;
    if (!active)
    {
      fault = true;
      events->Emit<gz::sim::events::Pause>(true);
      return;
    }
    const auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(
      info.dt).count();
    if (dt != PHYSICS_NS)
    {
      Fail("Physics dt is not exactly 1 ms");
      return;
    }
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      info.simTime).count();
    if (ns != startNs + static_cast<int64_t>(ticks + 1) * PHYSICS_NS)
    {
      Fail("Unexpected physical step timestamp inside the batch");
      return;
    }
    for (std::size_t i = 0; i < N; ++i)
    {
      const auto pose = links[i].WorldPose(ecm);
      const auto velocity = links[i].WorldLinearVelocity(ecm);
      if (!pose || !velocity)
      {
        Fail("Physical pose/velocity unavailable");
        return;
      }
      if (bootstrapActive)
      {
        // One excluded initialization step; SetLinearVelocity suppresses
        // wrench application on this step. Never used in mission batches.
        links[i].SetLinearVelocity(ecm, pose->Rot().RotateVectorReverse(initialVelocities[i]));
        continue;
      }
      auto force = pose->Rot().RotateVector(
        gz::math::Vector3d(0, 0, motors[i].thrust));
      force -= 0.012 * velocity->Length() * (*velocity);
      auto torque = pose->Rot().RotateVector(gz::math::Vector3d(
        motors[i].torque[0], motors[i].torque[1], motors[i].torque[2]));
      links[i].AddWorldWrench(ecm, force, torque);
    }
    if (!bootstrapActive) ++wrenchTicks;
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || !active) return;
    ++ticks;
    const unsigned int expectedTicks = bootstrapActive ? 1 : 10;
    if (ticks < expectedTicks) return;
    // Gazebo owns pausing after its tenth native step. Do not emit Pause
    // here: PostUpdate callbacks share this phase with physics bookkeeping.
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      info.simTime).count();
    if (ticks != expectedTicks || wrenchTicks != (bootstrapActive ? 0u : 10u) ||
        ns - startNs != static_cast<int64_t>(expectedTicks)*PHYSICS_NS)
    {
      Fail("Expected ten physical steps and ten wrench batches in 10 ms");
      return;
    }
    Step::Response response;
    response.success = true;
    response.status = bootstrapActive ? "Mission velocity initialization completed" :
      "Native batch completed: 10 physics steps, 10 wrench batches";
    response.physics_steps = ticks;
    response.wrench_steps = wrenchTicks;
    response.sequence = sequence;
    response.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    response.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    for (std::size_t i = 0; i < N; ++i)
    {
      const auto pose = links[i].WorldPose(ecm);
      const auto velocity = links[i].WorldLinearVelocity(ecm);
      const auto angular = links[i].WorldAngularVelocity(ecm);
      if (!pose || !velocity || !angular)
      {
        Fail("Final state unavailable");
        return;
      }
      nav_msgs::msg::Odometry s;
      s.header.stamp = response.stamp;
      s.header.frame_id = "world";
      s.child_frame_id = "cf" + std::to_string(i+1) + "/base_link";
      s.pose.pose.position.x = pose->Pos().X();
      s.pose.pose.position.y = pose->Pos().Y();
      s.pose.pose.position.z = pose->Pos().Z();
      s.pose.pose.orientation.x = pose->Rot().X();
      s.pose.pose.orientation.y = pose->Rot().Y();
      s.pose.pose.orientation.z = pose->Rot().Z();
      s.pose.pose.orientation.w = pose->Rot().W();
      const auto v = pose->Rot().RotateVectorReverse(*velocity);
      const auto w = pose->Rot().RotateVectorReverse(*angular);
      s.twist.twist.linear.x = v.X();
      s.twist.twist.linear.y = v.Y();
      s.twist.twist.linear.z = v.Z();
      s.twist.twist.angular.x = w.X();
      s.twist.twist.angular.y = w.Y();
      s.twist.twist.angular.z = w.Z();
      response.states.push_back(s);
      response.applied_forces.insert(response.applied_forces.end(),
        motors[i].motors.begin(), motors[i].motors.end());
      response.applied_limits.push_back(limits[i]);
      statePubs[i]->publish(s);
      std_msgs::msg::Float64MultiArray f;
      f.data = {motors[i].motors[0],motors[i].motors[1],
                motors[i].motors[2],motors[i].motors[3],limits[i]};
      motorPubs[i]->publish(f);
    }
    lastSequence = sequence;
    if (bootstrapActive)
    {
      bootstrapped = true;
      bootstrapCleanupPending = true;
    }
    active = false;
    {
      std::lock_guard<std::mutex> guard(mutex);
      result = std::move(response);
      responseReady = true;
    }
    cv.notify_all();
  }

private:
  void Discover(gz::sim::EntityComponentManager &ecm)
  {
    for (std::size_t i = 0; i < N; ++i)
    {
      auto entity = ecm.EntityByComponents(
        gz::sim::components::Model(),
        gz::sim::components::Name("cf" + std::to_string(i+1)),
        gz::sim::components::ParentEntity(worldEntity));
      if (entity == gz::sim::kNullEntity) return;
      auto model = gz::sim::Model(entity);
      links[i] = gz::sim::Link(model.LinkByName(ecm, "base_link"));
      if (!links[i].Valid(ecm)) return;
      links[i].EnableVelocityChecks(ecm);
    }
    modelsReady = true;
  }

  void OnRequest(const Step::Request::SharedPtr request,
                 Step::Response::SharedPtr response)
  {
    response->sequence = request->sequence;
    const bool bootstrapRequest = request->sequence == 0 && request->forces.empty() && request->limits.empty();
    if (!bootstrapRequest && (request->forces.size() != 4*N || request->limits.size() != N ||
        !std::all_of(request->forces.begin(), request->forces.end(),
          [](double x){return std::isfinite(x) && x>=0.0;}) ||
        !std::all_of(request->limits.begin(), request->limits.end(),
          [](double x){return std::isfinite(x) && x>=0.0 && x<=0.16;})))
    {
      response->status = "Expected 24 nonnegative finite forces and 6 limits in [0,0.16]";
      return;
    }
    std::unique_lock<std::mutex> guard(mutex);
    if (stopping || waiting)
    {
      response->status = "Backend stopping or busy";
      return;
    }
    waiting = true;
    responseReady = false;
    queued = request;
    const bool notified = cv.wait_for(guard, std::chrono::seconds(10),
      [this](){return responseReady || stopping;});
    if (notified && responseReady) *response = result;
    else
    {
      cancel = true;
      queued.reset();
      response->status = "Backend wall-time timeout or shutdown; restart launch";
    }
    waiting = false;
  }

  // Called on simulation thread with the mutex already held.
  void CompleteLocked(bool success, const std::string &status)
  {
    result = Step::Response();
    result.success = success;
    result.status = status;
    result.sequence = queued ? queued->sequence : sequence;
    queued.reset();
    responseReady = true;
    cv.notify_all();
  }
  void Fail(const std::string &status)
  {
    active = false;
    fault = true;
    events->Emit<gz::sim::events::Pause>(true);
    std::lock_guard<std::mutex> guard(mutex);
    CompleteLocked(false, status);
  }

  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  gz::sim::EventManager *events{nullptr};
  std::array<gz::sim::Link,N> links;
  std::array<safe_redcho_gazebo::MotorWrench,N> motors;
  std::array<double,N> limits{};
  bool modelsReady{false},active{false},fault{false};
  bool bootstrapEnabled{false},bootstrapActive{false},bootstrapped{false};
  bool bootstrapCleanupPending{false};
  std::array<gz::math::Vector3d,N> initialVelocities;
  uint64_t sequence{0},lastSequence{0};
  int64_t startNs{0};
  unsigned int ticks{0},wrenchTicks{0};
  gz::transport::Node transport;
  std::string controlService;
  std::shared_ptr<std::atomic<int>> controlResult;
  std::mutex mutex;
  std::condition_variable cv;
  bool waiting{false},responseReady{false},stopping{false},cancel{false};
  Step::Request::SharedPtr queued;
  Step::Response result;
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  rclcpp::Service<Step>::SharedPtr service;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread worker;
  std::array<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr,N> statePubs;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr,N> motorPubs;
};
}
GZ_ADD_PLUGIN(safe_redcho_lockstep::LockstepSystem,
  gz::sim::System,
  safe_redcho_lockstep::LockstepSystem::ISystemConfigure,
  safe_redcho_lockstep::LockstepSystem::ISystemPreUpdate,
  safe_redcho_lockstep::LockstepSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(safe_redcho_lockstep::LockstepSystem,
                   "safe_redcho_lockstep::LockstepSystem")
