#pragma once
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

// Sensor information describing the state of the robot.
struct SenseInfo {
  double measured_velocity{};
  double measured_x{};
  bool is_flinching{};
  bool is_knee_jerking{};
  std::chrono::steady_clock::time_point ts;
};

// Actuation command for the robot
struct ActuateCmd {
  double velocity;
};

// The output of an element after it is 'ticked'. The outcome contains the status of the element, 
// running, done and successful, or done and failed. It also contains the actuation command to the
// robot. This ensures the robot receives a command on every 'tick'.
struct Outcome {
  enum class Return { Running, Success, Fail };

  Outcome() = default;
  Return value{Return::Fail};
  ActuateCmd actuate{};
};

// Contains information populated by each element. The information is a product of initializing the element.
struct ElementMeta {
  const char *name{nullptr};
};

// Common services for all elements. The Services are passed to the element during its initialization.
class Services {
public:
  struct MessengerSvc {
    void notify(const char *source, const std::string &msg) {
      std::cout << "[" << source << "] " << msg << std::endl;
    }
  };
  struct ReactionSvc {
    void activate(uint32_t bitmap_flag) {}
    void release(uint32_t bitmap_flag) {}
  };

  MessengerSvc messenger;
  ReactionSvc reaction_svc;
};

// Compile Time Reaction Definition Values
enum ReactionDef {
  ReactionDef_REQUIRED = -1,
  ReactionDef_DISABLED = 0,
  ReactionDef_ENABLED = 1,
};