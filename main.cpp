#include "element.hpp"
#include "executor.hpp"
#include <future>

// This element commands the robot to stop its motion. The element does not
// complete until motion has stopped.
struct Stop : public MotionElement<Stop> {
  // Compile Time Configuration
  static constexpr char const *const NAME = "Stop";
  static constexpr ReactionDef KNEE_JERK_REACTION = ReactionDef_ENABLED;
  static constexpr ReactionDef FLINCH_REACTION = ReactionDef_ENABLED;

  static Outcome motion_element_tick(Stop &me, const SenseInfo &in) {
    Outcome out;

    bool is_zero = std::numeric_limits<double>::epsilon() >=
                   std::abs(in.measured_velocity);

    out.value = is_zero ? Outcome::Return::Success : Outcome::Return::Running;
    out.actuate.velocity = 0;

    return out;
  }
};

// The WalkToPosition element walks the robot along the x-axis towards the goal
// provided to the element
struct WalkToPosition : public MotionElement<WalkToPosition> {
  /// Constructor accepts the element's goal and additional params
  /// @param goal the absolute x coordinate in meters
  WalkToPosition(double goal) : goal_x(goal) {}

  // MotionElement Compile Time Requirements
  static constexpr char const *const NAME = "WalkToPosition";
  static constexpr ReactionDef FLINCH_REACTION =
      ReactionDef_DISABLED; // we don't care if we flinch while walking
  static constexpr ReactionDef KNEE_JERK_REACTION = ReactionDef_ENABLED;

  // MotionElement Static Overrides
  static void motion_element_data_initialize(WalkToPosition &me,
                                             const SenseInfo &s) {
    me.init_ts = s.ts;
  }

  // Tick Implementation
  static Outcome motion_element_tick(WalkToPosition &me, const SenseInfo &s) {
    // method constants
    static constexpr double MY_VELO = 1.0;        // m/s
    static constexpr double GOAL_THRESHOLD = 0.1; // m
    static constexpr std::chrono::milliseconds TIMEOUT{60000};

    Outcome o;
    // Always set the velocity, even if we are done
    // The next element will take control, ensuring there are no jerks between
    // elements
    o.actuate.velocity = MY_VELO;
    o.value = Outcome::Return::Running;

    // Determine the error between goal and measured and apply our control
    // command (velocity)
    auto dist_x = me.goal_x - s.measured_x;
    o.actuate.velocity = dist_x >= 0 ? MY_VELO : -MY_VELO;

    // Evaluate the exit conditions and adjust the actuate info if needed
    if (std::abs(dist_x) < GOAL_THRESHOLD) {
      // we reached the target successfully
      me.messenger().notify(NAME, "goal reached");
      o.value = Outcome::Return::Success;
    } else if (s.ts - me.init_ts > TIMEOUT) {
      // fail on timeout
      me.messenger().notify(NAME, "timeout");
      o.value = Outcome::Return::Fail;
    } else if (s.is_knee_jerking) {
      // for safety set the vel to 0 even though the reflex is in command now.
      // continue running, otherwise
      o.actuate.velocity = 0;
    } else {
      me.messenger().notify(NAME,
                            "velocity=" + std::to_string(o.actuate.velocity) +
                                " pos=" + std::to_string(s.measured_x) +
                                " dist=" + std::to_string(dist_x) +
                                " goal=" + std::to_string(me.goal_x));
    }

    return o;
  }

private:
  double goal_x{};
  std::chrono::steady_clock::time_point init_ts;
};

int main(int, char **) {
  // Create the behavior -> walk then stop
  Stop stop;
  WalkToPosition walk(4);
  SequenceElement sequence({std::ref(walk), std::ref(stop)});

  // run it asynchronously so we can do other work, like mapping or planning
  auto result = std::async(Executor::run, std::ref(sequence));
  result.wait();
}


