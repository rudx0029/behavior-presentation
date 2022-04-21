#pragma once
#include "element.hpp"
#include <thread>

struct Executor {
  static void run(BehaviorElement &element) {
    // Establish our Services and our Sense Input
    Services svc;
    Outcome out;
    SenseInfo sense;
    sense.ts = std::chrono::steady_clock::now();

    // init the element
    auto meta = element.initialize(svc);
    svc.messenger.notify(meta.name, "initialize");

    // run the element until done
    do {
      sense.ts = std::chrono::steady_clock::now();
      svc.messenger.notify(meta.name, "tick");
      out = element.tick(sense);

      // simulate walking
      constexpr std::chrono::milliseconds period{100};
      sense.measured_x += out.actuate.velocity * period.count() / 1000.0;
      sense.measured_velocity = out.actuate.velocity;
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    } while (out.value == Outcome::Return::Running);

    // finalize
    element.finalize();
    svc.messenger.notify(meta.name, "finalize");
  }
};