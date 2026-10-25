#include "path_searching/replan_coordinator.h"

#include <cassert>
#include <iostream>

int main()
{
  using path_searching::AvoidanceMode;
  using path_searching::ReplanCoordinator;
  using path_searching::ReplanReason;

  constexpr AvoidanceMode modes[]{
    AvoidanceMode::GlobalReplan,
    AvoidanceMode::GlobalMonitor,
    AvoidanceMode::GlobalOnly};

  for (const auto mode : modes)
  {
    auto triggers = ReplanCoordinator::defaultTriggersForMode(mode);
    assert(triggers.periodic);

    ReplanCoordinator::Config config;
    config.avoidance_mode = mode;
    config.triggers = triggers;

    ReplanCoordinator coordinator;
    coordinator.configure(config);
    assert(coordinator.triggerEnabled(ReplanReason::Periodic));
    const auto decision = coordinator.request(
      ReplanReason::Periodic,
      10.0,
      Eigen::Vector2d::Zero(),
      Eigen::Vector2d(1.0, 0.0));
    assert(decision.should_replan);

    config.triggers.periodic = false;
    coordinator.configure(config);
    assert(!coordinator.triggerEnabled(ReplanReason::Periodic));
  }

  std::cout << "replan_coordinator_periodic_test passed\n";
  return 0;
}
