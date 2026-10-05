#pragma once

#include <cstdint>
#include <string>

enum class NavState : uint8_t {
  WAITING = 0,
  GLOBAL_PLANNING = 1,
  TRACKING = 2,
  GOAL_ALIGN = 3,
  COMPLETED = 4,
  ABORTED = 5,
};

inline std::string nav_state_to_string(const NavState state)
{
  switch (state) {
    case NavState::WAITING: return "WAITING";
    case NavState::GLOBAL_PLANNING: return "GLOBAL_PLANNING";
    case NavState::TRACKING: return "TRACKING";
    case NavState::GOAL_ALIGN: return "GOAL_ALIGN";
    case NavState::COMPLETED: return "COMPLETED";
    case NavState::ABORTED: return "ABORTED";
    default: return "UNKNOWN";
  }
}
