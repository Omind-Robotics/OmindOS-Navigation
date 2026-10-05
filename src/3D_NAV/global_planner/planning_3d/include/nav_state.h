#ifndef NAV_STATE_H
#define NAV_STATE_H

#include <cstdint>
#include <string>

/**
 * @brief Navigation state machine enumeration
 *
 * Defines the state flow for the 3D navigation system:
 * WAITING -> GLOBAL_PLANNING -> TRACKING -> GOAL_ALIGN -> COMPLETED
 *                                     |
 *                                     +-----> ABORTED (on failure)
 */
enum class NavState : uint8_t {
    WAITING = 0,         // No goal, waiting for new target
    GLOBAL_PLANNING = 1, // Received goal, searching global path
    TRACKING = 2,        // Global path published, being tracked by local planner
    GOAL_ALIGN = 3,      // Position reached, adjusting orientation only
    COMPLETED = 4,       // Task completed, staying still
    ABORTED = 5          // Planning failed or fatal error
};

/**
 * @brief Convert NavState enum to human-readable string
 * @param state The navigation state
 * @return String representation of the state
 */
inline std::string navStateToString(NavState state) {
    switch (state) {
        case NavState::WAITING:         return "WAITING";
        case NavState::GLOBAL_PLANNING: return "GLOBAL_PLANNING";
        case NavState::TRACKING:        return "TRACKING";
        case NavState::GOAL_ALIGN:      return "GOAL_ALIGN";
        case NavState::COMPLETED:       return "COMPLETED";
        case NavState::ABORTED:         return "ABORTED";
        default:                        return "UNKNOWN";
    }
}

/**
 * @brief Convert uint8_t to NavState enum
 * @param value The uint8_t value
 * @return NavState enum value
 */
inline NavState uint8ToNavState(uint8_t value) {
    if (value <= static_cast<uint8_t>(NavState::ABORTED)) {
        return static_cast<NavState>(value);
    }
    return NavState::WAITING;  // Default fallback
}

#endif // NAV_STATE_H
