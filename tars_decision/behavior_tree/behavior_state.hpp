#pragma once

namespace tars_decision
{
/**
 * @brief Behavior state
 */
enum class BehaviorState {
  RUNNING,   ///< Running state in process
  SUCCESS,   ///< Success state as result
  FAILURE,   ///< Failure state as result
  IDLE,      ///< Idle state, state as default or after cancellation
};

/**
 * @brief Type of behavior tree node
 */
enum class BehaviorType {
  PARALLEL,       ///< Parallel Composite Node
  SELECTOR,       ///< Selector Composite Node
  SEQUENCE,       ///< Sequence Composite Node
  ACTION,         ///< Action Node
  PRECONDITION,   ///< Precondition Node
};

/**
 * @brief Abort Type of behavior tree precondition node
 */
enum class AbortType {
  NONE,           ///< Do not abort anything
  SELF,           ///< Abort self, and any sub-trees running under this node
  LOW_PRIORITY,   ///< Abort any nodes to the right of this node
  BOTH            ///< Abort self, any sub-trees running under me, and any nodes to the right of this node
};
} // namespace tars_decision

