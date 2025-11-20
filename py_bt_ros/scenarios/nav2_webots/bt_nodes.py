import math
from modules.base_bt_nodes import (
    BTNodeList,
    Status,
    Node,
    Sequence,
    Fallback,
    ReactiveSequence,
    ReactiveFallback,
)
from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics,
    ActionWithROSAction,
    ActionWithROSService,  
)

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger              
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


CUSTOM_ACTION_NODES = [
    "MoveToGoal",
    "CaptureImage", 
    "Return",  
]

CUSTOM_CONDITION_NODES = [
    "HasGoal",
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


class HasGoal(ConditionWithROSTopics):
    def __init__(self,
                 name,
                 agent,
                 goal_topic="/bt/goal_pose",
                 pose_topic="/amcl_pose"):
        super().__init__(name, agent, [
            (PoseStamped, goal_topic, "goal_pose_msg"),
            (PoseWithCovarianceStamped, pose_topic, "amcl_pose"),
        ])

    def _predicate(self, agent, blackboard) -> bool:
        cache = self._cache

        if "goal_pose_msg" in cache:
            blackboard["goal_pose"] = cache["goal_pose_msg"]
            del cache["goal_pose_msg"] 

        if blackboard.get("home_pose") is None and "amcl_pose" in cache:
            amcl: PoseWithCovarianceStamped = cache["amcl_pose"]
            home = PoseStamped()
            home.header = amcl.header
            home.pose = amcl.pose.pose
            blackboard["home_pose"] = home

        return blackboard.get("goal_pose") is not None


class MoveToGoal(ActionWithROSAction):

    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        ns = agent.ros_namespace or ""
        full_action_name = f"{ns}{action_name}" if ns else action_name

        super().__init__(
            name,
            agent,
            (NavigateToPose, full_action_name),
        )

    def _build_goal(self, agent, blackboard):
        ps: PoseStamped | None = blackboard.get("goal_pose")

        if ps is None:
            return None

        goal = NavigateToPose.Goal()
        goal.pose = ps

        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            blackboard["nav_result"] = "to_goal_succeeded"
            return Status.SUCCESS

        elif status_code == GoalStatus.STATUS_CANCELED:
            blackboard["nav_result"] = "to_goal_canceled"
            return Status.FAILURE

        else: 
            blackboard["nav_result"] = f"to_goal_failed_{status_code}"
            return Status.FAILURE
        
class Return(ActionWithROSAction):
    def __init__(self, name, agent, action_name="/navigate_to_pose"):
        ns = agent.ros_namespace or ""
        full_action_name = f"{ns}{action_name}" if ns else action_name

        super().__init__(
            name,
            agent,
            (NavigateToPose, full_action_name),
        )

    def _build_goal(self, agent, blackboard):
        home: PoseStamped | None = blackboard.get("home_pose")

        if home is None:
            return None

        goal = NavigateToPose.Goal()
        goal.pose = home
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            blackboard["nav_result"] = "back_home_succeeded"

            blackboard["goal_pose"] = None
            blackboard["home_pose"] = None
            blackboard["image_path"] = None
            blackboard["image_captured"] = False

            return Status.SUCCESS

        elif status_code == GoalStatus.STATUS_CANCELED:
            blackboard["nav_result"] = "back_home_canceled"
            return Status.FAILURE

        else:  # aborted 등
            blackboard["nav_result"] = f"back_home_failed_{status_code}"
            return Status.FAILURE


class CaptureImage(ActionWithROSService):
    def __init__(self, name, agent, srv_name="/bt/capture_image"):
        super().__init__(name, agent, (Trigger, srv_name))

    def update(self, agent, blackboard):
        if blackboard.get("image_captured"):
            return Status.SUCCESS
        return super().update(agent, blackboard)

    def _build_request(self, agent, blackboard):
        return Trigger.Request()

    def _interpret_response(self, response, agent, blackboard):
        if response.success:
            blackboard["image_captured"] = True
            blackboard["image_path"] = response.message
            return Status.SUCCESS
        else:
            blackboard["image_error"] = response.message
            return Status.FAILURE

