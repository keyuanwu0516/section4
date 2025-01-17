import numpy as np
import typing as T

from enum import Enum
from dataclasses import dataclass
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from scipy.interpolate import splev
from scipy.interpolate import splrep
from std_msgs.msg import Bool

from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.control import BaseController
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
from asl_tb3_lib.math_utils import wrap_angle, distance_linear, distance_angular
from asl_tb3_lib.navigation import BaseNavigator
from asl_tb3_lib.tf_utils import quaternion_to_yaw

@dataclass
class TrajectoryPlan:
    """ Data structure for holding a trajectory plan comming for A* planner and
        a trajectory smoother

    See https://docs.python.org/3.10/library/dataclasses.html for how to work
    with dataclasses. In short, __init__ function is implicitly created with
    arguments replaced by the following properties. For example, you can
    create a trajectory plan with

    ```
    my_plan = TrajectoryPlan(path=..., path_x_spline=..., path_y_spline=..., duration=...)
    ```
    """

    # raw planned path from A*
    path: np.ndarray

    # cubic spline fit of the x and y trajectory,
    # should be return values from scipy.interpolate.splrep
    #   see https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splrep.html
    path_x_spline: T.Tuple[np.ndarray, np.ndarray, int]
    path_y_spline: T.Tuple[np.ndarray, np.ndarray, int]

    # time duration of the trajectory plan
    duration: float

    def desired_state(self, t: float) -> TurtleBotState:
        """ Get state from the plan at specified time point

        Args:
            t (float): time in [seconds]

        Returns:
            TurtleBotState: desired state at t
        """
        x_d = splev(t, self.path_x_spline, der=1)
        y_d = splev(t, self.path_y_spline, der=1)

        return TurtleBotState(
            x=float(splev(t, self.path_x_spline, der=0)),
            y=float(splev(t, self.path_y_spline, der=0)),
            theta=float(np.arctan2(y_d, x_d)),
        )

    def smoothed_path(self, dt: float = 0.1) -> np.ndarray:
        """ Get the full smoothed path sampled with fixed time steps

        Args:
            dt (float): sampling duration in [seconds]

        Returns:
            np.ndarray: smoothed trajectory sampled @ dt
        """
        ts = np.arange(0., self.duration, dt)
        path = np.zeros((ts.shape[0], 2))
        path[:, 0] = splev(ts, self.path_x_spline, der=0)
        path[:, 1] = splev(ts, self.path_y_spline, der=0)

        return path


class NavMode(Enum):
    """ Navigation Mode """

    IDLE = 0    # robot does not move
    ALIGN = 1   # control angular velocity only to align orientation with the planned initial state
    TRACK = 2   # track planned trajectory using a tracking controller
    PARK = 3    # control angular velocity only to align orientation with the planned goal state


class BaseNavigator(BaseController):
    """ Student can inherit from this class to build a navigator node

    This node takes target pose from /cmd_nav, and control the robot towards the
    target pose using a switching controller that accounts for obstacles
    """

    def __init__(self, node_name: str = "navigator") -> None:
        super().__init__(node_name)

        self.mode = NavMode.IDLE

        self.is_planned = False
        self.plan_start_time = self.get_clock().now()
        self.plan: T.Optional[TrajectoryPlan] = None
        self.goal: T.Optional[TurtleBotState] = None
        self.occupancy: T.Optional[StochOccupancyGrid2D] = None

        self.cmd_nav_sub = self.create_subscription(TurtleBotState, "/cmd_nav", self.replan, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.nav_success_pub = self.create_publisher(Bool, "/nav_success", 10)
        self.planned_path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.smoothed_path_pub = self.create_publisher(Path, "/smoothed_path", 10)

        # parameters
        self.declare_parameter("theta_start_thresh", 0.05)  # threshold for heading controller
        self.declare_parameter("plan_thresh", 0.3)          # replan if at least this far from planned trajectory
        self.declare_parameter("near_thresh", 0.1)          # threshold to switch to NavMode.PARK
        self.declare_parameter("at_thresh_theta", 0.02)     # maximum angle delta from goal
        self.declare_parameter("plan_resolution", 0.1)      # resolution for A* planner in [m]
        self.declare_parameter("plan_horizon", 10.0)        # maximum grid dimension for planning

    def replan(self, goal: TurtleBotState) -> None:
        """ Re-plan the path towards some goal state

        Args:
            goal (TurtleBotState): goal state
        """
        if self.occupancy is None:
            self.get_logger().warn("Unable to replan: occupancy map not yet available")
            return

        # no need to plan if close to target
        self.goal = goal
        if self.near_goal():
            self.is_planned = True
            self.switch_mode(NavMode.PARK)
            return

        # stop the robot before planning with A* as it can take quite long to finish
        self.stop()

        # plan with A*
        new_plan = self.compute_trajectory_plan(
            state=self.state,
            goal=goal,
            occupancy=self.occupancy,
            resolution=self.get_parameter("plan_resolution").value,
            horizon=self.get_parameter("plan_horizon").value,
        )

        # planning failed
        if new_plan is None:
            self.is_planned = False
            self.get_logger().warn("Replanning failed")
            self.nav_success_pub.publish(Bool(data=False))
            return

        # planning succeeded
        self.is_planned = True
        self.plan = new_plan
        self.get_logger().info(f"Replanned to {goal}")

        # publish planned and smoothed trajectory for visualization in RVIZ
        self.publish_planned_path()
        self.publish_smooth_path()

        # no need to use heading controller if already aligned
        if self.aligned(self.plan.desired_state(0.0)):
            self.plan_start_time = self.get_clock().now()
            self.switch_mode(NavMode.TRACK)
        else:
            self.switch_mode(NavMode.ALIGN)

    def publish_planned_path(self) -> None:
        """ Publish planned path from A* """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for state in self.plan.path:
            pose_st = PoseStamped()
            pose_st.header.frame_id = "map"
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.planned_path_pub.publish(path_msg)

    def publish_smooth_path(self) -> None:
        """ Publish smoothed trajectory """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for state in self.plan.smoothed_path():
            pose_st = PoseStamped()
            pose_st.header.frame_id = "map"
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1.0
            path_msg.poses.append(pose_st)
        self.smoothed_path_pub.publish(path_msg)

    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated

        Args:
            msg (OccupancyGrid): updated map message
        """
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )

        # replan if the new map updates causes collision in the original plan
        if self.is_planned and not all([self.occupancy.is_free(s) for s in self.plan.path[1:]]):
            self.is_planned = False
            self.replan(self.goal)

    def switch_mode(self, new_mode: NavMode):
        """ Switch to some navigation mode

        Args:
            new_mode (NavMode): new navigation mode
        """
        if self.mode != new_mode:
            self.get_logger().info(f"Switching mode: {self.mode} -> {new_mode}")
            self.mode = new_mode

    def near_goal(self) -> bool:
        """ Check if current state is near the goal state in linear distance

        Returns:
            bool: True if the linear distance between current state and goal state
                  is below some threshold, False otherwise
        """
        near_thresh = self.get_parameter("near_thresh").value
        return distance_linear(self.state, self.goal) < near_thresh

    def aligned(self, target: TurtleBotState) -> bool:
        """ Check if the current state is aligned to the initial planned state in orientation

        Returns:
            bool: True if the angular distance between current state and the planned
                  initial state is below some threshold
        """
        theta_start_thresh = self.get_parameter("theta_start_thresh").value
        return distance_angular(self.state, target) < theta_start_thresh

    def close_to_plan(self) -> bool:
        """ Check whether the current state is staying close to the planned trajectory

        Returns:
            bool: True if the linear distance between current state and the planned
                  state at current time step is below some threshold, False otherwise
        """
        plan_thresh = self.get_parameter("plan_thresh").value
        t = (self.get_clock().now() - self.plan_start_time).nanoseconds * 1e-9
        return distance_linear(self.state, self.plan.desired_state(t)) < plan_thresh

    def compute_control(self) -> TurtleBotControl:
        """ High-level function for computing control targets.

        This function
            1) manipulates navigation mode transitions
            2) calls the corresponding controller depending on the current navigation mode

        Returns:
            TurtleBotControl: control target to send to the robot
        """
        # state machine switch
        if self.mode == NavMode.ALIGN:
            if self.aligned(self.plan.desired_state(0.0)):
                self.plan_start_time = self.get_clock().now()
                self.switch_mode(NavMode.TRACK)
        elif self.mode == NavMode.TRACK:
            if self.near_goal():
                self.switch_mode(NavMode.PARK)
                self.nav_success_pub.publish(Bool(data=True))
            elif self.get_clock().now() - self.plan_start_time > Duration(seconds=self.plan.duration):
                self.get_logger().info("Replanning because out of time or stuck")
                self.is_planned = False
                self.replan(self.goal)
            elif not self.close_to_plan():
                self.get_logger().info("Replanning because far from planned trajectory")
                self.is_planned = False
                self.replan(self.goal)
        elif self.mode == NavMode.PARK:
            if self.aligned(self.goal):
                self.is_planned = False
                self.switch_mode(NavMode.IDLE)

        # compute control
        if self.mode == NavMode.ALIGN:
            return self.compute_heading_control(self.state, self.plan.desired_state(0.0))
        elif self.mode == NavMode.TRACK:
            return self.compute_trajectory_tracking_control(
                state=self.state,
                plan=self.plan,
                t=(self.get_clock().now() - self.plan_start_time).nanoseconds * 1e-9,
            )
        elif self.mode == NavMode.PARK:
            return self.compute_heading_control(self.state, self.goal)
        else:   # NavMode.IDLE:
            return TurtleBotControl()

    def can_compute_control(self) -> bool:
        """ Can compute for a control only when planning succeed upon receiving a goal state

        Returns:
            bool: True if planning succeed on a goal state, False otherwise
        """
        return self.is_planned

    def compute_heading_control(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        """ Compute only orientation target (used for NavMode.ALIGN and NavMode.Park)

        Returns:
            TurtleBotControl: control target
        """
        heading_error = wrap_angle(goal.theta - state.theta)
        omega = self.kp * heading_error
        control_message = TurtleBotControl()
        control_message.omega = omega
        return control_message
        ##raise NotImplementedError("You need to implement this!")

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float,
    ) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """
        x, y, th = state.x, state.y, state.theta

        # Time difference
        dt = t - self.t_prev

        # Sample the desired state from the plan (spline)
        x_d = splev(t, plan.path_x_spline, der=0)
        xd_d = splev(t, plan.path_x_spline, der=1)
        xdd_d = splev(t, plan.path_x_spline, der=2)

        y_d = splev(t, plan.path_y_spline, der=0)
        yd_d = splev(t, plan.path_y_spline, der=1)
        ydd_d = splev(t, plan.path_y_spline, der=2)

        # Position and velocity errors
        ex = x_d - x
        ey = y_d - y
        ex_dot = xd_d - self.V_prev * np.cos(th)
        ey_dot = yd_d - self.V_prev * np.sin(th)

        # Virtual control inputs u1 and u2
        u1 = xdd_d + self.kpx * ex + self.kdx * ex_dot
        u2 = ydd_d + self.kpy * ey + self.kdy * ey_dot

        # Ensure velocity is above a threshold
        V_prev = max(self.V_prev, self.V_PREV_THRES)

        # Compute linear acceleration (a) and angular velocity (omega)
        a = np.cos(th) * u1 + np.sin(th) * u2
        om = (-np.sin(th) * u1 + np.cos(th) * u2) / V_prev

        # Update the velocity
        V = V_prev + a * dt

        # Save the previous values for the next timestep
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        # Return the control command with the calculated velocity and angular velocity
        return TurtleBotControl(velocity=V, angular_velocity=om)
        ##raise NotImplementedError("You need to implement this!")

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float,
    ) -> T.Optional[TrajectoryPlan]:
        """ Compute a trajectory plan using A* and cubic spline fitting

        Args:
            state (TurtleBotState): state
            goal (TurtleBotState): goal
            occupancy (StochOccupancyGrid2D): occupancy
            resolution (float): resolution
            horizon (float): horizon

        Returns:
            T.Optional[TrajectoryPlan]:
        """
        # Step 1: Initialize A* problem
        astar = AStar(
            statespace_lo=(-10, -10),
            statespace_hi=(10, 10),
            x_init=(state.x, state.y),
            x_goal=(goal.x, goal.y),
            occupancy=occupancy,
            resolution=resolution
        )

        # Step 2: Solve the A* problem
        if not astar.solve() or len(astar.path) < 4:
            return None

        # Step 3: Reset controller variables (for velocity and time)
        self.V_prev = 0.0
        self.t_prev = 0.0

        # Step 4: Path Time Computation - Assign times based on constant velocity heuristic
        path = np.array(astar.path)
        distances = np.linalg.norm(np.diff(path, axis=0), axis=1)
        total_distance = np.sum(distances)
        total_time = total_distance / self.V_max  # Using a constant velocity
        times = np.linspace(0, total_time, len(path))

        # Step 5: Trajectory Smoothing - Fit cubic splines to the path
        x_spline = splrep(times, path[:, 0])
        y_spline = splrep(times, path[:, 1])

        # Step 6: Construct and return the TrajectoryPlan
        trajectory_plan = TrajectoryPlan(
            path=path,
            path_x_spline=x_spline,
            path_y_spline=y_spline,
            duration=total_time
        )

        return trajectory_plan
        ##raise NotImplementedError("You need to implement this!")