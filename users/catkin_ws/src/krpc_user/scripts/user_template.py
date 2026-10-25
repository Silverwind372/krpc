#!/usr/bin/python3
# -*- coding:utf-8 -*-
# =====================================================================
# IMPROVED user_template.py   (Python only, krpc_user.launch untouched)
#
# TAGS USED IN COMMENTS
#   [CHANGED]  existing template code that I modified
#   [NEW]      code I added
#   [KEEP]     template code left as is
#   [DO NOT TOUCH]  required by the system
#
# SUMMARY OF WHAT CHANGED
#   1. Visits ALL checkpoints (ID order, goal point last), not just the first.
#   2. Every wait has a timeout; moves and pass requests can be cancelled.
#   3. Limited retries (never loops forever).
#   4. Waits until the robot is stable before asking for the checkpoint pass.
#   5. A failed checkpoint is skipped instead of ending the whole mission.
#   6. mission_finished is always sent at the end (unless stopped/shutdown).
#   7. Stop handler cancels BOTH actions and ends the flow.
#   8. Skips remaining checkpoints if time is running out, to reach the goal.
#   9. Stub for AR-marker aiming (__aim_camera_at_marker) for later work.
#
# NOT TESTED ON THE SIMULATOR. Run it on the local/Web simulator and read the logs.
# =====================================================================
import json
import math
import time
import rospy
import actionlib
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType
from ib2_msgs.msg import Navigation  # [CHANGED] uncommented: used to check the robot is stable

# ---------------------------------------------------------------------
# [NEW] TUNABLE SETTINGS. Adjust these after reading your simulator logs.
# ---------------------------------------------------------------------
SERVICE_TIMEOUT = 10.0        # seconds to wait for a service/server to appear
MOVE_TIMEOUT = 60.0           # max seconds for ONE move command
PASS_TIMEOUT = 10.0           # max seconds for ONE checkpoint_passed request
CANCEL_TIMEOUT = 5.0          # max seconds to confirm a cancel
MAX_MOVE_RETRIES = 2          # extra attempts if a move fails
MAX_PASS_RETRIES = 2          # extra attempts if a pass request fails
FETCH_RETRIES = 3             # attempts to get the checkpoint list
SETTLE_TIMEOUT = 5.0          # max seconds to wait for the robot to calm down
SETTLE_HOLD_SEC = 0.5         # must stay calm this long
SETTLE_LINEAR_MAX = 0.02      # m/s   (guess: tune it)
SETTLE_ANGULAR_MAX = 0.05     # rad/s (guess: tune it)
NAV_STALE_SEC = 1.0           # navigation data older than this is ignored
# ASSUMPTION: total mission time. The manual does not state it; the replay
# screenshot suggests about 5 minutes. CHECK THE RULEBOOK and edit.
MISSION_TIME_BUDGET = 300.0
GOAL_RESERVE_SEC = 45.0       # if less time than this remains, skip to the goal


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # [KEEP] Action clients
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # [KEEP] Service clients
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # [DO NOT TOUCH]
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE

        # [CHANGED] navigation subscriber enabled (used by __wait_until_stable)
        self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav)
        # [KEEP, still optional] camera image. Enable it when you add AR marker detection.
        # self.__subscriber_camera = rospy.Subscriber('/camera_main/image_raw', Image, self.__callback_camera_image)

        # [DO NOT TOUCH]
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        # [NEW] state flags
        self.__checkpoint_active = False   # a checkpoint_passed request is running
        self.__stop_requested = False      # set by the stop service
        self.__latest_nav = None           # most recent Navigation message
        self.__latest_nav_time = 0.0       # wall time it arrived
        self.__mission_t0 = 0.0            # wall time the flow started
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE

    # -----------------------------------------------------------------
    # [NEW] small helpers
    # -----------------------------------------------------------------
    def __should_quit(self):
        return rospy.is_shutdown() or self.__stop_requested

    def __wait_for_service(self, name):
        # [NEW] wait_for_service WITH a timeout
        try:
            rospy.wait_for_service(name, timeout=SERVICE_TIMEOUT)
            return True
        except rospy.ROSException:
            rospy.logerr('service %s not available within %.1fs', name, SERVICE_TIMEOUT)
            return False

    def __cancel_and_confirm(self, client, label):
        # [NEW] cancel an action, then wait for it to really end
        client.cancel_goal()
        if not client.wait_for_result(rospy.Duration(CANCEL_TIMEOUT)):
            rospy.logwarn('%s: cancel not confirmed within %.1fs', label, CANCEL_TIMEOUT)

    def __time_left(self):
        return MISSION_TIME_BUDGET - (time.time() - self.__mission_t0)

    # -----------------------------------------------------------------
    # Mission start / finished
    # -----------------------------------------------------------------
    def __notify_mission_start(self):
        # [CHANGED] timeout + exception handling
        if not self.__wait_for_service('/event/mission_start'):
            return False
        try:
            res = self.__mission_start()
        except rospy.ServiceException as exc:
            rospy.logerr('mission_start call failed: %s', exc)
            return False
        rospy.loginfo('mission_start result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_mission_finished(self):
        # [CHANGED] timeout + exception handling
        if not self.__wait_for_service('/event/mission_finished'):
            return False
        try:
            res = self.__mission_finished()
        except rospy.ServiceException as exc:
            rospy.logerr('mission_finished call failed: %s', exc)
            return False
        rospy.loginfo('mission_finished result: success=%s message=%s', res.success, res.message)
        return res.success

    # -----------------------------------------------------------------
    # Checkpoint pass request
    # -----------------------------------------------------------------
    def __notify_checkpoint_passed(self, checkpoint_id):
        # [CHANGED] now has a deadline, cancels on timeout, tracks its own active flag
        rospy.loginfo('notify checkpoint_id=%d passed', checkpoint_id)
        if not self.__checkpoint_client.wait_for_server(rospy.Duration(SERVICE_TIMEOUT)):
            rospy.logerr('checkpoint_passed server not available')
            return False
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_active = True
        try:
            self.__checkpoint_client.send_goal(goal)
            deadline = time.time() + PASS_TIMEOUT
            while not self.__should_quit():
                if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                    result = self.__checkpoint_client.get_result()
                    rospy.loginfo('checkpoint_passed result: %s', result)
                    return bool(result.success)
                if time.time() > deadline:
                    rospy.logwarn('checkpoint_passed timed out for id=%d', checkpoint_id)
                    break
            self.__cancel_and_confirm(self.__checkpoint_client, 'checkpoint_passed')
            return False
        finally:
            self.__checkpoint_active = False

    # -----------------------------------------------------------------
    # Reading checkpoints
    # -----------------------------------------------------------------
    def __get_checkpoint_id(self, checkpoint):
        # [KEEP]
        for key in ('checkpoint_id', 'checkpointid', 'id'):
            if key in checkpoint:
                return int(checkpoint[key])
        return None

    def __fetch_route(self):
        # [CHANGED] replaces __resolve_first_checkpoint_target.
        # Returns an ORDERED LIST of checkpoint dicts (ID order, goal last),
        # or None if it could not be read. No more "fake origin" target.
        for attempt in range(1, FETCH_RETRIES + 1):
            if self.__should_quit():
                return None
            if not self.__wait_for_service('/mission/get_checkpoints'):
                continue
            try:
                res = self.__checkpoints()
            except rospy.ServiceException as exc:
                rospy.logwarn('get_checkpoints call failed (try %d): %s', attempt, exc)
                continue
            rospy.loginfo('get_checkpoints result: success=%s', res.success)
            if not res.success:
                rospy.logwarn('get_checkpoints unsuccessful (try %d): %s', attempt, res.message)
                continue
            route = self.__parse_route(res.message)
            if route:
                return route
        rospy.logerr('could not obtain checkpoints after %d tries', FETCH_RETRIES)
        return None

    def __parse_route(self, message):
        # [NEW] parse the full JSON; keep position, attitude quaternion, marker_pose
        try:
            payload = json.loads(message)
        except Exception as exc:
            rospy.logwarn('Failed to parse checkpoints payload: %s', exc)
            return None

        raw = payload.get('checkpoints', payload.get('Checkpoints', []))
        goal_id = payload.get('goalpoint_id')
        try:
            goal_id = int(goal_id) if goal_id is not None else None
        except (TypeError, ValueError):
            goal_id = None

        items = []
        for cp in raw:
            if not isinstance(cp, dict):
                continue
            cp_id = self.__get_checkpoint_id(cp)
            position = cp.get('position')
            if cp_id is None or not (isinstance(position, list) and len(position) >= 3):
                rospy.logwarn('skipping malformed checkpoint entry: %s', cp)
                continue

            quaternion = [0.0, 0.0, 0.0, 1.0]
            attitude = cp.get('attitude')
            if isinstance(attitude, dict):
                q = attitude.get('quaternion')
                if isinstance(q, list) and len(q) >= 4:
                    quaternion = q
                else:
                    rospy.logwarn('checkpoint %d: no attitude quaternion, using identity', cp_id)

            items.append({
                'checkpoint_id': cp_id,
                'position': [float(position[0]), float(position[1]), float(position[2])],
                'quaternion': [float(quaternion[0]), float(quaternion[1]),
                               float(quaternion[2]), float(quaternion[3])],
                'marker_pose': cp.get('marker_pose'),   # kept for future AR-marker work
                'is_goal': (cp_id == goal_id),
            })

        # Order: normal checkpoints by ID, goal point last.
        # [EDIT LATER] A smarter route (shortest path) could replace this ordering.
        normal = sorted([c for c in items if not c['is_goal']], key=lambda c: c['checkpoint_id'])
        goal = [c for c in items if c['is_goal']]
        route = normal + goal
        rospy.loginfo('route: %s', [c['checkpoint_id'] for c in route])
        return route

    # -----------------------------------------------------------------
    # Movement
    # -----------------------------------------------------------------
    def __move_to_checkpoint(self, checkpoint_target):
        # [CHANGED] same logic as the template, plus a deadline and a confirmed cancel
        position = checkpoint_target['position']
        quaternion = checkpoint_target['quaternion']
        rospy.loginfo(
            'moving to checkpoint_id=%d: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
            checkpoint_target['checkpoint_id'],
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )

        if not self.__command.wait_for_server(rospy.Duration(SERVICE_TIMEOUT)):
            rospy.logerr('/ctl/command server not available')
            return False

        target = PoseStamped()
        target.pose.position.x = position[0]
        target.pose.position.y = position[1]
        target.pose.position.z = position[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)
        self.__action_active = True
        try:
            self.__command.send_goal(goal)
            deadline = time.time() + MOVE_TIMEOUT
            while not self.__should_quit():
                if self.__command.wait_for_result(rospy.Duration(0.2)):
                    result = self.__command.get_result()
                    rospy.loginfo('move command result: %s', result)
                    return result.type == CtlCommandResult.TERMINATE_SUCCESS
                if time.time() > deadline:
                    rospy.logwarn('move timed out after %.0fs', MOVE_TIMEOUT)
                    break
            self.__cancel_and_confirm(self.__command, 'move')
            return False
        finally:
            self.__action_active = False

    def __callback_nav(self, data):
        # [NEW] remember the latest navigation data
        self.__latest_nav = data
        self.__latest_nav_time = time.time()

    def __wait_until_stable(self):
        # [NEW] A finished move does not mean the robot has settled.
        # Waits until linear/angular speed stay under the limits for SETTLE_HOLD_SEC.
        # Falls back to a short fixed sleep if navigation data is missing or stale.
        if self.__latest_nav is None or (time.time() - self.__latest_nav_time) > NAV_STALE_SEC:
            rospy.logwarn('no fresh navigation data; using fixed 1.0s settle wait')
            rospy.sleep(1.0)
            return False

        deadline = time.time() + SETTLE_TIMEOUT
        stable_since = None
        while not self.__should_quit() and time.time() < deadline:
            nav = self.__latest_nav
            lin = nav.twist.linear
            ang = nav.twist.angular
            lin_speed = math.sqrt(lin.x ** 2 + lin.y ** 2 + lin.z ** 2)
            ang_speed = math.sqrt(ang.x ** 2 + ang.y ** 2 + ang.z ** 2)
            if lin_speed < SETTLE_LINEAR_MAX and ang_speed < SETTLE_ANGULAR_MAX:
                if stable_since is None:
                    stable_since = time.time()
                if time.time() - stable_since >= SETTLE_HOLD_SEC:
                    rospy.loginfo('stable (lin=%.4f ang=%.4f)', lin_speed, ang_speed)
                    return True
            else:
                stable_since = None
            rospy.sleep(0.1)
        rospy.logwarn('did not become stable within %.1fs', SETTLE_TIMEOUT)
        return False

    def __aim_camera_at_marker(self, checkpoint):
        # [NEW, STUB] Does nothing yet. The stored attitude quaternion already
        # points the camera roughly at the marker. To improve Lv1/Lv2/Lv3, add here:
        #   - subscribe to /camera_main/image_raw and /camera_main/camera_info
        #   - ArUco 4x4 detection, marker side 0.02 m (manual section 5.13)
        #   - small /ctl/command corrections to centre the marker and fix roll
        pass

    # -----------------------------------------------------------------
    # One checkpoint, start to finish
    # -----------------------------------------------------------------
    def __visit_checkpoint(self, checkpoint):
        # [NEW] move (with retries) -> aim -> settle -> pass request (with retries)
        cp_id = checkpoint['checkpoint_id']

        moved = False
        for attempt in range(1, MAX_MOVE_RETRIES + 2):
            if self.__should_quit():
                return False
            if self.__move_to_checkpoint(checkpoint):
                moved = True
                break
            rospy.logwarn('move to checkpoint %d failed (attempt %d)', cp_id, attempt)
        if not moved:
            rospy.logerr('giving up on checkpoint %d: could not move there', cp_id)
            return False

        self.__aim_camera_at_marker(checkpoint)
        self.__wait_until_stable()

        for attempt in range(1, MAX_PASS_RETRIES + 2):
            if self.__should_quit():
                return False
            if self.__notify_checkpoint_passed(cp_id):
                rospy.loginfo('checkpoint %d PASSED', cp_id)
                return True
            rospy.logwarn('checkpoint %d pass failed (attempt %d)', cp_id, attempt)
            self.__wait_until_stable()   # settle again, then retry
        rospy.logerr('checkpoint %d NOT passed after retries', cp_id)
        return False

    # -----------------------------------------------------------------
    # Main flow
    # -----------------------------------------------------------------
    def __run_user_flow(self, msg):
        try:
            if not self.__notify_mission_start():
                return

            # USER EDIT AREA: define route planning and game-specific flow here.
            # ***********************************************
            # [CHANGED] loop over ALL checkpoints instead of only the first one.
            self.__mission_t0 = time.time()
            route = self.__fetch_route()
            if route is None:
                rospy.logerr('no route available; finishing mission')
            else:
                passed = []
                failed = []
                for checkpoint in route:
                    if self.__should_quit():
                        break
                    # [NEW] low on time: skip normal checkpoints, go for the goal
                    if not checkpoint['is_goal'] and self.__time_left() < GOAL_RESERVE_SEC:
                        rospy.logwarn('low on time (%.0fs left): skipping checkpoint %d',
                                      self.__time_left(), checkpoint['checkpoint_id'])
                        continue
                    if self.__visit_checkpoint(checkpoint):
                        passed.append(checkpoint['checkpoint_id'])
                    else:
                        failed.append(checkpoint['checkpoint_id'])
                rospy.loginfo('SUMMARY passed=%s failed_or_skipped=%s', passed, failed)
            # ***********************************************

            # [CHANGED] always report mission finished (unless stopping)
            if not self.__should_quit():
                self.__notify_mission_finished()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)

    def __callback_start(self, msg):
        # [DO NOT TOUCH]
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __stop_processing(self, request):
        # [CHANGED] sets the stop flag and cancels BOTH actions before replying
        try:
            self.__stop_requested = True
            if self.__action_active:
                self.__command.cancel_goal()
            if self.__checkpoint_active:
                self.__checkpoint_client.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()