#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi
from moveit_msgs.msg import ExecuteTrajectoryGoal  # 添加必要的消息导入

class MyRobot:
    def __init__(self, Group_Name):
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('node_set_redefined_pose', anonymous=True)
        
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = Group_Name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        
                #设置速度和加速度比例因子
        self._group.set_max_velocity_scaling_factor(1.0)  
        self._group.set_max_acceleration_scaling_factor(1.0)  # 同理设置加速度
        
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', 
            moveit_msgs.msg.DisplayTrajectory, 
            queue_size=1
        )
        
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', 
            moveit_msgs.msg.DisplayTrajectory, 
            queue_size=1
        )

        self._execute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', 
            moveit_msgs.msg.ExecuteTrajectoryAction
        )
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        try:
            rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
            
            # 设置目标姿态
            self._group.set_named_target(arg_pose_name)
            
            # 执行规划并获取结果（返回元组）
            plan_result = self._group.plan()
            
            # 解析规划结果（success, trajectory, planning_time, error_code）
            if not plan_result[0]:
                rospy.logerr("规划失败!")
                return False
                
            # 构造Action目标
            goal = ExecuteTrajectoryGoal()
            goal.trajectory = plan_result[1]  # 提取轨迹对象
            
            # 发送执行目标
            self._execute_trajectory_client.send_goal(goal)
            if not self._execute_trajectory_client.wait_for_result(timeout=rospy.Duration(10)):
                rospy.logwarn("执行超时!")
                return False
                
            rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
            return True
            
        except Exception as e:
            rospy.logerr("操作失败: {}".format(str(e)))
            return False

    def __del__(self):
        if moveit_commander.roscpp_initialized():
            moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')

def main():
    try:
        arm = MyRobot("arm_group")
        grip = MyRobot("grip_group")

        poses_cycle = [
            ("zero_pose", 2),
            ("grip_open", 1),
            ("pick_obj", 2),
            ("grip_close", 1),
            ("lift_obj", 2),
            ("turn_pose", 2),
            ("drop_obj", 2),
            ("grip_open", 1),
            ("zero_pose", 2),
            ("grip_close", 1)
        ]

        while not rospy.is_shutdown():
            for pose_name, delay in poses_cycle:
                if "grip" in pose_name:
                    controller = grip
                else:
                    controller = arm
                
                if not controller.set_pose(pose_name):
                    rospy.logwarn("跳过当前位姿...")
                    continue
                    
                rospy.sleep(delay)

    except KeyboardInterrupt:
        rospy.loginfo("程序被用户中断")
    finally:
        if 'arm' in locals():
            del arm
        if 'grip' in locals():
            del grip

if __name__ == '__main__':
    main()
