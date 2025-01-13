#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from plansys2_domain_expert.domain_expert_client import DomainExpertClient
from plansys2_problem_expert.problem_expert_client import ProblemExpertClient
from plansys2_planner.planner_client import PlannerClient
from plansys2_executor.executor_client import ExecutorClient

class AssignmentController(Node):
    def __init__(self):
        super().__init__('assignment_controller')

        # Plansys2 clients
        self.domain_expert = DomainExpertClient()
        self.problem_expert = ProblemExpertClient()
        self.planner_client = PlannerClient()
        self.executor_client = ExecutorClient()

        # Static waypoints
        self.static_goals = ["wp0", "wp1", "wp2", "wp3"]
        self.current_goal_index = 0

        # Dynamic goal variables
        self.dynamic_goal_set = False
        self.dynamic_goal = ""

        # Subscriber to /dynamic_goal
        self.goal_sub = self.create_subscription(
            String,
            '/dynamic_goal',
            self.dynamic_goal_callback,
            10
        )

        # States
        self.state = "PLAN_STATIC_GOALS"

        self.init_knowledge()

    def init_knowledge(self):
        # Add robot and waypoints
        self.problem_expert.add_instance("robot1", "robot")
        self.problem_expert.add_instance("wp0", "waypoint")
        self.problem_expert.add_instance("wp1", "waypoint")
        self.problem_expert.add_instance("wp2", "waypoint")
        self.problem_expert.add_instance("wp3", "waypoint")

        # Initial position of the robot
        self.problem_expert.add_predicate("(robot_at robot1 wp0)")

        # Connectivity of waypoints
        self.problem_expert.add_predicate("(connected wp0 wp1)")
        self.problem_expert.add_predicate("(connected wp1 wp2)")
        self.problem_expert.add_predicate("(connected wp2 wp3)")
        self.problem_expert.add_predicate("(connected wp3 wp0)")

    def step(self):
        if self.state == "PLAN_STATIC_GOALS":
            self.plan_and_execute_static_goals()
        elif self.state == "PLAN_DYNAMIC_GOAL":
            if self.dynamic_goal_set:
                self.get_logger().info("Dynamic goal execution started.")
                self.plan_and_execute_dynamic_goal()
                self.dynamic_goal_set = False
        elif self.state == "SUCCESS":
            self.get_logger().info("All goals successfully completed!")
            rclpy.shutdown()
        elif self.state == "FAILURE":
            self.get_logger().error("Failed to execute plan.")
            rclpy.shutdown()

    def plan_and_execute_static_goals(self):
        if self.current_goal_index >= len(self.static_goals):
            self.get_logger().info("Static goals completed. Waiting for dynamic goal...")
            self.state = "PLAN_DYNAMIC_GOAL"
            return

        target_wp = self.static_goals[self.current_goal_index]
        self.problem_expert.set_goal(f"(and(robot_at robot1 {target_wp}))")

        if not self.execute_current_goal():
            self.state = "FAILURE"
            return

        self.current_goal_index += 1

    def plan_and_execute_dynamic_goal(self):
        self.problem_expert.set_goal(self.dynamic_goal)

        if not self.execute_current_goal():
            self.state = "FAILURE"
            return

        self.state = "SUCCESS"

    def execute_current_goal(self):
        # Compute the plan
        domain = self.domain_expert.get_domain()
        problem = self.problem_expert.get_problem()
        plan = self.planner_client.get_plan(domain, problem)

        if not plan:
            self.get_logger().error("Could not find plan for goal.")
            return False

        if not self.executor_client.start_plan_execution(plan):
            self.get_logger().error("Failed to start plan execution.")
            return False

        while not self.executor_client.execute_and_check_plan():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.executor_client.get_result().success:
            self.get_logger().error("Plan execution failed.")
            return False

        self.get_logger().info("Goal successfully reached.")
        return True

    def dynamic_goal_callback(self, msg):
        self.dynamic_goal = msg.data
        self.get_logger().info(f"Dynamic goal received: {self.dynamic_goal}")
        self.dynamic_goal_set = True


def main(args=None):
    rclpy.init(args=args)
    node = AssignmentController()

    try:
        while rclpy.ok():
            node.step()
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

