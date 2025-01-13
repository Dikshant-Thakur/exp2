#include <plansys2_pddl_parser/Utils.h>
#include <memory>
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node {
public:
  PatrollingController()
  : rclcpp::Node("patrolling_controller"), state_(STARTING) {
  }

  void init() {
    // Initialize Plansys2 Clients
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge() {
    

    RCLCPP_INFO(this->get_logger(), "Adding robot and waypoints...");
    problem_expert_->addInstance(plansys2::Instance{"robot1", "robot"}); // Updated Robot instance
    problem_expert_->addInstance(plansys2::Instance{"wp0", "waypoint"}); // Updated Waypoint instances
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"}); // Included wp4 for consistency


    RCLCPP_INFO(this->get_logger(), "Adding initial predicates...");
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at robot1 wp0)")); // Updated robot initial position
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp0 wp1)"));   // Updated connectivity predicates
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp4)"));

  }

  void step() {
    switch (state_) {
      case STARTING: {
        // Set the goal
        problem_expert_->setGoal(plansys2::Goal("(and (robot_at robot1 wp1))"));

        // Compute the plan
        auto domain = domain_expert_->getDomain();
        RCLCPP_INFO(this->get_logger(), "Domain: %s", domain.c_str());

        auto problem = problem_expert_->getProblem();
        RCLCPP_INFO(this->get_logger(), "Problem: %s", problem.c_str());

        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
          std::ostringstream plan_stream;
          for (const auto & action : plan.value().items) {
            plan_stream << action.action << " ";
          }
          RCLCPP_INFO(this->get_logger(), "Plan: %s", plan_stream.str().c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Plan not found! Cannot proceed.");

        }
        // Execute the plan
        if (executor_client_->start_plan_execution(plan.value())) {
          state_ = MOVING_TO_WP1;
        }
        break;
      }
      case MOVING_TO_WP1: {
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
          RCLCPP_INFO(this->get_logger(), "[%s %.2f%%]",
            action_feedback.action.c_str(),
            action_feedback.completion * 100.0);
        }

        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            RCLCPP_INFO(this->get_logger(), "Successfully reached wp1!");

            // Clean up and move to the next waypoint
            problem_expert_->removePredicate(plansys2::Predicate("(robot_at robot1 wp1)"));
            problem_expert_->addPredicate(plansys2::Predicate("(robot_at robot1 wp2)"));
            state_ = MOVING_TO_WP2;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Execution failed! Replanning...");

            // Replan
            auto domain = domain_expert_->getDomain();


            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              RCLCPP_ERROR(this->get_logger(), "Unsuccessful replan attempt to reach wp1.");
              break;
            }

            executor_client_->start_plan_execution(plan.value());
          }
        }
        break;
      }
      case MOVING_TO_WP2: {
        RCLCPP_INFO(this->get_logger(), "Patrolling complete!");
        rclcpp::shutdown();
        break;
      }
      default:
        break;
    }
  }

private:
  typedef enum {STARTING, MOVING_TO_WP1, MOVING_TO_WP2} StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
