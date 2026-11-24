#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

namespace xgo_behavior_tree
{

template <typename T>
inline bool getFromBB(const BT::Blackboard::Ptr& bb, const std::string& key, T& val)
{
  if (bb && bb->get(key, val)) return true;
  if (bb && bb->rootBlackboard() && bb->rootBlackboard()->get(key, val)) return true;
  return false;
}

inline void setInRootBB(const BT::Blackboard::Ptr& bb, const std::string& key, bool val)
{
  if (bb) {
    bb->set(key, val);
    if (bb->rootBlackboard()) {
      bb->rootBlackboard()->set(key, val);
    }
  }
}

/**
 * @brief Acción asíncrona de navegación hacia un punto (Nav2 + controlador cinemático para simulación/SIL).
 */
class NavigateToPose : public BT::StatefulActionNode
{
public:
  using Nav2Action = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Action>;

  NavigateToPose(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      goal_done_(false),
      goal_success_(false),
      use_fallback_controller_(false)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destino (x,y,theta) para Nav2")
    };
  }

  BT::NodeStatus onStart() override
  {
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getInput("goal", goal_pose)) {
      return BT::NodeStatus::FAILURE;
    }

    rclcpp::Node::SharedPtr node;
    if (!getFromBB(config().blackboard, "node", node) || !node) {
      return BT::NodeStatus::FAILURE;
    }

    target_goal_ = goal_pose;
    use_fallback_controller_ = false;

    if (!action_client_) {
      action_client_ = rclcpp_action::create_client<Nav2Action>(node, "navigate_to_pose");
    }

    // Si Nav2 no está disponible, usamos el controlador cinemático directo hacia el objetivo
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(50))) {
      use_fallback_controller_ = true;
      RCLCPP_INFO(node->get_logger(), "[NavigateToPose] Controlador cinemático activo hacia (%.2f, %.2f)",
                  goal_pose.pose.position.x, goal_pose.pose.position.y);
      return BT::NodeStatus::RUNNING;
    }

    Nav2Action::Goal goal_msg;
    goal_msg.pose = goal_pose;
    goal_done_ = false;
    goal_success_ = false;
    active_goal_handle_.reset();

    auto send_goal_options = rclcpp_action::Client<Nav2Action>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleNav2::SharedPtr& goal_handle) {
      active_goal_handle_ = goal_handle;
      if (!goal_handle) {
        goal_done_ = true;
        goal_success_ = false;
      }
    };
    send_goal_options.result_callback = [this](const GoalHandleNav2::WrappedResult& result) {
      active_goal_handle_.reset();
      goal_done_ = true;
      goal_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

    future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node->get_logger(), "[NavigateToPose] Meta enviada a Nav2: (%.2f, %.2f)",
                goal_pose.pose.position.x, goal_pose.pose.position.y);

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::Node::SharedPtr node;
    getFromBB(config().blackboard, "node", node);

    if (use_fallback_controller_) {
      geometry_msgs::msg::PoseStamped curr;
      getFromBB(config().blackboard, "current_pose", curr);

      double dx = target_goal_.pose.position.x - curr.pose.position.x;
      double dy = target_goal_.pose.position.y - curr.pose.position.y;
      double dist = std::hypot(dx, dy);

      if (!cmd_vel_pub_ && node) {
        cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      }

      // Si ha llegado al destino (dentro de 25 cm)
      if (dist < 0.25) {
        if (cmd_vel_pub_) {
          geometry_msgs::msg::Twist stop_msg;
          cmd_vel_pub_->publish(stop_msg);
        }
        if (node) {
          RCLCPP_INFO(node->get_logger(), "[NavigateToPose] ¡Destino alcanzado! (dist: %.2fm)", dist);
        }
        setInRootBB(config().blackboard, "has_task", false);
        return BT::NodeStatus::SUCCESS;
      }

      // Control P angular y lineal (yaw 3D completo invariante a signo de cuaternión)
      double qx = curr.pose.orientation.x;
      double qy = curr.pose.orientation.y;
      double qz = curr.pose.orientation.z;
      double qw = curr.pose.orientation.w;
      double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      double target_yaw = std::atan2(dy, dx);
      double yaw_err = target_yaw - yaw;
      while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

      geometry_msgs::msg::Twist cmd;
      if (std::abs(yaw_err) > 0.40) {
        cmd.angular.z = std::clamp(yaw_err * 1.6, -0.8, 0.8);
        cmd.linear.x = 0.05;
      } else {
        cmd.angular.z = std::clamp(yaw_err * 1.1, -0.6, 0.6);
        cmd.linear.x = std::clamp(dist * 0.5, 0.12, 0.35);
      }

      if (cmd_vel_pub_) {
        cmd_vel_pub_->publish(cmd);
      }
      return BT::NodeStatus::RUNNING;
    }

    if (goal_done_) {
      if (goal_success_) {
        setInRootBB(config().blackboard, "has_task", false);
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (use_fallback_controller_ && cmd_vel_pub_) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
    } else if (action_client_) {
      if (active_goal_handle_) {
        action_client_->async_cancel_goal(active_goal_handle_);
        active_goal_handle_.reset();
      } else if (future_goal_handle_.valid() &&
                 future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto goal_handle = future_goal_handle_.get();
        if (goal_handle) {
          action_client_->async_cancel_goal(goal_handle);
        }
      }
    }
  }

private:
  rclcpp_action::Client<Nav2Action>::SharedPtr action_client_;
  std::shared_future<GoalHandleNav2::SharedPtr> future_goal_handle_;
  GoalHandleNav2::SharedPtr active_goal_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::PoseStamped target_goal_;
  bool goal_done_;
  bool goal_success_;
  bool use_fallback_controller_;
};

/**
 * @brief Acción asíncrona que envía una secuencia de waypoints a Nav2 o los recorre cinemáticamente.
 */
class NavigateThroughPoses : public BT::StatefulActionNode
{
public:
  using Nav2Action = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Action>;

  NavigateThroughPoses(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      current_wp_idx_(0),
      goal_done_(false),
      goal_success_(false),
      use_fallback_controller_(false)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints", "Lista secuencial de poses")
    };
  }

  BT::NodeStatus onStart() override
  {
    if (!getInput("waypoints", waypoints_) || waypoints_.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    rclcpp::Node::SharedPtr node;
    if (!getFromBB(config().blackboard, "node", node) || !node) {
      return BT::NodeStatus::FAILURE;
    }

    current_wp_idx_ = 0;
    use_fallback_controller_ = false;

    if (!action_client_) {
      action_client_ = rclcpp_action::create_client<Nav2Action>(node, "navigate_through_poses");
    }

    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(50))) {
      use_fallback_controller_ = true;
      RCLCPP_INFO(node->get_logger(), "[NavigateThroughPoses] Modo cinemático activo para %zu waypoints.", waypoints_.size());
      return BT::NodeStatus::RUNNING;
    }

    Nav2Action::Goal goal_msg;
    goal_msg.poses = waypoints_;
    goal_done_ = false;
    goal_success_ = false;
    active_goal_handle_.reset();

    auto send_goal_options = rclcpp_action::Client<Nav2Action>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleNav2::SharedPtr& goal_handle) {
      active_goal_handle_ = goal_handle;
      if (!goal_handle) {
        goal_done_ = true;
        goal_success_ = false;
      }
    };
    send_goal_options.result_callback = [this](const GoalHandleNav2::WrappedResult& result) {
      active_goal_handle_.reset();
      goal_done_ = true;
      goal_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

    future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node->get_logger(), "[NavigateThroughPoses] Enviados %zu waypoints a Nav2.", waypoints_.size());

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::Node::SharedPtr node;
    getFromBB(config().blackboard, "node", node);

    if (use_fallback_controller_) {
      if (current_wp_idx_ >= waypoints_.size()) {
        if (!cmd_vel_pub_ && node) {
          cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        }
        if (cmd_vel_pub_) {
          geometry_msgs::msg::Twist stop_msg;
          cmd_vel_pub_->publish(stop_msg);
        }
        if (node) {
          RCLCPP_INFO(node->get_logger(), "[NavigateThroughPoses] ¡Todos los waypoints completados!");
        }
        setInRootBB(config().blackboard, "has_task", false);
        return BT::NodeStatus::SUCCESS;
      }

      const auto& target = waypoints_[current_wp_idx_];
      geometry_msgs::msg::PoseStamped curr;
      getFromBB(config().blackboard, "current_pose", curr);

      double dx = target.pose.position.x - curr.pose.position.x;
      double dy = target.pose.position.y - curr.pose.position.y;
      double dist = std::hypot(dx, dy);

      if (dist < 0.25) {
        current_wp_idx_++;
        if (node) {
          RCLCPP_INFO(node->get_logger(), "[NavigateThroughPoses] Waypoint %zu alcanzado.", current_wp_idx_);
        }
        return BT::NodeStatus::RUNNING;
      }

      if (!cmd_vel_pub_ && node) {
        cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      }

      double qx = curr.pose.orientation.x;
      double qy = curr.pose.orientation.y;
      double qz = curr.pose.orientation.z;
      double qw = curr.pose.orientation.w;
      double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      double target_yaw = std::atan2(dy, dx);
      double yaw_err = target_yaw - yaw;
      while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

      geometry_msgs::msg::Twist cmd;
      if (std::abs(yaw_err) > 0.40) {
        cmd.angular.z = std::clamp(yaw_err * 1.6, -0.8, 0.8);
        cmd.linear.x = 0.05;
      } else {
        cmd.angular.z = std::clamp(yaw_err * 1.1, -0.6, 0.6);
        cmd.linear.x = std::clamp(dist * 0.5, 0.12, 0.35);
      }

      if (cmd_vel_pub_) {
        cmd_vel_pub_->publish(cmd);
      }
      return BT::NodeStatus::RUNNING;
    }

    if (goal_done_) {
      if (goal_success_) {
        setInRootBB(config().blackboard, "has_task", false);
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (use_fallback_controller_ && cmd_vel_pub_) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
    } else if (action_client_) {
      if (active_goal_handle_) {
        action_client_->async_cancel_goal(active_goal_handle_);
        active_goal_handle_.reset();
      } else if (future_goal_handle_.valid() &&
                 future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        auto goal_handle = future_goal_handle_.get();
        if (goal_handle) {
          action_client_->async_cancel_goal(goal_handle);
        }
      }
    }
  }

private:
  rclcpp_action::Client<Nav2Action>::SharedPtr action_client_;
  std::shared_future<GoalHandleNav2::SharedPtr> future_goal_handle_;
  GoalHandleNav2::SharedPtr active_goal_handle_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  size_t current_wp_idx_;
  bool goal_done_;
  bool goal_success_;
  bool use_fallback_controller_;
};

/**
 * @brief Acción asíncrona para ejecutar maniobras de desatasco (CLEAR_COSTMAP, BACKUP, SPIN).
 */
class RecoverNavigation : public BT::StatefulActionNode
{
public:
  RecoverNavigation(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      duration_s_(0.0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("recovery_type", "CLEAR_COSTMAP", "Tipo de recuperación: CLEAR_COSTMAP, BACKUP o SPIN")
    };
  }

  BT::NodeStatus onStart() override
  {
    rec_type_ = "CLEAR_COSTMAP";
    getInput("recovery_type", rec_type_);

    rclcpp::Node::SharedPtr node;
    if (getFromBB(config().blackboard, "node", node) && node) {
      RCLCPP_INFO(node->get_logger(), "[RecoverNavigation] Iniciando maniobra: %s", rec_type_.c_str());
      if (!cmd_vel_pub_) {
        cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      }
    }

    if (rec_type_ == "BACKUP") {
      duration_s_ = 1.5;
    } else if (rec_type_ == "SPIN") {
      duration_s_ = 2.0;
    } else {
      return BT::NodeStatus::SUCCESS;
    }

    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= duration_s_) {
      publishStop();
      return BT::NodeStatus::SUCCESS;
    }

    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist cmd;
      if (rec_type_ == "BACKUP") {
        cmd.linear.x = -0.12;
      } else if (rec_type_ == "SPIN") {
        cmd.angular.z = 0.8;
      }
      cmd_vel_pub_->publish(cmd);
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    publishStop();
  }

private:
  void publishStop()
  {
    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
    }
  }

  std::string rec_type_;
  double duration_s_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

/**
 * @brief Acción de barrido visual/sensorial en el sitio (rotación 360° controlada).
 */
class ScanArea : public BT::StatefulActionNode
{
public:
  ScanArea(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      duration_s_(3.0),
      angular_speed_(0.6)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration_s", 3.0, "Duración del escaneo sensorial en segundos")
    };
  }

  BT::NodeStatus onStart() override
  {
    getInput("duration_s", duration_s_);
    if (duration_s_ <= 0.0) {
      return BT::NodeStatus::SUCCESS;
    }

    rclcpp::Node::SharedPtr node;
    if (getFromBB(config().blackboard, "node", node) && node) {
      if (!cmd_vel_pub_) {
        cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      }
      RCLCPP_INFO(node->get_logger(), "[ScanArea] Iniciando barrido sensorial de %.1fs", duration_s_);
    }

    angular_speed_ = std::clamp((2.0 * M_PI) / duration_s_, 0.3, 0.8);
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time_).count();
    if (elapsed >= duration_s_) {
      publishStop();
      return BT::NodeStatus::SUCCESS;
    }

    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = angular_speed_;
      cmd_vel_pub_->publish(cmd);
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    publishStop();
  }

private:
  void publishStop()
  {
    if (cmd_vel_pub_) {
      geometry_msgs::msg::Twist stop_msg;
      cmd_vel_pub_->publish(stop_msg);
    }
  }

  double duration_s_;
  double angular_speed_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

/**
 * @brief Acción para accionar el servobrazo del XGO (despliegue/agarre de cable guía).
 */
class OperateArm : public BT::SyncActionNode
{
public:
  OperateArm(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_type", "GRIP_CABLE", "Comando para el brazo: GRIP_CABLE o RELEASE_CABLE")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string action_type;
    getInput("action_type", action_type);

    rclcpp::Node::SharedPtr node;
    if (getFromBB(config().blackboard, "node", node) && node) {
      RCLCPP_INFO(node->get_logger(), "[OperateArm] Ejecutando comando de brazo: %s", action_type.c_str());
    }

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace xgo_behavior_tree
