#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

namespace xgo_behavior_tree
{

/**
 * @brief Acción instantánea que ordena parada física inmediata publicando cmd_vel = 0.
 */
class StopRobot : public BT::SyncActionNode
{
public:
  StopRobot(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    // Obtenemos el puntero al nodo ROS 2 desde el Blackboard
    rclcpp::Node::SharedPtr node;
    auto bb = config().blackboard;
    if ((bb && bb->get("node", node) && node) ||
        (bb && bb->rootBlackboard() && bb->rootBlackboard()->get("node", node) && node)) {
      if (!cmd_vel_pub_) {
        cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      }
      
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.linear.z = 0.0;
      stop_cmd.angular.x = 0.0;
      stop_cmd.angular.y = 0.0;
      stop_cmd.angular.z = 0.0;

      cmd_vel_pub_->publish(stop_cmd);
      RCLCPP_INFO_ONCE(node->get_logger(), "[StopRobot] Consigna de velocidad nula enviada.");
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

/**
 * @brief Acción asíncrona de espera temporizada sin bloquear el hilo de ejecución.
 * Devuelve RUNNING mientras el tiempo transcurre, y SUCCESS al completarse.
 */
class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config),
      duration_s_(1.0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration_s", 1.0, "Tiempo de espera en segundos")
    };
  }

  BT::NodeStatus onStart() override
  {
    getInput("duration_s", duration_s_);
    if (duration_s_ <= 0.0) {
      return BT::NodeStatus::SUCCESS;
    }

    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(now - start_time_).count();

    if (elapsed_s >= duration_s_) {
      return BT::NodeStatus::SUCCESS; // Espera finalizada
    }

    return BT::NodeStatus::RUNNING; // Todavía esperando
  }

  void onHalted() override
  {
    // Si una rama prioritaria cancela el Wait, salimos limpiamente
  }

private:
  double duration_s_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

} // namespace xgo_behavior_tree
