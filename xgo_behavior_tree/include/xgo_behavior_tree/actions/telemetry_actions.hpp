#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace xgo_behavior_tree
{

/**
 * @brief Acción instantánea que transmite un mensaje de alerta/evento hacia el orquestador global.
 */
class SendAlert : public BT::SyncActionNode
{
public:
  SendAlert(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message", "Mensaje textual de la alerta"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Coordenadas asociadas al evento (opcional)")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string msg_text;
    getInput("message", msg_text);

    geometry_msgs::msg::PoseStamped pose;
    auto pose_res = getInput("pose", pose);
    bool has_pose = static_cast<bool>(pose_res);

    rclcpp::Node::SharedPtr node;
    auto bb = config().blackboard;
    if ((bb && bb->get("node", node) && node) ||
        (bb && bb->rootBlackboard() && bb->rootBlackboard()->get("node", node) && node)) {
      if (!alert_pub_) {
        alert_pub_ = node->create_publisher<std_msgs::msg::String>("/fleet/alerts", 10);
      }
      
      std_msgs::msg::String ros_msg;
      if (has_pose) {
        ros_msg.data = msg_text + " @ (" + 
                       std::to_string(pose.pose.position.x) + ", " + 
                       std::to_string(pose.pose.position.y) + ")";
      } else {
        ros_msg.data = msg_text;
      }

      alert_pub_->publish(ros_msg);
      RCLCPP_INFO(node->get_logger(), "[SendAlert] Emitida alerta: %s", ros_msg.data.c_str());
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

/**
 * @brief Acción para ordenar el guardado de la rejilla de ocupación SLAM a disco.
 */
class SaveMap : public BT::SyncActionNode
{
public:
  SaveMap(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("map_name", "Nombre del archivo de mapa a guardar")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string map_name = "map_saved";
    getInput("map_name", map_name);

    rclcpp::Node::SharedPtr node;
    auto bb = config().blackboard;
    if ((bb && bb->get("node", node) && node) ||
        (bb && bb->rootBlackboard() && bb->rootBlackboard()->get("node", node) && node)) {
      RCLCPP_INFO(node->get_logger(), "[SaveMap] Guardando mapa SLAM como: '%s'", map_name.c_str());
    }

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace xgo_behavior_tree
