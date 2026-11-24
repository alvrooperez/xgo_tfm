#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace xgo_behavior_tree
{

/**
 * @brief Condición que comprueba si el robot tiene una misión activa asignada por el orquestador.
 */
class HasTaskAssigned : public BT::ConditionNode
{
public:
  HasTaskAssigned(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    bool has_task = false;
    auto bb = config().blackboard;
    if ((bb && bb->get("has_task", has_task) && has_task) ||
        (bb && bb->rootBlackboard() && bb->rootBlackboard()->get("has_task", has_task) && has_task)) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

/**
 * @brief Condición que comprueba si la tarea activa coincide con un tipo específico (M1 a M6).
 */
class IsTaskType : public BT::ConditionNode
{
public:
  IsTaskType(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("expected_type", "Tipo de tarea esperado (ej. GOTO_POINT, SWEEP_AREA, etc.)")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string expected_type;
    if (!getInput("expected_type", expected_type)) {
      return BT::NodeStatus::FAILURE;
    }

    std::string current_task_type;
    auto bb = config().blackboard;
    bool found = (bb && bb->get("task_type", current_task_type)) ||
                 (bb && bb->rootBlackboard() && bb->rootBlackboard()->get("task_type", current_task_type));
    if (found && current_task_type == expected_type) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

/**
 * @brief Condición que comprueba si el sistema de percepción ha detectado indicios de una víctima.
 * Si es positivo, exporta las coordenadas estimadas al puerto de salida.
 */
class IsTargetDetected : public BT::ConditionNode
{
public:
  IsTargetDetected(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "Coordenadas estimadas de la víctima detectada")
    };
  }

  BT::NodeStatus tick() override
  {
    bool detected = false;
    if (config().blackboard->get("target_detected", detected) && detected) {
      geometry_msgs::msg::PoseStamped victim_pose;
      if (config().blackboard->get("target_pose", victim_pose)) {
        setOutput("output_pose", victim_pose);
      }
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

} // namespace xgo_behavior_tree
