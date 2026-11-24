#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <cmath>

namespace xgo_behavior_tree
{

/**
 * @brief Condición que evalúa si el operador humano ha activado la parada de emergencia (E-STOP).
 */
class IsEmergencyStopActive : public BT::ConditionNode
{
public:
  IsEmergencyStopActive(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    bool estop_active = false;
    if (config().blackboard->get("emergency_stop_active", estop_active) && estop_active) {
      return BT::NodeStatus::SUCCESS; // ¡Alarma! E-STOP pulsado
    }
    return BT::NodeStatus::FAILURE;   // Nominal, no hay parada de emergencia
  }
};

/**
 * @brief Condición que evalúa si el perro robot ha volcado o se ha caído mediante la IMU.
 */
class HasFallen : public BT::ConditionNode
{
public:
  HasFallen(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("tilt_threshold_deg", 60.0, "Ángulo límite de inclinación (roll/pitch) en grados")
    };
  }

  BT::NodeStatus tick() override
  {
    double threshold_deg = 60.0;
    getInput("tilt_threshold_deg", threshold_deg);

    double roll_deg = 0.0;
    double pitch_deg = 0.0;

    // Leemos la orientación estimada de la IMU calculada en grados
    bool has_roll = config().blackboard->get("robot_roll_deg", roll_deg);
    bool has_pitch = config().blackboard->get("robot_pitch_deg", pitch_deg);

    if (has_roll && has_pitch) {
      if (std::abs(roll_deg) > threshold_deg || std::abs(pitch_deg) > threshold_deg) {
        return BT::NodeStatus::SUCCESS; // ¡Robot volcado!
      }
    }

    return BT::NodeStatus::FAILURE;   // Robot de pie / estable
  }
};

} // namespace xgo_behavior_tree
