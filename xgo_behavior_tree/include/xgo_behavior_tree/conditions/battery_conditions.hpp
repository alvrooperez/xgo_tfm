#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace xgo_behavior_tree
{

/**
 * @brief Condición que evalúa si la batería del robot ha caído por debajo del umbral crítico.
 * 
 * Lee el nivel actual de batería desde el Blackboard y lo compara contra el puerto de entrada "threshold".
 */
class IsBatteryLow : public BT::ConditionNode
{
public:
  IsBatteryLow(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("threshold", 15.0, "Umbral de porcentaje de batería para considerar nivel crítico")
    };
  }

  BT::NodeStatus tick() override
  {
    double threshold = 15.0;
    getInput("threshold", threshold);

    // Leemos el nivel actual de batería del Blackboard
    double battery_level = 100.0;
    if (!config().blackboard->get("battery_level", battery_level)) {
      // Si aún no se ha recibido telemetría de batería, asumimos que no hay emergencia
      return BT::NodeStatus::FAILURE;
    }

    if (battery_level <= threshold) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

} // namespace xgo_behavior_tree
