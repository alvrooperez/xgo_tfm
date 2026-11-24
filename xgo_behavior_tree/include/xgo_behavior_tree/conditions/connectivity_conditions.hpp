#pragma once

#include <behaviortree_cpp/condition_node.h>
#include <chrono>

namespace xgo_behavior_tree
{

/**
 * @brief Condición que comprueba si el enlace de comunicaciones Wi-Fi sigue activo.
 * 
 * Evalúa el tiempo transcurrido desde el último latido (heartbeat) recibido de la estación base.
 */
class HasWifiConnection : public BT::ConditionNode
{
public:
  HasWifiConnection(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_s", 5.0, "Tiempo máximo en segundos sin heartbeat para declarar pérdida de señal")
    };
  }

  BT::NodeStatus tick() override
  {
    double timeout_s = 5.0;
    getInput("timeout_s", timeout_s);

    double last_heartbeat_time_s = 0.0;
    double current_time_s = 0.0;

    // Obtenemos del Blackboard los tiempos registrados por el suscriptor de ROS 2
    if (!config().blackboard->get("last_heartbeat_time_s", last_heartbeat_time_s) ||
        !config().blackboard->get("current_time_s", current_time_s))
    {
      // Si aún no hemos recibido el primer mensaje, por seguridad asumimos que hay enlace
      return BT::NodeStatus::SUCCESS;
    }

    double elapsed_s = current_time_s - last_heartbeat_time_s;

    if (elapsed_s <= timeout_s) {
      return BT::NodeStatus::SUCCESS; // Conexión activa y saludable
    }

    return BT::NodeStatus::FAILURE;   // Timeout superado: enlace caído
  }
};

} // namespace xgo_behavior_tree
