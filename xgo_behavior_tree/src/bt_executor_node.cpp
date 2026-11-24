#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

// Condiciones
#include "xgo_behavior_tree/conditions/battery_conditions.hpp"
#include "xgo_behavior_tree/conditions/safety_conditions.hpp"
#include "xgo_behavior_tree/conditions/connectivity_conditions.hpp"
#include "xgo_behavior_tree/conditions/task_conditions.hpp"

// Acciones
#include "xgo_behavior_tree/actions/motion_actions.hpp"
#include "xgo_behavior_tree/actions/telemetry_actions.hpp"
#include "xgo_behavior_tree/actions/navigation_actions.hpp"

#include <cmath>
#include <filesystem>
#include <memory>
#include <sstream>
#include <vector>

namespace xgo_behavior_tree
{

class BTExecutorNode : public rclcpp::Node
{
public:
  BTExecutorNode()
    : Node("xgo_behavior_tree_node")
  {
  }

  void init()
  {
    std::string ns = this->get_namespace();
    if (ns.empty() || ns == "/") {
      ns = "robot";
    } else if (ns.front() == '/') {
      ns = ns.substr(1);
    }

    RCLCPP_INFO(this->get_logger(), "Iniciando nodo ejecutor BT (C++20) para namespace: [%s]", ns.c_str());

    // Parámetros configurables por robot
    this->declare_parameter<std::string>("tree_xml_path", "");
    this->declare_parameter<int>("groot_port", 1667);
    this->declare_parameter<std::string>("log_file", "");
    this->declare_parameter<bool>("simulate_heartbeat", true);
    simulate_heartbeat_ = this->get_parameter("simulate_heartbeat").as_bool();

    // 1. Crear el Blackboard compartido
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
    blackboard_->set<double>("battery_level", 100.0);
    blackboard_->set<double>("battery_threshold", 15.0);
    blackboard_->set<bool>("emergency_stop_active", false);
    blackboard_->set<double>("robot_roll_deg", 0.0);
    blackboard_->set<double>("robot_pitch_deg", 0.0);
    blackboard_->set<bool>("has_task", false);
    blackboard_->set<std::string>("task_type", "IDLE");
    blackboard_->set<double>("last_heartbeat_time_s", this->now().seconds());
    blackboard_->set<double>("current_time_s", this->now().seconds());

    // Inicialización de poses y waypoints de navegación
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header.frame_id = "map";
    init_pose.header.stamp = this->now();
    init_pose.pose.orientation.w = 1.0;
    blackboard_->set<geometry_msgs::msg::PoseStamped>("current_pose", init_pose);
    blackboard_->set<geometry_msgs::msg::PoseStamped>("goal_pose", init_pose);
    blackboard_->set<geometry_msgs::msg::PoseStamped>("dock_pose", init_pose);
    blackboard_->set<geometry_msgs::msg::PoseStamped>("last_known_wifi_pose", init_pose);
    blackboard_->set<geometry_msgs::msg::PoseStamped>("target_pose", init_pose);
    blackboard_->set<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints_list", {});
    blackboard_->set<std::string>("map_name", "zona_industrial");

    // 2. Suscriptores con topics relativos para aislamiento multi-robot
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state", 10,
      [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        double pct = msg->percentage;
        if (pct <= 1.0) {
          pct *= 100.0;
        }
        blackboard_->set("battery_level", pct);
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll_deg = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

        double sinp = 2.0 * (qw * qy - qz * qx);
        double pitch_deg = 0.0;
        if (std::abs(sinp) >= 1.0) {
          pitch_deg = std::copysign(90.0, sinp);
        } else {
          pitch_deg = std::asin(sinp) * 180.0 / M_PI;
        }

        blackboard_->set("robot_roll_deg", roll_deg);
        blackboard_->set("robot_pitch_deg", pitch_deg);
      });

    estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        blackboard_->set("emergency_stop_active", msg->data);
      });

    heartbeat_sub_ = this->create_subscription<std_msgs::msg::Header>(
      "heartbeat", 10,
      [this](const std_msgs::msg::Header::SharedPtr /*msg*/) {
        blackboard_->set("last_heartbeat_time_s", this->now().seconds());
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped p;
        p.header = msg->header;
        p.pose = msg->pose.pose;
        blackboard_->set("current_pose", p);

        // Si la conexión Wi-Fi es saludable, actualizamos la miga de pan
        double last_hb = 0.0;
        if (blackboard_->get("last_heartbeat_time_s", last_hb)) {
          if (this->now().seconds() - last_hb < 5.0) {
            blackboard_->set("last_known_wifi_pose", p);
          }
        }
      });

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        blackboard_->set("goal_pose", *msg);
        blackboard_->set("task_type", std::string("GOTO_POINT"));
        blackboard_->set("has_task", true);
        RCLCPP_INFO(this->get_logger(), "[BTExecutor] Misión recibida: GOTO_POINT hacia (%.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y);
      });

    task_sub_ = this->create_subscription<std_msgs::msg::String>(
      "task_command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        RCLCPP_INFO(this->get_logger(), "[BTExecutor] Comando de misión recibido: '%s'", cmd.c_str());

        std::istringstream iss(cmd);
        std::string type;
        iss >> type;

        if (type == "CANCEL" || type == "STOP" || type == "IDLE") {
          blackboard_->set("has_task", false);
          blackboard_->set("task_type", std::string("IDLE"));
          RCLCPP_INFO(this->get_logger(), "[BTExecutor] Misión cancelada / robot en IDLE.");
        } else if (type == "GOTO_POINT") {
          double x = 0.0, y = 0.0;
          if (iss >> x >> y) {
            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = this->now();
            goal.pose.position.x = x;
            goal.pose.position.y = y;
            goal.pose.orientation.w = 1.0;
            blackboard_->set("goal_pose", goal);
          }
          blackboard_->set("task_type", std::string("GOTO_POINT"));
          blackboard_->set("has_task", true);
        } else if (type == "SWEEP_AREA") {
          std::vector<geometry_msgs::msg::PoseStamped> wps;
          double x, y;
          while (iss >> x >> y) {
            geometry_msgs::msg::PoseStamped wp;
            wp.header.frame_id = "map";
            wp.header.stamp = this->now();
            wp.pose.position.x = x;
            wp.pose.position.y = y;
            wp.pose.orientation.w = 1.0;
            wps.push_back(wp);
          }
          if (wps.empty()) {
            geometry_msgs::msg::PoseStamped curr;
            blackboard_->get("current_pose", curr);
            double cx = curr.pose.position.x;
            double cy = curr.pose.position.y;
            for (auto offset : {std::make_pair(0.8, 0.0), std::make_pair(0.8, 0.6), std::make_pair(0.0, 0.6), std::make_pair(0.0, 0.0)}) {
              geometry_msgs::msg::PoseStamped wp;
              wp.header.frame_id = "map";
              wp.header.stamp = this->now();
              wp.pose.position.x = cx + offset.first;
              wp.pose.position.y = cy + offset.second;
              wp.pose.orientation.w = 1.0;
              wps.push_back(wp);
            }
          }
          blackboard_->set("waypoints_list", wps);
          blackboard_->set("task_type", std::string("SWEEP_AREA"));
          blackboard_->set("has_task", true);
          RCLCPP_INFO(this->get_logger(), "[BTExecutor] Misión SWEEP_AREA configurada con %zu waypoints.", wps.size());
        } else if (type == "MAP_UNKNOWN") {
          double x = 1.0, y = 0.5;
          iss >> x >> y;
          geometry_msgs::msg::PoseStamped goal;
          goal.header.frame_id = "map";
          goal.header.stamp = this->now();
          goal.pose.position.x = x;
          goal.pose.position.y = y;
          goal.pose.orientation.w = 1.0;
          blackboard_->set("goal_pose", goal);
          blackboard_->set("task_type", std::string("MAP_UNKNOWN"));
          blackboard_->set("has_task", true);
        } else if (type == "RTB") {
          double x = 0.0, y = 0.0;
          geometry_msgs::msg::PoseStamped goal;
          goal.header.frame_id = "map";
          goal.header.stamp = this->now();
          if (iss >> x >> y) {
            goal.pose.position.x = x;
            goal.pose.position.y = y;
          } else {
            goal.pose.position.x = 0.0;
            goal.pose.position.y = 0.0;
          }
          goal.pose.orientation.w = 1.0;
          blackboard_->set("goal_pose", goal);
          blackboard_->set("dock_pose", goal);
          blackboard_->set("task_type", std::string("RTB"));
          blackboard_->set("has_task", true);
          RCLCPP_INFO(this->get_logger(), "[BTExecutor] Misión RTB iniciada hacia (%.2f, %.2f)",
                      goal.pose.position.x, goal.pose.position.y);
        }
      });

    // 3. Registrar todos los nodos en la factoría de BehaviorTree.CPP
    registerNodes();

    // 4. Localizar y cargar el XML del árbol
    std::string xml_path = this->get_parameter("tree_xml_path").as_string();
    if (xml_path.empty()) {
      xml_path = findTreeXml();
    }
    RCLCPP_INFO(this->get_logger(), "Cargando árbol desde: %s", xml_path.c_str());
    tree_ = factory_.createTreeFromFile(xml_path, blackboard_);

    // 5. Instanciar publicador para Groot2 (puerto configurable por robot)
    int groot_port = this->get_parameter("groot_port").as_int();
    try {
      groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot_port);
      RCLCPP_INFO(this->get_logger(), "Groot2Publisher activo en el puerto %d.", groot_port);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "No se pudo iniciar Groot2Publisher en puerto %d: %s", groot_port, e.what());
    }

    // 5.1 Logger en consola en tiempo real
    cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);

    // 5.2 Grabador de fichero para reproducción en Groot2
    std::string log_file = this->get_parameter("log_file").as_string();
    if (log_file.empty()) {
      auto log_dir = std::filesystem::temp_directory_path() / "xgo_bt_logs";
      std::filesystem::create_directories(log_dir);
      log_file = (log_dir / ("ejecucion_" + ns + ".btlog")).string();
    } else {
      auto parent = std::filesystem::path(log_file).parent_path();
      if (!parent.empty()) {
        std::filesystem::create_directories(parent);
      }
    }
    file_logger_ = std::make_unique<BT::FileLogger2>(tree_, log_file);
    RCLCPP_INFO(this->get_logger(), "FileLogger2 grabando en: %s", log_file.c_str());

    // 6. Temporizador para ejecutar el tick del árbol periódicamente a 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { tickTree(); });

    RCLCPP_INFO(this->get_logger(), "Behavior Tree listo para la flota (robot: %s).", ns.c_str());
  }

private:
  void registerNodes()
  {
    // Condiciones
    factory_.registerNodeType<IsBatteryLow>("IsBatteryLow");
    factory_.registerNodeType<IsEmergencyStopActive>("IsEmergencyStopActive");
    factory_.registerNodeType<HasFallen>("HasFallen");
    factory_.registerNodeType<HasWifiConnection>("HasWifiConnection");
    factory_.registerNodeType<HasTaskAssigned>("HasTaskAssigned");
    factory_.registerNodeType<IsTaskType>("IsTaskType");
    factory_.registerNodeType<IsTargetDetected>("IsTargetDetected");

    // Acciones
    factory_.registerNodeType<StopRobot>("StopRobot");
    factory_.registerNodeType<Wait>("Wait");
    factory_.registerNodeType<SendAlert>("SendAlert");
    factory_.registerNodeType<SaveMap>("SaveMap");
    factory_.registerNodeType<NavigateToPose>("NavigateToPose");
    factory_.registerNodeType<NavigateThroughPoses>("NavigateThroughPoses");
    factory_.registerNodeType<RecoverNavigation>("RecoverNavigation");
    factory_.registerNodeType<ScanArea>("ScanArea");
    factory_.registerNodeType<OperateArm>("OperateArm");
  }

  std::string findTreeXml()
  {
    std::string path_in_share = ament_index_cpp::get_package_share_directory("xgo_behavior_tree") +
                                "/config/bt_trees/rastreo_bt.xml";
    if (std::filesystem::exists(path_in_share)) {
      return path_in_share;
    }
    std::filesystem::path local_fallback = std::filesystem::current_path() / "config/bt_trees/rastreo_bt.xml";
    if (std::filesystem::exists(local_fallback)) {
      return local_fallback.string();
    }
    throw std::runtime_error("No se encontró rastreo_bt.xml ni en share ni en ./config/bt_trees/");
  }

  void tickTree()
  {
    // Actualizamos el tiempo actual en el Blackboard
    blackboard_->set("current_time_s", this->now().seconds());
    if (simulate_heartbeat_) {
      blackboard_->set("last_heartbeat_time_s", this->now().seconds());
    }

    // Latido del árbol (tick)
    tree_.tickOnce();
  }

  bool simulate_heartbeat_{true};
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
  std::unique_ptr<BT::FileLogger2> file_logger_;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace xgo_behavior_tree

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<xgo_behavior_tree::BTExecutorNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
