/**
 * Minimal ranger-facing Qt dashboard.
 *
 * COMPLETE BASELINE: displays mission/environment/alert state and provides six
 * service-backed controls. Qt and ROS are processed on the same thread via QTimer,
 * which keeps the starter code understandable.
 *
 * TODO(student): add a map or embedded RViz view, stale-data indicators, alert
 * acknowledgement, event history and user testing with a non-robotics participant.
 */

#include <memory>
#include <string>
#include <utility>

#include <QApplication>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/perception_status.hpp"
#include "embertrace_interfaces/msg/fire_alert.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "embertrace_interfaces/srv/operator_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace embertrace_ui
{

class DashboardNode : public rclcpp::Node
{
public:
  DashboardNode()
  : Node("operator_dashboard")
  {
    state_subscription_ = create_subscription<embertrace_interfaces::msg::MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::MissionState::SharedPtr message) {
        state_name_ = message->state_name;
        detail_ = message->detail;
        progress_ = message->progress;
        connected_ = true;
        state_stamp_ = rclcpp::Time(message->header.stamp);
      });
    reading_subscription_ = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
      "/embertrace/environment", rclcpp::SensorDataQoS(),
      [this](const embertrace_interfaces::msg::EnvironmentalReading::SharedPtr message) {
        smoke_ppm_ = message->smoke_ppm;
        temperature_c_ = message->temperature_c;
        wind_speed_mps_ = message->wind_speed_mps;
        confidence_ = message->fused_confidence;
      });
    alert_subscription_ = create_subscription<embertrace_interfaces::msg::FireAlert>(
      "/embertrace/fire_alert", rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::FireAlert::SharedPtr message) {
        alert_text_ = message->confirmed ? "CONFIRMED FIRE" : "Suspected fire";
        alert_text_ += " — " + message->recommended_action;
      });
    rover_status_ = create_subscription<embertrace_interfaces::msg::PerceptionStatus>(
      "/embertrace/perception_status", 10,
      [this](const embertrace_interfaces::msg::PerceptionStatus::SharedPtr msg) {
        rover_fresh_ = msg->environment_fresh;
        rover_stamp_ = rclcpp::Time(msg->header.stamp);
      });
    drone_status_ = create_subscription<embertrace_interfaces::msg::PerceptionStatus>(
      "/drone1/perception_status", 10,
      [this](const embertrace_interfaces::msg::PerceptionStatus::SharedPtr msg) {
        drone_fresh_ = msg->environment_fresh;
        drone_stamp_ = rclcpp::Time(msg->header.stamp);
      });
    command_client_ = create_client<embertrace_interfaces::srv::OperatorCommand>(
      "/embertrace/operator_command");
  }

  void send_command(const std::string & command)
  {
    if (!command_client_->service_is_ready()) {
      command_result_ = "Command service is not ready";
      return;
    }
    auto request = std::make_shared<embertrace_interfaces::srv::OperatorCommand::Request>();
    request->command = command;
    command_result_ = "Sending " + command + "…";
    command_client_->async_send_request(
      request,
      [this](rclcpp::Client<embertrace_interfaces::srv::OperatorCommand>::SharedFuture future) {
        const auto response = future.get();
        command_result_ = response->accepted ? response->message : "Rejected: " + response->message;
      });
  }

  bool fresh(const rclcpp::Time & stamp) const
  {
    const double age = (now() - stamp).seconds();
    return stamp.nanoseconds() > 0 && age >= 0 && age < 2.0;
  }
  bool connected() const {return connected_ && fresh(state_stamp_);}
  bool rover_fresh() const {return rover_fresh_ && fresh(rover_stamp_);}
  std::string sensor_health() const
  {
    return std::string("Rover perception: ") + (rover_fresh() ? "fresh" : "waiting / stale") +
      " | Drone perception: " + (drone_fresh_ && fresh(drone_stamp_) ? "fresh" : "disabled / waiting / stale");
  }
  const std::string & state_name() const {return state_name_;}
  const std::string & detail() const {return detail_;}
  const std::string & alert_text() const {return alert_text_;}
  const std::string & command_result() const {return command_result_;}
  float progress() const {return progress_;}
  float smoke_ppm() const {return smoke_ppm_;}
  float temperature_c() const {return temperature_c_;}
  float wind_speed_mps() const {return wind_speed_mps_;}
  float confidence() const {return confidence_;}

private:
  bool connected_{false}, rover_fresh_{false}, drone_fresh_{false};
  rclcpp::Time state_stamp_{0, 0, RCL_ROS_TIME}, rover_stamp_{0, 0, RCL_ROS_TIME}, drone_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<embertrace_interfaces::msg::PerceptionStatus>::SharedPtr rover_status_, drone_status_;
  std::string state_name_{"WAITING"};
  std::string detail_{"Waiting for mission manager"};
  std::string alert_text_{"No active alert"};
  std::string command_result_{"Ready"};
  float progress_{0.0F};
  float smoke_ppm_{0.0F};
  float temperature_c_{0.0F};
  float wind_speed_mps_{0.0F};
  float confidence_{0.0F};

  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr
    reading_subscription_;
  rclcpp::Subscription<embertrace_interfaces::msg::FireAlert>::SharedPtr alert_subscription_;
  rclcpp::Client<embertrace_interfaces::srv::OperatorCommand>::SharedPtr command_client_;
};

class DashboardWindow : public QMainWindow
{
public:
  explicit DashboardWindow(std::shared_ptr<DashboardNode> node)
  : node_(std::move(node))
  {
    setWindowTitle("EmberTrace Ranger Console");
    resize(620, 520);

    auto * central = new QWidget(this);
    auto * root = new QVBoxLayout(central);

    title_ = new QLabel("EMBERTRACE  •  DISCONNECTED");
    title_->setStyleSheet("font-size: 20px; font-weight: 700; color: #d97706;");
    root->addWidget(title_);

    auto * status_box = new QGroupBox("Mission status");
    auto * status_layout = new QVBoxLayout(status_box);
    state_ = new QLabel("WAITING");
    state_->setStyleSheet("font-size: 24px; font-weight: 700;");
    detail_ = new QLabel("Waiting for mission manager");
    detail_->setWordWrap(true);
    progress_ = new QProgressBar();
    progress_->setRange(0, 100);
    status_layout->addWidget(state_);
    status_layout->addWidget(detail_);
    status_layout->addWidget(progress_);
    root->addWidget(status_box);

    auto * sensor_box = new QGroupBox("Environmental perception");
    auto * sensors = new QGridLayout(sensor_box);
    smoke_ = new QLabel("0.0 ppm");
    temperature_ = new QLabel("0.0 °C");
    wind_ = new QLabel("0.0 m/s");
    confidence_ = new QLabel("0 %");
    sensors->addWidget(new QLabel("Smoke"), 0, 0); sensors->addWidget(smoke_, 0, 1);
    sensors->addWidget(new QLabel("Temperature"), 1, 0); sensors->addWidget(temperature_, 1, 1);
    sensors->addWidget(new QLabel("Wind"), 0, 2); sensors->addWidget(wind_, 0, 3);
    sensors->addWidget(new QLabel("Fused confidence"), 1, 2); sensors->addWidget(confidence_, 1, 3);
    health_ = new QLabel("Waiting for sensor health");
    health_->setWordWrap(true);
    sensors->addWidget(health_, 2, 0, 1, 4);
    root->addWidget(sensor_box);

    alert_ = new QLabel("No active alert");
    alert_->setWordWrap(true);
    alert_->setStyleSheet("padding: 10px; background: #26332a; color: #d1fae5;");
    root->addWidget(alert_);

    auto * controls = new QGroupBox("Mission controls");
    auto * buttons = new QGridLayout(controls);
    add_button(buttons, "Start", "START", 0, 0);
    add_button(buttons, "Pause", "PAUSE", 0, 1);
    add_button(buttons, "Resume", "RESUME", 0, 2);
    add_button(buttons, "Return", "RETURN", 1, 0);
    add_button(buttons, "Reset", "RESET", 1, 1);
    auto * stop = add_button(buttons, "EMERGENCY STOP", "E_STOP", 1, 2);
    stop->setStyleSheet("background: #b91c1c; color: white; font-weight: 700; padding: 8px;");
    root->addWidget(controls);

    command_result_ = new QLabel("Ready");
    root->addWidget(command_result_);
    setCentralWidget(central);

    timer_ = new QTimer(this);
    QObject::connect(timer_, &QTimer::timeout, this, [this]() {
      rclcpp::spin_some(node_);
      refresh();
    });
    timer_->start(50);
  }

private:
  QPushButton * add_button(
    QGridLayout * layout, const QString & label, const std::string & command,
    const int row, const int column)
  {
    auto * button = new QPushButton(label);
    QObject::connect(button, &QPushButton::clicked, this, [this, command]() {
      node_->send_command(command);
    });
    layout->addWidget(button, row, column);
    return button;
  }

  void refresh()
  {
    title_->setText(node_->connected() ? "EMBERTRACE  •  CONNECTED" : "EMBERTRACE  •  DISCONNECTED");
    state_->setText(QString::fromStdString(node_->state_name()));
    detail_->setText(QString::fromStdString(node_->detail()));
    progress_->setValue(static_cast<int>(100.0F * node_->progress()));
    smoke_->setText(QString::number(node_->smoke_ppm(), 'f', 1) + " ppm");
    temperature_->setText(QString::number(node_->temperature_c(), 'f', 1) + " °C");
    wind_->setText(QString::number(node_->wind_speed_mps(), 'f', 1) + " m/s");
    confidence_->setText(QString::number(100.0F * node_->confidence(), 'f', 0) + " %");
    health_->setText(QString::fromStdString(node_->sensor_health()));
    if (!node_->rover_fresh()) {
      smoke_->setText("Unavailable"); temperature_->setText("Unavailable");
      wind_->setText("Unavailable"); confidence_->setText("Unavailable");
    }
    alert_->setText(QString::fromStdString(node_->alert_text()));
    if (node_->alert_text().find("CONFIRMED") != std::string::npos) {
      alert_->setStyleSheet("padding: 10px; background: #7f1d1d; color: white; font-weight: 700;");
    }
    command_result_->setText(QString::fromStdString(node_->command_result()));
  }

  std::shared_ptr<DashboardNode> node_;
  QLabel * title_{nullptr};
  QLabel * state_{nullptr};
  QLabel * detail_{nullptr};
  QLabel * smoke_{nullptr};
  QLabel * temperature_{nullptr};
  QLabel * wind_{nullptr};
  QLabel * confidence_{nullptr};
  QLabel * health_{nullptr};
  QLabel * alert_{nullptr};
  QLabel * command_result_{nullptr};
  QProgressBar * progress_{nullptr};
  QTimer * timer_{nullptr};
};

}  // namespace embertrace_ui

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication application(argc, argv);
  auto node = std::make_shared<embertrace_ui::DashboardNode>();
  embertrace_ui::DashboardWindow window(node);
  window.show();
  const int result = application.exec();
  rclcpp::shutdown();
  return result;
}

