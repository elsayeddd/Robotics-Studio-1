/** Operator console: selected-platform telemetry, map, alerts and bounded controls. */
#include <QApplication>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QMessageBox>
#include <QPainter>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "embertrace_interfaces/msg/environmental_reading.hpp"
#include "embertrace_interfaces/msg/fire_alert.hpp"
#include "embertrace_interfaces/msg/mission_state.hpp"
#include "embertrace_interfaces/msg/perception_status.hpp"
#include "embertrace_interfaces/srv/operator_command.hpp"
#include "embertrace_interfaces/srv/set_fire_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
namespace embertrace_ui {
class DashboardNode : public rclcpp::Node {
 public:
  using State = embertrace_interfaces::msg::MissionState;
  struct Vehicle {
    embertrace_interfaces::msg::EnvironmentalReading reading;
    geometry_msgs::msg::PoseStamped pose;
    double receipt{-1}, status_receipt{-1}, battery{-1}, battery_receipt{-1}, pose_receipt{-1};
    bool environment{}, co2{}, thermal{};
  };
  DashboardNode() : Node("operator_dashboard") {
    buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    const auto transient = rclcpp::QoS(1).transient_local();
    state_sub = create_subscription<State>("/embertrace/mission_state", transient,
                                           [this](State::SharedPtr m) {
                                             state = *m;
                                             state_receipt = wall();
                                           });
    alert_sub = create_subscription<embertrace_interfaces::msg::FireAlert>(
        "/embertrace/fire_alert", transient,
        [this](embertrace_interfaces::msg::FireAlert::SharedPtr m) {
          alert = *m;
          have_alert = true;
        });
    map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/husky1/map", transient, [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {
          const std::size_t cells = static_cast<std::size_t>(m->info.width) * m->info.height;
          const auto& p = m->info.origin.position;
          const auto& q = m->info.origin.orientation;
          if (!m->header.frame_id.empty() && m->info.width > 0 && m->info.height > 0 &&
              m->info.width <= 3000 && m->info.height <= 3000 && m->data.size() == cells &&
              std::isfinite(m->info.resolution) && m->info.resolution > 0 && std::isfinite(p.x) &&
              std::isfinite(p.y) && std::isfinite(q.x) && std::isfinite(q.y) &&
              std::isfinite(q.z) && std::isfinite(q.w) &&
              std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) < .1) {
            map = m;
            ++map_revision;
          }
        });
    events_sub = create_subscription<std_msgs::msg::String>(
        "/embertrace/mission_event", 50, [this](std_msgs::msg::String::SharedPtr m) {
          events.push_back(m->data);
          if (events.size() > 200) events.pop_front();
          ++event_revision;
        });
    for (std::size_t i = 0; i < 2; ++i) {
      const std::string root = i == 0 ? "/embertrace" : "/drone1",
                        robot = i == 0 ? "/husky1" : "/drone1";
      reading_sub[i] = create_subscription<embertrace_interfaces::msg::EnvironmentalReading>(
          root + "/environment", rclcpp::SensorDataQoS(),
          [this, i](embertrace_interfaces::msg::EnvironmentalReading::SharedPtr m) {
            vehicles[i].reading = *m;
            vehicles[i].receipt = wall();
          });
      status_sub[i] = create_subscription<embertrace_interfaces::msg::PerceptionStatus>(
          root + "/perception_status", 10,
          [this, i](embertrace_interfaces::msg::PerceptionStatus::SharedPtr m) {
            vehicles[i].environment = m->environment_fresh;
            vehicles[i].co2 = m->co2_valid;
            vehicles[i].thermal = m->thermal_fresh;
            vehicles[i].status_receipt = wall();
          });
      pose_sub[i] = create_subscription<nav_msgs::msg::Odometry>(
          robot + "/odometry", rclcpp::SensorDataQoS(),
          [this, i](nav_msgs::msg::Odometry::SharedPtr m) {
            const auto& p = m->pose.pose.position;
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
            vehicles[i].pose.header = m->header;
            vehicles[i].pose.pose = m->pose.pose;
            vehicles[i].pose_receipt = wall();
          });
      battery_sub[i] = create_subscription<sensor_msgs::msg::BatteryState>(
          robot + "/battery", 10, [this, i](sensor_msgs::msg::BatteryState::SharedPtr m) {
            if (std::isfinite(m->percentage) && m->percentage >= 0 && m->percentage <= 1) {
              vehicles[i].battery = m->percentage;
              vehicles[i].battery_receipt = wall();
            }
          });
    }
    operator_client =
        create_client<embertrace_interfaces::srv::OperatorCommand>("/embertrace/operator_command");
    fire_client =
        create_client<embertrace_interfaces::srv::SetFireState>("/embertrace/set_fire_state");
    parameters = std::make_shared<rclcpp::AsyncParametersClient>(this, "/mission_simulator");
    manual_pub = create_publisher<geometry_msgs::msg::Twist>("/embertrace/manual_cmd_vel", 10);
  }
  static double wall() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  bool connected() const {
    return wall() - state_receipt < 1.5;
  }
  bool fresh(std::size_t i) const {
    const auto& v = vehicles[i];
    const double stamp = v.reading.header.stamp.sec + v.reading.header.stamp.nanosec * 1e-9;
    const double age = now().seconds() - stamp;
    return v.environment && age >= -.1 && age <= 2 && wall() - v.receipt <= 2 &&
           wall() - v.status_receipt <= 2;
  }
  bool pose_fresh(std::size_t i) const {
    const auto& v = vehicles[i];
    const double age =
        now().seconds() - v.pose.header.stamp.sec - v.pose.header.stamp.nanosec * 1e-9;
    return v.pose_receipt >= 0 && wall() - v.pose_receipt <= 2 && age >= -.1 && age <= 2;
  }
  void command(const std::string& text) {
    if (!operator_client->service_is_ready()) {
      response = "Mission service unavailable";
      return;
    }
    auto r = std::make_shared<embertrace_interfaces::srv::OperatorCommand::Request>();
    r->command = text;
    operator_client->async_send_request(
        r, [this](rclcpp::Client<embertrace_interfaces::srv::OperatorCommand>::SharedFuture f) {
          const auto r = f.get();
          response = r->accepted ? r->message : "Rejected: " + r->message;
        });
  }
  void tree(const std::string& text) {
    if (!fire_client->service_is_ready()) {
      response = "Tree-state service unavailable";
      return;
    }
    auto r = std::make_shared<embertrace_interfaces::srv::SetFireState::Request>();
    r->state = text;
    fire_client->async_send_request(
        r, [this](rclcpp::Client<embertrace_interfaces::srv::SetFireState>::SharedFuture f) {
          response = f.get()->message;
        });
  }
  void wind(double speed) {
    if (!parameters->service_is_ready()) {
      response = "Wind simulator unavailable";
      return;
    }
    parameters->set_parameters(
        {rclcpp::Parameter("wind_speed_mps", speed)},
        [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
          const auto result = f.get();
          response = !result.empty() && result[0].successful ? "Wind setting updated"
                                                             : "Wind setting rejected";
        });
  }
  std::optional<geometry_msgs::msg::PoseStamped> in_map(geometry_msgs::msg::PoseStamped pose) {
    if (!map || pose.header.frame_id.empty()) return std::nullopt;
    pose.header.stamp.sec = 0;
    pose.header.stamp.nanosec = 0;
    try {
      return buffer->transform(pose, map->header.frame_id);
    } catch (const tf2::TransformException&) {
      return std::nullopt;
    }
  }
  State state;
  embertrace_interfaces::msg::FireAlert alert;
  bool have_alert{false};
  double state_receipt{-10};
  std::string response{"Ready"};
  std::array<Vehicle, 2> vehicles;
  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  std::size_t map_revision{0}, event_revision{0};
  std::deque<std::string> events;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_pub;

 private:
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  rclcpp::Subscription<State>::SharedPtr state_sub;
  rclcpp::Subscription<embertrace_interfaces::msg::FireAlert>::SharedPtr alert_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_sub;
  std::array<rclcpp::Subscription<embertrace_interfaces::msg::EnvironmentalReading>::SharedPtr, 2>
      reading_sub;
  std::array<rclcpp::Subscription<embertrace_interfaces::msg::PerceptionStatus>::SharedPtr, 2>
      status_sub;
  std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, 2> pose_sub;
  std::array<rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr, 2> battery_sub;
  rclcpp::Client<embertrace_interfaces::srv::OperatorCommand>::SharedPtr operator_client;
  rclcpp::Client<embertrace_interfaces::srv::SetFireState>::SharedPtr fire_client;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters;
};

/** Occupancy map and telemetry are projected into the map's actual origin and yaw. */
class MapWidget : public QWidget {
 public:
  explicit MapWidget(std::shared_ptr<DashboardNode> node) : node_(std::move(node)) {
    setMinimumSize(340, 300);
  }

 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter painter(this);
    painter.fillRect(rect(), QColor("#17212b"));
    if (!node_->map) {
      painter.setPen(Qt::white);
      painter.drawText(rect(), Qt::AlignCenter, "Waiting for SLAM map");
      return;
    }
    const auto& m = *node_->map;
    if (revision_ != node_->map_revision) {
      image_ = QImage(m.info.width, m.info.height, QImage::Format_RGB32);
      for (std::size_t y = 0; y < m.info.height; ++y)
        for (std::size_t x = 0; x < m.info.width; ++x) {
          const int value = m.data[y * m.info.width + x];
          const int shade = value < 0 ? 110 : 255 - std::clamp(value, 0, 100) * 255 / 100;
          image_.setPixel(static_cast<int>(x), static_cast<int>(m.info.height - 1 - y),
                          qRgb(shade, shade, shade));
        }
      revision_ = node_->map_revision;
    }
    QRect target = QRect(QPoint(0, 0), image_.size().scaled(size(), Qt::KeepAspectRatio));
    target.moveCenter(rect().center());
    painter.drawImage(target, image_);
    const auto& q = m.info.origin.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const auto marker = [&](const geometry_msgs::msg::PoseStamped& raw, const QColor& color,
                            const QString& label) {
      const auto pose = node_->in_map(raw);
      if (!pose) return;
      const double dx = pose->pose.position.x - m.info.origin.position.x,
                   dy = pose->pose.position.y - m.info.origin.position.y;
      const double x = (std::cos(yaw) * dx + std::sin(yaw) * dy) / m.info.resolution;
      const double y = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / m.info.resolution;
      QPointF point(target.left() + x / m.info.width * target.width(),
                    target.bottom() - y / m.info.height * target.height());
      painter.setPen(color);
      painter.setBrush(color);
      painter.drawEllipse(point, 5, 5);
      painter.drawText(point + QPointF(8, -8), label);
    };
    if (node_->pose_fresh(0)) marker(node_->vehicles[0].pose, QColor("#38bdf8"), "Rover");
    if (node_->pose_fresh(1)) marker(node_->vehicles[1].pose, QColor("#a78bfa"), "Drone");
    if (node_->have_alert)
      marker(node_->alert.estimated_source, QColor("#f97316"),
             node_->alert.source_resolved ? "Source estimate" : "Detection sample");
  }

 private:
  std::shared_ptr<DashboardNode> node_;
  QImage image_;
  std::size_t revision_{static_cast<std::size_t>(-1)};
};

class DashboardWindow : public QMainWindow {
 public:
  explicit DashboardWindow(std::shared_ptr<DashboardNode> node) : node_(std::move(node)) {
    setWindowTitle("EmberTrace Ranger Console");
    resize(1080, 820);
    auto* central = new QWidget(this);
    auto* root = new QVBoxLayout(central);
    title_ = new QLabel("Waiting for mission manager");
    title_->setStyleSheet("font-size:20px;font-weight:700;");
    root->addWidget(title_);
    detail_ = new QLabel;
    detail_->setWordWrap(true);
    root->addWidget(detail_);
    auto* panels = new QHBoxLayout;
    map_ = new MapWidget(node_);
    panels->addWidget(map_, 1);
    auto* telemetry = new QGroupBox("Vehicle telemetry");
    auto* grid = new QGridLayout(telemetry);
    for (std::size_t i = 0; i < 2; ++i) {
      telemetry_[i] = new QLabel;
      telemetry_[i]->setWordWrap(true);
      grid->addWidget(telemetry_[i], static_cast<int>(i), 0);
    }
    panels->addWidget(telemetry, 1);
    root->addLayout(panels);
    alert_ = new QLabel("No fire alert");
    alert_->setWordWrap(true);
    root->addWidget(alert_);
    auto* controls = new QGridLayout;
    for (const auto& item :
         std::array<std::pair<const char*, const char*>, 8>{{{"Start", "START"},
                                                             {"Pause", "PAUSE"},
                                                             {"Resume", "RESUME"},
                                                             {"Return", "RETURN"},
                                                             {"Reset", "RESET"},
                                                             {"EMERGENCY STOP", "E_STOP"},
                                                             {"Acknowledge alert", "ACK"},
                                                             {"Manual control", "MANUAL_ON"}}}) {
      auto* button = new QPushButton(item.first);
      const std::string command = item.second;
      if (command == "E_STOP")
        button->setStyleSheet("background:#b91c1c;color:white;font-weight:bold;");
      connect(button, &QPushButton::clicked, this, [this, command]() {
        if ((command == "RESET" || command == "RETURN") &&
            QMessageBox::question(this, "Confirm command",
                                  command == "RESET"
                                      ? "Stop and reset the mission?"
                                      : "Return the selected vehicle to its launch position?") !=
                QMessageBox::Yes)
          return;
        node_->command(command == "MANUAL_ON" && node_->state.manual_control ? "MANUAL_OFF"
                                                                             : command);
      });
      controls->addWidget(button, button_index_ / 4, button_index_ % 4);
      ++button_index_;
    }
    root->addLayout(controls);
    auto* manual = new QHBoxLayout;
    add_manual(manual, "Forward", 1, 0, 0);
    add_manual(manual, "Back", -1, 0, 0);
    add_manual(manual, "Left", 0, 1, 0);
    add_manual(manual, "Right", 0, -1, 0);
    add_manual(manual, "Up", 0, 0, 1);
    add_manual(manual, "Down", 0, 0, -1);
    root->addLayout(manual);
    auto* simulation = new QHBoxLayout;
    for (const std::string state : {"healthy", "burning", "burnt"}) {
      auto* button = new QPushButton(QString("Tree: ") + QString::fromStdString(state));
      connect(button, &QPushButton::clicked, this, [this, state]() { node_->tree(state); });
      simulation->addWidget(button);
    }
    auto* wind = new QDoubleSpinBox;
    wind->setRange(0, 30);
    wind->setSingleStep(.5);
    wind->setValue(2.5);
    wind->setSuffix(" m/s");
    simulation->addWidget(wind);
    auto* apply = new QPushButton("Set wind");
    connect(apply, &QPushButton::clicked, this, [this, wind]() { node_->wind(wind->value()); });
    simulation->addWidget(apply);
    root->addLayout(simulation);
    response_ = new QLabel("Ready");
    root->addWidget(response_);
    events_ = new QListWidget;
    events_->setMaximumHeight(125);
    root->addWidget(events_);
    setCentralWidget(central);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      if (!rclcpp::ok()) {
        QApplication::quit();
        return;
      }
      rclcpp::spin_some(node_);
      refresh();
    });
    timer_->start(50);
    connect(qApp, &QGuiApplication::applicationStateChanged, this,
            [this](Qt::ApplicationState state) {
              if (state != Qt::ApplicationActive) {
                manual_x_ = manual_y_ = manual_z_ = 0;
                publish_manual();
              }
            });
  }

 private:
  void add_manual(QHBoxLayout* layout, const char* label, double x, double y, double z) {
    auto* button = new QPushButton(label);
    layout->addWidget(button);
    connect(button, &QPushButton::pressed, this, [this, x, y, z]() {
      manual_x_ = x;
      manual_y_ = y;
      manual_z_ = z;
    });
    connect(button, &QPushButton::released, this, [this]() {
      manual_x_ = manual_y_ = manual_z_ = 0;
      publish_manual();
    });
  }
  void publish_manual() {
    geometry_msgs::msg::Twist command;
    if (node_->connected() && node_->state.manual_control) {
      command.linear.x = manual_x_ * .6;
      if (node_->state.active_platform == DashboardNode::State::PLATFORM_DRONE) {
        command.linear.y = manual_y_ * .6;
        command.linear.z = manual_z_ * .4;
      } else
        command.angular.z = manual_y_ * .7;
    }
    node_->manual_pub->publish(command);
  }
  void refresh() {
    const auto& s = node_->state;
    title_->setText(QString("EMBERTRACE | ") + (node_->connected() ? "CONNECTED" : "DISCONNECTED") +
                    " | " + QString::fromStdString(s.state_name));
    detail_->setText(QString::fromStdString(s.detail) + QString(" | Selected: ") +
                     (s.active_platform == 1   ? "rover"
                      : s.active_platform == 2 ? "drone"
                                               : "none"));
    for (std::size_t i = 0; i < 2; ++i) {
      const auto& v = node_->vehicles[i];
      const auto& r = v.reading;
      const QString battery =
          v.battery_receipt >= 0 && DashboardNode::wall() - v.battery_receipt <= 2
              ? QString::number(v.battery * 100, 'f', 0) + "%"
              : "Unavailable";
      QString text = QString(i == 0 ? "ROVER" : "DRONE") + " | Battery " + battery + "\n";
      text += node_->pose_fresh(i) ? QString("Position: %1, %2, %3 m\n")
                                         .arg(v.pose.pose.position.x, 0, 'f', 1)
                                         .arg(v.pose.pose.position.y, 0, 'f', 1)
                                         .arg(v.pose.pose.position.z, 0, 'f', 1)
                                   : "Position unavailable / stale\n";
      if (node_->fresh(i))
        text +=
            QString(
                "Smoke: %1 ppm\nLocal temperature: %2 C\nCO2: %3\nIR peak: %4\nWind: %5 "
                "m/s\nConfidence: %6%")
                .arg(r.smoke_ppm, 0, 'f', 1)
                .arg(r.temperature_c, 0, 'f', 1)
                .arg(v.co2 ? QString::number(r.co2_ppm, 'f', 1) + " ppm" : "Unavailable")
                .arg(v.thermal ? QString::number(r.thermal_peak_c, 'f', 1) + " C" : "Unavailable")
                .arg(r.wind_speed_mps, 0, 'f', 1)
                .arg(r.fused_confidence * 100, 0, 'f', 0);
      else
        text += "Sensor data unavailable / stale";
      telemetry_[i]->setText(text);
    }
    if (node_->have_alert) {
      const auto& a = node_->alert;
      alert_->setText(QString(a.confirmed ? "Confirmed fire %1 | %2 | %3\n%4"
                                          : "Suspected anomaly %1 | %2 | %3\n%4")
                          .arg(static_cast<qulonglong>(a.alert_id))
                          .arg(s.alert_acknowledged ? "ACKNOWLEDGED" : "UNACKNOWLEDGED")
                          .arg(a.source_resolved ? "estimated source"
                                                 : "location unresolved; detection sample shown")
                          .arg(QString::fromStdString(a.evidence)));
    }
    alert_->setStyleSheet(node_->have_alert && !s.alert_acknowledged
                              ? "padding:8px;background:#7f1d1d;color:white;"
                              : "padding:8px;background:#e2e8f0;color:#17212b;");
    if (last_events_ != node_->event_revision) {
      events_->clear();
      for (const auto& e : node_->events) events_->addItem(QString::fromStdString(e));
      events_->scrollToBottom();
      last_events_ = node_->event_revision;
    }
    response_->setText(QString::fromStdString(node_->response));
    map_->update();
    publish_manual();
  }
  std::shared_ptr<DashboardNode> node_;
  MapWidget* map_;
  QLabel* title_;
  QLabel* detail_;
  QLabel* alert_;
  QLabel* response_;
  std::array<QLabel*, 2> telemetry_;
  QListWidget* events_;
  QTimer* timer_;
  int button_index_{0};
  std::size_t last_events_{0};
  double manual_x_{0}, manual_y_{0}, manual_z_{0};
};
}  // namespace embertrace_ui
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  auto node = std::make_shared<embertrace_ui::DashboardNode>();
  embertrace_ui::DashboardWindow window(node);
  window.show();
  int result = app.exec();
  rclcpp::shutdown();
  return result;
}
