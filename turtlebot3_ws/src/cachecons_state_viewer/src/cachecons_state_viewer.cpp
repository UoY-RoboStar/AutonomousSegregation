#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>
#include "robot_selection.hpp"
#include <simulation_interfaces/msg/state_msg.hpp>
#include <rclcpp/rclcpp.hpp>

QString overviewDiagramPath(const QString &directory, const QString &filename)
{
  return directory + "/overview/hires/" + filename;
}

QPixmap loadOverviewDiagram(const QString &directory, const QString &filename)
{
  QPixmap diagram(overviewDiagramPath(directory, filename));
  diagram.setDevicePixelRatio(4.0);
  return diagram;
}

class DiagramView final : public QGraphicsView
{
public:
  explicit DiagramView(QWidget *parent = nullptr) : QGraphicsView(parent)
  {
    setScene(new QGraphicsScene(this));
    image_ = scene()->addPixmap(QPixmap());
    image_->setTransformationMode(Qt::SmoothTransformation);
    image_->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    // Stable viewport size avoids fitInView/scrollbar resize recursion.
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    setRenderHint(QPainter::SmoothPixmapTransform);
    setBackgroundBrush(Qt::white);
    setMinimumSize(420, 200);
  }

  void setDiagram(const QPixmap &pixmap)
  {
    // Qt 5's graphics pixmap bounds are not consistently DPR-aware. Preserve
    // every source pixel and apply its logical-size ratio explicitly to the item.
    const double ratio = pixmap.devicePixelRatio();
    QPixmap pixels = pixmap;
    if (!pixels.isNull()) {pixels.setDevicePixelRatio(1.0);}
    image_->setPixmap(pixels);
    image_->setScale(1.0 / ratio);
    setSceneRect(image_->sceneBoundingRect());
    if (fit_) {fitDiagram();}
  }

  void fitDiagram()
  {
    fit_ = true;
    if (!sceneRect().isEmpty()) {fitInView(sceneRect(), Qt::KeepAspectRatio);}
  }

  const QGraphicsPixmapItem *diagramItem() const {return image_;}

  void actualSize()
  {
    fit_ = false;
    resetTransform();
  }

  void zoomBy(double factor)
  {
    const double next = transform().m11() * factor;
    if (!std::isfinite(next) || next < 0.03 || next > 4.0) {return;}
    fit_ = false;
    scale(factor, factor);
  }

protected:
  void resizeEvent(QResizeEvent *event) override
  {
    QGraphicsView::resizeEvent(event);
    if (fit_) {fitDiagram();}
  }

  void wheelEvent(QWheelEvent *event) override
  {
    zoomBy(std::pow(1.0015, event->angleDelta().y()));
    event->accept();
  }

private:
  QGraphicsPixmapItem *image_{nullptr};
  bool fit_{true};
};

// Standalone ROS/Qt window; does not depend on the Gazebo GUI ABI.
class StateViewer final : public QWidget
{
public:
  explicit StateViewer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : node_(std::make_shared<rclcpp::Node>("cachecons_state_viewer", options))
  {
    const auto namespaces = node_->declare_parameter<std::vector<std::string>>(
      "robot_namespaces", {"T3A"});
    directory_ = node_->declare_parameter<std::string>("diagram_path", "");
    setWindowTitle("CacheCons state machine");
    auto *layout = new QVBoxLayout(this);
    selector_ = new QComboBox(this);
    label_ = new QLabel("Waiting for state", this);
    label_->setObjectName("currentPhase");
    diagram_ = new DiagramView(this);
    auto *controls = new QHBoxLayout;
    auto *actual = new QPushButton("100%", this);
    auto *out = new QPushButton("−", this);
    auto *in = new QPushButton("+", this);
    controls->addWidget(actual);
    controls->addWidget(out);
    controls->addWidget(in);
    controls->addStretch();
    connect(actual, &QPushButton::clicked, diagram_, &DiagramView::actualSize);
    connect(out, &QPushButton::clicked, this, [this]() {diagram_->zoomBy(0.8);});
    connect(in, &QPushButton::clicked, this, [this]() {diagram_->zoomBy(1.25);});
    layout->addWidget(selector_);
    layout->addWidget(label_);
    layout->addLayout(controls);
    layout->addWidget(diagram_, 1);
    for (auto ns : namespaces) {
      while (!ns.empty() && ns.front() == '/') {ns.erase(ns.begin());}
      while (!ns.empty() && ns.back() == '/') {ns.pop_back();}
      selector_->addItem(QString::fromStdString(ns));
      states_[ns] = -1;
      subscriptions_.push_back(node_->create_subscription<simulation_interfaces::msg::StateMsg>(
        "/" + ns + "/StateMsg", 5,
        [this, ns](simulation_interfaces::msg::StateMsg::ConstSharedPtr msg) {
          states_[ns] = msg->statemsg;
          refresh();
        }));
    }
    connect(selector_, qOverload<int>(&QComboBox::currentIndexChanged),
      this, [this](int) {refresh();});
    selection_ = std::make_unique<RobotSelection>(*selector_, node_,
      node_->declare_parameter<std::string>("selection_topic", "/cachecons/selected_robot"));
    auto *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
      if (rclcpp::ok()) {rclcpp::spin_some(node_); refresh();} else {close();}
    });
    timer->start(50);
    resize(1200, 760);
    refresh();
  }

protected:
  void closeEvent(QCloseEvent *event) override
  {
    selection_->clear();
    QWidget::closeEvent(event);
  }

private:
  void refresh()
  {
    static const std::array<const char *, 6> titles = {
      "Puck scan", "Puck target", "Homing", "Deposit push", "Deposit backup", "Exile"};
    static const std::array<const char *, 6> files = {
      "PU_SCAN.png", "PU_TARGET.png", "HOMING.png", "DE_PUSH.png", "DE_BACKUP.png", "EXILE.png"};
    auto ns = selector_->currentText().toStdString();
    const int state = states_.count(ns) ? states_.at(ns) : -1;
    if (state < 0 || state >= 6) {
      label_->setText(QString::fromStdString("Waiting for /" + ns + "/StateMsg"));
    } else {
      label_->setText(QString::fromStdString(ns) + " — " + titles[state]);
    }
    if (ns == displayed_namespace_ && state == displayed_state_) {return;}
    displayed_namespace_ = ns;
    displayed_state_ = state;
    if (state < 0 || state >= 6) {
      pixmap_ = QPixmap();
    } else {
      const auto directory = QString::fromStdString(directory_);
      pixmap_ = loadOverviewDiagram(directory, files[state]);
      if (pixmap_.isNull()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot load state diagram: %s",
          qUtf8Printable(overviewDiagramPath(directory, files[state])));
      }
    }
    diagram_->setDiagram(pixmap_);
  }
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<RobotSelection> selection_;
  std::vector<rclcpp::Subscription<simulation_interfaces::msg::StateMsg>::SharedPtr> subscriptions_;
  std::map<std::string, int> states_;
  std::string directory_, displayed_namespace_;
  int displayed_state_{-2};
  QComboBox *selector_{nullptr};
  QLabel *label_{nullptr};
  DiagramView *diagram_{nullptr};
  QPixmap pixmap_;
};

#ifndef CACHECONS_STATE_VIEWER_NO_MAIN
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication application(argc, argv);
  StateViewer viewer;
  viewer.show();
  const int result = application.exec();
  rclcpp::shutdown();
  return result;
}
#endif
