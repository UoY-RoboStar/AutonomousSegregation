#include <iostream>
#include <string>
#include <QTimer>
#include <gz/gui/MainWindow.hh>
#include <gz/sim/gui/Gui.hh>
#include "fullscreen_window.hpp"
#include "selection_highlighter.hpp"

int main(int argc, char **argv)
{
  GuiArguments arguments;
  try
  {
    arguments = parseGuiArguments(std::vector<std::string>(argv + 1, argv + argc));
  }
  catch (const std::invalid_argument &error)
  {
    std::cerr << error.what() << "\nUsage: cachecons_gazebo_gui <gui.config> "
              << "[--selection-topic /topic] [--robot namespace=entity]...\n";
    return 2;
  }
  // The normal Gazebo GUI startup discovers the server in our launch's
  // GZ_PARTITION and receives its scene; this process never controls physics.
  int qtArgc = 1;
  auto app = gz::sim::gui::createGui(qtArgc, argv, arguments.config.c_str());
  if (!app)
    return 1;
  auto *mainWindow = app->findChild<gz::gui::MainWindow *>();
  if (!mainWindow || !mainWindow->QuickWindow())
    return 1;
  FullscreenWindow presentation(mainWindow->QuickWindow());
  // The existing Gazebo handler retains ownership of SIGINT / SIGTERM.
  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto highlighter = std::make_shared<SelectionHighlighter>(arguments);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(highlighter);
  QTimer selectionTimer;
  QObject::connect(&selectionTimer, &QTimer::timeout, [&]()
    { executor.spin_some(); highlighter->update(); });
  selectionTimer.start(50);
  const int result = app->exec();
  selectionTimer.stop();
  executor.remove_node(highlighter);
  highlighter.reset();
  rclcpp::shutdown();
  return result;
}
