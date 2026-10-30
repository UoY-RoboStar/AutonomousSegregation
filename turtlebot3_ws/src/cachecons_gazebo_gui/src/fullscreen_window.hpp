#pragma once
#include <QKeyEvent>
#include <QQuickWindow>
#include <QVariant>

// Gazebo GUI 8's XML WindowConfig has no fullscreen or header-visibility
// setting. Apply these only to this GUI instance, never the desktop or ~/.gz.
class FullscreenWindow final : public QObject
{
public:
  explicit FullscreenWindow(QQuickWindow *window) : window_(window)
  {
    if (auto *header = window_->property("header").value<QObject *>())
    {
      header->setProperty("visible", false);
      header->setProperty("height", 0);
    }
    window_->installEventFilter(this);
    window_->showFullScreen();
  }

protected:
  bool eventFilter(QObject *object, QEvent *event) override
  {
    if (object == window_ && event->type() == QEvent::KeyPress)
    {
      auto *key = static_cast<QKeyEvent *>(event);
      if (!key->isAutoRepeat() && key->key() == Qt::Key_F11)
      {
        window_->visibility() == QWindow::FullScreen
          ? window_->showNormal() : window_->showFullScreen();
        return true;
      }
      if (key->key() == Qt::Key_Escape && window_->visibility() == QWindow::FullScreen)
      {
        window_->showNormal();
        return true;
      }
    }
    return QObject::eventFilter(object, event);
  }

private:
  QQuickWindow *window_;
};
