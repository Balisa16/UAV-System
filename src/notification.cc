#include <notification.hpp>

Notification::Notification() { notify_init("Copter Autonomous Flight"); }

void Notification::show(std::string message, std::string title = "Copter",
                        int timeout_ms = 3000)
{
    NotifyNotification *notification =
        notify_notification_new(title.c_str(), message.c_str(), 0);

    // Set the notification timeout (in milliseconds)
    notify_notification_set_timeout(notification, timeout_ms);

    // Show the notification
    notify_notification_show(notification, 0);

    // Clean up
    g_object_unref(G_OBJECT(notification));
}

Notification::~Notification() { notify_uninit(); }