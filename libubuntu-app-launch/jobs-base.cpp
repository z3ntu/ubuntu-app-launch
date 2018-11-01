/*
 * Copyright © 2016 Canonical Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranties of
 * MERCHANTABILITY, SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:
 *     Ted Gould <ted.gould@canonical.com>
 */

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <numeric>
#include <unity/util/GObjectMemory.h>
#include <unity/util/GlibMemory.h>
#include <unity/util/ResourcePtr.h>

#include "application-impl-base.h"
#include "helper-impl.h"
#include "jobs-base.h"
#include "jobs-systemd.h"
#include "jobs-openrc.h"
#include "registry-impl.h"
#include "string-util.h"

using namespace unity::util;

namespace ubuntu
{
namespace app_launch
{
namespace jobs
{
namespace manager
{

Base::Base(const std::shared_ptr<Registry::Impl>& registry)
    : registry_{registry}
    , allApplicationJobs_{"application-legacy", "application-snap"}
{
}

Base::~Base()
{
}

/** Should determine which jobs backend to use, but we only have
    one right now. */
std::shared_ptr<Base> Base::determineFactory(const std::shared_ptr<Registry::Impl>& registry)
{
//     g_debug("Building a systemd jobs manager");
//     return std::make_shared<jobs::manager::SystemD>(registry);
    g_debug("Building a OpenRC jobs manager");
    return std::make_shared<jobs::manager::OpenRC>(registry);
}

const std::list<std::string>& Base::getAllApplicationJobs() const
{
    return allApplicationJobs_;
}

core::Signal<const std::shared_ptr<Application>&, const std::shared_ptr<Application::Instance>&>& Base::appStarted()
{
    std::call_once(flag_appStarted, [this]() {
        jobStarted().connect([this](const std::string& job, const std::string& appid, const std::string& instanceid) {
            if (std::find(allApplicationJobs_.begin(), allApplicationJobs_.end(), job) == allApplicationJobs_.end())
            {
                /* Not an application, different signal */
                return;
            }

            try
            {
                auto reg = getReg();
                auto appId = reg->find(appid);
                auto app = reg->createApp(appId);
                auto inst = std::dynamic_pointer_cast<app_impls::Base>(app)->findInstance(instanceid);

                sig_appStarted(app, inst);
            }
            catch (std::runtime_error& e)
            {
                g_warning("Error in appStarted signal from job: %s", e.what());
            }
        });
    });

    return sig_appStarted;
}

core::Signal<const std::shared_ptr<Application>&, const std::shared_ptr<Application::Instance>&>& Base::appStopped()
{
    std::call_once(flag_appStopped, [this]() {
        jobStopped().connect([this](const std::string& job, const std::string& appid, const std::string& instanceid) {
            if (std::find(allApplicationJobs_.begin(), allApplicationJobs_.end(), job) == allApplicationJobs_.end())
            {
                /* Not an application, different signal */
                return;
            }

            try
            {
                auto reg = getReg();
                auto appId = reg->find(appid);
                auto app = reg->createApp(appId);
                auto inst = std::dynamic_pointer_cast<app_impls::Base>(app)->findInstance(instanceid);

                sig_appStopped(app, inst);
            }
            catch (std::runtime_error& e)
            {
                g_warning("Error in appStopped signal from job: %s", e.what());
            }
        });
    });

    return sig_appStopped;
}

core::Signal<const std::shared_ptr<Application>&, const std::shared_ptr<Application::Instance>&, Registry::FailureType>&
    Base::appFailed()
{
    std::call_once(flag_appFailed, [this]() {
        jobFailed().connect([this](const std::string& job, const std::string& appid, const std::string& instanceid,
                                   Registry::FailureType reason) {
            if (std::find(allApplicationJobs_.begin(), allApplicationJobs_.end(), job) == allApplicationJobs_.end())
            {
                /* Not an application, different signal */
                return;
            }

            try
            {
                auto reg = getReg();
                auto appId = reg->find(appid);
                auto app = reg->createApp(appId);
                auto inst = std::dynamic_pointer_cast<app_impls::Base>(app)->findInstance(instanceid);

                sig_appFailed(app, inst, reason);
            }
            catch (std::runtime_error& e)
            {
                g_warning("Error in appFailed signal from job: %s", e.what());
            }
        });
    });

    return sig_appFailed;
}

/** Structure to track the data needed for upstart events. This cleans
    up the lifecycle as we're passing this as a pointer through the
    GLib calls. */
struct upstartEventData
{
    /** Keeping a weak pointer because the handle is held by
        the registry implementation. */
    std::weak_ptr<Registry::Impl> weakReg;
};

/** Core handler for pause and resume events. Includes turning the GVariant
    pid list into a std::vector and getting the application object. */
void Base::pauseEventEmitted(core::Signal<const std::shared_ptr<Application>&,
                                          const std::shared_ptr<Application::Instance>&,
                                          const std::vector<pid_t>&>& signal,
                             const std::shared_ptr<GVariant>& params,
                             const std::shared_ptr<Registry::Impl>& reg)
{
    std::vector<pid_t> pids;
    auto vappid = unique_glib(g_variant_get_child_value(params.get(), 0));
    auto vinstid = unique_glib(g_variant_get_child_value(params.get(), 1));
    auto vpids = unique_glib(g_variant_get_child_value(params.get(), 2));
    guint64 pid;
    GVariantIter thispid;
    g_variant_iter_init(&thispid, vpids.get());

    while (g_variant_iter_loop(&thispid, "t", &pid))
    {
        pids.emplace_back(pid);
    }

    auto cappid = g_variant_get_string(vappid.get(), NULL);
    auto cinstid = g_variant_get_string(vinstid.get(), NULL);

    auto appid = reg->find(cappid);
    auto app = reg->createApp(appid);
    auto inst = std::dynamic_pointer_cast<app_impls::Base>(app)->findInstance(cinstid);

    signal(app, inst, pids);

    return;
}

/** Grab the signal object for application paused. If we're not already listing for
    those signals this sets up a listener for them. */
core::Signal<const std::shared_ptr<Application>&,
             const std::shared_ptr<Application::Instance>&,
             const std::vector<pid_t>&>&
    Base::appPaused()
{
    std::call_once(flag_appPaused, [this]() {
        auto reg = getReg();
        reg->thread.executeOnThread<bool>([this, reg]() {
            upstartEventData* data = new upstartEventData{reg};

            handle_appPaused = managedDBusSignalConnection(
                g_dbus_connection_signal_subscribe(reg->_dbus.get(),                /* bus */
                                                   nullptr,                         /* sender */
                                                   "com.canonical.UbuntuAppLaunch", /* interface */
                                                   "ApplicationPaused",             /* signal */
                                                   "/",                             /* path */
                                                   nullptr,                         /* arg0 */
                                                   G_DBUS_SIGNAL_FLAGS_NONE,
                                                   [](GDBusConnection*, const gchar*, const gchar*, const gchar*,
                                                      const gchar*, GVariant* params, gpointer user_data) -> void {
                                                       auto data = reinterpret_cast<upstartEventData*>(user_data);
                                                       auto reg = data->weakReg.lock();

                                                       if (!reg)
                                                       {
                                                           g_warning("Registry object invalid!");
                                                           return;
                                                       }

                                                       auto sparams = share_glib(g_variant_ref(params));
                                                       auto manager = std::dynamic_pointer_cast<Base>(reg->jobs());
                                                       manager->pauseEventEmitted(manager->sig_appPaused, sparams, reg);
                                                   },    /* callback */
                                                   data, /* user data */
                                                   [](gpointer user_data) {
                                                       auto data = reinterpret_cast<upstartEventData*>(user_data);
                                                       delete data;
                                                   }), /* user data destroy */
                reg->_dbus);

            return true;
        });
    });

    return sig_appPaused;
}

/** Grab the signal object for application resumed. If we're not already listing for
    those signals this sets up a listener for them. */
core::Signal<const std::shared_ptr<Application>&,
             const std::shared_ptr<Application::Instance>&,
             const std::vector<pid_t>&>&
    Base::appResumed()
{
    std::call_once(flag_appResumed, [this]() {
        auto reg = getReg();
        reg->thread.executeOnThread<bool>([this, reg]() {
            upstartEventData* data = new upstartEventData{reg};

            handle_appResumed = managedDBusSignalConnection(
                g_dbus_connection_signal_subscribe(reg->_dbus.get(),                /* bus */
                                                   nullptr,                         /* sender */
                                                   "com.canonical.UbuntuAppLaunch", /* interface */
                                                   "ApplicationResumed",            /* signal */
                                                   "/",                             /* path */
                                                   nullptr,                         /* arg0 */
                                                   G_DBUS_SIGNAL_FLAGS_NONE,
                                                   [](GDBusConnection*, const gchar*, const gchar*, const gchar*,
                                                      const gchar*, GVariant* params, gpointer user_data) -> void {
                                                       auto data = reinterpret_cast<upstartEventData*>(user_data);
                                                       auto reg = data->weakReg.lock();

                                                       if (!reg)
                                                       {
                                                           g_warning("Registry object invalid!");
                                                           return;
                                                       }

                                                       auto sparams = share_glib(g_variant_ref(params));
                                                       auto manager = std::dynamic_pointer_cast<Base>(reg->jobs());
                                                       manager->pauseEventEmitted(manager->sig_appResumed, sparams,
                                                                                  reg);
                                                   },    /* callback */
                                                   data, /* user data */
                                                   [](gpointer user_data) {
                                                       auto data = reinterpret_cast<upstartEventData*>(user_data);
                                                       delete data;
                                                   }), /* user data destroy */
                reg->_dbus);

            return true;
        });
    });

    return sig_appResumed;
}

core::Signal<const std::shared_ptr<Helper>&, const std::shared_ptr<Helper::Instance>&>& Base::helperStarted(
    Helper::Type type)
{
    try
    {
        return *sig_helpersStarted.at(type.value());
    }
    catch (std::out_of_range& e)
    {
        jobStarted().connect(
            [this, type](const std::string& job, const std::string& appid, const std::string& instanceid) {
                if (job != type.value())
                {
                    return;
                }

                try
                {
                    auto reg = getReg();
                    auto appId = reg->find(appid);
                    auto helper = reg->createHelper(type, appId, reg);
                    auto inst = std::dynamic_pointer_cast<helper_impls::Base>(helper)->existingInstance(instanceid);

                    (*sig_helpersStarted.at(type.value()))(helper, inst);
                }
                catch (...)
                {
                    g_warning("Unable to emit signal for helper type: %s", type.value().c_str());
                }
            });

        sig_helpersStarted.emplace(
            type.value(),
            std::make_shared<core::Signal<const std::shared_ptr<Helper>&, const std::shared_ptr<Helper::Instance>&>>());
    }

    return *sig_helpersStarted.at(type.value());
}

core::Signal<const std::shared_ptr<Helper>&, const std::shared_ptr<Helper::Instance>&>& Base::helperStopped(
    Helper::Type type)
{
    try
    {
        return *sig_helpersStopped.at(type.value());
    }
    catch (std::out_of_range& e)
    {
        jobStopped().connect(
            [this, type](const std::string& job, const std::string& appid, const std::string& instanceid) {
                if (job != type.value())
                {
                    return;
                }

                try
                {
                    auto reg = getReg();
                    auto appId = ubuntu::app_launch::AppID::parse(appid);
                    auto helper = reg->createHelper(type, appId, reg);
                    auto inst = std::dynamic_pointer_cast<helper_impls::Base>(helper)->existingInstance(instanceid);

                    (*sig_helpersStopped.at(type.value()))(helper, inst);
                }
                catch (...)
                {
                    g_warning("Unable to emit signal for helper type: %s", type.value().c_str());
                }
            });

        sig_helpersStopped.emplace(
            type.value(),
            std::make_shared<core::Signal<const std::shared_ptr<Helper>&, const std::shared_ptr<Helper::Instance>&>>());
    }

    return *sig_helpersStopped.at(type.value());
}

core::Signal<const std::shared_ptr<Helper>&, const std::shared_ptr<Helper::Instance>&, Registry::FailureType>&
    Base::helperFailed(Helper::Type type)
{
    try
    {
        return *sig_helpersFailed.at(type.value());
    }
    catch (std::out_of_range& e)
    {
        jobFailed().connect([this, type](const std::string& job, const std::string& appid,
                                         const std::string& instanceid, Registry::FailureType reason) {
            if (job != type.value())
            {
                return;
            }

            try
            {
                auto reg = getReg();
                auto appId = ubuntu::app_launch::AppID::parse(appid);
                auto helper = reg->createHelper(type, appId, reg);
                auto inst = std::dynamic_pointer_cast<helper_impls::Base>(helper)->existingInstance(instanceid);

                (*sig_helpersFailed.at(type.value()))(helper, inst, reason);
            }
            catch (...)
            {
                g_warning("Unable to emit signal for helper type: %s", type.value().c_str());
            }
        });

        sig_helpersFailed.emplace(
            type.value(),
            std::make_shared<core::Signal<const std::shared_ptr<Helper>&, const std::shared_ptr<Helper::Instance>&,
                                          Registry::FailureType>>());
    }

    return *sig_helpersFailed.at(type.value());
}

/** Take the GVariant of parameters and turn them into an application and
    and instance. Easier to read in the smaller function */
std::tuple<std::shared_ptr<Application>, std::shared_ptr<Application::Instance>> Base::managerParams(
    const std::shared_ptr<GVariant>& params, const std::shared_ptr<Registry::Impl>& reg)
{
    std::shared_ptr<Application> app;
    std::shared_ptr<Application::Instance> instance;

    const gchar* cappid = nullptr;
    const gchar* cinstid = nullptr;
    g_variant_get(params.get(), "(&s&s)", &cappid, &cinstid);

    auto appid = reg->find(cappid);
    app = reg->createApp(appid);

    /* TODO Instance */

    return std::make_tuple(app, instance);
}

/** Used to store data for manager based signal handlers. Has a link to the
    registry and the callback to use in a C++ style. */
struct managerEventData
{
    /* Keeping a weak pointer because the handle is held by
       the registry implementation. */
    std::weak_ptr<Registry::Impl> weakReg;
    std::function<void(const std::shared_ptr<Registry::Impl>& reg,
                       const std::shared_ptr<Application>& app,
                       const std::shared_ptr<Application::Instance>& instance,
                       const std::shared_ptr<GDBusConnection>&,
                       const std::string&,
                       const std::shared_ptr<GVariant>&)>
        func;
};

/** Register for a signal for the manager. All of the signals needed this same
    code so it got pulled out into a function. Takes the same of the signal, the registry
    that we're using and a function to call after we've messaged all the parameters
    into being something C++-ish. */
guint Base::managerSignalHelper(const std::string& signalname,
                                std::function<void(const std::shared_ptr<Registry::Impl>& reg,
                                                   const std::shared_ptr<Application>& app,
                                                   const std::shared_ptr<Application::Instance>& instance,
                                                   const std::shared_ptr<GDBusConnection>&,
                                                   const std::string&,
                                                   const std::shared_ptr<GVariant>&)> responsefunc)
{
    auto reg = getReg();
    managerEventData* focusdata = new managerEventData{reg, responsefunc};

    return g_dbus_connection_signal_subscribe(
        reg->_dbus.get(),                /* bus */
        nullptr,                         /* sender */
        "com.canonical.UbuntuAppLaunch", /* interface */
        signalname.c_str(),              /* signal */
        "/",                             /* path */
        nullptr,                         /* arg0 */
        G_DBUS_SIGNAL_FLAGS_NONE,
        [](GDBusConnection* cconn, const gchar* csender, const gchar*, const gchar*, const gchar*, GVariant* params,
           gpointer user_data) -> void {
            auto data = reinterpret_cast<managerEventData*>(user_data);
            auto reg = data->weakReg.lock();

            if (!reg)
            {
                g_warning("Registry object invalid!");
                return;
            }

            /* If we're still conneted and the manager has been cleared
               we'll just be a no-op */
            auto ljobs = std::dynamic_pointer_cast<Base>(reg->jobs());
            if (!ljobs->manager_)
            {
                return;
            }

            try
            {
                auto vparams = share_glib(g_variant_ref(params));
                auto conn = share_gobject(G_DBUS_CONNECTION(g_object_ref(cconn)));
                std::string sender = csender;
                std::shared_ptr<Application> app;
                std::shared_ptr<Application::Instance> instance;

                std::tie(app, instance) = managerParams(vparams, reg);

                data->func(reg, app, instance, conn, sender, vparams);
            }
            catch (std::runtime_error& e)
            {
                g_warning("Unable to call signal handler for manager signal: %s", e.what());
            }
        },
        focusdata,
        [](gpointer user_data) {
            auto data = reinterpret_cast<managerEventData*>(user_data);
            delete data;
        }); /* user data destroy */
}

/** Set the manager for the registry. This includes tracking the pointer
    as well as setting up the signals to call back into the manager. The
    signals are only setup once per registry even if the manager is cleared
    and changed again. They will just be no-op's in those cases.
*/
void Base::setManager(std::shared_ptr<Registry::Manager> manager)
{
    if (manager_)
    {
        throw std::runtime_error("Already have a manager and trying to set another");
    }

    g_debug("Setting a new manager");
    manager_ = manager;

    std::call_once(flag_managerSignals, [this]() {
        auto reg = getReg();

        if (!reg->thread.executeOnThread<bool>([this, reg]() {
                handle_managerSignalFocus = managedDBusSignalConnection(
                    managerSignalHelper(
                        "UnityFocusRequest",
                        [](const std::shared_ptr<Registry::Impl>& reg, const std::shared_ptr<Application>& app,
                           const std::shared_ptr<Application::Instance>& instance,
                           const std::shared_ptr<GDBusConnection>& conn, const std::string& sender,
                           const std::shared_ptr<GVariant>& params) {
                            /* Nothing to do today */
                            std::dynamic_pointer_cast<Base>(reg->jobs())
                                ->manager_->focusRequest(app, instance, [](bool response) {
                                    /* NOTE: We have no
                                       clue what thread
                                       this is gonna be
                                       executed on, but
                                       since we're just
                                       talking to the
                                       GDBus
                                       thread it isn't an
                                       issue today. Be
                                       careful in
                                       changing
                                       this code. */
                                });
                        }),
                    reg->_dbus);
                handle_managerSignalStarting = managedDBusSignalConnection(
                    managerSignalHelper(
                        "UnityStartingBroadcast",
                        [](const std::shared_ptr<Registry::Impl>& reg, const std::shared_ptr<Application>& app,
                           const std::shared_ptr<Application::Instance>& instance,
                           const std::shared_ptr<GDBusConnection>& conn, const std::string& sender,
                           const std::shared_ptr<GVariant>& params) {

                            std::dynamic_pointer_cast<Base>(reg->jobs())
                                ->manager_->startingRequest(app, instance, [conn, sender, params](bool response) {
                                    /* NOTE: We have no clue what thread this is gonna be
                                       executed on, but since we're just talking to the GDBus
                                       thread it isn't an issue today. Be careful in changing
                                       this code. */
                                    if (response)
                                    {
                                        g_dbus_connection_emit_signal(conn.get(), sender.c_str(),      /* destination */
                                                                      "/",                             /* path */
                                                                      "com.canonical.UbuntuAppLaunch", /* interface */
                                                                      "UnityStartingSignal",           /* signal */
                                                                      params.get(), /* params, the same */
                                                                      nullptr);     /* error */
                                    }
                                });
                        }),
                    reg->_dbus);
                handle_managerSignalResume = managedDBusSignalConnection(
                    managerSignalHelper(
                        "UnityResumeRequest",
                        [](const std::shared_ptr<Registry::Impl>& reg, const std::shared_ptr<Application>& app,
                           const std::shared_ptr<Application::Instance>& instance,
                           const std::shared_ptr<GDBusConnection>& conn, const std::string& sender,
                           const std::shared_ptr<GVariant>& params) {
                            std::dynamic_pointer_cast<Base>(reg->jobs())
                                ->manager_->resumeRequest(app, instance, [conn, sender, params](bool response) {
                                    /* NOTE: We have no clue what thread this is gonna be
                                       executed on, but since we're just talking to the GDBus
                                       thread it isn't an issue today. Be careful in changing
                                       this code. */
                                    if (response)
                                    {
                                        g_dbus_connection_emit_signal(conn.get(), sender.c_str(),      /* destination */
                                                                      "/",                             /* path */
                                                                      "com.canonical.UbuntuAppLaunch", /* interface */
                                                                      "UnityResumeResponse",           /* signal */
                                                                      params.get(), /* params, the same */
                                                                      nullptr);     /* error */
                                    }
                                });
                        }),
                    reg->_dbus);

                return true;
            }))
        {
            g_warning("Unable to install manager signals");
        }
    });
}

/** Clear the manager pointer */
void Base::clearManager()
{
    g_debug("Clearing the manager");
    manager_.reset();
}

/** Get application objects for all of the applications based
    on the appids associated with the application jobs */
std::list<std::shared_ptr<Application>> Base::runningApps()
{
    auto reg = getReg();
    auto appids = runningAppIds(allApplicationJobs_);

    std::list<std::shared_ptr<Application>> apps;
    for (const auto& appid : appids)
    {
        auto id = reg->find(appid);
        if (id.empty())
        {
            g_debug("Unable to handle AppID: %s", appid.c_str());
            continue;
        }

        try
        {
            apps.emplace_back(reg->createApp(id));
        }
        catch (std::runtime_error& e)
        {
            g_debug("Error adding appid '%s' to running apps list: %s", appid.c_str(), e.what());
        }
    }

    return apps;
}

/** Get application objects for all of the applications based
    on the appids associated with the application jobs */
std::list<std::shared_ptr<Helper>> Base::runningHelpers(const Helper::Type& type)
{
    auto reg = getReg();
    auto appids = runningAppIds({type.value()});

    std::list<std::shared_ptr<Helper>> helpers;
    for (const auto& appid : appids)
    {
        auto id = reg->find(appid);
        if (id.empty())
        {
            g_debug("Unable to handle AppID: %s", appid.c_str());
            continue;
        }

        helpers.emplace_back(reg->createHelper(type, id, reg));
    }

    return helpers;
}

}  // namespace manager

namespace instance
{

Base::Base(const AppID& appId,
           const std::string& job,
           const std::string& instance,
           const std::vector<Application::URL>& urls,
           const std::shared_ptr<Registry::Impl>& registry)
    : appId_(appId)
    , job_(job)
    , instance_(instance)
    , urls_(urls)
    , registry_(registry)
{
}

/** Checks to see if we have a primary PID for the instance */
bool Base::isRunning()
{
    return primaryPid() != 0;
}

/** Looks at the PIDs in the instance cgroup and checks to see if @pid
    is in the set.

    @param pid PID to look for
*/
bool Base::hasPid(pid_t pid)
{
    auto vpids = pids();
    bool hasit = std::find(vpids.begin(), vpids.end(), pid) != vpids.end();
    g_debug("Checking for PID %d on AppID '%s' result: %s", pid, std::string(appId_).c_str(), hasit ? "YES" : "NO");
    return hasit;
}

/** Pauses this application by sending SIGSTOP to all the PIDs in the
    cgroup and tells Zeitgeist that we've left the application. */
void Base::pause()
{
    g_debug("Pausing application: %s", std::string(appId_).c_str());
    registry_->zgSendEvent(appId_, ZEITGEIST_ZG_LEAVE_EVENT);

    auto pids = forAllPids([this](pid_t pid) {
        auto oomval = oom::paused();
        g_debug("Pausing PID: %d (%d)", pid, int(oomval));
        signalToPid(pid, SIGSTOP);
        oomValueToPid(pid, oomval);
    });

    pidListToDbus(registry_, appId_, instance_, pids, "ApplicationPaused");
}

/** Resumes this application by sending SIGCONT to all the PIDs in the
    cgroup and tells Zeitgeist that we're accessing the application. */
void Base::resume()
{
    g_debug("Resuming application: %s", std::string(appId_).c_str());
    registry_->zgSendEvent(appId_, ZEITGEIST_ZG_ACCESS_EVENT);

    auto pids = forAllPids([this](pid_t pid) {
        auto oomval = oom::focused();
        g_debug("Resuming PID: %d (%d)", pid, int(oomval));
        signalToPid(pid, SIGCONT);
        oomValueToPid(pid, oomval);
    });

    pidListToDbus(registry_, appId_, instance_, pids, "ApplicationResumed");
}

/** Focuses this application by sending SIGCONT to all the PIDs in the
    cgroup and tells the Shell to focus the application. */
void Base::focus()
{
    g_debug("Focusing application: %s", std::string(appId_).c_str());

    GError* error = nullptr;
    GVariantBuilder params;
    g_variant_builder_init(&params, G_VARIANT_TYPE_TUPLE);
    g_variant_builder_add_value(&params, g_variant_new_string(std::string(appId_).c_str()));
    g_variant_builder_add_value(&params, g_variant_new_string(instance_.c_str()));
    g_dbus_connection_emit_signal(registry_->_dbus.get(),          /* bus */
                                  nullptr,                         /* destination */
                                  "/",                             /* path */
                                  "com.canonical.UbuntuAppLaunch", /* interface */
                                  "UnityFocusRequest",             /* signal */
                                  g_variant_builder_end(&params),  /* params */
                                  &error);                         /* error */

    if (error != nullptr)
    {
        g_warning("Unable to emit signal 'UnityFocusRequest' for appid '%s': '%s'", std::string(appId_).c_str(),
                  error->message);
        g_error_free(error);
    }
    else
    {
        g_debug("Emmitted 'UnityFocusRequest' to DBus");
    }
}

/** Go through the list of PIDs calling a function and handling
    the issue with getting PIDs being a racey condition.

    \param eachPid Function to run on each PID
*/
std::vector<pid_t> Base::forAllPids(std::function<void(pid_t)> eachPid)
{
    std::set<pid_t> seenPids;
    bool added = true;

    while (added)
    {
        added = false;
        auto pidlist = pids();
        for (auto pid : pidlist)
        {
            if (seenPids.insert(pid).second)
            {
                eachPid(pid);
                added = true;
            }
        }
    }

    return std::vector<pid_t>(seenPids.begin(), seenPids.end());
}

/** Send a signal that we've change the application. Do this on the
    registry thread in an idle so that we don't block anyone.

    \param pids List of PIDs to turn into variants to send
    \param signal Name of the DBus signal to send
*/
void Base::pidListToDbus(const std::shared_ptr<Registry::Impl>& reg,
                         const AppID& appid,
                         const std::string& instanceid,
                         const std::vector<pid_t>& pids,
                         const std::string& signal)
{
    GVariantUPtr vpids;

    if (pids.empty())
    {
        vpids = unique_glib(g_variant_ref_sink(g_variant_new_array(G_VARIANT_TYPE_UINT64, nullptr, 0)));
    }
    else
    {
        GVariantBuilder builder;
        g_variant_builder_init(&builder, G_VARIANT_TYPE_ARRAY);

        for (auto pid : pids)
        {
            g_variant_builder_add_value(&builder, g_variant_new_uint64(pid));
        }

        vpids = unique_glib(g_variant_ref_sink(g_variant_builder_end(&builder)));
    }

    GVariantBuilder params;
    g_variant_builder_init(&params, G_VARIANT_TYPE_TUPLE);
    g_variant_builder_add_value(&params, g_variant_new_string(std::string(appid).c_str()));
    g_variant_builder_add_value(&params, g_variant_new_string(instanceid.c_str()));
    g_variant_builder_add_value(&params, vpids.get());

    GError* error = nullptr;
    g_dbus_connection_emit_signal(reg->_dbus.get(),                /* bus */
                                  nullptr,                         /* destination */
                                  "/",                             /* path */
                                  "com.canonical.UbuntuAppLaunch", /* interface */
                                  signal.c_str(),                  /* signal */
                                  g_variant_builder_end(&params),  /* params, the same */
                                  &error);                         /* error */

    if (error != nullptr)
    {
        g_warning("Unable to emit signal '%s' for appid '%s': %s", signal.c_str(), std::string(appid).c_str(),
                  error->message);
        g_error_free(error);
    }
    else
    {
        g_debug("Emmitted '%s' to DBus", signal.c_str());
    }
}

/** Sets the OOM adjustment by getting the list of PIDs and writing
    the value to each of their files in proc

    \param score OOM Score to set
*/
void Base::setOomAdjustment(const oom::Score score)
{
    forAllPids([this, &score](pid_t pid) { oomValueToPid(pid, score); });
}

/** Figures out the path to the primary PID of the application and
    then reads its OOM adjustment file. */
const oom::Score Base::getOomAdjustment()
{
    auto pid = primaryPid();
    if (pid == 0)
    {
        throw std::runtime_error("No PID for application: " + std::string(appId_));
    }

    auto path = pidToOomPath(pid);
    GError* error = nullptr;
    gchar* content = nullptr;

    g_file_get_contents(path.c_str(), /* path */
                        &content,     /* data */
                        nullptr,      /* size */
                        &error);      /* error */

    if (error != nullptr)
    {
        auto serror = unique_glib(error);
        error = nullptr;
        throw std::runtime_error("Unable to access OOM value for '" + std::string(appId_) + "' primary PID '" +
                                 std::to_string(pid) + "' because: " + serror->message);
    }

    auto score = static_cast<oom::Score>(std::atoi(content));
    g_free(content);
    return score;
}

/** Sends a signal to a PID with a warning if we can't send it.
    We could throw an exception, but we can't handle it usefully anyway

    \param pid PID to send the signal to
    \param signal signal to send
*/
void Base::signalToPid(pid_t pid, int signal)
{
    if (-1 == kill(pid, signal))
    {
        /* While that didn't work, we still want to try as many as we can */
        g_warning("Unable to send signal %d to pid %d", signal, pid);
    }
}

/** Get the path to the PID's OOM adjust path, with allowing for an
    override for testing using the environment variable
    UBUNTU_APP_LAUNCH_OOM_PROC_PATH

    \param pid PID to build path for
*/
std::string Base::pidToOomPath(pid_t pid)
{
    static std::string procpath;
    if (G_UNLIKELY(procpath.empty()))
    {
        /* Set by the test suite, probably not anyone else */
        auto envvar = g_getenv("UBUNTU_APP_LAUNCH_OOM_PROC_PATH");
        if (G_LIKELY(envvar == nullptr))
            procpath = "/proc";
        else
            procpath = envvar;
    }

    auto gpath =
        unique_gchar(g_build_filename(procpath.c_str(), std::to_string(pid).c_str(), "oom_score_adj", nullptr));
    return std::string(gpath.get());
}

/** Writes an OOM value to proc, assuming we have a string
    in the outer loop

    \param pid PID to change the OOM value of
    \param oomvalue OOM value to set
*/
void Base::oomValueToPid(pid_t pid, const oom::Score oomvalue)
{
    auto oomstr = std::to_string(static_cast<std::int32_t>(oomvalue));
    auto path = pidToOomPath(pid);
    ResourcePtr<FILE*, void (*)(FILE*)> adj(fopen(path.c_str(), "w"), [](FILE* fp) {
        if (fp != nullptr)
        {
            fclose(fp);
        }
    });
    int openerr = errno;

    if (adj.get() == nullptr)
    {
        switch (openerr)
        {
            case ENOENT:
                /* ENOENT happens a fair amount because of races, so it's not
                   worth printing a warning about */
                return;
            case EACCES:
            {
                /* We can get this error when trying to set the OOM value on
                   Oxide renderers because they're started by the sandbox and
                   don't have their adjustment value available for us to write.
                   We have a helper to deal with this, but it's kinda expensive
                   so we only use it when we have to. */
                oomValueToPidHelper(pid, oomvalue);
                return;
            }
            default:
                g_warning("Unable to set OOM value for '%d' to '%s': %s", int(pid), oomstr.c_str(),
                          std::strerror(openerr));
                return;
        }
    }

    size_t writesize = fwrite(oomstr.c_str(), 1, oomstr.size(), adj.get());
    int writeerr = errno;
    adj.dealloc();

    if (writesize == oomstr.size())
        return;

    if (writeerr != 0)
        g_warning("Unable to set OOM value for '%d' to '%s': %s", int(pid), oomstr.c_str(), strerror(writeerr));
    else
        /* No error, but yet, wrong size. Not sure, what could cause this. */
        g_debug("Unable to set OOM value for '%d' to '%s': Wrote %d bytes", int(pid), oomstr.c_str(), int(writesize));
}

/** Use a setuid root helper for setting the oom value of
    Chromium instances

    \param pid PID to change the OOM value of
    \param oomvalue OOM value to set
*/
void Base::oomValueToPidHelper(pid_t pid, const oom::Score oomvalue)
{
    GError* error = nullptr;
    std::string oomstr = std::to_string(static_cast<std::int32_t>(oomvalue));
    std::string pidstr = std::to_string(pid);
    std::array<const char*, 4> args = {OOM_HELPER, pidstr.c_str(), oomstr.c_str(), nullptr};

    g_debug("Excuting OOM Helper (pid: %d, score: %d): %s", int(pid), int(oomvalue),
            std::accumulate(args.begin(), args.end(), std::string{}, [](const std::string& instr,
                                                                        const char* output) -> std::string {
                if (instr.empty())
                {
                    return output;
                }
                else if (output != nullptr)
                {
                    return instr + " " + std::string(output);
                }
                else
                {
                    return instr;
                }
            }).c_str());

    g_spawn_async(nullptr,               /* working dir */
                  (char**)(args.data()), /* args */
                  nullptr,               /* env */
                  G_SPAWN_DEFAULT,       /* flags */
                  nullptr,               /* child setup */
                  nullptr,               /* child setup userdata*/
                  nullptr,               /* pid */
                  &error);               /* error */

    if (error != nullptr)
    {
        g_warning("Unable to launch OOM helper '" OOM_HELPER "' on PID '%d': %s", pid, error->message);
        g_error_free(error);
        return;
    }
}

/** Reformat a C++ vector of URLs into a C GStrv of strings

    \param urls Vector of URLs to make into C strings
*/
GCharVUPtr Base::urlsToStrv(const std::vector<Application::URL>& urls)
{
    if (urls.empty())
    {
        return GCharVUPtr(nullptr, &g_strfreev);
    }

    auto array = g_array_new(TRUE, FALSE, sizeof(gchar*));

    for (auto url : urls)
    {
        auto str = g_strdup(url.value().c_str());
        g_debug("Converting URL: %s", str);
        g_array_append_val(array, str);
    }

    return unique_gcharv((gchar**)g_array_free(array, FALSE));
}

}  // namespace instance

}  // namespace jobs
}  // namespace app_launch
}  // namespace ubuntu
