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

#include "jobs-openrc.h"
#include "application-impl-base.h"
#include "registry-impl.h"
#include "second-exec-core.h"
#include "string-util.h"
#include "utils.h"

extern "C" {
#include "ubuntu-app-launch-trace.h"
}

#include <gio/gio.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <numeric>
#include <regex>
#include <unity/util/GlibMemory.h>

using namespace unity::util;

namespace ubuntu
{
namespace app_launch
{
namespace jobs
{
namespace instance
{

class OpenRC : public instance::Base
{
    friend class manager::OpenRC;

public:
    explicit OpenRC(const AppID& appId,
                     const std::string& job,
                     const std::string& instance,
                     const std::vector<Application::URL>& urls,
                     const std::shared_ptr<Registry::Impl>& registry);
    virtual ~OpenRC()
    {
        g_debug("Destroying a OpenRC for '%s' instance '%s'", std::string(appId_).c_str(), instance_.c_str());
    }

    /* Query lifecycle */
    pid_t primaryPid() override;
    std::vector<pid_t> pids() override;

    /* Manage lifecycle */
    void stop() override;

};  // class OpenRC

OpenRC::OpenRC(const AppID& appId,
                 const std::string& job,
                 const std::string& instance,
                 const std::vector<Application::URL>& urls,
                 const std::shared_ptr<Registry::Impl>& registry)
    : Base(appId, job, instance, urls, registry)
{
    g_debug("Creating a new OpenRC for '%s' instance '%s'", std::string(appId).c_str(), instance.c_str());
}

pid_t OpenRC::primaryPid()
{
    auto manager = std::dynamic_pointer_cast<manager::OpenRC>(registry_->jobs());
    return manager->unitPrimaryPid(appId_, job_, instance_);
}

std::vector<pid_t> OpenRC::pids()
{
    auto manager = std::dynamic_pointer_cast<manager::OpenRC>(registry_->jobs());
    return manager->unitPids(appId_, job_, instance_);
}

void OpenRC::stop()
{
    auto manager = std::dynamic_pointer_cast<manager::OpenRC>(registry_->jobs());
    manager->stopUnit(appId_, job_, instance_);
}

}  // namespace instance

namespace manager
{

// static const char* SYSTEMD_DBUS_ADDRESS{"org.freedesktop.systemd1"};
// static const char* SYSTEMD_DBUS_IFACE_MANAGER{"org.freedesktop.systemd1.Manager"};
// static const char* SYSTEMD_DBUS_PATH_MANAGER{"/org/freedesktop/systemd1"};
// static const char * SYSTEMD_DBUS_IFACE_UNIT{"org.freedesktop.systemd1.Unit"};
// static const char* SYSTEMD_DBUS_IFACE_SERVICE{"org.freedesktop.systemd1.Service"};

OpenRC::OpenRC(const std::shared_ptr<Registry::Impl>& registry)
    : Base(registry)
{
    // TODO STUB
    g_debug("OpenRC constructor stub called...");
}

OpenRC::~OpenRC()
{
}

void OpenRC::copyEnv(const std::string& envname, std::list<std::pair<std::string, std::string>>& env)
{
    if (!findEnv(envname, env).empty())
    {
        g_debug("Already a value set for '%s' ignoring", envname.c_str());
        return;
    }

    auto cvalue = getenv(envname.c_str());
    g_debug("Copying Environment: %s", envname.c_str());
    if (cvalue != nullptr)
    {
        std::string value{cvalue};
        env.emplace_back(std::make_pair(envname, value));
    }
    else
    {
        g_debug("Unable to copy environment '%s'", envname.c_str());
    }
}

void OpenRC::copyEnvByPrefix(const std::string& prefix, std::list<std::pair<std::string, std::string>>& env)
{
    for (unsigned int i = 0; environ[i] != nullptr; i++)
    {
        if (g_str_has_prefix(environ[i], prefix.c_str()))
        {
            std::string envname = environ[i];
            envname.erase(envname.find('='));
            copyEnv(envname, env);
        }
    }
}

std::shared_ptr<Application::Instance> OpenRC::launch(
    const AppID& appId,
    const std::string& job,
    const std::string& instance,
    const std::vector<Application::URL>& urls,
    launchMode mode,
    std::function<std::list<std::pair<std::string, std::string>>(void)>& getenv)
{
    if (appId.empty())
        return {};

    auto appJobs = getAllApplicationJobs();
    bool isApplication = std::find(appJobs.begin(), appJobs.end(), job) != appJobs.end();

    auto reg = getReg();
    return reg->thread.executeOnThread<std::shared_ptr<instance::OpenRC>>([&]() -> std::shared_ptr<instance::OpenRC> {
        auto manager = std::dynamic_pointer_cast<manager::OpenRC>(reg->jobs());
        std::string appIdStr{appId};
        g_debug("Initializing params for an new instance::OpenRC for: %s", appIdStr.c_str());

        tracepoint(ubuntu_app_launch, libual_start, appIdStr.c_str());

        int timeout = 1;
        if (ubuntu::app_launch::Registry::Impl::isWatchingAppStarting())
        {
            timeout = 0;
        }

        handshake_t* handshake{nullptr};

        if (isApplication)
        {
            handshake = starting_handshake_start(appIdStr.c_str(), instance.c_str(), timeout);
            if (handshake == nullptr)
            {
                g_warning("Unable to setup starting handshake");
            }
        }

        /* Build up our environment */
        auto env = getenv();

        env.emplace_back(std::make_pair("APP_ID", appIdStr));                           /* Application ID */
        env.emplace_back(std::make_pair("APP_LAUNCHER_PID", std::to_string(getpid()))); /* Who we are, for bugs */

        copyEnv("DISPLAY", env);

        for (const auto& prefix : {"DBUS_", "MIR_", "UBUNTU_APP_LAUNCH_"})
        {
            copyEnvByPrefix(prefix, env);
        }

        /* If we're in deb mode and launching legacy apps, they're gonna need
         * more context, they really have no other way to get it. */
        if (g_getenv("SNAP") == nullptr && appId.package.value().empty())
        {
            copyEnvByPrefix("QT_", env);
            copyEnvByPrefix("XDG_", env);
            copyEnv("UBUNTU_APP_LAUNCH_XMIR_PATH", env);

            /* If we're in Unity8 we don't want to pass it's platform, we want
             * an application platform. */
            if (findEnv("QT_QPA_PLATFORM", env) == "mirserver" || findEnv("QT_QPA_PLATFORM", env) == "ubuntumirclient")
            {
                removeEnv("QT_QPA_PLATFORM", env);
                env.emplace_back(std::make_pair("QT_QPA_PLATFORM", "wayland"));
            }
        }

        /* Mir socket if we don't have one in our env */
        if (findEnv("MIR_SOCKET", env).empty())
        {
            env.emplace_back(std::make_pair("MIR_SOCKET", g_get_user_runtime_dir() + std::string{"/mir_socket"}));
        }

        if (!urls.empty())
        {
            auto accumfunc = [](const std::string& prev, Application::URL thisurl) -> std::string {
                gchar* gescaped = g_shell_quote(thisurl.value().c_str());
                std::string escaped;
                if (gescaped != nullptr)
                {
                    escaped = gescaped;
                    g_free(gescaped);
                }
                else
                {
                    g_warning("Unable to escape URL: %s", thisurl.value().c_str());
                    return prev;
                }

                if (prev.empty())
                {
                    return escaped;
                }
                else
                {
                    return prev + " " + escaped;
                }
            };
            auto urlstring = std::accumulate(urls.begin(), urls.end(), std::string{}, accumfunc);
            env.emplace_back(std::make_pair("APP_URIS", urlstring));
        }

        if (mode == launchMode::TEST)
        {
            env.emplace_back(std::make_pair("QT_LOAD_TESTABILITY", "1"));
        }

        /* ExecStart */
        auto commands = parseExec(env);
        if (commands.empty())
        {
            // TODO: Handle???
            g_debug("Commands stuff is empty!!");
            return nullptr;
        }

        /* Working Directory */
        if (!findEnv("APP_DIR", env).empty())
        {
//             g_variant_builder_open(&builder, G_VARIANT_TYPE_TUPLE);
//             g_variant_builder_add_value(&builder, g_variant_new_string("WorkingDirectory"));
//             g_variant_builder_open(&builder, G_VARIANT_TYPE_VARIANT);
//             g_variant_builder_add_value(&builder, g_variant_new_string(findEnv("APP_DIR", env).c_str()));
        }

        /* Clean up env before shipping it */
        for (const auto& rmenv :
             {"APP_XMIR_ENABLE", "APP_DIR", "APP_URIS", "APP_EXEC", "APP_EXEC_POLICY", "APP_LAUNCHER_PID",
              "INSTANCE_ID", "MIR_SERVER_PLATFORM_PATH", "MIR_SERVER_PROMPT_FILE", "MIR_SERVER_HOST_SOCKET",
              "UBUNTU_APP_LAUNCH_OOM_HELPER", "UBUNTU_APP_LAUNCH_LEGACY_ROOT", "UBUNTU_APP_LAUNCH_XMIR_HELPER"})
        {
            removeEnv(rmenv, env);
        }

        g_debug("Environment length: %d", envSize(env));

        /* Environment */
        for (const auto& envvar : env)
        {
            if (!envvar.first.empty() && !envvar.second.empty())
            {
//                 g_variant_builder_add_value(&builder, g_variant_new_take_string(g_strdup_printf(
//                                                           "%s=%s", envvar.first.c_str(), envvar.second.c_str())));
                // g_debug("Setting environment: %s=%s", envvar.first.c_str(), envvar.second.c_str());
            }
        }

        auto retval = std::make_shared<instance::OpenRC>(appId, job, instance, urls, reg);

        tracepoint(ubuntu_app_launch, handshake_wait, appIdStr.c_str());
        starting_handshake_wait(handshake);
        tracepoint(ubuntu_app_launch, handshake_complete, appIdStr.c_str());

        // From https://stackoverflow.com/a/7026414
        std::vector<char*> argv;
        for(auto loop = commands.begin(); loop != commands.end(); ++loop)
        {
            argv.push_back(&(*loop)[0]);
        }
        argv.push_back(NULL);

        std::vector<char*> envp;
        for(auto loop = env.begin(); loop != env.end(); ++loop)
        {
            std::string envStr;
            envStr += (*loop).first;
            envStr += "=";
            envStr += (*loop).second;
            argv.push_back(&envStr[0u]);
            g_debug(argv.back());
        }
        envp.push_back(NULL);

        pid_t child_pid = fork();
        if(child_pid == 0) {
            /* Call the job start function */
            g_debug("argv[0]: %s", argv[0]);
//             execvpe(argv[0], &argv[0], &envp[0]);

            g_debug("DOING SOMETHING ELSE");
            char *args2[] = { "system-settings", NULL }; /* Appears as `argv` in the main function */
            char *env2[] = { "APP_DESKTOP_FILE_PATH=/usr/share/applications/ubuntu-system-settings.desktop", "APP_ID=ubuntu-system-settings", "DBUS_SESSION_BUS_ADDRESS=unix:path=/var/run/dbus/system_bus_socket", "MIR_SERVER_CURSOR=null", "XDG_RUNTIME_DIR=/tmp/0-runtime-dir", "QT_QPA_PLATFORM=wayland", "MIR_SOCKET=/tmp/0-runtime-dir/mir_socket", NULL }; /* Env variables, appears in var `environ` */
            execvpe("system-settings", args2, env2);

            g_debug("execvpe failed from child.");
            g_debug("errno: %d", errno);
            g_debug(strerror(errno));
        }
        g_debug("child_pid: %d", child_pid);
        launchedpids.emplace_back(child_pid);

        tracepoint(ubuntu_app_launch, libual_start_message_sent, appIdStr.c_str());

        return retval;
    });
}

std::shared_ptr<Application::Instance> OpenRC::existing(const AppID& appId,
                                                         const std::string& job,
                                                         const std::string& instance,
                                                         const std::vector<Application::URL>& urls)
{
    return std::make_shared<instance::OpenRC>(appId, job, instance, urls, getReg());
}

std::vector<std::shared_ptr<instance::Base>> OpenRC::instances(const AppID& appID, const std::string& job)
{
    // TODO: STUB
    g_debug("instances stub called...");
    auto reg = getReg();

    std::vector<std::shared_ptr<instance::Base>> instances;
    std::vector<Application::URL> urls;
    std::string instance = "somestring";

    instances.emplace_back(std::make_shared<instance::OpenRC>(appID, job, instance, urls, reg));
    g_debug("Found %d instances for AppID '%s'", int(instances.size()), std::string(appID).c_str());

    return instances;
}

std::list<std::string> OpenRC::runningAppIds(const std::list<std::string>& allJobs)
{
    // TODO: STUB
    g_debug("runningAppIds stub called...");
    std::set<std::string> appids;
    return {appids.begin(), appids.end()};
}

pid_t OpenRC::unitPrimaryPid(const AppID& appId, const std::string& job, const std::string& instance)
{
    // TODO: STUB
    g_debug("unitPrimaryPid stub called...");
    return 0;
}

std::vector<pid_t> OpenRC::unitPids(const AppID& appId, const std::string& job, const std::string& instance)
{
    // TODO: STUB
    g_debug("unitPids stub called...");
    return launchedpids;
}

void OpenRC::stopUnit(const AppID& appId, const std::string& job, const std::string& instance)
{
    // TODO: STUB
    g_debug("stopUnit stub called...");
}

core::Signal<const std::string&, const std::string&, const std::string&>& OpenRC::jobStarted()
{
    /* Ensure we're connecting to the signals */
    return sig_jobStarted;
}

core::Signal<const std::string&, const std::string&, const std::string&>& OpenRC::jobStopped()
{
    /* Ensure we're connecting to the signals */
    return sig_jobStopped;
}

core::Signal<const std::string&, const std::string&, const std::string&, Registry::FailureType>& OpenRC::jobFailed()
{
    // TODO: STUB
    g_debug("jobFailed stub called...");
    return sig_jobFailed;
}

std::string OpenRC::findEnv(const std::string& value, std::list<std::pair<std::string, std::string>>& env)
{
    std::string retval;
    auto entry = std::find_if(env.begin(), env.end(),
                              [&value](std::pair<std::string, std::string>& entry) { return entry.first == value; });

    if (entry != env.end())
    {
        retval = entry->second;
    }

    return retval;
}

void OpenRC::removeEnv(const std::string& value, std::list<std::pair<std::string, std::string>>& env)
{
    auto entry = std::find_if(env.begin(), env.end(),
                              [&value](std::pair<std::string, std::string>& entry) { return entry.first == value; });

    if (entry != env.end())
    {
        env.erase(entry);
    }
}

int OpenRC::envSize(std::list<std::pair<std::string, std::string>>& env)
{
    int len = std::string{"Environment="}.length();

    for (const auto& entry : env)
    {
        len += 3; /* two quotes, one space */
        len += entry.first.length();
        len += entry.second.length();
    }

    len -= 1; /* We account for a space each time but the first doesn't have */

    return len;
}

std::vector<std::string> OpenRC::parseExec(std::list<std::pair<std::string, std::string>>& env)
{
    auto exec = findEnv("APP_EXEC", env);
    if (exec.empty())
    {
        g_warning("Application exec line is empty?!?!?");
        return {};
    }
    auto uris = findEnv("APP_URIS", env);

    g_debug("Exec line: %s", exec.c_str());
    g_debug("App URLS:  %s", uris.c_str());

    auto execarray = desktop_exec_parse(exec.c_str(), uris.c_str());

    std::vector<std::string> retval;
    for (unsigned int i = 0; i < execarray->len; i++)
    {
        auto cstr = g_array_index(execarray, gchar*, i);
        if (cstr != nullptr)
        {
            retval.emplace_back(cstr);
        }
    }

    /* This seems to work better than array_free(), I can't figure out why */
    auto strv = (gchar**)g_array_free(execarray, FALSE);
    g_strfreev(strv);

    if (retval.empty())
    {
        g_warning("After parsing 'APP_EXEC=%s' we ended up with no tokens", exec.c_str());
    }

    /* See if we need the xmir helper */
    if (findEnv("APP_XMIR_ENABLE", env) == "1" && getenv("DISPLAY") == nullptr)
    {
        retval.emplace(retval.begin(), findEnv("APP_ID", env));

        auto snapenv = getenv("SNAP");
        if (snapenv == nullptr)
        {
            auto xmirenv = getenv("UBUNTU_APP_LAUNCH_XMIR_HELPER");
            if (xmirenv == nullptr)
            {
                retval.emplace(retval.begin(), XMIR_HELPER);
            }
            else
            {
                retval.emplace(retval.begin(), xmirenv);
            }
        }
        else
        {
            /* If we're in a snap we need to use the utility which
               gets us back into the snap */
            std::string snappath{snapenv};

            retval.emplace(retval.begin(), snappath + SNAPPY_XMIR);
        }
    }

    /* See if we're doing apparmor by hand */
    auto appexecpolicy = findEnv("APP_EXEC_POLICY", env);
    if (!appexecpolicy.empty() && appexecpolicy != "unconfined")
    {
        retval.emplace(retval.begin(), appexecpolicy);
        retval.emplace(retval.begin(), "-p");
        retval.emplace(retval.begin(), "aa-exec");
    }

    return retval;
}

}  // namespace manager
}  // namespace jobs
}  // namespace app_launch
}  // namespace ubuntu
