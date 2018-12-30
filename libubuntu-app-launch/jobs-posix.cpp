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

#include "jobs-posix.h"
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

class POSIX : public instance::Base
{
    friend class manager::POSIX;

public:
    explicit POSIX(const AppID& appId,
                     const std::string& job,
                     const std::string& instance,
                     const std::vector<Application::URL>& urls,
                     const std::shared_ptr<Registry::Impl>& registry);
    virtual ~POSIX()
    {
        g_debug("Destroying a POSIX for '%s' instance '%s'", std::string(appId_).c_str(), instance_.c_str());
    }

    /* Query lifecycle */
    pid_t primaryPid() override;
    std::vector<pid_t> pids() override;

    /* Manage lifecycle */
    void stop() override;

};  // class POSIX

POSIX::POSIX(const AppID& appId,
                 const std::string& job,
                 const std::string& instance,
                 const std::vector<Application::URL>& urls,
                 const std::shared_ptr<Registry::Impl>& registry)
    : Base(appId, job, instance, urls, registry)
{
    g_debug("Creating a new POSIX for '%s' instance '%s'", std::string(appId).c_str(), instance.c_str());
}

pid_t POSIX::primaryPid()
{
    auto manager = std::dynamic_pointer_cast<manager::POSIX>(registry_->jobs());
    return manager->unitPrimaryPid(appId_, job_, instance_);
}

std::vector<pid_t> POSIX::pids()
{
    auto manager = std::dynamic_pointer_cast<manager::POSIX>(registry_->jobs());
    return manager->unitPids(appId_, job_, instance_);
}

void POSIX::stop()
{
    auto manager = std::dynamic_pointer_cast<manager::POSIX>(registry_->jobs());
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

POSIX::POSIX(const std::shared_ptr<Registry::Impl>& registry)
    : Base(registry)
{
    // TODO STUB
    g_debug("POSIX constructor stub called...");
}

POSIX::~POSIX()
{
}

void POSIX::copyEnv(const std::string& envname, std::list<std::pair<std::string, std::string>>& env)
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

void POSIX::copyEnvByPrefix(const std::string& prefix, std::list<std::pair<std::string, std::string>>& env)
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

std::shared_ptr<Application::Instance> POSIX::launch(
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
    return reg->thread.executeOnThread<std::shared_ptr<instance::POSIX>>([&]() -> std::shared_ptr<instance::POSIX> {
        auto manager = std::dynamic_pointer_cast<manager::POSIX>(reg->jobs());
        std::string appIdStr{appId};
        g_debug("Initializing params for an new instance::POSIX for: %s", appIdStr.c_str());

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
             {"APP_DIR", "APP_URIS", "APP_EXEC", "APP_EXEC_POLICY", "APP_LAUNCHER_PID",
              "INSTANCE_ID", "MIR_SERVER_PLATFORM_PATH", "MIR_SERVER_PROMPT_FILE", "MIR_SERVER_HOST_SOCKET",
              "UBUNTU_APP_LAUNCH_OOM_HELPER", "UBUNTU_APP_LAUNCH_LEGACY_ROOT"})
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

        auto retval = std::make_shared<instance::POSIX>(appId, job, instance, urls, reg);

        tracepoint(ubuntu_app_launch, handshake_wait, appIdStr.c_str());
        starting_handshake_wait(handshake);
        tracepoint(ubuntu_app_launch, handshake_complete, appIdStr.c_str());

        // From https://stackoverflow.com/a/7026414
        g_debug("PREPARING ARGV...");
        std::vector<char*> argv;
        for(auto loop = commands.begin(); loop != commands.end(); ++loop)
        {
            argv.push_back(&(*loop)[0]);
            g_debug(argv.back());
        }
        argv.push_back(NULL);

        g_debug("PREPARING ENVP...");
        std::vector<std::string> envp_str;
        std::vector<char*> envp;
        for(auto loop = env.begin(); loop != env.end(); ++loop)
        {
            std::string envStr;
            envStr += (*loop).first;
            envStr += "=";
            envStr += (*loop).second;
            envp_str.push_back(envStr);
            envp.push_back(&envp_str.back()[0]);
        }
        envp.push_back(NULL);

        std::cout << "printing members of envp:" << std::endl;
        for(char* n : envp) {
            std::cout << n << std::endl;
        }

        pid_t child_pid = fork();
        if(child_pid == 0) {
            /* Call the job start function */
            g_debug("argv[0]: %s", argv[0]);
            execvpe(argv[0], &argv[0], &envp[0]);

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

std::shared_ptr<Application::Instance> POSIX::existing(const AppID& appId,
                                                         const std::string& job,
                                                         const std::string& instance,
                                                         const std::vector<Application::URL>& urls)
{
    return std::make_shared<instance::POSIX>(appId, job, instance, urls, getReg());
}

std::vector<std::shared_ptr<instance::Base>> POSIX::instances(const AppID& appID, const std::string& job)
{
    // TODO: STUB
    g_debug("instances stub called...");
    auto reg = getReg();

    std::vector<std::shared_ptr<instance::Base>> instances;
    std::vector<Application::URL> urls;
    std::string instance = "somestring";

    instances.emplace_back(std::make_shared<instance::POSIX>(appID, job, instance, urls, reg));
    g_debug("Found %d instances for AppID '%s'", int(instances.size()), std::string(appID).c_str());

    return instances;
}

std::list<std::string> POSIX::runningAppIds(const std::list<std::string>& allJobs)
{
    // TODO: STUB
    g_debug("runningAppIds stub called...");
    std::set<std::string> appids;
    return {appids.begin(), appids.end()};
}

pid_t POSIX::unitPrimaryPid(const AppID& appId, const std::string& job, const std::string& instance)
{
    // TODO: STUB
    g_debug("unitPrimaryPid stub called...");
    return 0;
}

std::vector<pid_t> POSIX::unitPids(const AppID& appId, const std::string& job, const std::string& instance)
{
    // TODO: STUB
    g_debug("unitPids stub called...");
    return launchedpids;
}

void POSIX::stopUnit(const AppID& appId, const std::string& job, const std::string& instance)
{
    // TODO: STUB
    g_debug("stopUnit stub called...");
}

core::Signal<const std::string&, const std::string&, const std::string&>& POSIX::jobStarted()
{
    /* Ensure we're connecting to the signals */
    return sig_jobStarted;
}

core::Signal<const std::string&, const std::string&, const std::string&>& POSIX::jobStopped()
{
    /* Ensure we're connecting to the signals */
    return sig_jobStopped;
}

core::Signal<const std::string&, const std::string&, const std::string&, Registry::FailureType>& POSIX::jobFailed()
{
    // TODO: STUB
    g_debug("jobFailed stub called...");
    return sig_jobFailed;
}

std::string POSIX::findEnv(const std::string& value, std::list<std::pair<std::string, std::string>>& env)
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

void POSIX::removeEnv(const std::string& value, std::list<std::pair<std::string, std::string>>& env)
{
    auto entry = std::find_if(env.begin(), env.end(),
                              [&value](std::pair<std::string, std::string>& entry) { return entry.first == value; });

    if (entry != env.end())
    {
        env.erase(entry);
    }
}

int POSIX::envSize(std::list<std::pair<std::string, std::string>>& env)
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

std::vector<std::string> POSIX::parseExec(std::list<std::pair<std::string, std::string>>& env)
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
