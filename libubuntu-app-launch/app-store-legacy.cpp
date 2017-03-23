/*
 * Copyright © 2017 Canonical Ltd.
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

#include "app-store-legacy.h"
#include "application-impl-legacy.h"
#include "registry-impl.h"
#include "string-util.h"

#include <regex>

namespace ubuntu
{
namespace app_launch
{
namespace app_store
{

Legacy::Legacy()
{
}

Legacy::~Legacy()
{
}

/** Checks the AppID by ensuring the version and package are empty
    then looks for the application.

    \param appid AppID to check
    \param registry persistent connections to use
*/
bool Legacy::hasAppId(const AppID& appid, const std::shared_ptr<Registry>& registry)
{
    try
    {
        if (!appid.version.value().empty())
        {
            return false;
        }

        return verifyAppname(appid.package, appid.appname, registry);
    }
    catch (std::runtime_error& e)
    {
        return false;
    }
}

/** Ensure the package is empty

    \param package Container name
    \param registry persistent connections to use
*/
bool Legacy::verifyPackage(const AppID::Package& package, const std::shared_ptr<Registry>& registry)
{
    return package.value().empty();
}

/** Looks for an application by looking through the system and user
    application directories to find the desktop file.

    \param package Container name
    \param appname Application name to look for
    \param registry persistent connections to use
*/
bool Legacy::verifyAppname(const AppID::Package& package,
                           const AppID::AppName& appname,
                           const std::shared_ptr<Registry>& registry)
{
    if (!verifyPackage(package, registry))
    {
        throw std::runtime_error{"Invalid Legacy package: " + std::string(package)};
    }

    auto desktop = std::string(appname) + ".desktop";
    auto evaldir = [&desktop](const gchar* dir) {
        auto fulldir = unique_gchar(g_build_filename(dir, "applications", desktop.c_str(), nullptr));
        gboolean found = g_file_test(fulldir.get(), G_FILE_TEST_EXISTS);
        return found == TRUE;
    };

    if (evaldir(g_get_user_data_dir()))
    {
        return true;
    }

    auto&& data_dirs = g_get_system_data_dirs();
    for (int i = 0; data_dirs[i] != nullptr; i++)
    {
        if (evaldir(data_dirs[i]))
        {
            return true;
        }
    }

    return false;
}

/** We don't really have a way to implement this for Legacy, any
    search wouldn't really make sense. We just throw an error.

    \param package Container name
    \param card Application search paths
    \param registry persistent connections to use
*/
AppID::AppName Legacy::findAppname(const AppID::Package& package,
                                   AppID::ApplicationWildcard card,
                                   const std::shared_ptr<Registry>& registry)
{
    throw std::runtime_error("Legacy apps can't be discovered by package");
}

/** Function to return an empty string

    \param package Container name (unused)
    \param appname Application name (unused)
    \param registry persistent connections to use (unused)
*/
AppID::Version Legacy::findVersion(const AppID::Package& package,
                                   const AppID::AppName& appname,
                                   const std::shared_ptr<Registry>& registry)
{
    return AppID::Version::from_raw({});
}

static const std::regex desktop_remover("^(.*)\\.desktop$");

std::list<std::shared_ptr<Application>> Legacy::list(const std::shared_ptr<Registry>& registry)
{
    std::list<std::shared_ptr<Application>> list;
    std::unique_ptr<GList, decltype(&g_list_free)> head(g_app_info_get_all(),
                                                        [](GList* l) { g_list_free_full(l, g_object_unref); });
    for (GList* item = head.get(); item != nullptr; item = g_list_next(item))
    {
        GDesktopAppInfo* appinfo = G_DESKTOP_APP_INFO(item->data);

        if (appinfo == nullptr)
        {
            continue;
        }

        if (g_app_info_should_show(G_APP_INFO(appinfo)) == FALSE)
        {
            continue;
        }

        auto desktopappid = std::string(g_app_info_get_id(G_APP_INFO(appinfo)));
        std::string appname;
        std::smatch match;
        if (std::regex_match(desktopappid, match, desktop_remover))
        {
            appname = match[1].str();
        }
        else
        {
            continue;
        }

        /* Remove entries generated by the desktop hook in .local */
        if (g_desktop_app_info_has_key(appinfo, "X-Ubuntu-Application-ID"))
        {
            continue;
        }

        try
        {
            auto app = std::make_shared<app_impls::Legacy>(AppID::AppName::from_raw(appname), registry);
            list.push_back(app);
        }
        catch (std::runtime_error& e)
        {
            g_debug("Unable to create application for legacy appname '%s': %s", appname.c_str(), e.what());
        }
    }

    return list;
}

std::shared_ptr<app_impls::Base> Legacy::create(const AppID& appid, const std::shared_ptr<Registry>& registry)
{
    return std::make_shared<app_impls::Legacy>(appid.appname, registry);
}

/** Turns a directory changed event from a file monitor into an
 *  internal signal. Makes sure we can deal with it first, and
 *  then propegates up the stack. */
void Legacy::directoryChanged(GFile* file, GFileMonitorEvent type)
{
    if (g_file_query_file_type(file, G_FILE_QUERY_INFO_NONE, nullptr) != G_FILE_TYPE_REGULAR)
    {
        return;
    }

    auto cdesktopname = unique_gchar(g_file_get_basename(file));
    if (!cdesktopname)
        return;
    std::string desktopname{cdesktopname.get()};

    std::string appname;
    std::smatch match;
    if (std::regex_match(desktopname, match, desktop_remover))
    {
        appname = match[1].str();
    }
    else
    {
        return;
    }

    switch (type)
    {
        case G_FILE_MONITOR_EVENT_CREATED:
        {
            auto app = std::make_shared<app_impls::Legacy>(AppID::AppName::from_raw(appname), getReg());

            appAdded_(app);
            break;
        }
        case G_FILE_MONITOR_EVENT_CHANGED:
        {
            auto app = std::make_shared<app_impls::Legacy>(AppID::AppName::from_raw(appname), getReg());

            infoChanged_(app);
            break;
        }
        case G_FILE_MONITOR_EVENT_DELETED:
        {
            AppID appid{AppID::Package::from_raw({}), AppID::AppName::from_raw(appname), AppID::Version::from_raw({})};

            appRemoved_(appid);
            break;
        }
        default:
            break;
    };
}

/** Function that setups file monitors on all of the system application
 *  directories and the user application directory. Any time an application
 *  is added or removed or changed we send the appropriate signal up the
 *  stack. */
void Legacy::setupMonitors()
{
    std::call_once(monitorsSetup_, [this]() {
        auto reg = getReg();

        monitors_ =
            reg->impl->thread.executeOnThread<std::set<std::unique_ptr<GFileMonitor, unity::util::GObjectDeleter>>>(
                [this]() {
                    std::set<std::unique_ptr<GFileMonitor, unity::util::GObjectDeleter>> monitors;

                    auto monitorDir = [this](const gchar* dirname) {
                        auto appdir = unique_gchar(g_build_filename(dirname, "applications", nullptr));
                        auto gfile = unity::util::unique_gobject(g_file_new_for_path(appdir.get()));

                        if (!g_file_query_exists(gfile.get(), nullptr))
                        {
                            throw std::runtime_error("Directory doesn't exist");
                        }

                        if (g_file_query_file_type(gfile.get(), G_FILE_QUERY_INFO_NONE, nullptr) !=
                            G_FILE_TYPE_DIRECTORY)
                        {
                            throw std::runtime_error("Not a directory");
                        }

                        GError* error = nullptr;
                        auto monitor = unity::util::unique_gobject(
                            g_file_monitor_directory(gfile.get(), G_FILE_MONITOR_NONE, nullptr, &error));

                        if (error != nullptr)
                        {
                            std::string message = std::string{"Unable to create file monitor: "} + error->message;
                            g_error_free(error);
                            throw std::runtime_error{message};
                        }

                        g_signal_connect(monitor.get(), "changed",
                                         G_CALLBACK(+[](GFileMonitor*, GFile* file, GFile*, GFileMonitorEvent type,
                                                        gpointer user_data) {
                                             auto pthis = static_cast<Legacy*>(user_data);
                                             pthis->directoryChanged(file, type);
                                         }),
                                         this);

                        return monitor;
                    };

                    auto dirs = g_get_system_data_dirs();
                    for (int i = 0; dirs != nullptr && dirs[i] != nullptr; i++)
                    {
                        try
                        {
                            monitors.insert(monitorDir(dirs[i]));
                        }
                        catch (std::runtime_error& e)
                        {
                            g_debug("Unable to create directory monitor for system dir '%s': %s", dirs[i], e.what());
                        }
                    }

                    try
                    {
                        monitors.insert(monitorDir(g_get_user_data_dir()));
                    }
                    catch (std::runtime_error& e)
                    {
                        g_debug("Unable to create directory monitor for user data dir: %s", e.what());
                    }

                    return monitors;
                });
    });
}

/** Return the signal object, but make sure we have the
 *  monitors setup first */
core::Signal<const std::shared_ptr<Application>&>& Legacy::infoChanged()
{
    setupMonitors();
    return infoChanged_;
}

/** Return the signal object, but make sure we have the
 *  monitors setup first */
core::Signal<const std::shared_ptr<Application>&>& Legacy::appAdded()
{
    setupMonitors();
    return appAdded_;
}

/** Return the signal object, but make sure we have the
 *  monitors setup first */
core::Signal<const AppID&>& Legacy::appRemoved()
{
    setupMonitors();
    return appRemoved_;
}

}  // namespace app_store
}  // namespace app_launch
}  // namespace ubuntu
