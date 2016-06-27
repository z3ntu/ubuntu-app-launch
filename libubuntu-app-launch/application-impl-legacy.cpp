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

#include "application-impl-legacy.h"
#include "application-info-desktop.h"
#include "registry-impl.h"

#include <regex>

namespace ubuntu
{
namespace app_launch
{
namespace app_impls
{

std::pair<std::string, std::shared_ptr<GKeyFile>> keyfileForApp(const AppID::AppName& name);

void clear_keyfile(GKeyFile* keyfile)
{
    if (keyfile != nullptr)
    {
        g_key_file_free(keyfile);
    }
}

Legacy::Legacy(const AppID::AppName& appname,
               const std::string& basedir,
               const std::shared_ptr<GKeyFile>& keyfile,
               const std::shared_ptr<Registry>& registry)
    : Base(registry)
    , _appname(appname)
    , _basedir(basedir)
    , _keyfile(keyfile)
{
    if (!_keyfile)
        throw std::runtime_error{"Unable to find keyfile for legacy application: " + appname.value()};
}

Legacy::Legacy(const AppID::AppName& appname, const std::shared_ptr<Registry>& registry)
    : Base(registry)
    , _appname(appname)
{
    std::tie(_basedir, _keyfile) = keyfileForApp(appname);

    if (!_keyfile)
        throw std::runtime_error{"Unable to find keyfile for legacy application: " + appname.value()};
}

std::pair<std::string, std::shared_ptr<GKeyFile>> keyfileForApp(const AppID::AppName& name)
{
    std::string desktopName = name.value() + ".desktop";
    auto keyfilecheck = [desktopName](const std::string& dir) -> std::shared_ptr<GKeyFile> {
        auto fullname = g_build_filename(dir.c_str(), "applications", desktopName.c_str(), nullptr);
        if (!g_file_test(fullname, G_FILE_TEST_EXISTS))
        {
            g_free(fullname);
            return {};
        }

        auto keyfile = std::shared_ptr<GKeyFile>(g_key_file_new(), clear_keyfile);

        GError* error = nullptr;
        g_key_file_load_from_file(keyfile.get(), fullname, G_KEY_FILE_NONE, &error);
        g_free(fullname);

        if (error != nullptr)
        {
            g_debug("Unable to load keyfile '%s' becuase: %s", desktopName.c_str(), error->message);
            g_error_free(error);
            return {};
        }

        return keyfile;
    };

    std::string basedir = g_get_user_data_dir();
    auto retval = keyfilecheck(basedir);

    auto systemDirs = g_get_system_data_dirs();
    for (auto i = 0; !retval && systemDirs[i] != nullptr; i++)
    {
        basedir = systemDirs[i];
        retval = keyfilecheck(basedir);
    }

    return std::make_pair(basedir, retval);
}

std::shared_ptr<Application::Info> Legacy::info()
{
    if (!appinfo_)
    {
        appinfo_ = std::make_shared<app_info::Desktop>(_keyfile, _basedir, _registry, true);
    }
    return appinfo_;
}

const std::regex desktop_remover("^(.*)\\.desktop$");

std::list<std::shared_ptr<Application>> Legacy::list(const std::shared_ptr<Registry>& registry)
{
    std::list<std::shared_ptr<Application>> list;
    GList* head = g_app_info_get_all();
    for (GList* item = head; item != nullptr; item = g_list_next(item))
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

        auto fileappid = g_desktop_app_info_get_string(appinfo, "x-Ubuntu-UAL-Application-ID");
        if (fileappid != nullptr)
        {
            g_free(fileappid);
            continue;
        }

        try
        {
            auto app = std::make_shared<Legacy>(AppID::AppName::from_raw(appname), registry);
            list.push_back(app);
        }
        catch (std::runtime_error& e)
        {
            g_debug("Unable to create application for legacy appname: %s", appname.c_str());
        }
    }

    g_list_free_full(head, g_object_unref);

    return list;
}

std::vector<std::shared_ptr<Application::Instance>> Legacy::instances()
{
    std::vector<std::shared_ptr<Instance>> vect;
    auto startsWith = std::string(appId()) + "-";

    for (auto instance : _registry->impl->upstartInstancesForJob("application-legacy"))
    {
        g_debug("Looking at legacy instance: %s", instance.c_str());
        if (std::equal(startsWith.begin(), startsWith.end(), instance.begin()))
        {
            vect.emplace_back(std::make_shared<UpstartInstance>(appId(), "application-legacy", instance, _registry));
        }
    }

    g_debug("Legacy app '%s' has %d instances", std::string(appId()).c_str(), int(vect.size()));

    return vect;
}

std::shared_ptr<Application::Instance> Legacy::launch(const std::vector<Application::URL>& urls)
{
    return UpstartInstance::launch(appId(), "application-legacy", std::string(appId()) + "-", urls, _registry,
                                   UpstartInstance::launchMode::STANDARD,
                                   []() -> std::list<std::pair<std::string, std::string>> { return {}; });
}

std::shared_ptr<Application::Instance> Legacy::launchTest(const std::vector<Application::URL>& urls)
{
    return UpstartInstance::launch(appId(), "application-legacy", std::string(appId()) + "-", urls, _registry,
                                   UpstartInstance::launchMode::TEST,
                                   []() -> std::list<std::pair<std::string, std::string>> { return {}; });
}

};  // namespace app_impls
};  // namespace app_launch
};  // namespace ubuntu
