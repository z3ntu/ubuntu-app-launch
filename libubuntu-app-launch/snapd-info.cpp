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

#include "snapd-info.h"

#include <curl/curl.h>
#include <vector>

namespace ubuntu
{
namespace app_launch
{
namespace snapd
{

Info::Info()
{
    auto snapdEnv = g_getenv("UBUNTU_APP_LAUNCH_SNAPD_SOCKET");
    if (G_UNLIKELY(snapdEnv != nullptr))
    {
        snapdSocket = snapdEnv;
    }
    else
    {
        snapdSocket = "/run/snapd.socket";
    }

    if (g_file_test(snapdSocket.c_str(), G_FILE_TEST_EXISTS))
    {
        snapdExists = true;
    }
}

/** Gets package information out of snapd by using the REST
    interface and turning the JSON object into a C++ Struct */
std::shared_ptr<Info::PkgInfo> Info::pkgInfo(const AppID &appid) const
{
    if (!snapdExists)
    {
        return {};
    }

    try
    {
        auto snapnode = snapdJson("/v2/snap/" + appid.package.value());
        auto snapobject = json_node_get_object(snapnode.get());
        if (snapobject == nullptr)
        {
            throw std::runtime_error("Results returned by snapd were not a valid JSON object");
        }

        /******************************************/
        /* Validation of the object we got        */
        /******************************************/
        for (auto member : {"name", "status", "revision", "type", "apps", "version"})
        {
            if (!json_object_has_member(snapobject, member))
            {
                throw std::runtime_error("Snap JSON didn't have a '" + std::string(member) + "'");
            }
        }

        std::string namestr = json_object_get_string_member(snapobject, "name");
        if (namestr != appid.package.value())
        {
            throw std::runtime_error("Snapd returned information for snap '" + namestr + "' when we asked for '" +
                                     appid.package.value() + "'");
        }

        std::string statusstr = json_object_get_string_member(snapobject, "status");
        if (statusstr != "active")
        {
            throw std::runtime_error("Snap is not in the 'active' state.");
        }

        std::string typestr = json_object_get_string_member(snapobject, "type");
        if (typestr != "app")
        {
            throw std::runtime_error("Specified snap is not an application, we only support applications");
        }

        auto revision = json_object_get_int_member(snapobject, "revision");
        auto revisionstr = std::to_string(revision);
        if (revisionstr != appid.version.value())
        {
            std::string message = "Revision mismatch in request and info given. Expected: '" + appid.version.value() +
                                  "'  Given: '" + revisionstr + "'";
            if (appid.version.value() == "0")
            {
                /* We are special casing this to be a wild card for now
                   because we know that the interface code isn't returning
                   a revision, so we need to handle that. When the interface
                   interface gets fixed we can remove this special case. */
                g_warning("%s", message.c_str());
            }
            else
            {
                throw std::runtime_error(message);
            }
        }

        /******************************************/
        /* Validation complete — build the object */
        /******************************************/

        auto pkgstruct = std::make_shared<PkgInfo>();
        pkgstruct->name = namestr;
        pkgstruct->version = json_object_get_string_member(snapobject, "version");
        pkgstruct->revision = revision;

        auto apparray = json_object_get_array_member(snapobject, "apps");
        for (unsigned int i = 0; i < json_array_get_length(apparray); i++)
        {
            auto appname = json_array_get_string_element(apparray, i);
            pkgstruct->apps.emplace_back(std::string(appname));
        }

        /* TODO: Seems like snapd should give this to us */
        auto gdir = g_build_filename("snap", namestr.c_str(), revisionstr.c_str(), nullptr);
        pkgstruct->directory = gdir;
        g_free(gdir);

        return pkgstruct;
    }
    catch (std::runtime_error &e)
    {
        g_warning("Unable to get snap information for '%s': %s", std::string(appid).c_str(), e.what());
        return {};
    }
}

/** Asks the snapd process for some JSON. This function parses the basic
    response JSON that snapd returns and will error if a return code error
    is in the JSON. It then passes on the "result" part of the response
    to the caller. */
std::shared_ptr<JsonNode> Info::snapdJson(const std::string &endpoint) const
{
    /* Setup the CURL connection and suck some data */
    CURL *curl = curl_easy_init();
    if (curl == nullptr)
    {
        throw std::runtime_error("Unable to create new cURL connection");
    }

    std::vector<char> data;

    /* Configure the command */
    curl_easy_setopt(curl, CURLOPT_URL, ("http:" + endpoint).c_str());
    curl_easy_setopt(curl, CURLOPT_UNIX_SOCKET_PATH, snapdSocket.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &data);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, [](char *ptr, size_t size, size_t nmemb, void *userdata) -> size_t {
        unsigned int i;
        std::vector<char> *data = static_cast<std::vector<char> *>(userdata);
        data->reserve(data->size() + (size * nmemb)); /* allocate once */
        for (i = 0; i < size * nmemb; i++)
        {
            data->push_back(ptr[i]);
        }
        return i;
    });

    /* Run the actual request (blocking) */
    auto res = curl_easy_perform(curl);

    if (res != CURLE_OK)
    {
        curl_easy_cleanup(curl);
        throw std::runtime_error("snapd HTTP server returned an error");
    }

    curl_easy_cleanup(curl);

    /* Cool, we have data */
    auto parser = std::shared_ptr<JsonParser>(json_parser_new(), [](JsonParser *parser) { g_clear_object(&parser); });
    GError *error = nullptr;
    json_parser_load_from_data(parser.get(), /* parser */
                               data.data(),  /* array */
                               data.size(),  /* size */
                               &error);      /* error */

    if (error != nullptr)
    {
        g_warning("Can not parse! %s", error->message);
        g_error_free(error);
        throw std::runtime_error("Can not parse JSON response");
    }

    auto root = json_parser_get_root(parser.get());
    auto rootobj = json_node_get_object(root);

    if (rootobj == nullptr)
    {
        throw std::runtime_error("Root of JSON result isn't an object");
    }

    /* Check members */
    for (auto member : {"status",
                        "status-code"
                        "result",
                        "type"})
    {
        if (!json_object_has_member(rootobj, member))
        {
            throw std::runtime_error("Resulting JSON didn't have a '" + std::string(member) + "'");
        }
    }

    auto status = json_object_get_int_member(rootobj, "status-code");
    if (status != 200)
    {
        throw std::runtime_error("Status code is: " + std::to_string(status));
    }

    std::string statusstr = json_object_get_string_member(rootobj, "status");
    if (statusstr != "OK")
    {
        throw std::runtime_error("Status string is: " + statusstr);
    }

    std::string typestr = json_object_get_string_member(rootobj, "type");
    if (typestr != "sync")
    {
        throw std::runtime_error("We only support 'sync' results right now, but we got a: " + typestr);
    }

    auto result = std::shared_ptr<JsonNode>((JsonNode *)g_object_ref(json_object_get_member(rootobj, "result")),
                                            [](JsonNode *node) { g_clear_object(&node); });

    return result;
}

/** Gets all the apps that are available for a given interface. It asks snapd
    for the list of interfaces and then finds this one, turning it into a set
    of AppIDs */
std::set<AppID> Info::appsForInterface(const std::string &in_interface) const
{
    if (!snapdExists)
    {
        return {};
    }

    try
    {
        auto interfacesnode = snapdJson("/v2/interfaces");
        auto interface = json_node_get_object(interfacesnode.get());
        if (interface != nullptr)
        {
            throw std::runtime_error("Interfaces result isn't an object");
        }

        for (auto member : {"plugs", "slots"})
        {
            if (!json_object_has_member(interface, member))
            {
                throw std::runtime_error("Interface JSON didn't have a '" + std::string(member) + "'");
            }
        }

        auto slotarray = json_object_get_array_member(interface, "slots");
        std::set<AppID> appids;
        for (unsigned int i = 0; i < json_array_get_length(slotarray); i++)
        {
            auto ifaceobj = json_array_get_object_element(slotarray, i);
            try
            {
                for (auto member : {"snap", "interface", "connections"})
                {
                    if (!json_object_has_member(ifaceobj, member))
                    {
                        throw std::runtime_error("Interface JSON didn't have a '" + std::string(member) + "'");
                    }
                }

                std::string snapname = json_object_get_string_member(ifaceobj, "snap");
                /* Everything is ubuntu-core right now, in the future this will
                   change, but we'll change this code then */
                if (snapname != "ubuntu-core")
                {
                    continue;
                }

                std::string interfacename = json_object_get_string_member(ifaceobj, "interface");
                if (interfacename != in_interface)
                {
                    continue;
                }

                auto connections = json_object_get_array_member(ifaceobj, "connections");
                for (unsigned int j = 0; j < json_array_get_length(connections); j++)
                {
                    auto connectionobj = json_array_get_object_element(connections, j);

                    for (auto member : {"snap", "apps"})
                    {
                        if (!json_object_has_member(ifaceobj, member))
                        {
                            continue;
                        }
                    }

                    std::string snapname = json_object_get_string_member(connectionobj, "snap");
                    int revision = 0;  // TODO: We don't get the revision from snapd today :-(

                    auto apps = json_object_get_array_member(connectionobj, "apps");
                    for (unsigned int k = 0; k < json_array_get_length(apps); k++)
                    {
                        std::string appname = json_array_get_string_element(apps, j);

                        appids.emplace(AppID(AppID::Package::from_raw(snapname),                   /* package */
                                             AppID::AppName::from_raw(appname),                    /* appname */
                                             AppID::Version::from_raw(std::to_string(revision)))); /* version */
                    }
                }
            }
            catch (std::runtime_error &e)
            {
                /* We'll check the others even if one is bad */
                g_warning("Malformed inteface instance: %s", e.what());
                continue;
            }
        }

        return appids;
    }
    catch (std::runtime_error &e)
    {
        g_warning("Unable to get interface information: %s", e.what());
        return {};
    }

    return {};
}

}  // namespace snapd
}  // namespace app_launch
}  // namespace ubuntu
