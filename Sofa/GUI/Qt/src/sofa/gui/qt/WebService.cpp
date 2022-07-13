/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <iostream>
#include <thread>
#include "WebService.h"

namespace sofa
{

WebService::WebService(){}

void WebService::internalStart()
{
    server.Get("/sofa", [](const httplib::Request &, httplib::Response &res) {
        res.set_content("Sofa version: v22.06", "text/plain");

    });

    int port = server.bind_to_any_port("0.0.0.0");
    std::cout << "NOW LISTENING TO: " << port << std::endl;
    server.listen_after_bind();
}

WebService srv;
std::thread worker;
void WebService::start()
{
    new std::thread([]()
    {
        srv.internalStart();
    });
}

}
