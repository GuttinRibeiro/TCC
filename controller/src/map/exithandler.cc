/***
 * Warthog Robotics
 * University of Sao Paulo (USP) at Sao Carlos
 * http://www.warthog.sc.usp.br/
 *
 * This file is part of WRCoach project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***/

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include "exithandler.hh"
#include <iostream>
#include <signal.h>

QApplication* ExitHandler::_app = nullptr;
int ExitHandler::_counter = 0;

void ExitHandler::setup(QApplication *app) {
    ExitHandler::_app = app;

    // Setup CTRL+C hook
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ExitHandler::run;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);
}

void ExitHandler::run(int s) {
    ExitHandler::_counter++;

    switch(ExitHandler::_counter) {
        case 1: {
            std::cout << "\n[EXIT HANDLER] Closing WorldMap...\n";

            // Close application
            ExitHandler::_app->exit();
        } break;

        default:
        case 2: {
            std::cout << "\n[EXIT HANDLER] Halting WorldMap...\n";
            exit(EXIT_FAILURE);
        }
    }
}
