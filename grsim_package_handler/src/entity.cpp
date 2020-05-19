/***
 * Warthog Robotics
 * University of Sao Paulo (USP) at Sao Carlos
 * http://www.warthog.sc.usp.br/
 *
 * This file is part of WREye project.
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

#include "entity.hpp"
#include <iostream>

Entity::Entity(QObject *parent) : QThread(parent) {
    _running = true; // set running by default
    _loopTime = 10;
}

void Entity::run() {
    initialization();

    WRTimer t;
    while(isRunning())	{
        t.start();
        loop();
        t.stop();

        if(loopTime()>0) {
            long rest = loopTime() - t.timemsec();
            if(rest >= 0)
                msleep(rest);
            else
                std::cout << "[TIMER OVEREXTENDED] " << name().toStdString() << " for " <<  -rest  << " ms.\n";
        }
    }

    finalization();
}

void Entity::restart() {
    if(isRunning())
        return;

    _mutexRunning.lock();
    _running = true;
    _mutexRunning.unlock();

    this->start();
}

bool Entity::isRunning() {
    bool result;

    _mutexRunning.lock();
    result = _running;
    _mutexRunning.unlock();

    return result;
}

void Entity::stopRunning() {
    _mutexRunning.lock();
    _running = false;
    _mutexRunning.unlock();
}

void Entity::setLoopTime(int t) {
    _mutexLoopTime.lock();
    _loopTime = t;
    _mutexLoopTime.unlock();
}

void Entity::setLoopFrequency(int hz) {
    _mutexLoopTime.lock();
    if(hz==0)
        _loopTime = 0;
    else
        _loopTime = 1000/hz;
    _mutexLoopTime.unlock();
}

int Entity::loopTime() {
    _mutexLoopTime.lock();
    int result = _loopTime;
    _mutexLoopTime.unlock();

    return result;
}

int Entity::loopFrequency() {
    _mutexLoopTime.lock();
    int result = 1000/_loopTime;
    _mutexLoopTime.unlock();

    return result;
}
