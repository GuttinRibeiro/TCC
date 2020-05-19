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

#ifndef WRTIMER_HH_
#define WRTIMER_HH_

#include <ctime>

class WRTimer {
public:
    WRTimer() { }

    void start() {
        clock_gettime(CLOCK_REALTIME, &time1);
    }
    void stop() {
        clock_gettime(CLOCK_REALTIME, &time2);
    }
    double timesec() {
        return timensec()/1E9;
    }
    double timemsec()	{
        return timensec()/1E6;
    }
    double timeusec()	{
        return timensec()/1E3;
    }
    double timensec()	{
        return (time2.tv_sec*1E9 + time2.tv_nsec) - (time1.tv_sec*1E9 + time1.tv_nsec);
    }
private:
    timespec time1, time2;
};

#endif // WRTIMER_HH_
