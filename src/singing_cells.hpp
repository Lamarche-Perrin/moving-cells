/*
 * This file is part of Moving Cells.
 *
 * Moving Cells is is a digital installation building on a depth sensor to
 * allow spectators to interact with a cloud of particles through their movements.
 * It has been developed and first displayed in June 2015 by Robin Lamarche-Perrin
 * and Bruno Pace for the eponymous dance festival, in Leipzig.
 * See: http://www.movingcells.org
 * 
 * The current version of the program is implemented on Kinect for Windows v2 (K4W2)
 * through the open source driver libreenect2.
 * See: https://github.com/OpenKinect/libfreenect2
 * 
 * Copyright © 2015-2017 Robin Lamarche-Perrin and Bruno Pace
 * (<Robin.Lamarche-Perrin@lip6.fr>)
 * 
 * Moving Cells is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 * 
 * Moving Cells is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */


int testOpenAL ();
int makeSOund ();
void mouseEvents (int event, int x, int y, int flags, void *userdata);
void displayFrame ();

void sinWave (std::vector<ALshort> &samples, float amplitude, float frequency);
void addWave (std::vector<ALshort> &samples, const std::vector<ALshort> &samplesToAdd);
void amWave (std::vector<ALshort> &samples, double amplitude, double frequency);
void fmWave (std::vector<ALshort> &samples, double amplitude, double frequency, double deltaFrequency = 0);
void plotWave (std::vector<ALshort> &samples);
void plotParameters (double amplitude1, double frequency1, double amplitude2, double frequency2);
