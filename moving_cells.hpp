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



// DEFINE ENUM

#define NO_BORDER                 0
#define CYCLIC_BORDER             1
#define MIRROR_BORDER             2

#define UNIFORM_INIT              0
#define RANDOM_INIT               1

#define SYMMETRIC_GRAVITATION     0
#define QUADRANT_GRAVITATION      1
#define LINEAR_GRAVITATION        2
#define EXPONENTIAL_GRAVITATION   3


// PRE-DEFINITIONS

struct Body;
class Particles;
typedef std::list<Body*> BodyList;


// FUNCTIONS

void extractBodies (float *dPixel);
void displaySensor (cv::Mat *depthFrame);
void calibrateKinect (int key);

void sigint_handler (int s);
int scale (float z);
float linearMap (float value, float min1, float max1, float min2, float max2);

void loop ();
void setup ();
void initParticles (int type);
void getTime ();
void computeBodies ();
void computeParticles ();
void draw ();
int ms_sleep (unsigned int ms);

void *loop (void *arg);
	
void *updateParticles (void *arg);
void *moveParticles (void *arg);
void *applyParticles (void *arg);

void *clearPixels (void *arg);
void *applyPixels (void *arg);


// CLASSES

struct Pixel
{
    int x; int y; int z;
    Pixel (int cx, int cy, int cz) : x(cx), y(cy), z(cz) {}
};


struct Body
{
public:	
    int index;
    Body *closestBody;
    double minDist;

    int xMin, xMoy, xMax, yMin, yMoy, yMax, zMin, zMoy, zMax, pixelNb;
    int xMinS, xMoyS, xMaxS, yMinS, yMoyS, yMaxS, zMinS, zMoyS, zMaxS;

    Body ();
    void getClosestBody (BodyList *list);
    void update (double delay);
    double getDistance (Body *body);
    void print ();
};


class Particle
{
public:
	float px, py, dx, dy;
	Particle ();

	void update ();
	void move ();
	void apply ();
};

