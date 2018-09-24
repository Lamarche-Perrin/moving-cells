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

#include <random>

// DEFINE ENUM

#define MILLION 1000000L;
#define PI 3.14159265

#define SWITCH_ON_CLICK           0
#define KEEP_PRESSED              1

#define UNIFORM_INIT              0
#define RANDOM_INIT               1

#define MIRROR_BORDERS            0
#define NO_BORDERS                1


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
void setupEvents ();
void setupParameters ();
void setupStencil ();
void setupDistribution ();
void setupScreens ();
void setupThreads ();
void setupColor ();
void setupPhysics ();

void initParticles (int type);
void getTime ();

void computeBodies ();
void computeDistributedBodies ();
void computeParticles ();
void draw ();
void display ();
void record ();
//void readConfigFile (std::string configFile);
int ms_sleep (unsigned int ms);

void saveConfig (int index);
void loadConfig (int index);
void loadConfig (std::string filename);

void *loop (void *arg);
	
void *updateParticles (void *arg);
void *updateParticlesWithDistribution (void *arg);
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
    float minDist;

    int xMin, xMoy, xMax, yMin, yMoy, yMax, zMin, zMoy, zMax, pixelNb;
    int xMinS, xMoyS, xMaxS, yMinS, yMoyS, yMaxS, zMinS, zMoyS, zMaxS;

    Body ();
    void getClosestBody (BodyList *list);
    void update (float delay);
    float getDistance (Body *body);
    void print ();
};


class Particle
{
public:
	bool alive;
	float px, py, dx, dy;
	cv::Point body;
	Particle ();

	void update ();
	void updateWithDistribution ();
	void move ();
	void apply ();
};



// DEFINE PARAMETER METHODS

#define PARTICLE_DAMPING    0
#define PARTICLE_SPEED      1

#define GRAVITATION_FACTOR  2
#define GRAVITATION_ANGLE   3

#define BODY_WEIGHT         4
#define BODY_RADIUS         5
#define BODY_ATTRACT_FACTOR 6
#define BODY_REPEL_FACTOR   7

#define PIXEL_INTENSITY     8

#define PARAMETER_NUMBER    9


struct Parameter
{
public:
	int id;
	std::string name;
	int scancode;
	int keycode;

	float max;
	float aadd;
	float add;
	float moy;
	float sub;
	float ssub;
	float min;

	bool physics;
};

typedef std::vector<Parameter> ParameterVector;


float getParameterValue (int parameter);
void setParameterValue (int parameter, float value);
void addParameterValue (int parameter, float value);


// DEFINE EVENT CLASSES

class Event
{
public:
	double startTime;
	double currentTime;
	float duration;
	bool started = false;
	bool stopped = false;

	int parameter;

	Event (int parameter, float duration = 0);
	bool hasStarted ();
	bool hasStopped ();

	virtual void start ();
	virtual void stop ();
	virtual float step (float delay);
};


class EventList
{
public:
	std::vector<std::list<Event*>> events;

	EventList ();

	void clear (int parameter);
	void push_back (Event *event);
	void interrupt (Event *event);

	float step (float delay);
};


typedef std::vector<EventList> EventLists;



class InstantaneousVariation : public Event
{
public:
	float endValue;

	InstantaneousVariation (int parameter, float endValue);

	void start ();
	void stop ();
	float step (float delay);
};


class LinearVariation : public Event
{
public:
	float startValue;
	float endValue;

	LinearVariation (int parameter, float endValue, float duration);

	void start ();
	void stop ();
	float step (float delay);
};


class SinusoidalVariation : public Event
{
public:
	float startValue;
	float endValue;
	float amplitude;
	float frequency;

	SinusoidalVariation (int parameter, float amplitude, float frequency, float duration = 0);

	void start ();
	void stop ();
	float step (float delay);
};



class SawtoothVariation : public Event
{
public:
	float startValue;
	float endValue;
	float frequency;
	double intermediateTime;
	
	SawtoothVariation (int parameter, float endValue, float frequency, float duration = 0);

	void start ();
	void stop ();
	float step (float delay);
};



// DEFINE SCREEN CLASS

struct Screen {
	char i;
	bool active;
	float time;
	int delay;
	
	int sx, sy, sdx, sdy;
	int cx, cy, cdx, cdy;
	cv::Rect srect, crect;
	cv::Rect srectp, crectp;
	
	Screen (char vi, int vx, int vy, int vdx, int vdy, int vdelay = 0);

	void set ();
	void set (int vx, int vy, int vdx, int vdy, int vdelay = 0);
	void unset ();
	void swap ();
};

typedef std::vector<Screen> ScreenVector;


// DEFINE COLOR CLASSES

typedef struct RgbColor
{
	RgbColor () {};
	RgbColor (unsigned char vr, unsigned char vg, unsigned char vb) : r (vr), g (vg), b (vb) {};
	unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
	HsvColor () {};
	HsvColor (unsigned char vh, unsigned char vs, unsigned char vv) : h (vh), s (vs), v (vv) {};
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6; 

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
	case 0:
		rgb.r = hsv.v; rgb.g = t; rgb.b = p;
		break;
	case 1:
		rgb.r = q; rgb.g = hsv.v; rgb.b = p;
		break;
	case 2:
		rgb.r = p; rgb.g = hsv.v; rgb.b = t;
		break;
	case 3:
		rgb.r = p; rgb.g = q; rgb.b = hsv.v;
		break;
	case 4:
		rgb.r = t; rgb.g = p; rgb.b = hsv.v;
		break;
	default:
		rgb.r = hsv.v; rgb.g = p; rgb.b = q;
		break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}

