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


#include <fstream>
#include <random>
#include <vector>
#include <list>

#include <SDL.h>
#include <opencv2/opencv.hpp>


#define VERBOSE 0
#define MILLION 1000000L
#define BILLION 1000000000L
#define PI 3.14159265


// DEFINE ENUM

#define SWITCH_ON_CLICK           0
#define KEEP_PRESSED              1

#define UNIFORM_INIT              0
#define RANDOM_INIT               1
#define DYNAMIC_INIT              2

#define NO_BORDERS                0
#define MIRROR_BORDERS            1


// PARAMETER METHODS

#define GRAVITATION_FACTOR    0
#define GRAVITATION_ANGLE     1

#define PARTICLE_DAMPING      2

#define BODY_X                3
#define BODY_Y                4
#define BODY_WEIGHT           5
#define BODY_RADIUS           6

#define PIXEL_INTENSITY       7
#define TIME_FACTOR           8

#define PARAMETER_NUMBER      9


// CLASS PREDIFINITIONS

class Cloud;
class Body;
typedef std::vector<Body*> BodyList;

struct ArgStruct {
	Cloud *cloud;
	int id;
	ArgStruct (Cloud *vCloud, int vId) : cloud (vCloud), id (vId) {};
};


// PARAMETER STRUCTURE

struct Parameter
{
public:
	int id;
	std::string name;
	std::string str;
	int scancode;
	int keycode;

	float max;
	float aadd;
	float add;
	float moy;
	float sub;
	float ssub;
	float min;
};

typedef std::vector<Parameter> ParameterVector;



// EVENT CLASSES

class Event
{
public:
	Cloud *cloud;

	double startTime;
	double currentTime;
	float duration;
	bool started = false;
	bool stopped = false;

	int parameter;

	Event (Cloud *cloud, int parameter, float duration = 0);
	virtual ~Event ();
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

	InstantaneousVariation (Cloud *cloud, int parameter, float endValue);

	void start ();
	void stop ();
	float step (float delay);
};


class LinearVariation : public Event
{
public:
	float startValue;
	float endValue;

	LinearVariation (Cloud *cloud, int parameter, float endValue, float duration);

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

	SinusoidalVariation (Cloud *cloud, int parameter, float amplitude, float frequency, float duration = 0);

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
	
	SawtoothVariation (Cloud *cloud, int parameter, float endValue, float frequency, float duration = 0);

	void start ();
	void stop ();
	float step (float delay);
};




// COLOR CLASSES

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



// FUNCTIONS

// void extractBodies (float *dPixel);
// void displaySensor (cv::Mat *depthFrame);
// void calibrateKinect (int key);

int ms_sleep (unsigned int ms);


// SIMPLE STRUCTURES

// struct Pixel
// {
//     int x; int y; int z;
//     Pixel (int cx, int cy, int cz) : x (cx), y (cy), z (cz) {}
// };


struct Body
{
public:
	int id;
	float x; float y;
	float rX; float rY;
	float weight;
	float radius;

    Body () : id (-1), x (0), y (0), rX (0), rY (0), weight (0), radius (0) {};
    Body (float vX, float vY, float vWeight) : id (-1), x (vX), y (vY), rX (0), rY (0), weight (vWeight), radius (0) {};
};


struct Particle
{
public:
	float x, y, dx, dy;
	Particle () : x (0), y (0), dx (0), dy (0) {}
};



// OTHER CLASSES

class Cloud
{
public:
	// GRAPHICS PARAMETERS
	bool displayParticles     = true;
	bool displayBodies        = true;
	bool displayParameters    = false;
	bool displayCoordinates   = false;
	bool hideMouse            = true;
	bool displayFullscreen    = true;

	bool recordParticles      = false;
	bool recordParameters     = false;
	bool readParameters       = !recordParameters;
	
	std::string inputParameterFilename  = "static-cells-input-sequence.csv";
	std::string outputParameterFilename = "static-cells-output-sequence.csv";
	std::string inoutParameterFilename  = "static-cells-inout-sequence.csv";

	float framePerSecond      = 0;
	float frameLogFrequency   = 0;
	int frameFrequency        = 0;
	int frameLimit            = 0;
	float constantDelay       = 0;

	int graphicsWidth         = 1920;
	int graphicsHeight        = 1080;
	int threadNumber          = 8;

// PHYSICS PARAMETER
	int borderMode            = MIRROR_BORDERS;
	int particleInitMode      = UNIFORM_INIT;

	int particleNumber        = 1920 * 1080 / 9;
	float particleWeight      = 1.;
	float particleDamping     = 1.;

	float gravitationFactor   = 0.;
	float gravitationAngle    = 0.;
	float timeFactor          = 1.;

	pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
	BodyList *bodyList = new BodyList ();
	BodyList *newBodyList = new BodyList ();
	Body *mouseBody = new Body ();

    // float bodyX               = 0.5;
	// float bodyY               = 0.5;
	// float bodyWeight          = 0.;
	// float bodyRadius          = 0.;

	// bool withFixedBody        = false;
	// float fixedBodyX          = 1./3;
	// float fixedBodyY          = 1./3;
	// float fixedBodyWeight     = 4;

	RgbColor particleColorMin = RgbColor (  0,   0,   0);
	RgbColor particleColorMoy = RgbColor (  3,   6,  16);
	RgbColor particleColorMax = RgbColor (255, 255, 255);

	float particleRatioMin    = 0.;
	float particleRatioMoy    = 0.5;
	float particleRatioMax    = 1000.;

	float pixelIntensity      = 1.;

// PROGRAM PARAMETERS
	static const int maxParticleNumber   = 1920 * 1080 * 4;
	static const int maxThreadNumber     = 16;
	const float maxParticleSpeed         = 1000000.;
	
// PROGRAM VARIABLES
	int graphicsFps = 0;

	std::string configFilename = "";
	std::string outputFilename = "";
	std::ifstream inputParameterFile;
	std::ofstream outputParameterFile;
	std::string inputParameterLine;

	int mouseX, mouseY;
	SDL_Window *window;
	SDL_Renderer *renderer;
	SDL_Texture *texture;
	SDL_Event event;
	ParameterVector parameters;

	bool stop;
	int pixelNumber;
	EventList events;

	int frameNb;
	int sumFrameNb;
	float delay;
	float sumDelay;
	float currentDelay;

	struct timeval startTimer;
	struct timeval endTimer;
	struct timeval parameterTimer;

// PHYSICS VARIABLES
	float rDistance;
	float rDelay;
	float rGravitationFactor;
	float rGravitationAngle;
	// float rBodyRadius;
	// float rBodyX;
	// float rBodyY;
	// float rBodyWeight;
	float rParticleDamping;
	float rPixelSize;
	float rWidthBorder;
	float rWidthBorderDoubled;
	float rHeightBorder;
	float rHeightBorderDoubled;

// PARTICLE VARIABLES
	Particle *particles;
	int *pixels;
	cv::Mat *frame;
	cv::Mat finalFrame;
	int frameIndex;
	int firstFrameIndex = 0;

	int *particleRedArray;
	int *particleGreenArray;
	int *particleBlueArray;

	float bodyLeftWeight;
	float bodyRightWeight;
	float bodyTopWeight;
	float bodyBottomWeight;

// THREAD VARIABLES
	void *status;
	pthread_attr_t attr;

	pthread_t threads [maxThreadNumber];
	int firstParticle [maxThreadNumber];
	int lastParticle [maxThreadNumber];
	int firstPixel [maxThreadNumber];
	int lastPixel [maxThreadNumber];

	Cloud ();
	~Cloud ();

	void init ();
	void setup ();
	void setupEvents ();
	void setupParameters ();
	void setupThreads ();
	void setupColor ();
	void setdown ();

	static void *run (void *arg);
	void run ();

	void initParticles (int type);
	void getTime ();

	void updateBodies ();
	void updatePhysics ();
	void computeParticles ();
	void computeFrame ();
	void displayFrame ();
	void recordFrame ();

	void addBody (Body *body);
	void clearBodies ();

	void recordParticlePositions (int index);
	void recordParticlePositions (std::string filename);
	void readParticlePositions (int index);
	void readParticlePositions (std::string filename);

	static void *updateAndMoveParticles (void *args);
	void updateAndMoveParticles (int id);

	static void *clearPixels (void *args);
	void clearPixels (int id);

	static void *applyPixels (void *args);
	void applyPixels (int id);

	void openOutputParameterFile (std::string filename);
	void writeOutputParameterFile ();
	void closeOutputParameterFile ();

	void openInputParameterFile (std::string filename);
	void readInputParameterFile ();
	void closeInputParameterFile ();

	int getParameterId (std::string name);
	float getParameter (int parameter);
	void setParameter (int parameter, float value, bool write = true);
	void addParameter (int parameter, float value, bool write = true);
};




