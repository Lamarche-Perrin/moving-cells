
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

