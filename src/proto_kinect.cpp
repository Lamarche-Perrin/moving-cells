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

#define CL_TARGET_OPENCL_VERSION 220

#include <unistd.h>
#include <sys/time.h>

#include "kinect.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <random>
#include <vector>
#include <chrono>
#include <math.h>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/compute/function.hpp>
#include <boost/compute/types/builtin.hpp>
#include <boost/compute/container/vector.hpp>
#include <boost/compute/algorithm/copy.hpp>
#include <boost/compute/algorithm/transform.hpp>
#include <boost/compute/algorithm/max_element.hpp>
#include <boost/compute/system.hpp>
#include <boost/compute/interop/opencv/core.hpp>
#include <boost/compute/interop/opencv/highgui.hpp>
#include <boost/compute/utility/source.hpp>

namespace cl = boost::compute;
typedef std::vector<float> floats;

#define PI 3.14159265
#define SEED 2
#define withKinect true

std::string doDisplay =      "dye";

float speedAdditionRadius =       15.0;
float speedAdditionQuant =       200.;

float dyeAdditionRadius =          6.;
float dyeAdditionQuant =           0.9;

float envAdditionRadius =        120.;
float envAdditionQuant =           0.990;
float envSubstractionQuant =       0.010;

float colorAdditionQuant =         0.900;
float colorSubstractionQuant =     0.030;

std::string feedAxis = "y";
float varFeedMin = 0.50;
float varFeedMax = 0.30;
float valFeedMin = 0.020;
float valFeedMax = 0.034;

std::string killAxis = "x"; //"xz"; //"x";
float varKillMin = 0.10; //0.02; //0.12;
float varKillMax = 0.33; //0.04; //0.23;
float valKillMin = 0.064;
float valKillMax = 0.048;

std::string IntensityAxis = "z";
float varIntensityMin = 0.05;
float varIntensityMax = 0.20;
float valIntensityMin = 0.50;
float valIntensityMax = 0.25;

float varColorMin = 0.02;
float varColorMax = 2.00;
float valColorMin = 1.0;
float valColorMax = -0.2;


#define width       1024/2
#define height      768/2
// #define width       1280/2 //1920/4
// #define height      720/2 //1080/4

int picWidth  = 500;
int picHeight  = 500;
int picTarget = 10;
float picMaxX = 1;
float picMaxY = 1;
float picMaxZ = 4;

class Coord
{
public:
  float x = 0; float y = 0; float z = 0;
  Coord () {};
  Coord (float _x, float _y, float _z) : x (_x), y (_y), z (_z) {};
  Coord (const Coord &c) : x (c.x), y (c.y), z (c.z) {};

  Coord &operator+= (double a) { x += a; y += a; z += a; return *this; }
  Coord &operator-= (double a) { x -= a; y -= a; z -= a; return *this; }
  Coord &operator*= (double a) { x *= a; y *= a; z *= a; return *this; }
  Coord &operator/= (double a) { x /= a; y /= a; z /= a; return *this; }

  Coord &operator+= (const Coord &c) { x += c.x; y += c.y; z += c.z; return *this; }
  Coord &operator-= (const Coord &c) { x -= c.x; y -= c.y; z -= c.z; return *this; }
  Coord &operator*= (const Coord &c) { x *= c.x; y *= c.y; z *= c.z; return *this; }
  Coord &operator/= (const Coord &c) { x /= c.x; y /= c.y; z /= c.z; return *this; }
};

Coord operator+ (double a, const Coord &c) { return Coord (c.x + a, c.y + a, c.z + a); }
Coord operator- (double a, const Coord &c) { return Coord (c.x - a, c.y - a, c.z - a); }
Coord operator* (double a, const Coord &c) { return Coord (c.x * a, c.y * a, c.z * a); }
Coord operator/ (double a, const Coord &c) { return Coord (c.x / a, c.y / a, c.z / a); }

Coord operator+ (const Coord &c, double a) { return Coord (c.x + a, c.y + a, c.z + a); }
Coord operator- (const Coord &c, double a) { return Coord (c.x - a, c.y - a, c.z - a); }
Coord operator* (const Coord &c, double a) { return Coord (c.x * a, c.y * a, c.z * a); }
Coord operator/ (const Coord &c, double a) { return Coord (c.x / a, c.y / a, c.z / a); }

Coord operator+ (const Coord &c1, const Coord &c2) { return Coord (c1.x + c2.x, c1.y + c2.y, c1.z + c2.z); }
Coord operator- (const Coord &c1, const Coord &c2) { return Coord (c1.x - c2.x, c1.y - c2.y, c1.z - c2.z); }
Coord operator* (const Coord &c1, const Coord &c2) { return Coord (c1.x * c2.x, c1.y * c2.y, c1.z * c2.z); }
Coord operator/ (const Coord &c1, const Coord &c2) { return Coord (c1.x / c2.x, c1.y / c2.y, c1.z / c2.z); }

Coord pow (const Coord &c, double exponent) { return Coord (pow (c.x, exponent), pow (c.y, exponent), pow (c.z, exponent)); }

class Data
{
public:
  float time = 0;
  Coord mean;
  Coord var;
  float varxz = 0;
  float varxyz = 0;
  Data *diff = NULL;

  Data () {};
  Data (float _time) : time (_time) {};

  Data *copy ()
  {
    Data *d = new Data (time);

    d->mean = mean;
    d->var = var;
    d->varxz = varxz;
    d->varxyz = varxyz;

    if (diff != NULL) d->diff = diff->copy();
    return d;
  }

  void computeDiff (Data *data)
  {
    float dt = time - data->time;
    if (diff != NULL) delete diff;
    diff = new Data (dt);
    diff->mean = (mean - data->mean) / dt;
    diff->var = (var - data->var) / dt;
    diff->varxz = (varxz - data->varxz) / dt;
    diff->varxyz = (varxyz - data->varxyz) / dt;
  }

  void add (Data *data)
  {
    mean += data->mean;
    var += data->var;
    varxyz += data->varxyz;

    diff->mean += data->diff->mean;
    diff->var += data->diff->var;
    diff->varxz += data->diff->varxz;
    diff->varxyz += data->diff->varxyz;
  }

  void div (float d)
  {
    mean /= d;
    var /= d;
    varxz /= d;
    varxyz /= d;

    diff->mean /= d;
    diff->var /= d;
    diff->varxz /= d;
    diff->varxyz /= d;
  }

  void draw (cv::Mat &pic, bool line = false)
  {
    int x = picWidth/2 * (1 + mean.x / picMaxX);
    if (x < 0) x = 0; else if (x >= picWidth) x = picWidth-1;
    int y = picHeight/2 * (1 + mean.y / picMaxY);
    if (y < 0) y = 0; else if (y >= picHeight) y = picHeight-1;
    int z = picHeight/2 * (1 + mean.z / picMaxZ);
    if (z < 0) z = 0; else if (z >= picHeight) z = picHeight-1;

    int dx = picWidth/2 * var.x / picMaxX;
    int dy = picHeight/2 * var.y / picMaxY;
    // int dz = picHeight/2 * var.z / picMaxZ;

    cv::line (pic, cv::Point (x-picTarget,z), cv::Point (x+picTarget,z), 1);
    cv::line (pic, cv::Point (x,z-picTarget), cv::Point (x,z+picTarget), 1);
    cv::rectangle (pic, cv::Point (x-dx,z-dy), cv::Point (x+dx,z+dy), 1);
 
    if (line) cv::line (pic, cv::Point (x,z), cv::Point (picWidth/2, picHeight/2), 1);
    
    // for (int px = -picTarget; px <= picTarget; px++) if (x+px >= 0 && x+px < picWidth) pic.at<float> (cv::Point (x+px, y)) = 1;
    // for (int py = -picTarget; py <= picTarget; py++) if (y+py >= 0 && y+py < picHeight) pic.at<float> (cv::Point (x, y+py)) = 1;
  }

  ~Data () { if (diff != NULL) delete diff; }
};

class Series
{
public:
  std::list<Data*> datas;

  Series () {};

  void addData (Data *data)
  {
    if (datas.size() > 0) data->computeDiff (datas.back()); else data->diff = new Data ();
    datas.push_back (data);
  }

  Data *getSmoothData (int steps)
  {
    if (datas.empty())
      {
	Data *d = new Data ();
	d->diff = new Data ();
	return d;
      }
    
    Data *smooth = datas.back()->copy();
    std::list<Data*>::reverse_iterator it = datas.rbegin();
    it++;
    unsigned int k = 1;
    while (steps > 0 && it != datas.rend())
      {
	smooth->add (*it);
	it++; steps--; k++;
      }
    smooth->div (k);
    return smooth;
  }
};


uint pointNb;
Series series;




bool verbose = false;

#define timeLength     0

#define toScreenWait  1

bool toScreen =              true;
bool toFile =                false;
std::string filename = "output/bug1.png";

bool doSpeedAddition =       true;
bool doSpeedBoundary =       false;
bool doSpeedAdvection =      true;
bool doSpeedPressure =       true;

bool doDyeAddition =         true;
bool doDyeBoundary =         false;
bool doDyeDiffusion =        true;
bool doDyeAdvection =        false;
bool doDyeReaction =         true;

unsigned int speedPressureIter =  10;
unsigned int dyeReactionIter =    10;

float dt =                         0.9;
float ds =                         1.;

float speedInit =                  0.;
float dyeInitA =                   1.;
float dyeInitB =                   0.;

float dyeDiffusionFactorA =        0.2097;
float dyeDiffusionFactorB =        0.105;
cl::float2_ dyeDiffusionFactors = {dyeDiffusionFactorA, dyeDiffusionFactorB};

unsigned int envInit = 0;
unsigned int envTarget = 3;

unsigned int envNb = 5;
cl::float4_ *envs;
cl::float4_ envCurrent;

void initEnvs ()
{
  envs = new cl::float4_ [envNb];
  envs[0] = {0.030, 0.062, 0.2097, 0.105}; // SOLITONS
  envs[1] = {0.014, 0.054, 0.2097, 0.105}; // MOVING SPOTS
  envs[2] = {0.029, 0.057, 0.2097, 0.105}; // MAZES 
  envs[3] = {0.034, 0.056, 0.2097, 0.105}; // CHAOS AND HOLES
  envs[4] = {0.020, 0.065, 0.2097, 0.105}; // NOTHING
}

float speedDisplayMin =            0.;
float speedDisplayMax =            1.;
float dyeDisplayMin =              0.;
float dyeDisplayMax =              0.5;
float currentDyeDisplayMax = dyeDisplayMax;

cl::float2_ envDisplayMin =       {0.010, 0.050};
cl::float2_ envDisplayMax =       {0.040, 0.070};


// GPU VARIABLES

cl::device GPU_device;
cl::context GPU_context;
cl::command_queue GPU_queue;

const size_t origin [2] = {0, 0};
const size_t region [2] = {width, height};

cl::image2d GPU_speed;
cl::image2d GPU_speedNew;

cl::image2d GPU_dye;
cl::image2d GPU_dyeNew;

cl::image2d GPU_div;
cl::image2d GPU_divNew;

cl::image2d GPU_other;
cl::image2d GPU_otherNew;

cv::Mat frame;
cl::image2d GPU_frame;
cl::image_format bgraInt8 (CL_BGRA, CL_UNORM_INT8);
cl::image_format rgbaFloat (CL_RGBA, CL_FLOAT);

cl::uint2_ size;
cl::float4_ diff;


// OTHER VARIABLES

bool stop = false;
unsigned int currentLoop = 0;
unsigned int previousLoop = currentLoop;
unsigned int currentTime;


// GPU PROGRAMS

cl::program GPU_speedAddition_program;
cl::program GPU_speedAdvection_program;
cl::program GPU_speedDivergence_program;
cl::program GPU_speedPressure_program;
cl::program GPU_speedGradient_program;

cl::program GPU_dyeAddition_program;
cl::program GPU_dyeReaction_program;
cl::program GPU_dyeDiffusion_program;
cl::program GPU_dyeAdvection_program;
cl::program GPU_environment_program;

cl::program GPU_vectorBoundary_program;
cl::program GPU_scalarBoundary_program;

cl::program GPU_speedDisplay_program;
cl::program GPU_dyeDisplay_program;
cl::program GPU_environmentDisplay_program;


// FUNCTIONS

void GPU_initVariables ();
void GPU_initPrograms ();

void GPU_doSpeedAddition ();
void GPU_doSpeedAdvection ();
void GPU_doSpeedPressure ();

void GPU_doDyeAddition (cl::vector <cl::int2_> &GPU_sources);
void GPU_doDyeDiffusion ();
void GPU_doDyeAdvection ();
void GPU_doDyeReaction ();

void GPU_doSpeedBoundary ();
void GPU_doDivergenceBoundary ();
void GPU_doPressureBoundary ();
void GPU_doDyeBoundary ();

void GPU_doSpeedDisplay ();
void GPU_doDyeDisplay (cl::vector <cl::float4_> &GPU_mycolors1, cl::vector <cl::float4_> &GPU_mycolors2);
void GPU_doEnvironmentDisplay ();

void keyPressed (int key);

// COLORS
const unsigned int colorNb = 1000;
const std::string colorFilename1 = "colorsGreen";
const std::string colorFilename2 = "colorsBlue";

float colorRatio = 1;
float currentColorRatio = colorRatio;

float *colors1;
float *colors2;
// cl::vector <cl::float4_> GPU_mycolors;

void getColors (std::string file, float *colors[])
{
  *colors = new float [colorNb*4];

  std::string line;
  std::ifstream colorFile (file);

  std::getline (colorFile, line);
  std::istringstream iss (line);
  for (unsigned int k = 0; k < colorNb; k++)
    {
      std::string colorHex;
      iss >> colorHex;

      int colorRGB;
      std::stringstream ss;
      ss << std::hex << colorHex.substr(1,6);
      ss >> colorRGB;

      (*colors)[4*k+0] = (float) ((colorRGB >> 16) & 0xFF) / 255.;
      (*colors)[4*k+1] = (float) ((colorRGB >> 8) & 0xFF) / 255.;
      (*colors)[4*k+2] = (float) ((colorRGB) & 0xFF) / 255.;
      (*colors)[4*k+3] = 1.;
    }
  
  colorFile.close();
  
  // GPU_mycolors = cl::vector <cl::float4_> (colorNb, GPU_context);

  // cl::copy (
  // 	    reinterpret_cast <cl::float4_*> (colors),
  // 	    reinterpret_cast <cl::float4_*> (colors) + colorNb,
  // 	    GPU_mycolors.begin(),
  // 	    GPU_queue
  // 	    );
}


// ADDITION VARIABLES

int speedAdditionX, speedAdditionY;
float speedAdditionDX, speedAdditionDY;

bool speedAddition = false;
// int speedAdditionX1, speedAdditionY1, speedAdditionX2, speedAdditionY2;

// bool dyeAdditionA = false;
// bool dyeAdditionB = false;
// int dyeAdditionX, dyeAdditionY;

// bool env = false;
// int envX, envY;

// void callbackFunction (int event, int x, int y, int flags, void *userdata)
// {
//   switch (event)
//     {
//     case cv::EVENT_LBUTTONDOWN :
//       // speedAddition = true;
//       // speedAdditionX1 = x;
//       // speedAdditionY1 = y;
//       dyeAdditionB = true;
//       break;

//     case cv::EVENT_LBUTTONUP :
//       // speedAddition = false;
//       dyeAdditionB = false;
//       break;

//     case cv::EVENT_RBUTTONDOWN :
//       speedAddition = true;
//       speedAdditionX1 = x;
//       speedAdditionY1 = y;
//       env = true;
//       break;
      
//     case cv::EVENT_RBUTTONUP :
//       speedAddition = false;
//       env = false;
//       break;
      
//     case cv::EVENT_MOUSEMOVE :
//       speedAdditionX2 = x;
//       speedAdditionY2 = y;
//       dyeAdditionX = x;
//       dyeAdditionY = y;
//       envX = x;
//       envY = y;
//       break;
//     }
// }


float scale (float in, float inMin, float inMax, float outMin, float outMax, bool clip = true)
{
  float r = (in - inMin) / (inMax - inMin);
  if (clip) { if (r < 0) r = 0; else if (r > 1) r = 1; }
  return r * (outMax - outMin) + outMin;
}


void GPU_initVariables ()
{
  // GPU DEVICE
  if (verbose) printf ("INIT GPU DEVICE\n");
  
  for (const auto &device : cl::system::devices()) { std::cout << device.name() << std::endl; GPU_device = device; }
  GPU_device = cl::system::default_device();
  std::cout << "selected device: " << GPU_device.name() << std::endl;
	
  GPU_context = cl::context (GPU_device);
  GPU_queue = cl::command_queue (GPU_context, GPU_device);

  // GPU ARGS
  if (verbose) printf ("INIT GPU ARGS\n");
  size = {width, height};
  diff = {ds, ds, dt, 0};

  // GPU VARIABLES
  if (verbose) printf ("INIT GPU VARIABLES\n");

  GPU_speed = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  GPU_speedNew = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  cl::float4_ initSpeed (speedInit, speedInit, envs[envInit].x, envs[envInit].y);
  GPU_queue.enqueue_fill_image (GPU_speed, &initSpeed, GPU_speed.origin(), GPU_speed.size());

  GPU_other = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  GPU_otherNew = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  cl::float4_ initOther (currentDyeDisplayMax, currentColorRatio, 0, 0);
  GPU_queue.enqueue_fill_image (GPU_other, &initOther, GPU_other.origin(), GPU_other.size());

  GPU_dye = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  GPU_dyeNew = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  cl::float4_ initDye (dyeInitA, dyeInitB, 0., 0.);
  GPU_queue.enqueue_fill_image (GPU_dye, &initDye, GPU_dye.origin(), GPU_dye.size());
  
  GPU_div = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  GPU_divNew = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);

  frame = cv::Mat (height, width, CV_8UC4, cv::Scalar (1,1,1));
  GPU_frame = cl::image2d (GPU_context, width, height, bgraInt8, cl::image2d::read_write);
  GPU_queue.enqueue_write_image (GPU_frame, GPU_frame.origin(), GPU_frame.size(), frame.data);

  getColors (colorFilename1, &colors1);
  getColors (colorFilename2, &colors2);
}


void GPU_initPrograms ()
{
  if (verbose) printf ("INIT GPU PROGRAMS\n");

  // SPEED ADDITION
  if (doSpeedAddition)
    {    
      const char GPU_speedAddition_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_speedAddition_kernel
	 (
	  __write_only image2d_t speedOut,
	  __read_only image2d_t speedIn,
	  __write_only image2d_t otherOut,
	  __read_only image2d_t otherIn,
	  int2 xy,
	  float2 dxy,
	  float2 radius,
	  float4 quant,
	  float4 target,
	  float4 otherTarget,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};
	   const float r = sqrt (pow ((float) x - xy.x, 2) + pow ((float) y - xy.y, 2)); // TODO TORUS

	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;

	   float4 value = read_imagef (speedIn, sampler, coord);
	   value.xy += diff.z * quant.x * dxy * exp (- r / radius.x);
	   value.zw += diff.z * quant.y * exp (- r / radius.y) * (target.xy - value.zw);
	   write_imagef (speedOut, coord, value);

	   value = read_imagef (otherIn, sampler, coord);
	   value.xy += diff.z * quant.z * exp (- r / radius.y) * (otherTarget.xy - value.xy);
	   write_imagef (otherOut, coord, value);
	 }
	 );

      GPU_speedAddition_program = cl::program::create_with_source (GPU_speedAddition_source, GPU_context);
      try { GPU_speedAddition_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_speedAddition_program.build_log() << std::endl; }
    }

  
  // SPEED ADVECTION
  if (doSpeedAdvection)
    {
      const char GPU_speedAdvection_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_speedAdvection_kernel
	 (
	  __write_only image2d_t speedOut,
	  __read_only image2d_t speedIn,
	  __write_only image2d_t otherOut,
	  __read_only image2d_t otherIn,
	  __read_only image2d_t dyeIn,
	  float2 quant,
	  float4 init,
	  float4 otherInit,
	  uint2 size,
	  float4 diff
	  )
	 {			
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};

	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
	   float4 value = read_imagef (speedIn, sampler, coord);

	   const float2 coord2 = convert_float2 (coord) - value.xy * diff.z / diff.xy + (float2) {0.5, 0.5};
	   const sampler_t sampler2 = CLK_ADDRESS_NONE | CLK_FILTER_LINEAR;

	   float4 value2 = read_imagef (speedIn, sampler2, coord2);

	   float4 value1 = read_imagef (dyeIn, sampler, coord);
	   float dye = value1.y * 2;	   
	   value2.xy = value2.xy * (diff.z * (1.f - 0.05f * dye));

	   value2.zw += diff.z * quant.x * (init.xy - value2.zw);
	   write_imagef (speedOut, coord, value2);

	   value2 = read_imagef (otherIn, sampler2, coord2);
	   value2.xy += diff.z * quant.y * (otherInit.xy - value2.xy);
	   write_imagef (otherOut, coord, value2);
	 }
	 );

      GPU_speedAdvection_program = cl::program::create_with_source (GPU_speedAdvection_source, GPU_context);
      try { GPU_speedAdvection_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_speedAdvection_program.build_log() << std::endl; }
    }


  if (doSpeedPressure)
    {
      // SPEED DIVERGENCE
      const char GPU_speedDivergence_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_speedDivergence_kernel
	 (
	  __write_only image2d_t divOut,
	  __read_only image2d_t speedIn,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};
	   
	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;

	   float div = 0;
	   div += read_imagef (speedIn, sampler, coord + (int2) {1,0}).x;
	   div -= read_imagef (speedIn, sampler, coord + (int2) {-1,0}).x;
	   div += read_imagef (speedIn, sampler, coord + (int2) {0,1}).y;
	   div -= read_imagef (speedIn, sampler, coord + (int2) {0,-1}).y;	   
	   div /= (diff.x + diff.y);

	   write_imagef (divOut, coord, (float4) {div, 0., 0., 0.});
	 }
	 );

      GPU_speedDivergence_program = cl::program::create_with_source (GPU_speedDivergence_source, GPU_context);
      try { GPU_speedDivergence_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_speedDivergence_program.build_log() << std::endl; }


      // SPEED PRESSURE
      const char GPU_speedPressure_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_speedPressure_kernel
	 (
	  __write_only image2d_t divOut,
	  __read_only image2d_t divIn,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};
	   
	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
	   
	   float div = read_imagef (divIn, sampler, coord).x;

	   float press = 0;
	   press += read_imagef (divIn, sampler, coord + (int2) {1,0}).y;
	   press += read_imagef (divIn, sampler, coord + (int2) {-1,0}).y;
	   press += read_imagef (divIn, sampler, coord + (int2) {0,1}).y;
	   press += read_imagef (divIn, sampler, coord + (int2) {0,-1}).y;	   
	   press = (press - diff.x * diff.y * div) / 4;
	   
	   write_imagef (divOut, coord, (float4) {div, press, 0., 0.});
	 }
	 );

      GPU_speedPressure_program = cl::program::create_with_source (GPU_speedPressure_source, GPU_context);
      try { GPU_speedPressure_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_speedPressure_program.build_log() << std::endl; }


      // SPEED GRADIENT
      const char GPU_speedGradient_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_speedGradient_kernel
	 (
	  __write_only image2d_t speedOut,
	  __read_only image2d_t speedIn,
	  __read_only image2d_t divIn,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};
	   
	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;

	   float pressX = 0;
	   pressX += read_imagef (divIn, sampler, coord + (int2) {1,0}).y;
	   pressX -= read_imagef (divIn, sampler, coord + (int2) {-1,0}).y;
	   
	   float pressY = 0;
	   pressY += read_imagef (divIn, sampler, coord + (int2) {0,1}).y;
	   pressY -= read_imagef (divIn, sampler, coord + (int2) {0,-1}).y;

	   float4 speed = read_imagef (speedIn, sampler, coord);
	   speed.xy -= (float2) {pressX, pressY} / (diff.x + diff.y);
	   write_imagef (speedOut, coord, speed);
	 }
	 );
	
      GPU_speedGradient_program = cl::program::create_with_source (GPU_speedGradient_source, GPU_context);
      try { GPU_speedGradient_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_speedGradient_program.build_log() << std::endl; }
    }
  
  
  // DYE ADDITION
  if (doDyeAddition)
    {
      const char GPU_dyeAddition_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_dyeAddition_kernel
	 (
	  __write_only image2d_t dyeOut,
	  __read_only image2d_t dyeIn,
	  // uint x1,
	  // uint y1,
	  float radius,
	  float2 quant,
	  __global int2 *sources,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};

	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
	   float4 value = read_imagef (dyeIn, sampler, coord);
	   
	   // if ((x % 100 == 0 && (y == 0 || y == size.y-1)) || (y % 100 == 0 && (x == 0 || x == size.x-1)))
	   //   {
	   //     value.xy = quant;
	   //   }

	   for (int k = 0; k < 40; k++)
	     {
	       if (x == sources[k].x && y == sources[k].y) value.xy = quant;
	     }
	   write_imagef (dyeOut, coord, value);
	 }
	 );
 
      GPU_dyeAddition_program = cl::program::create_with_source (GPU_dyeAddition_source, GPU_context);
      try { GPU_dyeAddition_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_dyeAddition_program.build_log() << std::endl; }
    }

  
  // DYE DIFFUSION
  if (doDyeDiffusion)
    {
      const char GPU_dyeDiffusion_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_dyeDiffusion_kernel
	 (
	  __write_only image2d_t dyeOut,
	  __read_only image2d_t dyeIn,
	  float2 factor,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};

	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;

	   float4 dye = read_imagef (dyeIn, sampler, coord);

	   float4 dyeSum = {0., 0., 0., 0.};
	   dyeSum += read_imagef (dyeIn, sampler, coord + (int2) {0,1});
	   dyeSum += read_imagef (dyeIn, sampler, coord + (int2) {1,0});
	   dyeSum += read_imagef (dyeIn, sampler, coord + (int2) {0,-1});
	   dyeSum += read_imagef (dyeIn, sampler, coord + (int2) {-1,0});
	     
	   dye.xy += (dyeSum.xy - 4 * dye.xy) * factor * diff.z;
	   write_imagef (dyeOut, coord, dye);
	 }
	 );

      GPU_dyeDiffusion_program = cl::program::create_with_source (GPU_dyeDiffusion_source, GPU_context);
      try { GPU_dyeDiffusion_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_dyeDiffusion_program.build_log() << std::endl; }
    }
  
  
  // DYE ADVECTION
  if (doDyeAdvection)
    {
      const char GPU_dyeAdvection_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_dyeAdvection_kernel (__global float2 *dyeOut, __global const float2 *dyeIn, __global const float2 *speedIn, uint2 size, float4 diff)
	 {
	   const uint i = get_global_id (0);

	   if (block[i] == 2) dyeOut[i] = dyeIn[i];
	   else {
	     const uint x = i % size.x;
	     const uint y = i / size.x;
       
	     float nx = (float) x - speedIn[i].x * diff.z / diff.x;
	     float ny = (float) y - speedIn[i].y * diff.z / diff.y;

	     if (nx < 0.5) nx = 0.5; else if (nx > size.x - 1.5) nx = size.x - 1.5;
	     if (ny < 0.5) ny = 0.5; else if (ny > size.y - 1.5) ny = size.y - 1.5;

	     const uint x1 = floor (nx);
	     const uint x2 = x1 + 1;
	     const float dx = nx - x1;

	     const uint y1 = floor (ny);
	     const uint y2 = y1 + 1;
	     const float dy = ny - y1;

	     const uint i11 = x1 + y1 * size.x;
	     const uint i12 = x1 + y2 * size.x;
	     const uint i21 = x2 + y1 * size.x;
	     const uint i22 = x2 + y2 * size.x;
	
	     const float dfx = dyeIn[i21] - dyeIn[i11];
	     const float dfy = dyeIn[i12] - dyeIn[i11];
	     const float dfxy = dyeIn[i11] + dyeIn[i22] - dyeIn[i21] - dyeIn[i12];

	     const float fxy = dfx * dx / diff.x + dfy * dy / diff.y + dfxy * dx / diff.x * dy / diff.y + dyeIn[i11];
	     dyeOut[i] = fxy;
	   }
	 }
	 );

      GPU_dyeAdvection_program = cl::program::create_with_source (GPU_dyeAdvection_source, GPU_context);
      try { GPU_dyeAdvection_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_dyeAdvection_program.build_log() << std::endl; }
    }
  

  // DYE REACTION
  if (doDyeReaction)
    {
      const char GPU_dyeReaction_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_dyeReaction_kernel
	 (
	  __write_only image2d_t dyeOut,
	  __read_only image2d_t dyeIn,
	  __read_only image2d_t speedIn,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};

	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;

	   float4 dye = read_imagef (dyeIn, sampler, coord);
	   float4 speed = read_imagef (speedIn, sampler, coord);
	   
	   const float d = dye.x * dye.y * dye.y;
	   const float dA = - d + speed.z * (1. - dye.x);
	   const float dB = d - (speed.z + speed.w) * dye.y;
	   dye.xy += diff.z * (float2) {dA, dB};
	   write_imagef (dyeOut, coord, dye);
	 }
	 );

      GPU_dyeReaction_program = cl::program::create_with_source (GPU_dyeReaction_source, GPU_context);
      try { GPU_dyeReaction_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_dyeReaction_program.build_log() << std::endl; }
    }
  
  
  if (doSpeedBoundary)
    {
      // VECTOR BOUNDARY
      const char GPU_vectorBoundary_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_vectorBoundary_kernel (__global float2 *speedInOut, uint2 size, float4 diff)
	 {
	   const uint i = get_global_id (0);

	   if (block[i] > 1)
	     {
	       const uint x = i % size.x;
	       const uint y = i / size.x;

	       uint n = 0;
	       float2 speedSum = {0, 0};

	       if (x > 0)
		 {
		   const uint im0 = (x-1) + y * size.x;
		   if (block[im0] <= 1) { speedSum.x -= speedInOut[im0].x; speedSum.y += speedInOut[im0].y; n++; }
		 }

	       if (x < size.x-1)
		 {
		   const uint ip0 = (x+1) + y * size.x;
		   if (block[ip0] <= 1) { speedSum.x -= speedInOut[ip0].x; speedSum.y += speedInOut[ip0].y; n++; }
		 }

	       if (y > 0)
		 {
		   const uint i0m = x + (y-1) * size.x;
		   if (block[i0m] <= 1) { speedSum.x += speedInOut[i0m].x; speedSum.y -= speedInOut[i0m].y; n++; }
		 }

	       if (y < size.y-1)
		 {
		   const uint i0p = x + (y+1) * size.x;
		   if (block[i0p] <= 1) { speedSum.x += speedInOut[i0p].x; speedSum.y -= speedInOut[i0p].y; n++; }
		 }
	       
	       if (n > 0) speedInOut[i] = speedSum / n;
	     }
	 }
	 );

      GPU_vectorBoundary_program = cl::program::create_with_source (GPU_vectorBoundary_source, GPU_context);
      try { GPU_vectorBoundary_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_vectorBoundary_program.build_log() << std::endl; }
    }


  if (doSpeedBoundary || doDyeBoundary)
    {
      // SCALAR BOUNDARY
      const char GPU_scalarBoundary_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
	(
	 __kernel void GPU_scalarBoundary_kernel (__global float *scalarInOut, uint2 size, float4 diff)
	 {
	   const uint i = get_global_id (0);

	   if (block[i] > 1)
	     {
	       const uint x = i % size.x;
	       const uint y = i / size.x;

	       uint n = 0;
	       float scalarSum = 0;

	       if (x > 0)
		 {
		   const uint im0 = (x-1) + y * size.x;
		   if (block[im0] <= 1) { scalarSum += scalarInOut[im0]; n++; }
		 }

	       if (x < size.x-1)
		 {
		   const uint ip0 = (x+1) + y * size.x;
		   if (block[ip0] <= 1) { scalarSum += scalarInOut[ip0]; n++; }
		 }

	       if (y > 0)
		 {
		   const uint i0m = x + (y-1) * size.x;
		   if (block[i0m] <= 1) { scalarSum += scalarInOut[i0m]; n++; }
		 }

	       if (y < size.y-1)
		 {
		   const uint i0p = x + (y+1) * size.x;
		   if (block[i0p] <= 1) { scalarSum += scalarInOut[i0p]; n++; }
		 }
	       
	       if (n > 0) scalarInOut[i] = scalarSum / n;
	     }
	 }
	 );

      GPU_scalarBoundary_program = cl::program::create_with_source (GPU_scalarBoundary_source, GPU_context);
      try { GPU_scalarBoundary_program.build(); }
      catch (cl::opencl_error &e) { std::cout << GPU_scalarBoundary_program.build_log() << std::endl; }
    }
  

  // DYE DISPLAY
  const char GPU_dyeDisplay_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
    (
     __kernel void GPU_dyeDisplay_kernel
     (
      __write_only image2d_t frameOut,
      __read_only image2d_t dyeIn,
      __read_only image2d_t otherIn,
      __global const float4 *colors1,
      __global const float4 *colors2,
      int colorNb,
      float min,
      float max,
      uint2 size,
      float4 diff
      )
     {
       const uint x = get_global_id (0);
       const uint y = get_global_id (1);
       const int2 coord = {x, y};

       const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
       float4 value = read_imagef (dyeIn, sampler, coord);
       float4 other = read_imagef (otherIn, sampler, coord);

       int colorIndex = (value.y - min) / (other.x - min) * colorNb;
       if (colorIndex < 0) colorIndex = 0; else if (colorIndex >= colorNb) colorIndex = colorNb-1;
	   
       write_imagef (frameOut, coord, other.y * colors1[colorIndex] + (1 - other.y) * colors2[colorIndex]);
       // float v = (value.y - min) / (max - min);
       // write_imagef (frameOut, coord, (float4) {v, v, v, 1.});
     }
     );

  GPU_dyeDisplay_program = cl::program::create_with_source (GPU_dyeDisplay_source, GPU_context);
  try { GPU_dyeDisplay_program.build(); }
  catch (cl::opencl_error &e) { std::cout << GPU_dyeDisplay_program.build_log() << std::endl; }
  
  // SPEED DISPLAY
  const char GPU_speedDisplay_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
    (
     __kernel void GPU_speedDisplay_kernel
     (
      __write_only image2d_t frameOut,
      __read_only image2d_t speedIn,
      // __global const float4 *colors,
      int colorNb,
      float min,
      float max,
      uint2 size,
      float4 diff
      )
     {
       const uint x = get_global_id (0);
       const uint y = get_global_id (1);
       const int2 coord = {x, y};

       const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
       float4 value = read_imagef (speedIn, sampler, coord);

       int colorIndex = (sqrt (value.x*value.x + value.y*value.y) - min) / (max - min) * colorNb;
       if (colorIndex < 0) colorIndex = 0; else if (colorIndex >= colorNb) colorIndex = colorNb-1;
	 
       // write_imagef (frameOut, coord, colors[colorIndex]);
       float v = (sqrt (value.x*value.x + value.y*value.y) - min) / (max - min);
       write_imagef (frameOut, coord, (float4) {v, v, v, 1.});
     }
     );
	
  GPU_speedDisplay_program = cl::program::create_with_source (GPU_speedDisplay_source, GPU_context);
  try { GPU_speedDisplay_program.build(); }
  catch (cl::opencl_error &e) { std::cout << GPU_speedDisplay_program.build_log() << std::endl; }


  // ENVIRONMENT DISPLAY
  const char GPU_environmentDisplay_source [] = BOOST_COMPUTE_STRINGIZE_SOURCE
    (
     __kernel void GPU_environmentDisplay_kernel
     (
      __write_only image2d_t frameOut,
      __read_only image2d_t speedIn,
      float2 min,
      float2 max,
      uint2 size,
      float4 diff
      )
     {
       const uint x = get_global_id (0);
       const uint y = get_global_id (1);
       const int2 coord = {x, y};

       const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
       float4 value = read_imagef (speedIn, sampler, coord);

       float2 colorValue = (value.zw - min) / (max - min);
       if (colorValue.x < 0) colorValue.x = 0; else if (colorValue.x > 1) colorValue.x = 1;
       if (colorValue.y < 0) colorValue.y = 0; else if (colorValue.y > 1) colorValue.y = 1;
	 
       write_imagef (frameOut, coord, (float4) {colorValue.x, colorValue.y, 0, 1});
     }
     );
	
  GPU_environmentDisplay_program = cl::program::create_with_source (GPU_environmentDisplay_source, GPU_context);
  try { GPU_environmentDisplay_program.build(); }
  catch (cl::opencl_error &e) { std::cout << GPU_environmentDisplay_program.build_log() << std::endl; }
}


void GPU_doSpeedAddition ()
{
  if (! speedAddition) return;

  if (verbose) printf ("SPEED ADDITION\n");

  // printf ("%i %i %f %f\n", speedAdditionX, speedAdditionY, speedAdditionDX, speedAdditionDY);
  cl::kernel GPU_speedAddition_kernel (GPU_speedAddition_program, "GPU_speedAddition_kernel");
  GPU_speedAddition_kernel.set_arg (0, GPU_speedNew);
  GPU_speedAddition_kernel.set_arg (1, GPU_speed);
  GPU_speedAddition_kernel.set_arg (2, GPU_otherNew);
  GPU_speedAddition_kernel.set_arg (3, GPU_other);
  GPU_speedAddition_kernel.set_arg (4, (cl::int2_) {speedAdditionX, speedAdditionY});
  GPU_speedAddition_kernel.set_arg (5, (cl::float2_) {speedAdditionDX, speedAdditionDY});
  GPU_speedAddition_kernel.set_arg (6, (cl::float2_) {speedAdditionRadius, envAdditionRadius});
  GPU_speedAddition_kernel.set_arg (7, (cl::float4_) {speedAdditionQuant, envAdditionQuant, colorAdditionQuant, 0});
  GPU_speedAddition_kernel.set_arg (8, envCurrent);
  GPU_speedAddition_kernel.set_arg (9, (cl::float4_) {currentDyeDisplayMax, currentColorRatio, 0, 0});
  GPU_speedAddition_kernel.set_arg (10, size);
  GPU_speedAddition_kernel.set_arg (11, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_speedAddition_kernel, 2, origin, region, 0);

  GPU_speed = GPU_speedNew;
  GPU_other = GPU_otherNew;
}


void GPU_doSpeedAdvection ()
{
  if (verbose) printf ("SPEED ADVECTION\n");

  cl::kernel GPU_speedAdvection_kernel (GPU_speedAdvection_program, "GPU_speedAdvection_kernel");
  GPU_speedAdvection_kernel.set_arg (0, GPU_speedNew);
  GPU_speedAdvection_kernel.set_arg (1, GPU_speed);
  GPU_speedAdvection_kernel.set_arg (2, GPU_otherNew);
  GPU_speedAdvection_kernel.set_arg (3, GPU_other);
  GPU_speedAdvection_kernel.set_arg (4, GPU_dye);
  GPU_speedAdvection_kernel.set_arg (5, (cl::float2_) {envSubstractionQuant, colorSubstractionQuant});
  GPU_speedAdvection_kernel.set_arg (6, envs[envInit]);
  GPU_speedAdvection_kernel.set_arg (7, (cl::float4_) {dyeDisplayMax, colorRatio, 0, 0});
  GPU_speedAdvection_kernel.set_arg (8, size);
  GPU_speedAdvection_kernel.set_arg (9, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_speedAdvection_kernel, 2, origin, region, 0);

  GPU_speed = GPU_speedNew;
  GPU_other = GPU_otherNew;

  if (doSpeedBoundary) GPU_doSpeedBoundary();
}


void GPU_doSpeedPressure ()
{
  if (verbose) printf ("SPEED PRESSURE\n");

  cl::kernel GPU_speedDivergence_kernel (GPU_speedDivergence_program, "GPU_speedDivergence_kernel");
  GPU_speedDivergence_kernel.set_arg (0, GPU_div);
  GPU_speedDivergence_kernel.set_arg (1, GPU_speed);
  GPU_speedDivergence_kernel.set_arg (2, size);
  GPU_speedDivergence_kernel.set_arg (3, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_speedDivergence_kernel, 2, origin, region, 0);

  if (doSpeedBoundary) GPU_doDivergenceBoundary();

  for (unsigned int n = 0; n < speedPressureIter; n++)
    {
      cl::kernel GPU_speedPressure_kernel (GPU_speedPressure_program, "GPU_speedPressure_kernel");
      GPU_speedPressure_kernel.set_arg (0, GPU_divNew);
      GPU_speedPressure_kernel.set_arg (1, GPU_div);
      GPU_speedPressure_kernel.set_arg (2, size);
      GPU_speedPressure_kernel.set_arg (3, diff);
      GPU_queue.enqueue_nd_range_kernel (GPU_speedPressure_kernel, 2, origin, region, 0);

      GPU_div = GPU_divNew;

      if (doSpeedBoundary) GPU_doPressureBoundary();
    }

  cl::kernel GPU_speedGradient_kernel (GPU_speedGradient_program, "GPU_speedGradient_kernel");
  GPU_speedGradient_kernel.set_arg (0, GPU_speedNew);
  GPU_speedGradient_kernel.set_arg (1, GPU_speed);
  GPU_speedGradient_kernel.set_arg (2, GPU_div);
  GPU_speedGradient_kernel.set_arg (3, size);
  GPU_speedGradient_kernel.set_arg (4, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_speedGradient_kernel, 2, origin, region, 0);

  GPU_speed = GPU_speedNew;

  if (doSpeedBoundary) GPU_doSpeedBoundary();
}


void GPU_doDyeAddition (cl::vector <cl::int2_> &GPU_sources)
{
  // if (dyeAdditionA || dyeAdditionB || true)
  //   {
      if (verbose) printf ("DYE ADDITION\n");

      cl::float2_ dyeAdditionQuant = {0.0, 0.9};

      // cl::float2_ dyeAdditionQuant = {0, 0};
      // if (dyeAdditionA) dyeAdditionQuant.x = dyeAdditionA;
      // if (dyeAdditionB) dyeAdditionQuant.y = dyeAdditionB;

      cl::kernel GPU_dyeAddition_kernel (GPU_dyeAddition_program, "GPU_dyeAddition_kernel");
      GPU_dyeAddition_kernel.set_arg (0, GPU_dyeNew);
      GPU_dyeAddition_kernel.set_arg (1, GPU_dye);
      // GPU_dyeAddition_kernel.set_arg (2, dyeAdditionX);
      // GPU_dyeAddition_kernel.set_arg (3, dyeAdditionY);
      GPU_dyeAddition_kernel.set_arg (2, dyeAdditionRadius);
      GPU_dyeAddition_kernel.set_arg (3, dyeAdditionQuant);
      GPU_dyeAddition_kernel.set_arg (4, GPU_sources);
      GPU_dyeAddition_kernel.set_arg (5, size);
      GPU_dyeAddition_kernel.set_arg (6, diff);
      GPU_queue.enqueue_nd_range_kernel (GPU_dyeAddition_kernel, 2, origin, region, 0);

      GPU_dye = GPU_dyeNew;

      if (doDyeBoundary) GPU_doDyeBoundary();
    // }
}


void GPU_doDyeDiffusion ()
{
  if (verbose) printf ("DYE DIFFUSION\n");

  cl::kernel GPU_dyeDiffusion_kernel (GPU_dyeDiffusion_program, "GPU_dyeDiffusion_kernel");
  GPU_dyeDiffusion_kernel.set_arg (0, GPU_dyeNew);
  GPU_dyeDiffusion_kernel.set_arg (1, GPU_dye);
  GPU_dyeDiffusion_kernel.set_arg (2, dyeDiffusionFactors);
  GPU_dyeDiffusion_kernel.set_arg (3, size);
  GPU_dyeDiffusion_kernel.set_arg (4, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_dyeDiffusion_kernel, 2, origin, region, 0);

  GPU_dye = GPU_dyeNew;

  if (doDyeBoundary) GPU_doDyeBoundary();
}


void GPU_doDyeAdvection ()
{
  if (verbose) printf ("DYE ADVECTION\n");

  cl::kernel GPU_dyeAdvection_kernel (GPU_dyeAdvection_program, "GPU_dyeAdvection_kernel");
  GPU_dyeAdvection_kernel.set_arg (0, GPU_dyeNew);
  GPU_dyeAdvection_kernel.set_arg (1, GPU_dye);
  GPU_dyeAdvection_kernel.set_arg (2, GPU_speed);
  GPU_dyeAdvection_kernel.set_arg (3, size);
  GPU_dyeAdvection_kernel.set_arg (4, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_dyeAdvection_kernel, 2, origin, region, 0);

  GPU_dye = GPU_dyeNew;

  if (doDyeBoundary) GPU_doDyeBoundary();
}


void GPU_doDyeReaction ()
{
  if (verbose) printf ("DYE REACTION\n");
  
  cl::kernel GPU_dyeReaction_kernel = cl::kernel (GPU_dyeReaction_program, "GPU_dyeReaction_kernel");
  GPU_dyeReaction_kernel.set_arg (0, GPU_dyeNew);
  GPU_dyeReaction_kernel.set_arg (1, GPU_dye);
  GPU_dyeReaction_kernel.set_arg (2, GPU_speed);
  GPU_dyeReaction_kernel.set_arg (3, size);
  GPU_dyeReaction_kernel.set_arg (4, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_dyeReaction_kernel, 2, origin, region, 0);

  GPU_dye = GPU_dyeNew;

  if (doDyeBoundary) GPU_doDyeBoundary();
}


void GPU_doSpeedBoundary ()
{
  if (verbose) printf ("SPEED BOUNDARY\n");

  cl::kernel GPU_vectorBoundary_kernel (GPU_vectorBoundary_program, "GPU_vectorBoundary_kernel");
  GPU_vectorBoundary_kernel.set_arg (0, GPU_speed);
  GPU_vectorBoundary_kernel.set_arg (1, size);
  GPU_vectorBoundary_kernel.set_arg (2, diff);
  GPU_queue.enqueue_1d_range_kernel (GPU_vectorBoundary_kernel, 0, width * height, 0);
}


void GPU_doDivergenceBoundary ()
{
  if (verbose) printf ("DIVERGENCE BOUNDARY\n");

  cl::kernel GPU_scalarBoundary_kernel (GPU_scalarBoundary_program, "GPU_scalarBoundary_kernel");
  GPU_scalarBoundary_kernel.set_arg (0, GPU_div);
  GPU_scalarBoundary_kernel.set_arg (1, size);
  GPU_scalarBoundary_kernel.set_arg (2, diff);
  GPU_queue.enqueue_1d_range_kernel (GPU_scalarBoundary_kernel, 0, width * height, 0);
}


void GPU_doPressureBoundary ()
{
  if (verbose) printf ("PRESSURE BOUNDARY\n");

  cl::kernel GPU_scalarBoundary_kernel (GPU_scalarBoundary_program, "GPU_scalarBoundary_kernel");
  GPU_scalarBoundary_kernel.set_arg (0, GPU_div);
  GPU_scalarBoundary_kernel.set_arg (1, size);
  GPU_scalarBoundary_kernel.set_arg (2, diff);
  GPU_queue.enqueue_1d_range_kernel (GPU_scalarBoundary_kernel, 0, width * height, 0);
}


void GPU_doDyeBoundary ()
{
  if (verbose) printf ("DYE BOUNDARY\n");

  cl::kernel GPU_scalarBoundary_kernel (GPU_scalarBoundary_program, "GPU_scalarBoundary_kernel");
  GPU_scalarBoundary_kernel.set_arg (0, GPU_dye);
  GPU_scalarBoundary_kernel.set_arg (1, size);
  GPU_scalarBoundary_kernel.set_arg (2, diff);
  GPU_queue.enqueue_1d_range_kernel (GPU_scalarBoundary_kernel, 0, width * height, 0);
}


void GPU_doDyeDisplay (cl::vector <cl::float4_> &GPU_mycolors1, cl::vector <cl::float4_> &GPU_mycolors2)
{
  if (verbose) printf ("DYE DISPLAY\n");

  cl::kernel GPU_dyeDisplay_kernel (GPU_dyeDisplay_program, "GPU_dyeDisplay_kernel");
  GPU_dyeDisplay_kernel.set_arg (0, GPU_frame);
  GPU_dyeDisplay_kernel.set_arg (1, GPU_dye);
  GPU_dyeDisplay_kernel.set_arg (2, GPU_other);
  GPU_dyeDisplay_kernel.set_arg (3, GPU_mycolors1);
  GPU_dyeDisplay_kernel.set_arg (4, GPU_mycolors2);
  GPU_dyeDisplay_kernel.set_arg (5, colorNb);
  GPU_dyeDisplay_kernel.set_arg (6, dyeDisplayMin);
  GPU_dyeDisplay_kernel.set_arg (7, dyeDisplayMax);
  GPU_dyeDisplay_kernel.set_arg (8, size);
  GPU_dyeDisplay_kernel.set_arg (9, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_dyeDisplay_kernel, 2, origin, region, 0);
  
  if (toScreen)
    {
      cl::opencv_imshow ("incarnasuono", GPU_frame, GPU_queue);
      int key = cv::waitKey (toScreenWait);
      keyPressed (key);
    }

  if (toFile)
    {
      frame = cv::Mat (height, width, CV_8UC4);
      GPU_queue.enqueue_read_image (GPU_frame, GPU_frame.origin(), GPU_frame.size(), frame.data);

      std::string outputBase = filename.substr (0, filename.length()-4);
      std::string outputExt = filename.substr (filename.length()-3, 3);
      std::string filename2 = outputBase + "." + std::string (5 - (int) log10 (currentLoop), '0') + std::to_string (currentLoop) + "." + outputExt;
      std::cout << filename << std::endl;
      cv::imwrite (filename2, frame);
    }
}


void GPU_doSpeedDisplay ()
{
  if (verbose) printf ("SPEED DISPLAY\n");

  cl::kernel GPU_speedDisplay_kernel (GPU_speedDisplay_program, "GPU_speedDisplay_kernel");
  GPU_speedDisplay_kernel.set_arg (0, GPU_frame);
  GPU_speedDisplay_kernel.set_arg (1, GPU_speed);
  // GPU_speedDisplay_kernel.set_arg (2, GPU_mycolors);
  GPU_speedDisplay_kernel.set_arg (2, colorNb);
  GPU_speedDisplay_kernel.set_arg (3, speedDisplayMin);
  GPU_speedDisplay_kernel.set_arg (4, speedDisplayMax);
  GPU_speedDisplay_kernel.set_arg (5, size);
  GPU_speedDisplay_kernel.set_arg (6, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_speedDisplay_kernel, 2, origin, region, 0);

  if (toScreen)
    {
      cl::opencv_imshow ("incarnasuono", GPU_frame, GPU_queue);
      int key = cv::waitKey (toScreenWait);
      keyPressed (key);
    }

  if (toFile)
    {
      frame = cv::Mat (height, width, CV_8UC4);
      GPU_queue.enqueue_read_image (GPU_frame, GPU_frame.origin(), GPU_frame.size(), frame.data);

      std::string outputBase = filename.substr (0, filename.length()-4);
      std::string outputExt = filename.substr (filename.length()-3, 3);
      std::string filename2 = outputBase + "." + std::string (5 - (int) log10 (currentLoop), '0') + std::to_string (currentLoop) + "." + outputExt;
      cv::imwrite (filename2, frame);
    }
}


void GPU_doEnvironmentDisplay ()
{
  if (verbose) printf ("ENVIRONMENT DISPLAY\n");

  cl::kernel GPU_environmentDisplay_kernel (GPU_environmentDisplay_program, "GPU_environmentDisplay_kernel");
  GPU_environmentDisplay_kernel.set_arg (0, GPU_frame);
  GPU_environmentDisplay_kernel.set_arg (1, GPU_speed);
  GPU_environmentDisplay_kernel.set_arg (2, envDisplayMin);
  GPU_environmentDisplay_kernel.set_arg (3, envDisplayMax);
  GPU_environmentDisplay_kernel.set_arg (4, size);
  GPU_environmentDisplay_kernel.set_arg (5, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_environmentDisplay_kernel, 2, origin, region, 0);

  if (toScreen)
    {
      cl::opencv_imshow ("incarnasuono", GPU_frame, GPU_queue);
      int key = cv::waitKey (toScreenWait);
      keyPressed (key);
    }

  if (toFile)
    {
      frame = cv::Mat (height, width, CV_8UC4);
      GPU_queue.enqueue_read_image (GPU_frame, GPU_frame.origin(), GPU_frame.size(), frame.data);

      std::string outputBase = filename.substr (0, filename.length()-4);
      std::string outputExt = filename.substr (filename.length()-3, 3);
      std::string filename2 = outputBase + "." + std::string (5 - (int) log10 (currentLoop), '0') + std::to_string (currentLoop) + "." + outputExt;
      cv::imwrite (filename2, frame);
    }
}


void keyPressed (int key)
{
  if (key > 0)
    {
      key = key & 0xFF;
      std::cout << "KINECT KEY PRESSED: " << key << std::endl;
      
      switch (key)
	{
	case  27 : stop = true; break; // Escape
	case 176 :
	  if (doDisplay == "dye") doDisplay = "speed";
	  else if (doDisplay == "speed") doDisplay = "env";
	  else if (doDisplay == "env") doDisplay = "dye";
	  break; // 0 NUM

	case 177 : envTarget = 0; break; // 1 NUM
	case 178 : envTarget = 1; break; // 2 NUM
	case 179 : envTarget = 2; break; // 3 NUM
	case 180 : envTarget = 3; break; // 4 NUM
	case 181 : envTarget = 4; break; // 5 NUM
	}
    }
}



int main (int argc, char *argv[])
{
  srand (time (NULL));

  Kinect *kinect = new Kinect ();
  pthread_t kinectThread;
  int rcKinect;
  if (withKinect)
    {
      kinect->thresholdFromFile = true;
      kinect->realPositioning = true;
      kinect->allowSensorDisplay = false;
      kinect->allowObjectDetection = false;
      kinect->allowPointExtraction = true;
      kinect->waitingTime = 30000;
      kinect->thresholdBlurSize = 0;
      kinect->init();
  
      rcKinect = pthread_create (&kinectThread, NULL, &Kinect::run, (void *) kinect);
      if (rcKinect) { std::cout << "Error: Unable to create thread " << rcKinect << std::endl; exit (-1); }
    }

  // cv::namedWindow ("proto");
  struct timeval startTimer, stepTimer;
  gettimeofday (&startTimer, NULL);

  // INCARNASUONO
  initEnvs ();

  // INIT GPU
  GPU_initVariables();
  GPU_initPrograms();
  
  // INIT DISPLAY
  if (verbose) printf ("INIT WINDOW\n");
  cv::namedWindow ("incarnasuono", cv::WINDOW_NORMAL);
  cv::resizeWindow ("incarnasuono", 1920, 1080);
  cv::setWindowProperty ("incarnasuono", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
  // cv::setMouseCallback ("incarnasuono", callbackFunction, NULL);

  // LOOP
  if (verbose) printf ("START\n");
  std::chrono::steady_clock::time_point previousTime = std::chrono::steady_clock::now();

  cl::vector <cl::float4_> GPU_mycolors1 = cl::vector <cl::float4_> (colorNb, GPU_context);
  cl::vector <cl::float4_> GPU_mycolors2 = cl::vector <cl::float4_> (colorNb, GPU_context);

  cl::copy (
	    reinterpret_cast <cl::float4_*> (colors1),
	    reinterpret_cast <cl::float4_*> (colors1) + colorNb,
	    GPU_mycolors1.begin(),
	    GPU_queue
	    );

  cl::copy (
	    reinterpret_cast <cl::float4_*> (colors2),
	    reinterpret_cast <cl::float4_*> (colors2) + colorNb,
	    GPU_mycolors2.begin(),
	    GPU_queue
	    );

  unsigned int sourceNb = 40;
  int *sources = new int [sourceNb*2];
  for (unsigned int k = 0; k < sourceNb; k++)
    {
      sources[k*2] = rand() % width;
      sources[k*2+1] = rand() % height;
      // printf ("%i %i\n", sources[k*2], sources[k*2+1]);
    }
    
  cl::vector <cl::int2_> GPU_sources = cl::vector <cl::int2_> (sourceNb, GPU_context);
  cl::copy (
	    reinterpret_cast <cl::int2_*> (sources),
	    reinterpret_cast <cl::int2_*> (sources) + sourceNb,
	    GPU_sources.begin(),
	    GPU_queue
	    );

  stop = false;
  int t = 0;
  while ((timeLength == 0 || t++ < timeLength) && ! stop)
    {
      // FRAME COUNT
      if (verbose) printf ("\nLOOP %u\n", currentLoop);
      currentLoop++;

      std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();

      int ms = std::chrono::duration_cast <std::chrono::milliseconds> (currentTime - previousTime).count();
      if (ms > 5000)
	{
	  float fps = 1000 * (float) (currentLoop - previousLoop) / ms;
	  printf ("FPS = %f (%u)\n", fps, currentLoop);
				
	  previousLoop = currentLoop;
	  previousTime = currentTime;
	}

      if (withKinect)
	{
	  pthread_mutex_lock (&kinect->mutex);
	  if (kinect->pointList != NULL)
	    {
	      // std::cout << "a1\n" << std::flush;
	      gettimeofday (&stepTimer, NULL);
	      float time = (stepTimer.tv_sec - startTimer.tv_sec) + (float)(stepTimer.tv_usec - startTimer.tv_usec) / 1000000L;
	      Data *data = new Data (time);
	  
	      pointNb = kinect->pointList->size();

	      for (PointList::iterator it = kinect->pointList->begin(); it != kinect->pointList->end(); ++it) data->mean += Coord (it->x, it->y, it->z);
	      if (pointNb != 0) data->mean /= pointNb;

	      for (PointList::iterator it = kinect->pointList->begin(); it != kinect->pointList->end(); ++it)
		{
		  data->var += pow (Coord (it->x, it->y, it->z) - data->mean, 2);
		  data->varxz += pow ((float) it->x - data->mean.x, 2) + pow ((float) it->z - data->mean.z, 2);
		  data->varxyz += pow ((float) it->x - data->mean.x, 2) + pow ((float) it->y - data->mean.y, 2) + pow ((float) it->z - data->mean.z, 2);
		}
	      if (pointNb != 0)
		{
		  data->var /= pointNb;
		  data->varxz /= pointNb;
		  data->varxyz /= pointNb;
		}
	      data->var = pow (data->var, 0.5);
	      data->varxz = pow (data->varxz, 0.5);
	      data->varxyz = pow (data->varxyz, 0.5);

	      // std::cout << "a2\n" << std::flush;
	      series.addData (data);
	    }
	  pthread_mutex_unlock (&kinect->mutex);

	  while (series.datas.size() > 100)
	    {
	      delete series.datas.front();
	      series.datas.pop_front();
	    }

	  cv::Mat pic (picHeight, picWidth, CV_32FC1, cv::Scalar(0));

	  // std::cout << "b\n" << std::flush;
	  Data *smooth = series.getSmoothData (5);
	  smooth->draw (pic);
	  smooth->diff->draw (pic, true);

	  // std::cout << "d\n" << std::flush;
	  // cv::imshow ("proto", pic);
	  // cv::waitKey (1);

	  if (false)
	    {
	      printf(" mean -> x=% 05.3f y=% 05.3f z=% 05.3f\n", smooth->mean.x, smooth->mean.y, smooth->mean.z);
	      printf("  var -> x=% 05.3f y=% 05.3f z=% 05.3f xz=% 05.3f xyz=% 05.3f\n", smooth->var.x, smooth->var.y, smooth->var.z, smooth->varxz, smooth->varxyz);
	      printf("dmean -> x=% 05.3f y=% 05.3f z=% 05.3f\n", smooth->diff->mean.x, smooth->diff->mean.y, smooth->diff->mean.z);
	      printf(" dvar -> x=% 05.3f y=% 05.3f z=% 05.3f xz=% 05.3f xyz=% 05.3f\n", smooth->diff->var.x, smooth->diff->var.y, smooth->diff->var.z, smooth->diff->varxz, smooth->diff->varxyz);
	      printf ("\n");
	    }

	  // INCARNASUONO
	  speedAddition = pointNb > 100;
	  speedAdditionX = width/2;
	  speedAdditionY = height/2;

	  float dmeanx = smooth->diff->mean.x;
	  float dmeanz = smooth->diff->mean.z;
	  if (abs (dmeanx) < 0.2) dmeanx = 0;
	  if (abs (dmeanz) < 0.2) dmeanz = 0;
	  speedAdditionDX = dmeanx;
	  speedAdditionDY = dmeanz;

	  float dmean = sqrt (pow (smooth->diff->mean.x, 2) + pow (smooth->diff->mean.y, 2) + pow (smooth->diff->mean.z, 2));
	  // if (abs (dmean) < 0.2) dmean = 0;
	  // printf ("dmean=% 5.3f\n", dmean);
	  

	  float varFeed = 0;
	  if (feedAxis == "x") varFeed = smooth->var.x;
	  else if (feedAxis == "y") varFeed = smooth->var.y;
	  else if (feedAxis == "z") varFeed = smooth->var.z;
	  else if (feedAxis == "xz") varFeed = smooth->varxz;
	  else if (feedAxis == "xyz") varFeed = smooth->varxyz;
	  float valFeed = scale (varFeed, varFeedMin, varFeedMax, valFeedMin, valFeedMax);

	  float varKill = 0;
	  if (killAxis == "x") varKill = smooth->var.x;
	  else if (killAxis == "y") varKill = smooth->var.y;
	  else if (killAxis == "z") varKill = smooth->var.z;
	  else if (killAxis == "xz") varKill = smooth->varxz;
	  else if (killAxis == "xyz") varKill = smooth->varxyz;
	  float valKill = scale (varKill, varKillMin, varKillMax, valKillMin, valKillMax);

	  envCurrent = {valFeed, valKill, dyeDiffusionFactorA, dyeDiffusionFactorB};

	  float varIntensity = 0;
	  if (IntensityAxis == "x") varIntensity = smooth->var.x;
	  else if (IntensityAxis == "y") varIntensity = smooth->var.y;
	  else if (IntensityAxis == "z") varIntensity = smooth->var.z;
	  else if (IntensityAxis == "xz") varIntensity = smooth->varxz;
	  else if (IntensityAxis == "xyz") varIntensity = smooth->varxyz;
	  float valIntensity = scale (varIntensity, varIntensityMin, varIntensityMax, valIntensityMin, valIntensityMax);
	  currentDyeDisplayMax = valIntensity;

	  float valColor = scale (dmean, varColorMin, varColorMax, valColorMin, valColorMax);
	  // printf("%f\n",valColor);
	  currentColorRatio = valColor;

	  delete smooth;

	  // printf("%i %i %i %i\n", speedAdditionX1, speedAdditionY1, speedAdditionX2, speedAdditionY2);
	}

      // MAIN CODE
      if (doSpeedAddition) GPU_doSpeedAddition();
      if (doSpeedPressure) GPU_doSpeedPressure();
      
      if (doSpeedAdvection) GPU_doSpeedAdvection();
      if (doSpeedPressure) GPU_doSpeedPressure();

      if (doDyeAddition) GPU_doDyeAddition(GPU_sources);

      for (unsigned int k = 0; k < dyeReactionIter; k++)
	{
	  if (doDyeDiffusion) GPU_doDyeDiffusion();
	  if (doDyeAdvection) GPU_doDyeAdvection();
	  if (doDyeReaction) GPU_doDyeReaction();
	}

      if (doDisplay == "dye") GPU_doDyeDisplay(GPU_mycolors1, GPU_mycolors2);
      else if (doDisplay == "speed") GPU_doSpeedDisplay();
      else if (doDisplay == "env") GPU_doEnvironmentDisplay();

      // usleep (60000);
    }

  if (verbose) printf ("STOP\n");

  if (withKinect)
    {
      void *status;
      rcKinect = pthread_join (kinectThread, &status);
      if (rcKinect) { std::cout << "Error: Unable to join thread " << rcKinect << std::endl; exit(-1); }
     }
      
  delete kinect;
	
  return 0;
}

