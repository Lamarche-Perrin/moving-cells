//  # -*- compile-command: "g++ -w -O3 -std=c++11 -o incarnasuono incarnasuono-V2.cpp -I/usr/local/include/compute/ -lOpenCL `pkg-config --cflags --libs opencv4`" -*-

/* SOURCES
 * http://developer.download.nvidia.com/books/HTML/gpugems/gpugems_ch38.html
 * https://www.dgp.toronto.edu/public_user/stam/reality/Research/pdf/GDC03.pdf
 * http://www.degeneratestate.org/posts/2017/May/05/turing-patterns/
 * https://mathematica.stackexchange.com/questions/199577/turing-patterns
 * http://experiences.math.cnrs.fr/Structures-de-Turing.html
 *
 * https://www.lanevol.org/resources/gray-scott
 * https://github.com/pmneila/jsexp/blob/master/grayscott/index.html
 */

#define CL_TARGET_OPENCL_VERSION 220

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#include <random>
#include <vector>
#include <chrono>
#include <math.h>

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

bool verbose = false;

#define width       1920/4
#define height      1080/4
#define time          0

#define toScreenWait  1

std::string doDisplay =      "dye";
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

float speedAdditionRadius =        1.0;
float speedAdditionQuant =         0.5;
float dyeAdditionRadius =          5.;
float dyeAdditionQuant =           0.9;
float envAdditionRadius =         10.;
float envAdditionQuant =           0.900;
float envSubstractionQuant =       0.002;

float dyeDiffusionFactorA =        0.2097;
float dyeDiffusionFactorB =        0.105;
cl::float2_ dyeDiffusionFactors = {dyeDiffusionFactorA, dyeDiffusionFactorB};

unsigned int envInit = 0;
unsigned int envTarget = 3;

float envRadius =      50.;
float envRatio =        0.1;

unsigned int envNb = 5;
cl::float4_ *envs;

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

void GPU_doDyeAddition ();
void GPU_doDyeDiffusion ();
void GPU_doDyeAdvection ();
void GPU_doDyeReaction ();

void GPU_doSpeedBoundary ();
void GPU_doDivergenceBoundary ();
void GPU_doPressureBoundary ();
void GPU_doDyeBoundary ();

void GPU_doSpeedDisplay ();
void GPU_doDyeDisplay ();
void GPU_doEnvironmentDisplay ();

void keyPressed (int key);

// COLORS
const unsigned int colorNb = 1000;
const std::string colorFilename = "colors";

float *colors;
cl::vector <cl::float4_> GPU_colors;

void getColors ()
{
  colors = new float [colorNb*4];

  std::string line;
  std::ifstream colorFile (colorFilename);

  std::getline (colorFile, line);
  std::istringstream iss (line);
  for (int k = 0; k < colorNb; k++)
    {
      std::string colorHex;
      iss >> colorHex;

      int colorRGB;
      std::stringstream ss;
      ss << std::hex << colorHex.substr(1,6);
      ss >> colorRGB;

      colors[4*k+0] = (float) ((colorRGB >> 16) & 0xFF) / 255.;
      colors[4*k+1] = (float) ((colorRGB >> 8) & 0xFF) / 255.;
      colors[4*k+2] = (float) ((colorRGB) & 0xFF) / 255.;
      colors[4*k+3] = 1.;
    }
  
  colorFile.close();

  GPU_colors = cl::vector <cl::float4_> (colorNb, GPU_context);

  cl::copy (
	    reinterpret_cast <cl::float4_*> (colors),
	    reinterpret_cast <cl::float4_*> (colors) + colorNb,
	    GPU_colors.begin(),
	    GPU_queue
	    );
}


// ADDITION VARIABLES

bool speedAddition = false;
int speedAdditionX1, speedAdditionY1, speedAdditionX2, speedAdditionY2;

bool dyeAdditionA = false;
bool dyeAdditionB = false;
int dyeAdditionX, dyeAdditionY;

bool env = false;
int envX, envY;

void callbackFunction (int event, int x, int y, int flags, void *userdata)
{
  switch (event)
    {
    case cv::EVENT_LBUTTONDOWN :
      // speedAddition = true;
      // speedAdditionX1 = x;
      // speedAdditionY1 = y;
      dyeAdditionB = true;
      break;

    case cv::EVENT_LBUTTONUP :
      // speedAddition = false;
      dyeAdditionB = false;
      break;

    case cv::EVENT_RBUTTONDOWN :
      speedAddition = true;
      speedAdditionX1 = x;
      speedAdditionY1 = y;
      env = true;
      break;
      
    case cv::EVENT_RBUTTONUP :
      speedAddition = false;
      env = false;
      break;
      
    case cv::EVENT_MOUSEMOVE :
      speedAdditionX2 = x;
      speedAdditionY2 = y;
      dyeAdditionX = x;
      dyeAdditionY = y;
      envX = x;
      envY = y;
      break;
    }
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

  // origin = new size_t [2];
  // origin[0] = 0;
  // origin[1] = 0;

  // GPU VARIABLES
  if (verbose) printf ("INIT GPU VARIABLES\n");

  GPU_speed = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  GPU_speedNew = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  cl::float4_ initSpeed (speedInit, speedInit, envs[envInit].x, envs[envInit].y);
  GPU_queue.enqueue_fill_image (GPU_speed, &initSpeed, GPU_speed.origin(), GPU_speed.size());

  GPU_dye = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  GPU_dyeNew = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  cl::float4_ initDye (dyeInitA, dyeInitB, 0., 0.);
  GPU_queue.enqueue_fill_image (GPU_dye, &initDye, GPU_dye.origin(), GPU_dye.size());
  
  GPU_div = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);
  GPU_divNew = cl::image2d (GPU_context, width, height, rgbaFloat, cl::image2d::read_write);

  frame = cv::Mat (height, width, CV_8UC4, cv::Scalar (1,1,1));
  GPU_frame = cl::image2d (GPU_context, width, height, bgraInt8, cl::image2d::read_write);
  GPU_queue.enqueue_write_image (GPU_frame, GPU_frame.origin(), GPU_frame.size(), frame.data);

  getColors();
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
	  int x1,
	  int y1,
	  int x2,
	  int y2,
	  float2 radius,
	  float2 quant,
	  float4 target,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};

	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
	   float4 value = read_imagef (speedIn, sampler, coord);

	   const float2 q = {(x2 - x1), (y2 - y1)};
	   const float r = sqrt (pow ((float) x - x2, 2) + pow ((float) y - y2, 2)); // TODO TORUS
	   value.xy += diff.z * quant.x * q * exp (- r / radius.x);
	   value.zw += diff.z * quant.y * exp (- r / radius.y) * (target.xy - value.zw);

	   write_imagef (speedOut, coord, value);
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
	  float quant,
	  float4 init,
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

	   value2.zw += diff.z * quant * (init.xy - value2.zw);
	   
	   write_imagef (speedOut, coord, value2);
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
	  uint x1,
	  uint y1,
	  float radius,
	  float2 quant,
	  uint2 size,
	  float4 diff
	  )
	 {
	   const uint x = get_global_id (0);
	   const uint y = get_global_id (1);
	   const int2 coord = {x, y};

	   const sampler_t sampler = CLK_ADDRESS_NONE | CLK_FILTER_NEAREST;
	   float4 value = read_imagef (dyeIn, sampler, coord);
	   
	   const float r = sqrt (pow ((float) x - x1, 2) + pow ((float) y - y1, 2));
	   if (r < radius) value.xy = quant;
	   
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
      __global const float4 *colors,
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

       int colorIndex = (value.y - min) / (max - min) * colorNb;
       if (colorIndex < 0) colorIndex = 0; else if (colorIndex >= colorNb) colorIndex = colorNb-1;
	   
       write_imagef (frameOut, coord, colors[colorIndex]);
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
      __global const float4 *colors,
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
	 
       write_imagef (frameOut, coord, colors[colorIndex]);
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
  if (! speedAddition || (speedAdditionX1 == speedAdditionX2 && speedAdditionY1 == speedAdditionY2)) return;

  if (verbose) printf ("SPEED ADDITION\n");

  cl::kernel GPU_speedAddition_kernel (GPU_speedAddition_program, "GPU_speedAddition_kernel");
  GPU_speedAddition_kernel.set_arg (0, GPU_speedNew);
  GPU_speedAddition_kernel.set_arg (1, GPU_speed);
  GPU_speedAddition_kernel.set_arg (2, speedAdditionX1);
  GPU_speedAddition_kernel.set_arg (3, speedAdditionY1);
  GPU_speedAddition_kernel.set_arg (4, speedAdditionX2);
  GPU_speedAddition_kernel.set_arg (5, speedAdditionY2);
  GPU_speedAddition_kernel.set_arg (6, (cl::float2_) {speedAdditionRadius, envAdditionRadius});
  GPU_speedAddition_kernel.set_arg (7, (cl::float2_) {speedAdditionQuant, envAdditionQuant});
  GPU_speedAddition_kernel.set_arg (8, envs[envTarget]);
  GPU_speedAddition_kernel.set_arg (9, size);
  GPU_speedAddition_kernel.set_arg (10, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_speedAddition_kernel, 2, origin, region, 0);

  GPU_speed = GPU_speedNew;

  speedAdditionX2 = speedAdditionX1;
  speedAdditionY2 = speedAdditionY1;
}


void GPU_doSpeedAdvection ()
{
  if (verbose) printf ("SPEED ADVECTION\n");

  cl::kernel GPU_speedAdvection_kernel (GPU_speedAdvection_program, "GPU_speedAdvection_kernel");
  GPU_speedAdvection_kernel.set_arg (0, GPU_speedNew);
  GPU_speedAdvection_kernel.set_arg (1, GPU_speed);
  GPU_speedAdvection_kernel.set_arg (2, envSubstractionQuant);
  GPU_speedAdvection_kernel.set_arg (3, envs[envInit]);
  GPU_speedAdvection_kernel.set_arg (4, size);
  GPU_speedAdvection_kernel.set_arg (5, diff);
  GPU_queue.enqueue_nd_range_kernel (GPU_speedAdvection_kernel, 2, origin, region, 0);

  GPU_speed = GPU_speedNew;

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

  for (int n = 0; n < speedPressureIter; n++)
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


void GPU_doDyeAddition ()
{
  if (dyeAdditionA || dyeAdditionB)
    {
      if (verbose) printf ("DYE ADDITION\n");

      cl::float2_ dyeAdditionQuant = {0, 0};
      if (dyeAdditionA) dyeAdditionQuant.x = dyeAdditionA;
      if (dyeAdditionB) dyeAdditionQuant.y = dyeAdditionB;

      cl::kernel GPU_dyeAddition_kernel (GPU_dyeAddition_program, "GPU_dyeAddition_kernel");
      GPU_dyeAddition_kernel.set_arg (0, GPU_dyeNew);
      GPU_dyeAddition_kernel.set_arg (1, GPU_dye);
      GPU_dyeAddition_kernel.set_arg (2, dyeAdditionX);
      GPU_dyeAddition_kernel.set_arg (3, dyeAdditionY);
      GPU_dyeAddition_kernel.set_arg (4, dyeAdditionRadius);
      GPU_dyeAddition_kernel.set_arg (5, dyeAdditionQuant);
      GPU_dyeAddition_kernel.set_arg (6, size);
      GPU_dyeAddition_kernel.set_arg (7, diff);
      GPU_queue.enqueue_nd_range_kernel (GPU_dyeAddition_kernel, 2, origin, region, 0);

      GPU_dye = GPU_dyeNew;

      if (doDyeBoundary) GPU_doDyeBoundary();
    }
}


void GPU_doDyeDiffusion ()
{
  if (verbose) printf ("DYE DIFFUSION\n");

  cl::float2_ dyeDiffusionFactor = {dyeDiffusionFactorA, dyeDiffusionFactorB};
  
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


void GPU_doDyeDisplay ()
{
  if (verbose) printf ("DYE DISPLAY\n");

  cl::kernel GPU_dyeDisplay_kernel (GPU_dyeDisplay_program, "GPU_dyeDisplay_kernel");
  GPU_dyeDisplay_kernel.set_arg (0, GPU_frame);
  GPU_dyeDisplay_kernel.set_arg (1, GPU_dye);
  GPU_dyeDisplay_kernel.set_arg (2, GPU_colors);
  GPU_dyeDisplay_kernel.set_arg (3, colorNb);
  GPU_dyeDisplay_kernel.set_arg (4, dyeDisplayMin);
  GPU_dyeDisplay_kernel.set_arg (5, dyeDisplayMax);
  GPU_dyeDisplay_kernel.set_arg (6, size);
  GPU_dyeDisplay_kernel.set_arg (7, diff);
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
  GPU_speedDisplay_kernel.set_arg (2, GPU_colors);
  GPU_speedDisplay_kernel.set_arg (3, colorNb);
  GPU_speedDisplay_kernel.set_arg (4, speedDisplayMin);
  GPU_speedDisplay_kernel.set_arg (5, speedDisplayMax);
  GPU_speedDisplay_kernel.set_arg (6, size);
  GPU_speedDisplay_kernel.set_arg (7, diff);
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
