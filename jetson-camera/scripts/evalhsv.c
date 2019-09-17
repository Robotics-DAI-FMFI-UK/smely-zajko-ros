#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// pose of the camera relative to the vertical axis; cm, radians
#define CAMERA_POSITION_ABOVE_GROUND 58.0
#define CAMERA_INCLINATION (25 / 180.0 * M_PI)  

// when a perpendicular surface is placed 1m in front of the camera, the view field of the camera has the following dimensions on that surface:
#define CALIBRATED_HEIGHT 98.0
#define CALIBRATED_WIDTH 132.0

// how many cells the output grid contains
#define OUTPUT_SIZE 60 

static uint8_t positive[255][255];
static uint8_t *evaluated;
static double **x_precomputed_displacements, *y_precomputed_distances;
static int w, h;
static int out_counts[OUTPUT_SIZE][OUTPUT_SIZE];
static long out_vals[OUTPUT_SIZE][OUTPUT_SIZE];

void reallocate(int new_w)
{
  if (evaluated) free(evaluated);
  if (x_precomputed_displacements) 
  {
    for (int i = 0; i < w; i++)
      free(x_precomputed_displacements[i]);
    free(x_precomputed_displacements);
  }
  if (y_precomputed_distances) free(y_precomputed_distances);
  w = new_w;

  evaluated = (uint8_t *) malloc(w * h);
  x_precomputed_displacements = (double **) malloc(w * sizeof(double *));
  for (int i = 0; i < w; i++)
    x_precomputed_displacements[i] = (double *) malloc(h * sizeof(double));
  y_precomputed_distances = (double *) malloc(h * sizeof(double));

  for (int y = 0; y < h; y++)
    y_precomputed_distances[y] = CAMERA_POSITION_ABOVE_GROUND / tan(CAMERA_INCLINATION + atan((y - h / 2) / (double)h * CALIBRATED_HEIGHT / 100.0));
  
  for (int x = 0; x < w; x++)
    for (int y = 0; y < h; y++)
    {
      double x_in_cm = (x - (w / 2)) / (double) w * CALIBRATED_WIDTH;
      x_precomputed_displacements[x][y] = (y_precomputed_distances[y] * x_in_cm / 100.0);
    }

/*
  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
      printf("%.2lf ", x_precomputed_displacements[x][y]);
    printf("\n");
  }
*/

  // rescale to 10cm grid 60x60
  for (int y = 0; y < h; y++)
  {
    y_precomputed_distances[y] = y_precomputed_distances[y] / 10.0;
    if (y_precomputed_distances[y] >= OUTPUT_SIZE) y_precomputed_distances[y] = -1;
  }
  for (int x = 0; x < w; x++)
  {
    for (int y = 0; y < h; y++)
    {
      x_precomputed_displacements[x][y] = x_precomputed_displacements[x][y] / 10.0 + OUTPUT_SIZE / 2.0;
      if (x_precomputed_displacements[x][y] >= OUTPUT_SIZE) x_precomputed_displacements[x][y] = -1;
    } 
  }
}

int init(uint8_t *img)
{
  for (int i = 0; i < 255; i++) 
    for (int j = 0; j < 255; j++)
      positive[i][j] = img[i * 255 * 3 + j * 3];
  w = 640;
  h = 480;

  evaluated = 0;
  x_precomputed_displacements = 0;
  y_precomputed_distances = 0;

  reallocate(w);
  return 0;
}

uint8_t max(uint8_t a, uint8_t b)
{
  if (a > b) return a;
  return b;
}

int evaluate(uint8_t *img, uint8_t *out, int width, int height ) //, uint8_t *dbg)
{
  if ((width != w) || (height != h))
  {
    h = height;
    reallocate(width);
  }

  uint8_t *q = out;
  for (int i = 0; i < OUTPUT_SIZE * OUTPUT_SIZE; i++)
     *(q++) = 0;

  uint8_t *p = img;
  //uint8_t *qq = dbg;
  for (int y = 0; y < OUTPUT_SIZE; y++)
    for (int x = 0; x < OUTPUT_SIZE; x++)
    {
      out_vals[y][x] = 0;
      out_counts[y][x] = 0;
    }

  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
//      uint8_t v = *(p++);
//      uint8_t s = *(p++);
//      uint8_t h = *(p++);
//   

//      uint8_t val = 255 - positive[h][s];

      //*(qq++) = val;
      //*(qq++) = val;
      //*(qq++) = val;

      uint8_t val = (*(p++) > 0);

      int floor_y = (int)y_precomputed_distances[y];
      int floor_x = (int)x_precomputed_displacements[x][y];

      if ((floor_x >= 0) && (floor_y >= 0) && (floor_x < 60) && (floor_y < 60))
      {
         out_vals[floor_y][floor_x] += val;
         out_counts[floor_y][floor_x]++;
      }
    }
  }
  for (int y = 0; y < OUTPUT_SIZE; y++)
    for (int x = 0; x < OUTPUT_SIZE; x++)
      if (out_counts[y][x]) out[y * OUTPUT_SIZE + x] = 1 + (uint8_t)((254 * out_vals[y][x]) / out_counts[y][x]);
      else out[y * OUTPUT_SIZE + x] = 0;

  return 0;
}

