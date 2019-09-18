#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define TRIANGLE_WIDTH 58
#define TRIANGLE_HEIGHT 430

// pose of the camera relative to the vertical axis; cm, radians
#define CAMERA_POSITION_ABOVE_GROUND 58.0
#define CAMERA_INCLINATION (25 / 180.0 * M_PI)  

// when a perpendicular surface is placed 1m in front of the camera, the view field of the camera has the following dimensions on that surface:
#define CALIBRATED_HEIGHT 98.0
#define CALIBRATED_WIDTH 132.0

// how many cells the output grid contains
#define OUTPUT_SIZE 60 

static uint8_t positive[255][255];
static uint8_t **interm;
static uint8_t *evaluated;
static double **x_precomputed_displacements, *y_precomputed_distances;
static int w, h;
static int out_counts[OUTPUT_SIZE][OUTPUT_SIZE];
static long out_vals[OUTPUT_SIZE][OUTPUT_SIZE];

static void reallocate(int new_w)
{
  if (evaluated) free(evaluated);
  if (x_precomputed_displacements) 
  {
    for (int i = 0; i < w; i++)
    {
      free(x_precomputed_displacements[i]);
      free(interm[i]);
    }
    free(interm);
    free(x_precomputed_displacements);
  }
  if (y_precomputed_distances) free(y_precomputed_distances);
  w = new_w;

  evaluated = (uint8_t *) malloc(w * h);
  x_precomputed_displacements = (double **) malloc(w * sizeof(double *));
  interm = (uint8_t **) malloc(w * sizeof(uint8_t *));
  for (int i = 0; i < w; i++)
  {
    x_precomputed_displacements[i] = (double *) malloc(h * sizeof(double));
    interm[i] = (uint8_t *) malloc(h * sizeof(uint8_t));
  }
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

int init_triangle(uint8_t *img)
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

static uint8_t max(uint8_t a, uint8_t b)
{
  if (a > b) return a;
  return b;
}

int evaluate_triangles(uint8_t *img, uint8_t *preview, double *out, int width, int height ) //, uint8_t *dbg)
{
  if ((width != w) || (height != h))
  {
    h = height;
    reallocate(width);
  }

  uint8_t *p = img;

  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      uint8_t v = *(p++);
      uint8_t s = *(p++);
      uint8_t h = *(p++);

      uint8_t val = positive[h][s];

      interm[x][y] = val;
      preview[y * 640 + x] = val;
    }
  }

  for (int i = 0; i < 11; i++) out[i] = 0.0;

  for (int x = 0; x < 11; x++)
    for (int y = 50; y < h; y++)
    {
      double s = 0;
      int triangle_width_in_this_y = y * TRIANGLE_WIDTH/TRIANGLE_HEIGHT;
      for (int x_in_this_y = x * TRIANGLE_WIDTH - triangle_width_in_this_y / 2; 
           x_in_this_y < x * TRIANGLE_WIDTH + triangle_width_in_this_y / 2; 
           x_in_this_y++)
      {
        if ((x_in_this_y > 0) && (x_in_this_y < w))
          out[x] += interm[x_in_this_y][y];
        s += 255;
      }
      out[x] /= s;
    }

  return 0;
}

