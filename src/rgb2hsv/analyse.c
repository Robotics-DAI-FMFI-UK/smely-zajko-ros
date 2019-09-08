#include <stdio.h>
#include <stdint.h>
#include <math.h>

static int positive[255][255];
static int negative[255][255];
static int count = 0;

static void ana_count(uint8_t *p, uint8_t *output)
{
  uint8_t v = *(p++);
  uint8_t s = *(p++);
  uint8_t h = *p; 

  if (*output) positive[h][s]++;
  else negative[h][s]++;
  count++;
}

static double logf_2;

int init()
{
  for (int i = 0; i < 255; i++) 
    for (int j = 0; j < 255; j++)
    {
      positive[i][j] = 0;
      negative[i][j] = 0;
    }
  logf_2 = logf(2);
  return 0;
}


int ana(uint8_t *img, uint8_t *output, int width, int height)
{
  uint8_t *p = img;
  uint8_t *q = output;
  int len = width * height;
  for (int i = 0; i < len; i++)
  {
      ana_count(p, q);
      p += 3;
      q += 3;
  }

  return 0;
}

static int fn1(int x)
{
  int y = (int)(20.0 * logf(x) / logf_2);
  if (y > 255) return 255;
  else return y;
}

static int fn2(int x)
{
  int y = (int)(20.0 * logf(x) / logf_2);
  if (y > 255) return 255;
  else return y;
}

int dump_results(uint8_t *img1, uint8_t *img2)
{
  int max_p = 0;
  int max_n = 0;
  for (int i = 0; i < 255; i++)
  {
    for (int j = 0; j < 255; j++)
    {
      printf("%5d", positive[i][j]);
       img1[i * 255 * 3 + j * 3] = fn1(positive[i][j]);
       img1[i * 255 * 3 + j * 3 + 1] = fn1(positive[i][j]);
       img1[i * 255 * 3 + j * 3 + 2] = fn1(positive[i][j]);
      if (positive[i][j] > max_p) max_p = positive[i][j];
    }
    printf("\n");
  }
  printf("\n");
  
  for (int i = 0; i < 255; i++)
  {
    for (int j = 0; j < 255; j++)
    {
      printf("%5d", negative[i][j]);
       img2[i * 255 * 3 + j * 3] = fn2(negative[i][j]);
       img2[i * 255 * 3 + j * 3 + 1] = fn2(negative[i][j]);
       img2[i * 255 * 3 + j * 3 + 2] = fn2(negative[i][j]);
      if (negative[i][j] > max_n) max_n = negative[i][j];
    }
    printf("\n");
  }
  printf("\n");
  printf("p:%d n:%d\n", max_p, max_n);
  return count;
}

int transform(uint8_t *img, int width, int height, uint8_t *trans)
{
  uint8_t *p = img;
  int len = width * height;
  for (int i = 0; i < len; i++)
  { 
      uint8_t v = *(p++);
      uint8_t s = *(p++);
      uint8_t h = *p;
      p -= 2;

      uint8_t val = trans[h * 255 * 3 + s * 3];
      *(p++) = val;
      *(p++) = val;
      *(p++) = val;
  }

  return 0;
}

