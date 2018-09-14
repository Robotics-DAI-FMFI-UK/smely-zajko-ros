#include <stdint.h>


static void bgr_to_hsv(uint8_t *p)
{
  uint8_t h, s, v;
  double hD;
  double r, g, b, rgb_c, rgb_max, rgb_min;

  b = *(p++);
  g = *(p++);
  r = *p;

  if (r > g)
  {
    if (r > b)
    {
      rgb_max = r;
      if (g > b) rgb_min  = b;
      else rgb_min = g;
    }
    else
    {
      rgb_max = b;
      rgb_min = g;
    }
  }
  else if (g > b)
  {
    rgb_max = g;
    if (r > b) rgb_min = b;
    else rgb_min = r;
  }
  else
  {
    rgb_max = b;
    rgb_min = r;
  }
  rgb_c = rgb_max - rgb_min;
  if (rgb_c == 0) hD = 0;
  else if (rgb_max == r) hD = (g - b) * 42.5 / rgb_c;
  else if (rgb_max == g) hD = (b - r) * 42.5 / rgb_c + 85;
  else hD = (r - g) * 42.5 / rgb_c + 170;
  if (hD < 0) hD += 255;
  h = (uint8_t)(hD + 0.5);
  if (rgb_max == 0) s = 0;
  else s = (uint8_t)(0.5 + (rgb_max - rgb_min) / (double)rgb_max * 255.0);
  v = rgb_max;

//  *(p--) = r / 2;
//  *(p--) = g;
//  *p = b; 
  *(p--) = h;
  *(p--) = s;
  *p = v; 
}


int rgb2hsv(uint8_t *img, int width, int height)
{
  uint8_t *p = img;
  int len = width * height;
  for (int i = 0; i < len; i++)
  {
      bgr_to_hsv(p);
      p += 3;
  }

  return 0;
}

