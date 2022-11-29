#ifndef IMAGE_INTERFACE_H
#define IMAGE_INTERFACE_H

#include "image.h"
#include "opencv2/core/types_c.h"

static float get_pixel(image m, int x, int y, int c);
image** load_alphabet_with_file(char* datafile);
void generate_image(image p, IplImage* disp);

#endif
