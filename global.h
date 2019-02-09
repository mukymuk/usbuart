#include "ide.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define ARRAY_COUNT(x)  (sizeof(x)/sizeof(*(x)))
#define MIN(x,y)  ( ((x) > (y)) ? (y) : (x) )
