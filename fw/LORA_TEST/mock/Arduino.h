#include <stdint.h>
#include <math.h>
typedef uint8_t byte;
#define millis() 0
static double sq(double m) {
	return m*m;
}
#define TWO_PI (2*M_PI)
static double radians(double m) {
	return m*M_PI/180;
}
static double degrees(double m) {
	return m*180/M_PI;
}
