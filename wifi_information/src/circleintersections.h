#include <iostream>
#include <cmath>
#include <vector>
#include <string.h>

// Define the X,Y positions of the accesspoints HERE
#define STUDENTROOM_LEFT_X 23.3
#define STUDENTROOM_LEFT_Y 7.33
#define STUDENTROOM_RIGHT_X 9.91
#define STUDENTROOM_RIGHT_Y 7.2
#define LABORATORY_LEFT_X 21.8
#define LABORATORY_LEFT_Y 20.6
#define LABORATORY_RIGHT_X 14.066
#define LABORATORY_RIGHT_Y 21.011


#define MACADRESS_STUDENTROOM_LEFT "54:A0:50:5B:39:F8"
#define MACADRESS_STUDENTROOM_RIGHT "54:A0:50:5B:32:58"
#define MACADRESS_LABORATORY_LEFT "54:A0:50:5B:20:80"
#define MACADRESS_LABORATORY_RIGHT "54:A0:50:5B:4D:E8"


std::vector<float> compute_circle_intersections(float radius1, float radius2, const char* MAC_ID_1, const char* MAC_ID_2);
