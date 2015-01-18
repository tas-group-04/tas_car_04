#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

#define MACADRESS_STUDENTROOM_LEFT "54:A0:50:5B:39:F8"
#define MACADRESS_STUDENTROOM_RIGHT "54:A0:50:5B:32:58"
#define MACADRESS_LABORATORY_LEFT "54:A0:50:5B:20:80"
#define MACADRESS_LABORATORY_RIGHT "54:A0:50:5B:4D:E8"



struct WifiSignals
{
   struct Studentroom_Left{
                    const char* MAC_id;
                    int signal_level;
                    float freqInMHz;
                    float signalLevelInDb;
   } SL;
   struct Studentroom_Right{
                    const char* MAC_id;
                    int signal_level;
                    float freqInMHz;
                    float signalLevelInDb;
   } SR;
   struct Laboratory_Left{
                    const char* MAC_id;
                    int signal_level;
                    float freqInMHz;
                    float signalLevelInDb;
   } LL;
   struct Laboratory_Right{
                    const char* MAC_id;
                    int signal_level;
                    float freqInMHz;
                    float signalLevelInDb;
   } LR;
};



std::string exec(char const* cmd);


struct WifiSignals readWifiStrength();


float calculateDistance(float signalLevelInDb, float freqInMHz);
