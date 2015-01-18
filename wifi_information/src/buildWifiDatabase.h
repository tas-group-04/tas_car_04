#include "readwifistrength.h"
#include <vector>
#include <unistd.h> // for usleep()
#include <cmath>

#include <iterator> //for std::copy copying arrays

// write to text file
#include <iostream>
#include <fstream>


#define NR_OF_SCANS 30
#define NR_OF_AP    4

struct PositionEstimate{
    float x;
    float y;
};


struct fingerprint
{
   struct position{
                    float x;
                    float y;

   } pos;
   float meanSignalStrengthArray[NR_OF_AP];

};

fingerprint createWifiFingerprint(float posX, float posY);


void saveWifiFingerprintToBinaryFile(struct fingerprint, const char *path);


fingerprint readWifiFingerprintFromBinaryFile(const char *path);


std::vector <fingerprint> parseWifiDatabase();

PositionEstimate GetPositionEstimateByFingerprinting(std::vector <fingerprint> Db, fingerprint fp);

void printDatabase(std::vector <fingerprint> db);
