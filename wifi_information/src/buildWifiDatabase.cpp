#include "buildWifiDatabase.h"

fingerprint createWifiFingerprint(float posX, float posY){
    // a vector that stores the mean signal strength for every accesspoints scanned for 300 seconds collecting 30 samples
    float meanSignalStrength[NR_OF_AP];
    fingerprint currentFingerprint;

    WifiSignals wifi;
    int unsigned scan_interval = 10000000; //10s

    float meanSL = 0.0;
    int counterSL = 0;


    float meanSR = 0.0;
    int counterSR = 0;

    float meanLL = 0.0;
    int counterLL = 0;

    float meanLR = 0.0;
    int counterLR = 0;

    // Scan wifi signals for 300 seconds and take 30 samples
    for(int i=0; i<NR_OF_SCANS;i++){
        wifi = readWifiStrength();


        if (wifi.SL.signal_level != 0){
            meanSL = meanSL + (float)wifi.SL.signal_level;
            counterSL = counterSL + 1;
        }

        if (wifi.SR.signal_level != 0){
        meanSR = meanSR + (float)wifi.SR.signal_level;
        counterSR = counterSR + 1;
        }
        if (wifi.LL.signal_level != 0){
        meanLL = meanLL + (float)wifi.LL.signal_level;
        counterLL = counterLL + 1;
        }
        if (wifi.LR.signal_level != 0){
        meanLR = meanLR + (float)wifi.LR.signal_level;
        counterLR = counterLR + 1;
        }

        std::cout << "scanning..." << "sample number: " << i+1 << " of " << NR_OF_SCANS << std::endl;

        usleep(scan_interval);
    }

    if (counterSL != 0){
    meanSL = meanSL / (float)counterSL;
    }
    if (counterSR != 0){
    meanSR = meanSR / (float)counterSR;
    }
    if (counterLL != 0){
    meanLL = meanLL / (float)counterLL;
    }
    if (counterLR != 0){
    meanLR = meanLR / (float)counterLR;
    }

    meanSignalStrength[0] = meanSL;
    meanSignalStrength[1] = meanSR;
    meanSignalStrength[2] = meanLL;
    meanSignalStrength[3] = meanLR;


    currentFingerprint.pos.x = posX;
    currentFingerprint.pos.y = posY;
    // currentFingerprint.meanSignalStrengthArray = meanSignalStrength;
    std::copy(meanSignalStrength, meanSignalStrength+NR_OF_AP, currentFingerprint.meanSignalStrengthArray);


    std::cout << "posX: " << currentFingerprint.pos.x << std::endl;
    std::cout << "posY: " << currentFingerprint.pos.y << std::endl;

    for(int i=0; i < NR_OF_AP; i++){
        std::cout << "mean signal strength: " << currentFingerprint.meanSignalStrengthArray[i] << std::endl;
    }

    return currentFingerprint;
}

/***************Function that saves a Fingerprint to a Binary File********************/
void saveWifiFingerprintToBinaryFile(fingerprint fp, const char *path){

    std::ofstream outfile(path, std::ios::binary);
    outfile.write((char *)&fp, sizeof(fp));
    std::cout << "Fingerprint successfully written to binary file: " << path << std::endl;
}

fingerprint readWifiFingerprintFromBinaryFile(const char *path){

    fingerprint fp;
    std::ifstream ifs(path, std::ios::binary);
    ifs.read((char *)&fp, sizeof(fp));

    return fp;
}

/***************Function that parses the Fingerprints from binary files to the programm as the database ********************/
std::vector <fingerprint> parseWifiDatabase(){

    std::vector <fingerprint> WifiDatabase;
    const char *mypath1="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint1.dat";                     // set correct path to binary files here!!
    const char *mypath2="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint2.dat";
    const char *mypath3="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint3.dat";
    const char *mypath4="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint4.dat";
    const char *mypath5="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint5.dat";
    const char *mypath6="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint6.dat";
    const char *mypath7="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint7.dat";
    const char *mypath8="/home/tas_group_04/catkin_ws/src/tas_car_04/wifi_information/WifiDatabase/fingerprint8.dat";

    fingerprint myfp1,myfp2,myfp3,myfp4,myfp5,myfp6,myfp7,myfp8;


    myfp1 = readWifiFingerprintFromBinaryFile(mypath1);
    myfp2 = readWifiFingerprintFromBinaryFile(mypath2);
    myfp3 = readWifiFingerprintFromBinaryFile(mypath3);
    myfp4 = readWifiFingerprintFromBinaryFile(mypath4);
    myfp5 = readWifiFingerprintFromBinaryFile(mypath5);
    myfp6 = readWifiFingerprintFromBinaryFile(mypath6);
    myfp7 = readWifiFingerprintFromBinaryFile(mypath7);
    myfp8 = readWifiFingerprintFromBinaryFile(mypath8);


    WifiDatabase.push_back(myfp1);
    WifiDatabase.push_back(myfp2);
    WifiDatabase.push_back(myfp3);
    WifiDatabase.push_back(myfp4);
    WifiDatabase.push_back(myfp5);
    WifiDatabase.push_back(myfp6);
    WifiDatabase.push_back(myfp7);
    WifiDatabase.push_back(myfp8);

    return WifiDatabase;
}

/***************Function that matches a fingerprint to the database by using the manhattan distance between each accesspoints wifi strength********************/
PositionEstimate GetPositionEstimateByFingerprinting(std::vector <fingerprint> Db, fingerprint fp){
    // ignore x and y pos of fingerprint fp

    PositionEstimate pos_estimate;
    float manDist = 0.0;
    float manDist_old = 9999.0;
    fingerprint currentFingerprint;
	
    int minindex = 0;
    int min = 0;
    min = fp.meanSignalStrengthArray[0];
    float IndicesOfThreeHighestSignalStrength[NR_OF_AP-1];

    //FIND THE 3 STRONGEST SIGNALS of the Fingerprint fp and compare manDist only to those 3 Strongest accesspoints
    for(int n=0; n<NR_OF_AP; n++){
		if(min>fp.meanSignalStrengthArray[n]){
    			min=fp.meanSignalStrengthArray[n];
    			minindex = n;
  		}
	}
    
    int idx_counter = 0;	
    for (int n=0; n<NR_OF_AP;n++){
		if (n != minindex){
			IndicesOfThreeHighestSignalStrength[idx_counter] = n;
			idx_counter = idx_counter+1;		
		}
	}

/*    for(int j=0; j < NR_OF_AP-1; j++){
        std::cout << "strongest strength: " << IndicesOfThreeHighestSignalStrength[j] << std::endl;
    	}
*/
     int curIndex = 0;
    // Nearest Neighbour method: use manhatten distance as a metric to evaluate similarity with database signal patterns
    for (int i=0; i<Db.size(); i++){
        currentFingerprint = Db.at(i);
	
	

        for (int j=0; j<NR_OF_AP-1; j++){
	    curIndex = IndicesOfThreeHighestSignalStrength[j];
            manDist = manDist + fabs(currentFingerprint.meanSignalStrengthArray[curIndex] - fp.meanSignalStrengthArray[curIndex]);  
        }

//	std::cout << "manhattan distance: " << manDist << "to fingerprint from database with number" << i << std::endl;

        if (manDist < manDist_old){
                pos_estimate.x = currentFingerprint.pos.x;
                pos_estimate.y = currentFingerprint.pos.y;
                manDist_old = manDist;
        }
	manDist = 0.0;
    }
    return pos_estimate;
}

/***************Function that prints the entire database to the console********************/
void printDatabase(std::vector <fingerprint> db){
	fingerprint fp;	

	for(int i=0; i<db.size(); i++){
		fp = db.at(i);
		std::cout << "fingerprint number: " << i << std::endl;
		std::cout << "pos x: " << fp.pos.x << std::endl;
    		std::cout << "pos y: " << fp.pos.y << std::endl;
		for(int j=0; j < NR_OF_AP; j++){
        		std::cout << "mean signal strength: " << fp.meanSignalStrengthArray[j] << " to accesspoint: " << j+1 << std::endl;
    		}
	}	
}


