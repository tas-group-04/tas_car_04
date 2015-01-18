#include "readwifistrength.h"




/*-------------------------------------------- function that calculates relative distance in meter to wifi router given a RSSI----------------------------------------------*/
float calculateDistance(float signalLevelInDb, float freqInMHz) {
    // freq in MHz, signalLevel in Dbm (POSITIVE value!)

    float exp = (27.55 - (20 * log10(freqInMHz)) + std::abs(signalLevelInDb)) / 20.0;
    return pow(10.0, exp);
}





/*-------------------------------------------- function that sends shell command----------------------------------------------*/
std::string exec(char const* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);
    return result;
}


/*-------------------------------------------- function that reads the wifi strength to TAS-NET----------------------------------------------*/
struct WifiSignals readWifiStrength(){

        WifiSignals ws;
        std::size_t found_position = 0;
        std::string SignalLevel,Frequency;
        int buf = 0;
        float buf_float = 0.0;


        ws.SR.signalLevelInDb = 0.0;
        ws.SL.signalLevelInDb = 0.0;
        ws.LL.signalLevelInDb = 0.0;
        ws.LR.signalLevelInDb = 0.0;

        ws.SR.freqInMHz = 0.0;
        ws.SL.freqInMHz = 0.0;
        ws.LL.freqInMHz = 0.0;
        ws.LR.freqInMHz = 0.0;

        /*------------------------ send shell command----------------------------------------------*/
        /*
         *
        ATTENTION: EVERY CAR MAY BE DIFFERENT HERE!!! -- "wlan3" check with iwconfig
        Read terminal command: iwlist wlan3 scan
        futurama: wlan3
        vettel: wlan1
        gerty: wlan3
        *
        */
        char const* wifi_scan_command = "iwlist wlan3 scan";
        std::string scan_result;
        scan_result =  exec(wifi_scan_command);
        //std::cout << scan_result << std::endl;

        /*------------------------- search for MAC Adresses----------------------------------------------*/

        /*------------------------- MACADRESS_STUDENTROOM_LEFT----------------------------------------------*/
        ws.SL.MAC_id = MACADRESS_STUDENTROOM_LEFT;
        found_position = scan_result.find(MACADRESS_STUDENTROOM_LEFT);
        if (found_position!=std::string::npos){
            //std::cout << "Found MACADRESS_STUDENTROOM_LEFT: " << found_position << std::endl;

            // find frequency
            found_position=scan_result.find("Frequency:",found_position+1);
            if (found_position!=std::string::npos){
                Frequency = scan_result.substr(found_position+10,5);
                buf_float = atof(Frequency.c_str());
                buf_float = buf_float * 1000;
                ws.SL.freqInMHz = buf_float;
                //std::cout << "ws.SL.freqInMHz " << ws.SL.freqInMHz << std::endl;
            }

            // find signal level
            found_position=scan_result.find("Signal level=",found_position+1);
            if (found_position!=std::string::npos){

                //std::cout << "Signal level found at: " << found_position << '\n';
                SignalLevel = scan_result.substr(found_position+13,7);
                //std::cout << "Signal Level: " << SignalLevel << std::endl;

                // convert string SignalLevel to integer
                found_position = SignalLevel.find("/");
                SignalLevel = SignalLevel.substr(found_position-2,2); // writes the string into substring of only 2 characters, so "00" means the value "100"
                if(SignalLevel.compare("00") == 0){
                    SignalLevel = "100";
                }
                buf = atoi(SignalLevel.c_str());

                ws.SL.signal_level = buf;
                //std::cout << "ws.SL.signal_level " << ws.SL.signal_level << std::endl;

                ws.SL.signalLevelInDb = (ws.SL.signal_level / 2) - 100;

            }
        }
        else{
//            std::cout << "MACADRESS_STUDENTROOM_LEFT NOT FOUND" << std::endl;
            ws.SL.signal_level = 0;    // 0 means macadress not found
        }



        /*-------------------------- MACADRESS_STUDENTROOM_RIGHT----------------------------------------------*/
        ws.SR.MAC_id = MACADRESS_STUDENTROOM_RIGHT;
        found_position = scan_result.find(MACADRESS_STUDENTROOM_RIGHT);
        if (found_position!=std::string::npos){
            //std::cout << "Found MACADRESS_STUDENTROOM_RIGHT: " << found_position << std::endl;

            // find frequency
            found_position=scan_result.find("Frequency:",found_position+1);
            if (found_position!=std::string::npos){
                Frequency = scan_result.substr(found_position+10,5);
                buf_float = atof(Frequency.c_str());
                buf_float = buf_float * 1000;
                ws.SR.freqInMHz = buf_float;
                //std::cout << "ws.SR.freqInMHz " << ws.SR.freqInMHz << std::endl;
            }

            // find signal level
            found_position=scan_result.find("Signal level=",found_position+1);
            if (found_position!=std::string::npos){

                //std::cout << "Signal level found at: " << found_position << '\n';
                SignalLevel = scan_result.substr(found_position+13,7);
                //std::cout << "Signal Level: " << SignalLevel << std::endl;

                // convert string SignalLevel to integer
                found_position = SignalLevel.find("/");
                SignalLevel = SignalLevel.substr(found_position-2,2); // writes the string into substring of only 2 characters, so "00" means the value "100"
                if(SignalLevel.compare("00") == 0){
                    SignalLevel = "100";
                }
                buf = atoi(SignalLevel.c_str());

                ws.SR.signal_level = buf;
                //std::cout << "ws.SR.signal_level " << ws.SR.signal_level << std::endl;

                ws.SR.signalLevelInDb = (ws.SR.signal_level / 2) - 100;
            }
        }
        else{
 //           std::cout << "MACADRESS_STUDENTROOM_RIGHT NOT FOUND" << std::endl;
            ws.SR.signal_level = 0;    // -1 means macadress not found
        }



        /*--------------------------- MACADRESS_LABORATORY_LEFT----------------------------------------------*/
        ws.LL.MAC_id = MACADRESS_LABORATORY_LEFT;
        found_position = scan_result.find(MACADRESS_LABORATORY_LEFT);
        if (found_position!=std::string::npos){
            //std::cout << "Found MACADRESS_LABORATORY_LEFT: " << found_position << std::endl;

            // find frequency
            found_position=scan_result.find("Frequency:",found_position+1);
            if (found_position!=std::string::npos){
                Frequency = scan_result.substr(found_position+10,5);
                buf_float = atof(Frequency.c_str());
                buf_float = buf_float * 1000;
                ws.LL.freqInMHz = buf_float;
                //std::cout << "ws.LL.freqInMHz " << ws.LL.freqInMHz << std::endl;
            }
            // find signal level
            found_position=scan_result.find("Signal level=",found_position+1);
            if (found_position!=std::string::npos){

                //std::cout << "Signal level found at: " << found_position << '\n';
                SignalLevel = scan_result.substr(found_position+13,7);
                //std::cout << "Signal Level: " << SignalLevel << std::endl;

                // convert string SignalLevel to integer
                found_position = SignalLevel.find("/");
                SignalLevel = SignalLevel.substr(found_position-2,2); // writes the string into substring of only 2 characters, so "00" means the value "100"
                if(SignalLevel.compare("00") == 0){
                    SignalLevel = "100";
                }
                buf = atoi(SignalLevel.c_str());

                ws.LL.signal_level = buf;
                //std::cout << "ws.LL.signal_level " << ws.LL.signal_level << std::endl;

                ws.LL.signalLevelInDb = (ws.LL.signal_level / 2) - 100;
            }
        }
        else{
 //           std::cout << "MACADRESS_LABORATORY_LEFT NOT FOUND" << std::endl;
            ws.LL.signal_level = 0;    // -1 means macadress not found
        }



        /*-------------------------- MACADRESS_LABORATORY_RIGHT----------------------------------------------*/
        ws.LR.MAC_id = MACADRESS_LABORATORY_RIGHT;
        found_position = scan_result.find(MACADRESS_LABORATORY_RIGHT);
        if (found_position!=std::string::npos){
            //std::cout << "Found MACADRESS_LABORATORY_RIGHT: " << found_position << std::endl;


            // find frequency
            found_position=scan_result.find("Frequency:",found_position+1);
            if (found_position!=std::string::npos){
                Frequency = scan_result.substr(found_position+10,5);
                buf_float = atof(Frequency.c_str());
                buf_float = buf_float * 1000;
                ws.LR.freqInMHz = buf_float;
                //std::cout << "ws.LR.freqInMHz " << ws.LR.freqInMHz << std::endl;
            }

            // find signal level
            found_position=scan_result.find("Signal level=",found_position+1);
            if (found_position!=std::string::npos){

                //std::cout << "Signal level found at: " << found_position << '\n';
                SignalLevel = scan_result.substr(found_position+13,7);
                //std::cout << "Signal Level: " << SignalLevel << std::endl;

                // convert string SignalLevel to integer
                found_position = SignalLevel.find("/");
                SignalLevel = SignalLevel.substr(found_position-2,2); // writes the string into substring of only 2 characters, so "00" means the value "100"
                if(SignalLevel.compare("00") == 0){
                    SignalLevel = "100";
                }
                buf = atoi(SignalLevel.c_str());

                ws.LR.signal_level = buf;
                //std::cout << "ws.LR.signal_level " << ws.LR.signal_level << std::endl;

                ws.LR.signalLevelInDb = (ws.LR.signal_level / 2) - 100;

            }
        }
        else{
 //           std::cout << "MACADRESS_LABORATORY_RIGHT NOT FOUND" << std::endl;
            ws.LR.signal_level = 0;    // -1 means macadress not found
        }
/*
        std::cout << "ws.SR.signal_level " << ws.SR.signal_level << std::endl;
        std::cout << "ws.SL.signal_level " << ws.SL.signal_level << std::endl;
        std::cout << "ws.LL.signal_level " << ws.LL.signal_level << std::endl;
        std::cout << "ws.LR.signal_level " << ws.LR.signal_level << std::endl;
        std::cout << "ws.SR.signalLevelInDb " << ws.SR.signalLevelInDb << std::endl;
        std::cout << "ws.SL.signalLevelInDb " << ws.SL.signalLevelInDb << std::endl;
        std::cout << "ws.LL.signalLevelInDb " << ws.LL.signalLevelInDb << std::endl;
        std::cout << "ws.LR.signalLevelInDb " << ws.LR.signalLevelInDb << std::endl;
        std::cout << "ws.SR.freqInMHz " << ws.SR.freqInMHz << std::endl;
        std::cout << "ws.SL.freqInMHz " << ws.SL.freqInMHz << std::endl;
        std::cout << "ws.LL.freqInMHz " << ws.LL.freqInMHz << std::endl;
        std::cout << "ws.LR.freqInMHz " << ws.LR.freqInMHz << std::endl;
*/
        return ws;
}
