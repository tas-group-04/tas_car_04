#include "findlocation.h"


std::vector <float> readLocationGuess(std::vector <std::vector <float> > allvec){

    std::vector <float> tmp,tmp2,cluster1,cluster2,saved_cluster_x,saved_cluster_y, guesses;

    float value1,value2,comparevalue0,comparevalue1;
    float guess_x = 0.0;
    float guess_y = 0.0;

    int counter1 = 0;
    int counter2 = 0;
    int saved_counter_x = 0;
    int saved_counter_y = 0;


    /*-----------------------------------------------------X VALUES-------------------------------------------------------*/
    // find the x value that has the most "neighbours" given a tolerance
    for (int i=0; i<allvec.size();i++){
        tmp = allvec.at(i);


        // Compare the first x entry of the first vector in the stack with all the other x values and count how often another x value is in its tolerance neighbourhood
        value1 = tmp.at(0);
        cluster1.push_back(value1);

        for(int j=0;j<allvec.size();j++){
            tmp2 = allvec.at(j);
            comparevalue0 = tmp2.at(0);
            comparevalue1 = tmp2.at(2);

            if (comparevalue0 != value1){
                if(std::abs(value1-comparevalue0)<TOLERANCE){
                    counter1 = counter1 + 1;
                    cluster1.push_back(comparevalue0);
                }
            }
            if (comparevalue1 != value1){
                if(std::abs(value1-comparevalue1)<TOLERANCE){
                    counter1 = counter1 + 1;
                    cluster1.push_back(comparevalue1);
                }
            }
        }



        // Compare the second x entry of the first vector in the stack with all the other x values and count how often another x value is in its tolerance neighbourhood
        value2 = tmp.at(2);
        cluster2.push_back(value2);

        for(int j=0;j<allvec.size();j++){
            tmp2 = allvec.at(j);
            comparevalue0 = tmp2.at(0);
            comparevalue1 = tmp2.at(2);

            if (comparevalue0 != value2){
                if(std::abs(value2-comparevalue0)<TOLERANCE){
                    counter2 = counter2 + 1;
                    cluster2.push_back(comparevalue0);
                }
            }
            if (comparevalue1 != value2){
                if(std::abs(value2-comparevalue1)<TOLERANCE){
                    counter2 = counter2 + 1;
                    cluster2.push_back(comparevalue1);
                }
            }
        }


        // get the cluster with most entries
        if (counter1 > counter2){
            if (counter1>saved_counter_x){
                saved_counter_x = counter1;
                saved_cluster_x = cluster1;
            }
        }
        else{
            if (counter2 > saved_counter_x){
                saved_counter_x = counter2;
                saved_cluster_x = cluster2;
            }
        }

        counter1 = 0;
        counter2 = 0;
        cluster1.clear();
        cluster2.clear();
    }

    /*-----------------------------------------------------Y VALUES-------------------------------------------------------*/
    // find the y value that has the most "neighbours" given a tolerance
    for (int i=0; i<allvec.size();i++){
        tmp = allvec.at(i);


        // Compare the first y entry of the first vector in the stack with all the other y values and count how often another y value is in its tolerance neighbourhood
        value1 = tmp.at(1);
        cluster1.push_back(value1);

        for(int j=0;j<allvec.size();j++){
            tmp2 = allvec.at(j);
            comparevalue0 = tmp2.at(1);
            comparevalue1 = tmp2.at(3);

            if (comparevalue0 != value1){
                if(std::abs(value1-comparevalue0)<TOLERANCE){
                    counter1 = counter1 + 1;
                    cluster1.push_back(comparevalue0);
                }
            }
            if (comparevalue1 != value1){
                if(std::abs(value1-comparevalue1)<TOLERANCE){
                    counter1 = counter1 + 1;
                    cluster1.push_back(comparevalue1);
                }
            }
        }



        // Compare the second y entry of the first vector in the stack with all the other y values and count how often another y value is in its tolerance neighbourhood
        value2 = tmp.at(3);
        cluster2.push_back(value2);

        for(int j=0;j<allvec.size();j++){
            tmp2 = allvec.at(j);
            comparevalue0 = tmp2.at(1);
            comparevalue1 = tmp2.at(3);

            if (comparevalue0 != value2){
                if(std::abs(value2-comparevalue0)<TOLERANCE){
                    counter2 = counter2 + 1;
                    cluster2.push_back(comparevalue0);
                }
            }
            if (comparevalue1 != value2){
                if(std::abs(value2-comparevalue1)<TOLERANCE){
                    counter2 = counter2 + 1;
                    cluster2.push_back(comparevalue1);
                }
            }
        }


        // get the cluster with most entries
        if (counter1 > counter2){
            if (counter1>saved_counter_y){
                saved_counter_y = counter1;
                saved_cluster_y = cluster1;
            }
        }
        else{
            if (counter2 > saved_counter_x){
                saved_counter_y = counter2;
                saved_cluster_y = cluster2;
            }
        }

        counter1 = 0;
        counter2 = 0;
        cluster1.clear();
        cluster2.clear();
    }


  //  std::cout << "saved_counter_x: " << saved_counter_x << std::endl;
    for(int i=0;i<saved_cluster_x.size();i++){
  //      std::cout << "saved_cluster_x: " << saved_cluster_x.at(i) << std::endl;
        guess_x = guess_x + saved_cluster_x.at(i);
    }

  //  std::cout << "saved_counter_y: " << saved_counter_y << std::endl;
    for(int i=0;i<saved_cluster_y.size();i++){
  //      std::cout << "saved_cluster_y: " << saved_cluster_y.at(i) << std::endl;
        guess_y = guess_y + saved_cluster_y.at(i);
    }

    guess_x = guess_x / saved_cluster_x.size();
    guess_y = guess_y / saved_cluster_y.size();

    std::cout << "guess_x: " << guess_x << std::endl;
    std::cout << "guess_y: " << guess_y << std::endl;

    guesses.push_back(guess_x);
    guesses.push_back(guess_y);

    return guesses;
}
