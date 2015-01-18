#include "circleintersections.h"



std::vector<float> compute_circle_intersections(float radius1, float radius2, const char* MAC_ID_1, const char* MAC_ID_2){

    std::vector<float> intersections;
    float dx1, dy1, dx2, dy2, D, d, x1, y1, x2, y2;
    
    /*
     circle1 with radius radius1 (depending on signal strength)
     dx1 distance in x direction to origin of coordinate system
     dy1 distance in y direction to origin of coordinate system
     (x-dx1)^2 + (y-dy1)^2 = r1^2
    */

    if(strcmp(MAC_ID_1,MACADRESS_STUDENTROOM_LEFT) == 0){
        dx1 = STUDENTROOM_LEFT_X;
        dy1 = STUDENTROOM_LEFT_Y;
    }
    else if(strcmp(MAC_ID_1,MACADRESS_STUDENTROOM_RIGHT) == 0){
        dx1 = STUDENTROOM_RIGHT_X;
        dy1 = STUDENTROOM_RIGHT_Y;
    }
    else if(strcmp(MAC_ID_1,MACADRESS_LABORATORY_LEFT) == 0){
        dx1 = LABORATORY_LEFT_X;
        dy1 = LABORATORY_LEFT_Y;
    }
    else if(strcmp(MAC_ID_1,MACADRESS_LABORATORY_RIGHT) == 0){
        dx1 = LABORATORY_RIGHT_X;
        dy1 = LABORATORY_RIGHT_Y;
    }
//    std::cout << "Die Koordinaten des ersten Routers sind: dx1 = " << dx1 << " und dy1 = " << dy1 << std::endl;

    /*
     circle1 with radius radius2 (depending on signal strength)
     dx2 distance in x direction to origin of coordinate system
     dy2 distance in y direction to origin of coordinate system
     (x-dx2)^2 + (y-dy2)^2 = r2^2
    */

    if(strcmp(MAC_ID_2,MACADRESS_STUDENTROOM_LEFT) == 0){
        dx2 = STUDENTROOM_LEFT_X;
        dy2 = STUDENTROOM_LEFT_Y;
    }
    else if(strcmp(MAC_ID_1,MACADRESS_STUDENTROOM_RIGHT) == 0){
        dx2 = STUDENTROOM_RIGHT_X;
        dy2 = STUDENTROOM_RIGHT_Y;
    }
    else if(strcmp(MAC_ID_1,MACADRESS_LABORATORY_LEFT) == 0){
        dx2 = LABORATORY_LEFT_X;
        dy2 = LABORATORY_LEFT_Y;
    }
    else if(strcmp(MAC_ID_1,MACADRESS_LABORATORY_RIGHT) == 0){
        dx2 = LABORATORY_RIGHT_X;
        dy2 = LABORATORY_RIGHT_Y;
    }
//    std::cout << "Die Koordinaten des zweiten Routers sind: dx2 = " << dx2 << " und dy2 = " << dy2 << std::endl;

    
    // evtl. nötig Radien zu vergrößern, da sich sonst kein Schnittpunkt ergibt (Ungenauigkeit des Mappings der Signalstärke zu Distanz)
    //r1 = 1.001 * r1; // 0.1% Toleranz
    //r2 = 1.001 * r2; // 0.1% Toleranz
    
    D = sqrt(pow((dx2-dx1),2.0) + pow((dy2-dy1),2.0));
    d = 0.25*sqrt((D+radius1+radius2)*(D+radius1-radius2)*(D-radius1+radius2)*(-D+radius1+radius2));
    // std::cout << "d = " << d << std::endl;
    
    x1 = (dx1+dx2)/2 + ((dx2-dx1)*(pow(radius1,2.0)-pow(radius2,2.0)))/(2*pow(D,2.0)) + 2*(dy1-dy2)/(pow(D,2.0))*d;
    y1 = (dy1+dy2)/2 + ((dy2-dy1)*(pow(radius1,2.0)-pow(radius2,2.0)))/(2*pow(D,2.0)) - 2*(dx1-dx2)/(pow(D,2.0))*d;
    
    x2 = (dx1+dx2)/2 + ((dx2-dx1)*(pow(radius1,2.0)-pow(radius2,2.0)))/(2*pow(D,2.0)) - 2*(dy1-dy2)/(pow(D,2.0))*d;
    y2 = (dy1+dy2)/2 + ((dy2-dy1)*(pow(radius1,2.0)-pow(radius2,2.0)))/(2*pow(D,2.0)) + 2*(dx1-dx2)/(pow(D,2.0))*d;
    
    // std::cout << x1 << ", " << y1 << std::endl;
    // std::cout << x2 << ", " << y2 << std::endl;


    /*
    if (d == 0 && isnan(x1) && isnan(y1) && isnan(x2) && isnan(y2)) {
        std::cout << "Die Kreise sind identisch." << std::endl;
    }
    
    else if (isnan(d)) {
        std::cout << "Die Kreise haben keinen Schnittpunkt." << std::endl;
    }

    else if (d == 0 && (isnan(x1) == false || isnan(y1) == false || isnan(x2) == false || isnan(y2) == false)) {
        std::cout << "Es gibt nur einen Schnittpunkt:  x = " << x1 << ",   y = " << y1 << std::endl;
    }
    
    else {
        std::cout << "Der erste Schnittpunkt hat die Koordinaten: x = " << x1 << ",   y = " << y1 << std::endl;
        std::cout << "Der zweite Schnittpunkt hat die Koordinaten: x = " << x2 << ",   y = " << y2 << std::endl;
    }
    */

    intersections.push_back(x1);
    intersections.push_back(y1);
    intersections.push_back(x2);
    intersections.push_back(y2);

    return intersections;
}
