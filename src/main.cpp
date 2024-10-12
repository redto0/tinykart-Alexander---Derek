#include <Arduino.h>
#include "pins.hpp"
#include "kart.hpp"
#include "ld06.hpp"
#include "dma.hpp"
#include "logger.hpp"
#include "pure_pursuit.hpp"
#include "f1tenth_gap_follow.hpp"
#include "naive_gap_follow.hpp"
#include <cmath>
// added by me
// #include <bits/stdc++.h>

// Robot control
TinyKart *tinyKart;

// LiDAR
LD06 ld06{};

// Scan processor
// was 180, 360
///  ScanPoint{0.1524, 0}
ScanBuilder scan_builder{180, 360, ScanPoint{0.1524, 0}}; /// LIDAR SCAN SHAPE & SCAN INIT

/// Starts/stops the kart
void estop() {
    logger.printf("Toggle Pause\n");

    tinyKart->toggle_pause();
    if (true){
        tinyKart->set_neutral();
        // tinyKart->set_steering(0.0);
    }
    digitalToggle(LED_YELLOW);
}

void setup() {
    // LEDs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, HIGH);

    // Setup blue user button on the board to stop the kart
    pinMode(USER_BTN, INPUT);
    attachInterrupt(digitalPinToInterrupt(USER_BTN), estop, FALLING);

    // Init PWM
    analogWriteResolution(PWM_BITS); // Range of 0-4096
    analogWriteFrequency(PWM_FREQ);

    // Prepare kart for motion
    ESC esc{THROTTLE_PIN, PWM_MAX_DUTY, PWM_FREQ};
    // max third para is 0.3, range >1.6
    tinyKart = new TinyKart{STEERING_PIN, esc, 0.3, 4.5}; /// LAST FLOAT IS BIAS 
    // Init DMA and UART for LiDAR
    dmaSerialRx5.begin(230'400, [&](volatile LD06Buffer buffer) {
        // On each packet received, copy over to driver.
        ld06.add_buffer(buffer, 47);
    });

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
}



float maxSpeed = 0.20;
float startBrakingDistance = 12.686 * maxSpeed - 0.7757;
float brakingPercentage = -1;
float slopeBreaking = 1 / (0.5 - startBrakingDistance);
float max_braking_trick_angle = 20;
// in degrees converting to rads
float max_braking_angle_constant = tan(30 * 0.01745329);


/// Calculates the command to move the kart to some target point.
double calculate_command_to_point(const TinyKart *tinyKart, ScanPoint target_point,
                                            float max_lookahead) {
    auto x = target_point.x;
    auto y = target_point.y; 
    auto magnitude = target_point.dist( ScanPoint::zero());
    auto Aphfa = asinf( ( sqrtf( x * x + y * y ) ) ) ; /// * 57.2957795;
    auto R = magnitude / 2 * sin( Aphfa );
    auto steering_angle = atan ( tinyKart->get_wheelbase() / R ) * 57.2957795;

    // LIMIT STEERING ANGLE
    if (steering_angle >= 24) {
        steering_angle = 23;
    }
    if (steering_angle <= -24) {
        steering_angle = -23;
    }
/* 
    AckermannCommand commands_to_point{};
    commands_to_point.throttle_percent = 0.15;
    if (x >= 0){
        commands_to_point.steering_angle = steering_angle;
        return commands_to_point;
    } else if ( x < 0 && steering_angle > 0){
        commands_to_point.steering_angle = steering_angle * -1;
        return commands_to_point; */
    //} 
    //commands_to_point.steering_angle = steering_angle;
    
    return steering_angle;
    //commands_to_point;
    
}


float pure_pursuit_but_with_glasses(TinyKart tinyKart, const ScanPoint &scan, float max_viewing_dist) 
{
    //non-const local variable and initializing it with the values of scan
    ScanPoint copy_scan = scan; // Create a mutable copy of scan because scan is const & read-only

    if (auto target_dist = scan.dist(ScanPoint::zero()); target_dist > max_viewing_dist) {
        auto turn_angle = atan2f(scan.y, scan.x);

        auto y_2 = sin(turn_angle) * max_viewing_dist;
        auto x_2 = cos(turn_angle) * max_viewing_dist;

        copy_scan.x = x_2;
        copy_scan.y = y_2;
    }

    float alpha = atan2f(copy_scan.y, copy_scan.x); // CALC ANGLE

    float steering_angle = atanf(2.0 * 0.335 * sinf(alpha) / copy_scan.dist(ScanPoint::zero())); // SET STEERING ANGLE

    steering_angle = steering_angle * float(180.0) / float(3.1415); // CONVERT TO DEGREES

    // LIMIT STEERING ANGLE
    if (steering_angle >= 24) {
        steering_angle = 23;
    }
    if (steering_angle <= -24) {
        steering_angle = -23;
    }

    return steering_angle;

    /* // formula is arc sin ( || vecter || * || y axis vector || )
    float ans = asinf( ( sqrtf( x * x + y * y ) ) ) * 57.2957795;
    /// ans = clamp( ans, -45, 45);
    if ( x < 0 && ans > 0 ){
        ans = ans * -1;
    }
    return ans; */
}


std::optional<ScanPoint> find_closest_point(const std::vector<ScanPoint> &scan, float max_dist_from_ldar, float maxClusterDistance){
    double distance_array[scan.size()];
    for(auto i = 0; i < scan.size(); i++){
        if( scan[i].x != 0 && (scan[i].y) != 0 ) {
            distance_array[i] = scan[i].dist(ScanPoint::zero());
        }
    }

    int current_first = 0;
    ScanPoint closest_scan ;
    closest_scan.x = 1000;
    bool is_last_a_zero = false;
    int start = 0;
    if(distance_array[0] != 0){
        is_last_a_zero= true;
    }
    for(auto i = 1; i < scan.size(); i++){
        // check if its a non zero
        if(distance_array[i] != 0 && distance_array[i] < max_dist_from_ldar) {
            if(is_last_a_zero = true){
                // set the begining of the cluster
                current_first = i;
                is_last_a_zero = false;
            } else {
                // check if this cluster is within size limits
                if( scan[current_first].dist(scan[i]) > maxClusterDistance ) {
                    // check to see if this cluster has more than one point
                    if( scan[i-1].dist(scan[i]) > maxClusterDistance ){
                        // this is a new cluster. 
                        // sets the pointer to the next non zero element. 
                        current_first = i;
                        // also ends this literation of the loop. 
                    } else {
                        // check the object
                        // create a mid point from the past ones
                        ScanPoint new_scan;
                        new_scan.x = ( scan[current_first].x + scan[i].x ) / 2;
                        new_scan.y = ( scan[current_first].y + scan[i].y ) / 2;

                        // check is there is a closest point
                        if( (closest_scan.x == 1000  ) ) {
                            // add this as the first closest point
                            closest_scan.x = new_scan.x;
                            closest_scan.y = new_scan.y;

                            // check if this new point is closer than the last one
                        } else if ( closest_scan.dist(ScanPoint::zero()) > new_scan.dist(ScanPoint::zero() ) ){
                            // set to the new scan
                            closest_scan.x = new_scan.x;
                            closest_scan.y = new_scan.y;
                        }
                    }
                }
            }
        // switch to if the element is zero
        } else {
            i--;
            if (current_first = i) {continue; }
            // check if this cluster is within size limits
                if( scan[current_first].dist(scan[i]) > maxClusterDistance){
                    // check to see if this cluster has more than one point
                    if( (current_first + 1) == i){
                        // fail this cluster and set i to the next point.
                        // set the pointer to the next non zero element. 
                        current_first = i;
                    } else {
                        // create a mid point from the past ones
                        ScanPoint new_scan;
                        new_scan.x = ( scan[current_first].x + scan[i].x ) / 2;
                        new_scan.y = ( scan[current_first].y + scan[i].y ) / 2;

                        // check is there is a closest point
                        if( (closest_scan.x == 1000  ) ){
                            // add this as the first closest point
                            closest_scan.x = new_scan.x;
                            closest_scan.y = new_scan.y;

                            // check if this new point is closer than the last one
                        } else if ( closest_scan.dist(ScanPoint::zero()) > new_scan.dist(ScanPoint::zero() ) ){
                            // set to the new scan
                            closest_scan.x = new_scan.x;
                            closest_scan.y = new_scan.y;
                        }
                    }
                }
            i++;
            is_last_a_zero = true;
        }// end if chain
    }// end loop
    if(closest_scan.x == 1000){
        return std::nullopt;
    } else {
        return closest_scan;
    }
    
}// end fuction

struct vector_with_angle /// : std::vector
{   
    /* data */
    public:
    float x = this->x;
    float y = this->y;
    float angle = this->angle;
    float distance;
    /* constructor */
    public:
    vector_with_angle(float xf, float yf, float anglef, float distancef){
        x = xf;
        y = yf;
        angle = anglef;
        distance = distancef;
    };
};

/*
std::optional<ScanPoint> find_gap_naive_new_algo(const std::vector<ScanPoint> &scan, uint8_t max_gap_size, float max_dist, 
                                        float min_dist, float rDist, int doMidpoint, float ideal_angle) {
    /// this is the start of andys ideas

    // we should wholesale reuse the enire perivous code of find native gap.
    
    int angle_array_length = 0;
    for (int i = 0; i > scan.size(); i++){
        if( scan[i].dist( ScanPoint::zero() ) > 0){
            angle_array_length++;
        }
    }

    vector_with_angle angle_array [angle_array_length] = {};
    
    // ai code buit its quicker than learning it myself
    float calculateAngle(float p1x, float p1y, float p2x, float p2y) {
        float deltaY = p2y - p1y;
        float deltaX = p2x - p1x;
        float angleInRadians = atan2(deltaY, deltaX);
        float angleInDegrees = angleInRadians * 180 / M_PI;
        return angleInDegrees;
    }
    int j = 0;
    for (int i = 0; i < scan.size(); i++){
        float distf = scan[i].dist(ScanPoint::zero);
        if( distf > 0){
            if (j > 0) {
                // uses the perivous data point
                angle_array[j] = vector_with_angle( scan[i].x, scan[i].y, 
                                calculateAngle( scan[i].x, scan[i].y, scan[j - 1], scan[j - 1] ), 
                                distf );
            } else {
                //intailize the begin as 0 because it does not matter much
                angle_array[j] = vector_with_angle( scan[i].x, scan[i].y, 
                                0, distf );
            }
            j++;
        }
    }

    for(int i = 0; i < scan.size() - 1; i++){ 
        angle_array[i].x = scan[i].x;
        angle_array[i].y = scan[i].y;
        angle_array[i].angle = calculateAngled(scan[i].x, scan[i].y, scan[i+1].x, scan[i+1].y );
        // todo cal angle betweeen car points
    }

    float average_additon = 0;
    for ( auto i = 1; i < angle_array.size(), i++){
        average_additon += angle_array[i].angle;
    }

    float   average_angle              = average_additon / angle_array_length;
    float   greatest_deviation_score   = 0;
    int     greatest_Deviant           = 0;
    // end point in the mind the part of the calulations

    for ( auto i = 1; i < angle_array.size(), i++){
        if( abs( angle_array[i].angle - average_angle) > greatest_deviation_score) {
            greatest_deviation_score = abs( angle_array[i].angle - average_angle);
            greatest_deviat = i;
        }
    }
    
}/// end new algo
*/
std::optional<ScanPoint> find_gap_naive(const std::vector<ScanPoint> &scan, uint8_t max_gap_size, float max_dist, 
                                        float min_dist, float rDist, int doMidpoint) {
    // TODO
    
    /// float rDist = 0.5;

    // INITS
    float distance_array [scan.size()];
    distance_array[0] = scan[0].dist(ScanPoint::zero());
    auto nearest_Point_index = -1;
    auto nearest_Point_Dist = 10;
    int hi = round(scan.size() / 2);
    distance_array[ hi ] = 0;

    // find closest point
    for(int i = 1; i < scan.size(); i++){
        distance_array[i] = scan[i].dist(ScanPoint::zero()); // CALC DISTANCE OF EACH POINT
        if ( distance_array[i] > max_dist || distance_array[i] < min_dist){
            distance_array[i] = 0; // IGNORE EXTREMELY FAR POINTS -d?
                                    // and close ones now -AFB
        }
        if( distance_array[i] > 0 && nearest_Point_Dist > distance_array[i]){
            nearest_Point_index = i;
            nearest_Point_Dist = distance_array[i]; // POPULATseE DISTANCE ARRAY
        }
        // If the distance is non-zero and smaller than the current closest point, 
        // update the closest point index and distance
        /// if ( distance[i] > max_dist )
    }
    // zero out other points
    if ( !(nearest_Point_index == -1) ){ // IF CLOSEST POINT IS FOUND...

        for(auto i = 0; i < scan.size(); i++){ // LOOP THROUGH POINTS
            if( scan[nearest_Point_index].dist(scan[i]) <= rDist ){
                distance_array[i] = 0; // SET THEM TO ZERO
            }
        }
    }
    distance_array[nearest_Point_index] = 0; // ZERO CLOSEST POINT
    /// finding the gaps
    /// the gap flag
    // INIT
    bool is_a_gap = false;
    int begin_of_cluster = 0;
    int begin_max_cluster = 0;
    int length_max_cluster = 0;
    // need to inizate new scanPoint;
    ScanPoint scan_center_biggest_cluster{ 0, 0 };
    if ( distance_array[0] != 0 ){
        is_a_gap = true;
    } 
    int number_of_gaps = 0;
    for(auto i = 0; i < scan.size(); i++){
        if(distance_array[i] == 0.0){
            if(is_a_gap == true ){
            /// check if the gap is bigger than the last one!
                if ( (i - 1 ) - begin_of_cluster > length_max_cluster ){
                    /// assgin as biggest
                    length_max_cluster = scan[ i - 1 ].dist( scan[ begin_of_cluster ]);
                    begin_max_cluster = begin_of_cluster;

                }
                is_a_gap = false;
            }
        }  else if( i  == scan.size() && is_a_gap == true ){
            // check if wall is legit
            if ( i - begin_of_cluster > length_max_cluster && i - begin_of_cluster > 0 ){
                begin_max_cluster = begin_of_cluster;
                length_max_cluster = scan[ i ].dist( scan[ begin_of_cluster ]);
            }
        } else if( is_a_gap ){
            // check if the distance is too long
            if( scan[i].dist(scan[i - 1]) >  max_gap_size ){

                auto length_of_this_gap = scan[ i - 1 ].dist( scan[ begin_of_cluster ]);
                if ( !( i - 1 == begin_of_cluster) && length_of_this_gap > length_max_cluster ){
                    /// assgin as biggest
                    length_max_cluster = length_of_this_gap;
                    begin_max_cluster = begin_of_cluster;

                }
                is_a_gap = false;
                
            }

        } else if ( is_a_gap == false ) {
            begin_of_cluster = i;
            is_a_gap = true;
            number_of_gaps++;
        }
    } logger.printf(" %i \n", (int16_t) number_of_gaps);
    /// logger.printf("%hi \n", (int16_t)(number_of_gaps));
    if( length_max_cluster == 0){
        /// logger.printf(" no target acquired \n"); // REMOVED FOR GUI QUALITY OF LIFE
        return std::nullopt;
    } else {
        switch(doMidpoint){
            case 1:
                float largest_distance = 0.0;
                int pointer_Biggest_distance = 0.0;
                /// finds the largest distance is the biggest gap
                for ( auto i = begin_max_cluster; i < length_max_cluster; i++ ){
                    if (scan[i].dist(ScanPoint::zero()) > largest_distance ){
                        largest_distance = scan[i].dist(ScanPoint::zero());
                        pointer_Biggest_distance = i;
                    }
                }
                /// 
                if ( pointer_Biggest_distance > 0  ){

                    scan_center_biggest_cluster. x = scan[ pointer_Biggest_distance ].x ;
                    scan_center_biggest_cluster. y = scan[ pointer_Biggest_distance ].y ;
                }   
                    else if (scan[begin_max_cluster].dist( ScanPoint::zero() ) > scan[ length_max_cluster ].dist( ScanPoint::zero() ) ){
                        scan_center_biggest_cluster. x = scan[ begin_max_cluster ].x ;
                        scan_center_biggest_cluster. y = scan[ begin_max_cluster ].y ;
                } else {
                    scan_center_biggest_cluster. x = scan[ length_max_cluster ].x ;
                    scan_center_biggest_cluster. y = scan[ length_max_cluster ].y ;
                } 
                // logger.printf( " %hi \n", (int16_t) (number_of_gaps) );
                // logger.printf( " (%hi, %hi) \n", (int32_t)(scan_center_biggest_cluster.x *1000) , (int32_t)(scan_center_biggest_cluster.y *1000) );
                return scan_center_biggest_cluster;
            break;
        }
    }
    return std::nullopt;
}

std::optional<ScanPoint> angle_find_gap(const std::vector<ScanPoint> &scan, float max_dist_from_ldar, float maxClusterDistance){
    struct Node {    
          float data;    
          Node* next;
    };

    int number_not_zero = 0;
    for (auto &pt : scan){
        if( pt.dist(ScanPoint::zero()) ) {
            number_not_zero++;
        }
    }
    std::vector<ScanPoint> valid_array[number_not_zero];

    int i = 0;
    for (auto pt : scan){
        if( pt.dist(ScanPoint::zero()) ) {
            valid_array[i] = pt;
        }
    }

};

void doTinyKartBrakingTrick(TinyKart TinyKart, float closestY){
    brakingPercentage = slopeBreaking * closestY + 1;

    if(brakingPercentage > 1){
        brakingPercentage = 1;
    }// end braking check

    //logger.printf( "(%hi,%hi) \n", (int16_t) brakingPercentage*1000, (int16_t) (closestY * 1000 ) );

    if(brakingPercentage > 0 && brakingPercentage < .50){
        //logger.printf("We are stopping");
        tinyKart->set_neutral();
        /// tinyKart->set_forward(maxSpeed);

        //estop();
    } else  if ( brakingPercentage > 0){
        tinyKart->set_reverse(brakingPercentage * maxSpeed);
        //logger.printf("cart goes back\n");
                    
    } else {
        // manual trim, range (0,24)
        // positive is left
        tinyKart->set_forward(maxSpeed);
    }// end forward/back code
}


void doTinyKartBrakingTrick(TinyKart TinyKart, const std::vector<ScanPoint> &scan, float netural_zone, int angle){
    float closestY = 1000;

    double angle_constant = tan(angle * 0.01745329);
    double distance_array[scan.size()];
    int i = 0;
    for (auto &pt: scan) {
        //logger.printf("Point: (%hu,%hu)\n", (uint16_t) (pt.x * 1000), (uint16_t) (pt.y * 1000));
        // speed control
        if(closestY > pt.y && (pt.y > 0.001) ){
            if(pt.x == 0){
                closestY = pt.y;
            } else if (pt.y/pt.x > angle_constant || pt.y/pt.x < -angle_constant ){
                closestY = pt.y;
            }
        }
    }
    ///logger.printf(" %hi \n", (int16_t) (closestY * 1000));
    brakingPercentage = slopeBreaking * closestY + 1;

    if(brakingPercentage > 1){
        brakingPercentage = 1;
    }// end braking check

    //logger.printf( "(%hi,%hi) \n", (int16_t) brakingPercentage*1000, (int16_t) (closestY * 1000 ) );

    if(brakingPercentage > 0 && brakingPercentage < netural_zone){
        tinyKart->set_neutral();
    } else  if ( brakingPercentage > 0){
        tinyKart->set_reverse(brakingPercentage * maxSpeed);
    } else {
        tinyKart->set_forward(maxSpeed);
    }// end forward/back code
}


void loop() {
    noInterrupts();
    auto res = ld06.get_scan();
    interrupts();
    // digitalWrite(LED_RED, LOW); // LED RED LIGHT OFFF
    /// logger.printf("lopper \n");
    
    // Check if we have a scan frame
    if (res) {
        
        auto scan_res = *res;
        // Check if frame erred
        if (scan_res) {
            auto maybe_scan = scan_builder.add_frame(scan_res.scan);
            /// logger.printf("Yes res\n");
            // Check if we have a 180 degree scan built
            if (maybe_scan) {
                auto scan = *maybe_scan;

                /// logger.printf("maybe scan");
                
             // run pio device monitor -b 115200



                ///                          scan, min_gap_size, max_dist, min_dist, rDist, leave as one
                /**
                 * min_gap_size is the smallest that the gap can be
                 * max_dist is the maxium length from the lidar that the point can be.
                 * min_dist is the minium length between points. 
                 * rDist is the radius of exlucdion
                 * the last point is left as one
                */
                auto target_pt = find_gap_naive( scan, 0.1, 10, 0.001, 0.00, 1);
                if( !(target_pt.has_value()) ){
                                                // scan, max distance from idar, max cluster size
                    /// target_pt = find_closest_point(scan, 8, 1 );
                }

                
                tinyKart->set_forward(.20);
                if( (target_pt.has_value()) ){

                    ///float steering_angle = pure_pursuit_but_with_glasses(tinyKart, target_pt.value(), 0.2);
                     double steering_angle = calculate_command_to_point(tinyKart, target_pt.value(), 4);
                    /*
                    if( target_pt.value().x > 0 ){
                        steering_angle = steering_angle*-1;
                    }*/
                    tinyKart->set_steering(steering_angle); // STEER WITH ANGLE
                    tinyKart->set_forward(16);
                    /// GUI PRINTING
                    /// logger.printf( " goal (x %i [mm], y %i [mm]) \n ang [degrees] %i \n", (int32_t)(target_pt.value().x*1000), (int32_t)(target_pt.value().y*1000), 
                    ///    (int32_t) (steering_angle * 100 ) );
                    if(steering_angle > 0){
                        logger.printf("Left \n");   
                    } else {
                        logger.printf("Right \n");
                    }
                    logger.printf(" %i \n", (int16_t) scan.size());
                    tinyKart->set_forward(0.16);
                    digitalWrite(LED_RED, HIGH); // RED LIGHT ON IF TARGET FOUND
                } else {

                    //tinyKart->set_steering(0.0); // NO TARGET SO CONTINUE FORWARD
                    tinyKart->set_forward(0.16);
                    /// doTinyKartBrakingTrick(tinyKart, scan, 0, 45);
                    tinyKart->set_neutral();
                    digitalWrite(LED_RED, LOW); // RED LIGHT OFF NO TARGET
                    // logger.printf("no target \n no ang\n");
                }
            }
        } else {
            switch (scan_res.error) {
                case ScanResult::Error::CRCFail:
                    logger.printf("CRC error!\n");
                   
                    // GETTING CRC ERRORS ON KART 3 --- LIDAR NOT WORKING!!!
                    break;
                case ScanResult::Error::HeaderByteWrong:
                    logger.printf("Header byte wrong!\n");
                    break;
            }
        }
    }else {
        // logger.printf("no res\n");
        /// tinyKart->set_forward(0.1);
        /// tinyKart->set_steering(0.0);
        
    }
}
