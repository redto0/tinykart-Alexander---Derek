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
ScanBuilder scan_builder{180, 360, ScanPoint{0.1524, 0}};

/// Starts/stops the kart
void estop() {
    logger.printf("Toggle Pause\n");

    tinyKart->toggle_pause();
    if (true){
        tinyKart->set_neutral();
        tinyKart->set_steering(0.0);
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
    tinyKart = new TinyKart{STEERING_PIN, esc, 0.3, 4.0};
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
float clamp(float value, int min_value, int max_value) {
    return std::max(static_cast<float>(min_value), std::min(value, static_cast<float>(max_value)));
}

float pure_pursuit_but_cooler( auto tinyKart, const ScanPoint &scan, float hello ){
    auto x = scan.x;
    auto y = scan.y; 
    /* auto normalizer = sqrt( x * x + y * y);
    if ( y > abs ( x ) ) {
        normalizer = y;
    } else { 
        normalizer = abs( x );
    }
    x = x / normalizer;
    y = y / normalizer; */
    // the angle between (0,1) and (x,y) in degrees
    float ans = acosf( ( y ) / ( sqrtf( x * x + y * y ) ) ) * 57.2957795;
    /// ans = clamp( ans, -45, 45);
    if ( x < 0 && ans > 0 ){
        ans = ans * -1;
    }
    return ans;
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

std::optional<ScanPoint> find_gap_naive(const std::vector<ScanPoint> &scan, uint8_t min_gap_size, float max_dist, float rDist) {
    // TODO
    
    /// float rDist = 0.5;
    float distance_array[scan.size()];
    distance_array[0] = scan[0].dist(ScanPoint::zero());
    auto closetPoint = -1;
    auto closetPointDist = 10;

    // find closest point
    for(int i = 1; i < scan.size(); i++){
        distance_array[i] = scan[i].dist(ScanPoint::zero());
        if ( distance_array[i] > max_dist ){
            distance_array[i] = 0;
        }
        if( distance_array[i] > 0 && closetPointDist > distance_array[i]){
            closetPoint = i;
            closetPointDist = distance_array[i];
        }
        /// if ( distance[i] > max_dist )
    } 
    // zero out other points
    if ( !(closetPoint == -1) ){ 

        for(auto i = 0; i < scan.size(); i++){
            if( scan[closetPoint].dist(scan[i]) <= rDist ){
                distance_array[i] = 0;
            }
        }
    }
    distance_array[closetPoint] = 0;
    /// finding the gaps
    /// the gap flag
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
                if ( (i - 1) - begin_of_cluster > length_max_cluster ){
                    /// assgin as biggest
                    length_max_cluster = ( i - 1) - begin_of_cluster;
                    begin_max_cluster = begin_of_cluster;

                }
                is_a_gap = false;
            }
        } else if( is_a_gap ){
            // check if the distance is too long
            if( scan[i].dist(scan[i-1]) >  min_gap_size ){

                if ( !( i - 1 == begin_of_cluster) && (i - 1) - begin_of_cluster > length_max_cluster ){
                    /// assgin as biggest
                    length_max_cluster = ( i - 1) - begin_of_cluster;
                    begin_max_cluster = begin_of_cluster;

                }
                is_a_gap = false;
                
            }

        } else if( (i - 1) == scan.size() && is_a_gap == true ){
            // check if wall is legit
            if ( i - begin_of_cluster > length_max_cluster && i - begin_of_cluster > 0 ){
                begin_max_cluster = begin_of_cluster;
                length_max_cluster = ( i ) - begin_of_cluster;
            }
        } else if ( is_a_gap == false ) {
            begin_of_cluster = i;
            is_a_gap = true;
            number_of_gaps++;
        }
    }
    /// logger.printf("%hi \n", (int16_t)(number_of_gaps));
    if( length_max_cluster == 0){
        logger.printf(" no target acquired \n");
        return std::nullopt;
    } else {
        scan_center_biggest_cluster.x = scan[begin_max_cluster].x + scan[ length_max_cluster ].x;
        scan_center_biggest_cluster.x = (scan_center_biggest_cluster.x ) / 2;
        scan_center_biggest_cluster.y = scan[begin_max_cluster].y + scan[ length_max_cluster ].y;
        scan_center_biggest_cluster.y = (scan_center_biggest_cluster.y) / 2;
        
        // logger.printf( " %hi \n", (int16_t) (number_of_gaps) );
        // logger.printf( " (%hi, %hi) \n", (int32_t)(scan_center_biggest_cluster.x *1000) , (int32_t)(scan_center_biggest_cluster.y *1000) );
        return scan_center_biggest_cluster;
    }
}

void doTinyKartBrakingTrick(auto TinyKart, float closestY){
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


void doTinyKartBrakingTrick(auto TinyKart, const std::vector<ScanPoint> &scan, float netural_zone, int angle){
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
    
    // Check if we have a scan frame
    if (res) {
        
        auto scan_res = *res;
        // Check if frame erred
        if (scan_res) {
            auto maybe_scan = scan_builder.add_frame(scan_res.scan);
            // Check if we have a 180 degree scan built
            if (maybe_scan) {
                auto scan = *maybe_scan;
                
             // run pio device monitor -b 115200
             // online research ingores the fact that most of this is custom writtern

                auto target_pt = find_gap_naive( scan, 1.5, 10, 0.5);
                
                
                if( (target_pt.has_value()) ){
                    /// auto steering_angle = tan( target_pt.value().x / target_pt.value().y) *10;
                    /// tinyKart->set_steering(steering_angle);
                    auto steering_angle = pure_pursuit_but_cooler(tinyKart, target_pt.value(), 0);
                    // auto command = pure_pursuit::calculate_command_to_point ( tinyKart, target_pt.value(), 5.0 );
                    // tinyKart->set_steering(command.steering_angle);
                    tinyKart->set_steering(steering_angle);
                    /// logger.printf("steering %i\n", (int32_t)(command.steering_angle * 1000));
                    logger.printf( "(%i x, %i y) ang %i \n", (int32_t)(target_pt.value().x*1000), (int32_t)(target_pt.value().y*1000), 
                    (int32_t) (steering_angle * 10 ) );
                    
                    /// doTinyKartBrakingTrick(tinyKart, target_pt.value().y);
                    // tinyKart->set_forward(0.20);
                    // tinyKart->set_forward(command.throttle_percent);
                    /// doTinyKartBrakingTrick(tinyKart, target_pt.value().y);
                    tinyKart->set_forward(0.14);
                    digitalWrite(LED_RED, HIGH);
                } else {

                    //logger.printf("steering 0.0 \n");
                    // tinyKart->set_steering(0.0);
                   // doTinyKartBrakingTrick(tinyKart, scan);
                    // tinyKart->set_steering(0.0);
                    tinyKart->set_forward(0.14);
                    /// doTinyKartBrakingTrick(tinyKart, scan, .20, 15);
                    digitalWrite(LED_RED, LOW);
                }
            }
        } else {
            switch (scan_res.error) {
                case ScanResult::Error::CRCFail:
                    logger.printf("CRC error!\n");
                    break;

                case ScanResult::Error::HeaderByteWrong:
                    logger.printf("Header byte wrong!\n");
                    break;
            }
        }
    }
}
