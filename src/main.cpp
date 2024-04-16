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


float pure_pursuit_but_sillier( auto tinyKart, const ScanPoint &scan, float hello ){
    auto x = scan.x;
    auto y = scan.y; 
    auto magnitude = scan.dist( ScanPoint::zero());
    auto Aphfa = asinf( ( sqrtf( x * x + y * y ) ) ) ; /// * 57.2957795;

    auto R = magnitude / 2 * sin( Aphfa );
    auto steering_angle = (magnitude); 
    return 0;
}

/// Calculates the command to move the kart to some target point.
AckermannCommand calculate_command_to_point(const TinyKart *tinyKart, ScanPoint target_point,
                                            float max_lookahead) {
    auto x = target_point.x;
    auto y = target_point.y; 
    auto magnitude = target_point.dist( ScanPoint::zero());
    auto Aphfa = asinf( ( sqrtf( x * x + y * y ) ) ) ; /// * 57.2957795;

    auto R = magnitude / 2 * sin( Aphfa );
    auto steering_angle = atan ( tinyKart->get_wheelbase() / R );

    AckermannCommand commands_to_point{};
    
    commands_to_point.throttle_percent = 0.15;
    if (x >= 0){
        commands_to_point.steering_angle = steering_angle * 57.2957795;
        return commands_to_point;
    } else if ( x < 0 && steering_angle > 0){
        commands_to_point.steering_angle = steering_angle * -57.2957795;
        return commands_to_point;
    } 
    commands_to_point.steering_angle = steering_angle * 57.2957795;
    return commands_to_point;
    
}
 //TODO    


std::optional<ScanPoint> find_gap_naive(const std::vector<ScanPoint> &scan, uint8_t min_gap_size, float max_dist, float rDist) {
    // TODO
    
    /// float rDist = 0.5;

    // INITS
    float distance_array[scan.size()];
    distance_array[0] = scan[0].dist(ScanPoint::zero());
    auto closetPoint = -1;
    auto closetPointDist = 10;

    // find closest point
    for(int i = 1; i < scan.size(); i++){
        distance_array[i] = scan[i].dist(ScanPoint::zero()); // CALC DISTANCE OF EACH POINT
        if ( distance_array[i] > max_dist ){
            distance_array[i] = 0; // IGNORE EXTREMELY FAR POINTS?
        }
        if( distance_array[i] > 0 && closetPointDist > distance_array[i]){
            closetPoint = i;
            closetPointDist = distance_array[i]; // POPULATE DISTANCE ARRAY
        }
        // If the distance is non-zero and smaller than the current closest point, 
        // update the closest point index and distance
        /// if ( distance[i] > max_dist )
    } 
    // zero out other points
    if ( !(closetPoint == -1) ){ // IF CLOSEST POINT IS FOUND...

        for(auto i = 0; i < scan.size(); i++){ // LOOP THROUGH POINTS
            if( scan[closetPoint].dist(scan[i]) <= rDist ){
                distance_array[i] = 0; // SET THEM TO ZERO
            }
        }
    }
    distance_array[closetPoint] = 0; // ZERO CLOSEST POINT
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
        float largest_distance = 0.0;
        int pointer_Biggest_ditance = 0.0;
        for ( auto i = begin_max_cluster; i < length_max_cluster; i++ ){
            if (scan[i].dist(ScanPoint::zero()) > largest_distance ){
                largest_distance = scan[i].dist(ScanPoint::zero());
                pointer_Biggest_ditance = i;
            }
        }
        if ( pointer_Biggest_ditance > 0  ){

            scan_center_biggest_cluster. x = scan[ pointer_Biggest_ditance ].x ;
            scan_center_biggest_cluster. y = scan[ pointer_Biggest_ditance ].y ;
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
    // digitalWrite(LED_RED, LOW); // LED RED LIGHT OFFF
    /// logger.printf("lopper \n");
    
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
                    /// float steering_angle = pure_pursuit_but_sillier(tinyKart, target_pt.value(), 0);
                    auto steering_angle = calculate_command_to_point(tinyKart, target_pt.value(), 4).steering_angle;
                    tinyKart->set_steering(steering_angle); // STEER WITH ANGLE FROM SILLIER
                    logger.printf( "(%i x, %i y) ang %i \n", (int32_t)(target_pt.value().x*1000), (int32_t)(target_pt.value().y*1000), 
                        (int32_t) (steering_angle * 10 ) );

                    tinyKart->set_forward(0.16);
                    digitalWrite(LED_RED, HIGH); // RED LIGHT ON IF TARGET FOUND
                } else {

                    tinyKart->set_steering(0.0); // NO TARGET SO CONTINUE FORWARD
                    tinyKart->set_forward(0.16);
                    /// doTinyKartBrakingTrick(tinyKart, scan, 0, 45);
                    /// tinyKart->set_neutral();
                    digitalWrite(LED_RED, LOW); // RED LIGHT OFF NO TARGET
                    logger.printf("no target \n\n");
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
