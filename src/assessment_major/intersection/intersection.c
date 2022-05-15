#include "intersection.h"
#include "../fsm.h"
#include "../sensor.h"

/** Original Transitions
const FSM_TRANSITION broadway_to_broadway             = {BROADWAY, trigger_bw_to_bw,                  noAction, &STATE_BROADWAY};
const FSM_TRANSITION broadway_to_broadway_southbound  = {BROADWAY, trigger_bw_to_bw_south,            noAction, &STATE_BROADWAY_SOUTHBOUND};
const FSM_TRANSITION broadway_to_broadway_turn_ped    = {BROADWAY, trigger_bw_to_bw_right_and_ped,    noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS};
const FSM_TRANSITION broadway_to_little_street        = {BROADWAY, trigger_bw_to_little,              noAction, &STATE_LITTLE_STREET};
const FSM_TRANSITION broadway_to_broadway_little_ped  = {BROADWAY, trigger_bw_to_bw_and_little_ped,   noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN};

FSM_TRANSITION broadway_south_to_broadway_turn_ped      = {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw_right_and_ped,   noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS};
FSM_TRANSITION broadway_south_to_little_street          = {BROADWAY_SOUTHBOUND, trigger_bw_south_to_little,             noAction, &STATE_LITTLE_STREET};
FSM_TRANSITION broadway_south_to_broadway_little_ped    = {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw_and_little_ped,  noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN};
FSM_TRANSITION broadway_south_to_broadway               = {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw,                 noAction, &STATE_BROADWAY};

FSM_TRANSITION broadway_turn_ped_to_little_street       = {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_little,               noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS};
FSM_TRANSITION broadway_turn_ped_to_broadway_little_ped = {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_and_little_ped,    noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN};
FSM_TRANSITION broadway_turn_ped_to_broadway            = {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw,                   noAction, &STATE_BROADWAY};
FSM_TRANSITION broadway_turn_ped_to_broadway_southbound = {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_south,             noAction, &STATE_BROADWAY_SOUTHBOUND};

FSM_TRANSITION little_street_to_broadway_little_ped = {LITTLE_STREET, trigger_little_to_bw_and_little_ped,  noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN};
FSM_TRANSITION little_street_to_broadway            = {LITTLE_STREET, trigger_little_to_bw,                 noAction, &STATE_BROADWAY};
FSM_TRANSITION little_street_to_broadway_southbound = {LITTLE_STREET, trigger_little_to_bw_south,           noAction, &STATE_BROADWAY_SOUTHBOUND};
FSM_TRANSITION little_street_to_broadway_turn_ped   = {LITTLE_STREET, trigger_little_to_bw_right_and_ped,   noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS};

FSM_TRANSITION broadway_little_ped_to_broadway              = {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw, noAction, &STATE_BROADWAY};
FSM_TRANSITION broadway_little_ped_to_broadway_south        = {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_south, noAction, &STATE_BROADWAY_SOUTHBOUND};
FSM_TRANSITION broadway_little_ped_to_broadway_turn_ped     = {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_right_and_ped, noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS};
FSM_TRANSITION broadway_little_ped_to_little                = {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_little, noAction, &STATE_LITTLE_STREET};
*/


FSM_TRANSITION_TABLE transitionTable = {BROADWAY, {
    // Away from Broadway Transitions
    {BROADWAY, trigger_bw_to_bw,                  noAction, &STATE_BROADWAY},
    {BROADWAY, trigger_bw_to_bw_south,            noAction, &STATE_BROADWAY_SOUTHBOUND},
    {BROADWAY, trigger_bw_to_bw_right_and_ped,    noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {BROADWAY, trigger_bw_to_little,              noAction, &STATE_LITTLE_STREET},
    {BROADWAY, trigger_bw_to_bw_and_little_ped,   noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    // Away from Broadway South Transitions
    {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw_right_and_ped,   noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {BROADWAY_SOUTHBOUND, trigger_bw_south_to_little,             noAction, &STATE_LITTLE_STREET},
    {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw_and_little_ped,  noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw,                 noAction, &STATE_BROADWAY},
    // Away from Broadway Turn and Pedestrian Transitions
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_little,               noAction, &STATE_LITTLE_STREET},
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_and_little_ped,    noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw,                   noAction, &STATE_BROADWAY},
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_south,             noAction, &STATE_BROADWAY_SOUTHBOUND},
    // Away from Little Street Transitions
    {LITTLE_STREET, trigger_little_to_bw_and_little_ped,  noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {LITTLE_STREET, trigger_little_to_bw,                 noAction, &STATE_BROADWAY},
    {LITTLE_STREET, trigger_little_to_bw_south,           noAction, &STATE_BROADWAY_SOUTHBOUND},
    {LITTLE_STREET, trigger_little_to_bw_right_and_ped,   noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    // Away from Broadway and Little Street Pedestrian Transitions
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_south, noAction, &STATE_BROADWAY_SOUTHBOUND},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_right_and_ped, noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_little, noAction, &STATE_LITTLE_STREET},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, noTrigger, noAction, &STATE_BROADWAY},
    // Intermediate Transitions/Transitions
    {INTERSECTION_AMBER, noTrigger, noAction, &STATE_RED},
    {INTERSECTION_RED, noTrigger, noAction, &STATE_AFTER_RED}
}};

Initialiser initialiseIntersectionStates() { 
    intersectionStates[BROADWAY] = (IntersectionLightState){
        RED,    //Little Street
        RED,    //Little Street Ped
        RED,    //Broadway South Turn
        GREEN,  //Broadway South Straight
        GREEN,  //Broadway North
        GREEN,  //Broadway North Turn
        RED     //Broadway Pedestrian
    };
    intersectionStates[BROADWAY_SOUTHBOUND] = (IntersectionLightState){
        RED,    //Little Street
        RED,    //Little Street Ped
        GREEN,  //Broadway South Turn
        GREEN,  //Broadway South Straight
        RED,    //Broadway North
        RED,    //Broadway North Turn
        RED     //Broadway Pedestrian
    };
    intersectionStates[BROADWAY_TURN_AND_PEDESTRIANS] = (IntersectionLightState){
        RED,    //Little Street
        RED,    //Little Street Ped
        GREEN,  //Broadway South Turn
        RED,    //Broadway South Straight
        RED,    //Broadway North
        RED,    //Broadway North Turn
        GREEN   //Broadway Pedestrian
    };
    intersectionStates[LITTLE_STREET] = (IntersectionLightState){
        GREEN,  //Little Street
        RED,    //Little Street Ped
        RED,    //Broadway South Turn
        RED,    //Broadway South Straight
        RED,    //Broadway North
        RED,    //Broadway North Turn
        RED     //Broadway Pedestrian
    };
    intersectionStates[BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN] = (IntersectionLightState){
        RED,    //Little Street
        GREEN,  //Little Street Ped
        RED,    //Broadway South Turn
        GREEN,  //Broadway South Straight
        GREEN,  //Broadway North
        RED,    //Broadway North Turn
        RED     //Broadway Pedestrian
    };
}

extern Sensor sensor0;
extern Sensor sensor1;
extern Sensor sensor2;
extern Sensor sensor3;
extern Sensor sensor4;
extern Sensor sensor5;
extern Sensor sensor6;
/**
 * Broadway State Triggers
 */
bool trigger_bw_to_bw() { return !sensor1.triggered && !sensor2.triggered && !sensor5.triggered && !sensor6.triggered; }
bool trigger_bw_to_bw_south() { return sensor1.triggered && !sensor5.triggered; }
bool trigger_bw_to_bw_right_and_ped() { return sensor5.triggered; }
bool trigger_bw_to_little() { return !sensor1.triggered && sensor2.triggered && !sensor5.triggered; }
bool trigger_bw_to_bw_and_little_ped() { return !sensor1.triggered && !sensor2.triggered && !sensor5.triggered && sensor6.triggered; }

/**
 * Broadway Southbound State Triggers
 */
bool trigger_bw_south_to_bw_right_and_ped() { return sensor5.triggered; }
bool trigger_bw_south_to_little() {return sensor2.triggered && !sensor5.triggered;}
bool trigger_bw_south_to_bw_and_little_ped() { return !sensor2.triggered && !sensor5.triggered && sensor6.triggered; }
bool trigger_bw_south_to_bw() { return !sensor2.triggered && !sensor5.triggered && !sensor6.triggered; }

/**
 * Broadway Right Turn && Pedestrian State Triggers
 */
bool trigger_bw_right_and_ped_to_little() { return sensor2.triggered; }
bool trigger_bw_right_and_ped_to_bw_and_little_ped() { return !sensor2.triggered && sensor6.triggered; }
bool trigger_bw_right_and_ped_to_bw() { return !sensor2.triggered && !sensor6.triggered && (sensor3.triggered || sensor4.triggered || (sensor0.triggered && !sensor1.triggered)); }
bool trigger_bw_right_and_ped_to_bw_south() { return sensor1.triggered && !sensor2.triggered && !sensor3.triggered && !sensor4.triggered && !sensor6.triggered; }

/**
 * Little Street State Triggers
 */
bool trigger_little_to_bw_and_little_ped() { return sensor6.triggered; }
bool trigger_little_to_bw() { return !sensor6.triggered && (sensor3.triggered || sensor4.triggered || (sensor0.triggered && !sensor1.triggered)); }
bool trigger_little_to_bw_south() { return sensor1.triggered && !sensor3.triggered && !sensor4.triggered && !sensor6.triggered; }
bool trigger_little_to_bw_right_and_ped() { return !sensor0.triggered && !sensor3.triggered && !sensor4.triggered && sensor5.triggered && !sensor6.triggered; }

/**
 * Broadway and Little Street Pedestrian State Triggers
 */
bool trigger_bw_and_little_ped_to_bw() { return sensor3.triggered || sensor4.triggered || (sensor0.triggered && !sensor1.triggered); }
bool trigger_bw_and_little_ped_to_bw_south() { return sensor1.triggered && !sensor3.triggered && !sensor4.triggered; }
bool trigger_bw_and_little_ped_to_bw_right_and_ped() { return !sensor0.triggered && !sensor3.triggered && !sensor4.triggered && sensor5.triggered; }
bool trigger_bw_and_little_ped_to_little() { return !sensor0.triggered && !sensor1.triggered && sensor2.triggered && !sensor3.triggered && !sensor4.triggered && !sensor5.triggered; }