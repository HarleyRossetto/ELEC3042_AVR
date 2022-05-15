#include "intersection.h"
#include "../fsm.h"
#include "../sensor.h"
#include "../drivers/mcp23s17/mcp23s17.h"
#include "trafficLight.h"

extern Sensor sensor0;
extern Sensor sensor1;
extern Sensor sensor2;
extern Sensor sensor3;
extern Sensor sensor4;
extern Sensor sensor5;
extern Sensor sensor6;

extern TrafficLight tl_Broadway_North;
extern TrafficLight tl_Broadway_North_Turn;
extern TrafficLight tl_Broadway_South;
extern TrafficLight tl_Broadway_South_Turn;
extern TrafficLight tl_Broadway_Pedestrian;
extern TrafficLight tl_Little_Street_Pedestrian;
extern TrafficLight tl_Little_Street;

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
    {BROADWAY_SOUTHBOUND, noTrigger,                              noAction, &STATE_BROADWAY},
    // Away from Broadway Turn and Pedestrian Transitions
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_little,               noAction, &STATE_LITTLE_STREET},
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_and_little_ped,    noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_south,             noAction, &STATE_BROADWAY_SOUTHBOUND},
    {BROADWAY_TURN_AND_PEDESTRIANS, noTrigger,                                        noAction, &STATE_BROADWAY},
    // Away from Little Street Transitions
    {LITTLE_STREET, trigger_little_to_bw_and_little_ped,  noAction, &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {LITTLE_STREET, trigger_little_to_bw_south,           noAction, &STATE_BROADWAY_SOUTHBOUND},
    {LITTLE_STREET, trigger_little_to_bw_right_and_ped,   noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {LITTLE_STREET, noTrigger,                            noAction, &STATE_BROADWAY},
    // Away from Broadway and Little Street Pedestrian Transitions
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_south,         noAction, &STATE_BROADWAY_SOUTHBOUND},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_right_and_ped, noAction, &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_little,           noAction, &STATE_LITTLE_STREET},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, noTrigger,                                     noAction, &STATE_BROADWAY},
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
    intersectionStates[INTERSECTION_AMBER] = (IntersectionLightState){
        YELLOW,  //Little Street
        YELLOW,  //Little Street Ped
        YELLOW,  //Broadway South Turn
        YELLOW,  //Broadway South Straight
        YELLOW,  //Broadway North
        YELLOW,  //Broadway North Turn
        YELLOW   //Broadway Pedestrian
    };
     intersectionStates[INTERSECTION_RED] = (IntersectionLightState){
        RED,  //Little Street
        RED,  //Little Street Ped
        RED,  //Broadway South Turn
        RED,  //Broadway South Straight
        RED,  //Broadway North
        RED,  //Broadway North Turn
        RED   //Broadway Pedestrian
    };
}

void applyIntersectionState(IntersectionLightState *state) {
    TrafficLight_SetState(&tl_Broadway_North, state->broadway_north_state);
    TrafficLight_SetState(&tl_Broadway_North_Turn, state->broadway_north_turn_state);
    TrafficLight_SetState(&tl_Broadway_South, state->broadway_south_straight_state);
    TrafficLight_SetState(&tl_Broadway_South_Turn, state->broadway_south_turn_state);
    TrafficLight_SetState(&tl_Broadway_Pedestrian, state->broadway_pedestrian_state);
    TrafficLight_SetState(&tl_Little_Street_Pedestrian, state->little_street_pedestrian_state);
    TrafficLight_SetState(&tl_Little_Street, state->little_street_state);
}

IntersectionLightState mixIntersectionStates(IntersectionLightState *from, IntersectionLightState *to, TrafficLightState mixPhase) {
    IntersectionLightState mixed;
    
    TrafficLightState mixValue = (mixPhase == YELLOW) ? YELLOW : RED;

    if (from->broadway_north_state != to->broadway_north_state) {
        mixed.broadway_north_state = mixValue;
    } else {
        mixed.broadway_north_state = to->broadway_north_state;
    }

    if (from->broadway_north_turn_state != to->broadway_north_turn_state) {
        mixed.broadway_north_turn_state = mixValue;
    } else {
        mixed.broadway_north_turn_state = to->broadway_north_turn_state;
    }

    if (from->broadway_south_straight_state != to->broadway_south_straight_state) {
        mixed.broadway_south_straight_state = mixValue;
    } else {
        mixed.broadway_south_straight_state = to->broadway_south_straight_state;
    }

    if (from->broadway_south_turn_state != to->broadway_south_turn_state) {
        mixed.broadway_south_turn_state = mixValue;
    } else {
        mixed.broadway_south_turn_state = to->broadway_south_turn_state;
    }

    if (from->broadway_pedestrian_state != to->broadway_pedestrian_state) {
        mixed.broadway_pedestrian_state = mixValue;
    } else {
        mixed.broadway_pedestrian_state = to->broadway_pedestrian_state;
    }

    if (from->little_street_pedestrian_state != to->little_street_pedestrian_state) {
        mixed.little_street_pedestrian_state = mixValue;
    } else {
        mixed.little_street_pedestrian_state = to->little_street_pedestrian_state;
    }

    if (from->little_street_state != to->little_street_state) {
        mixed.little_street_state = mixValue;
    } else {
        mixed.little_street_state = to->little_street_state;
    }

    return mixed;
}

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