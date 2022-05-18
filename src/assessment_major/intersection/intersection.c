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
    {BROADWAY, trigger_bw_to_bw,                  action_clear_triggers_bw_to_bw,                   &STATE_BROADWAY},
    {BROADWAY, trigger_bw_to_bw_south,            action_clear_triggers_bw_to_bw_south,             &STATE_BROADWAY_SOUTHBOUND},
    {BROADWAY, trigger_bw_to_bw_right_and_ped,    action_clear_triggers_bw_to_bw_right_and_ped,     &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {BROADWAY, trigger_bw_to_little,              action_clear_triggers_bw_to_little,               &STATE_LITTLE_STREET},
    {BROADWAY, trigger_bw_to_bw_and_little_ped,   action_clear_triggers_bw_to_bw_and_little_ped,    &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    // Away from Broadway South Transitions
    {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw_right_and_ped,   action_clear_triggers_bw_south_to_bw_right_and_ped,   &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {BROADWAY_SOUTHBOUND, trigger_bw_south_to_little,             action_clear_triggers_bw_south_to_little,             &STATE_LITTLE_STREET},
    {BROADWAY_SOUTHBOUND, trigger_bw_south_to_bw_and_little_ped,  action_clear_triggers_bw_south_to_bw_and_little_ped,  &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {BROADWAY_SOUTHBOUND, noTrigger,                              action_clear_triggers_bw_to_bw,                       &STATE_BROADWAY},
    // Away from Broadway Turn and Pedestrian Transitions
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_little,               action_clear_triggers_bw_right_and_ped_to_little,             &STATE_LITTLE_STREET},
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_and_little_ped,    action_clear_triggers_bw_right_and_ped_to_bw_and_little_ped,  &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {BROADWAY_TURN_AND_PEDESTRIANS, trigger_bw_right_and_ped_to_bw_south,             action_clear_triggers_bw_right_and_ped_to_bw_south,           &STATE_BROADWAY_SOUTHBOUND},
    {BROADWAY_TURN_AND_PEDESTRIANS, noTrigger,                                        action_clear_triggers_bw_to_bw,                                                     &STATE_BROADWAY},
    // Away from Little Street Transitions
    {LITTLE_STREET, trigger_little_to_bw_and_little_ped,  action_clear_triggers_little_to_bw_and_little_ped,    &STATE_BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN},
    {LITTLE_STREET, trigger_little_to_bw_south,           action_clear_triggers_little_to_bw_south,             &STATE_BROADWAY_SOUTHBOUND},
    {LITTLE_STREET, trigger_little_to_bw_right_and_ped,   action_clear_triggers_little_to_bw_right_and_ped,     &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {LITTLE_STREET, noTrigger,                            action_clear_triggers_bw_to_bw,                       &STATE_BROADWAY},
    // Away from Broadway and Little Street Pedestrian Transitions
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_south,         action_clear_triggers_bw_and_little_ped_to_bw_south,            &STATE_BROADWAY_SOUTHBOUND},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_bw_right_and_ped, action_clear_triggers_bw_and_little_ped_to_bw_right_and_ped,    &STATE_BROADWAY_TURN_AND_PEDESTRIANS},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, trigger_bw_and_little_ped_to_little,           action_clear_triggers_bw_and_little_ped_to_little,              &STATE_LITTLE_STREET},
    {BROADWAY_STRAIGHT_AND_LITTLE_STREET_PEDESTRIAN, noTrigger,                                     action_clear_triggers_bw_to_bw,                                 &STATE_BROADWAY},
    // Intermediate Transitions/Transitions
    {INTERSECTION_AMBER,    noTrigger, noAction, &STATE_RED},
    {INTERSECTION_RED,      noTrigger, noAction, &STATE_AFTER_RED}
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

    // Are we mixing to yellow or red?
    TrafficLightState mixValue = (mixPhase == YELLOW) ? YELLOW : RED;

    // If the current and next state are different, take the mix value.
    if (from->broadway_north_state != to->broadway_north_state && from->broadway_north_state != RED) {
        mixed.broadway_north_state = mixValue;
    } else {
        //Otherwise they are the same so just assume the current state.
        mixed.broadway_north_state = from->broadway_north_state;
    }

    if (from->broadway_north_turn_state != to->broadway_north_turn_state && from->broadway_north_turn_state != RED) {
        mixed.broadway_north_turn_state = mixValue;
    } else {
        mixed.broadway_north_turn_state = from->broadway_north_turn_state;
    }

    if (from->broadway_south_straight_state != to->broadway_south_straight_state && from->broadway_south_straight_state != RED) {
        mixed.broadway_south_straight_state = mixValue;
    } else {
        mixed.broadway_south_straight_state = from->broadway_south_straight_state;
    }

    if (from->broadway_south_turn_state != to->broadway_south_turn_state && from->broadway_south_turn_state != RED) {
        mixed.broadway_south_turn_state = mixValue;
    } else {
        mixed.broadway_south_turn_state = from->broadway_south_turn_state;
    }

    if (from->broadway_pedestrian_state != to->broadway_pedestrian_state && from->broadway_pedestrian_state != RED) {
        mixed.broadway_pedestrian_state = mixValue;
    } else {
        mixed.broadway_pedestrian_state = from->broadway_pedestrian_state;
    }

    if (from->little_street_pedestrian_state != to->little_street_pedestrian_state && from->little_street_pedestrian_state != RED) {
        mixed.little_street_pedestrian_state = mixValue;
    } else {
        mixed.little_street_pedestrian_state = from->little_street_pedestrian_state;
    }

    if (from->little_street_state != to->little_street_state && from->little_street_state != RED) {
        mixed.little_street_state = mixValue;
    } else {
        mixed.little_street_state = from->little_street_state;
    }

    return mixed;
}

/**
 * Broadway State Triggers
 */
bool trigger_bw_to_bw() { return !sensor1.triggered && !sensor2.triggered && !sensor5.triggered && !sensor6.triggered; }
bool trigger_bw_to_bw_south() { return sensor1.triggered; }
bool trigger_bw_to_bw_right_and_ped() { return sensor5.triggered; }
bool trigger_bw_to_little() { return !sensor1.triggered && sensor2.triggered && !sensor5.triggered; }
bool trigger_bw_to_bw_and_little_ped() { return !sensor1.triggered && !sensor2.triggered && !sensor5.triggered && sensor6.triggered; }
/** Actions */
extern Sensor intersection_change_trigger_sensor;
Action action_clear_triggers_bw_to_bw() {

    Sensor sensorWithHighestWaitTime = sensor0;
    if (sensor3.periods_held > sensorWithHighestWaitTime.periods_held)
        sensorWithHighestWaitTime = sensor3;
    if (sensor4.periods_held > sensorWithHighestWaitTime.periods_held)
        sensorWithHighestWaitTime = sensor4;

    Sensor_ClearTriggeredFlag(&sensor0); 
    Sensor_ClearTriggeredFlag(&sensor3); 
    Sensor_ClearTriggeredFlag(&sensor4);

    intersection_change_trigger_sensor = sensorWithHighestWaitTime;
}
Action action_clear_triggers_bw_to_bw_south() { Sensor_ClearTriggeredFlag(&sensor1); }
Action action_clear_triggers_bw_to_bw_right_and_ped() { Sensor_ClearTriggeredFlag(&sensor5); }
Action action_clear_triggers_bw_to_little() { Sensor_ClearTriggeredFlag(&sensor2); }
Action action_clear_triggers_bw_to_bw_and_little_ped() { Sensor_ClearTriggeredFlag(&sensor6); }

/**
 * Broadway Southbound State Triggers
 */
bool trigger_bw_south_to_bw_right_and_ped() { return sensor5.triggered; }
bool trigger_bw_south_to_little() {return sensor2.triggered && !sensor5.triggered;}
bool trigger_bw_south_to_bw_and_little_ped() { return !sensor2.triggered && !sensor5.triggered && sensor6.triggered; }
bool trigger_bw_south_to_bw() { return !sensor2.triggered && !sensor5.triggered && !sensor6.triggered; }
/** Actions */
Action action_clear_triggers_bw_south_to_bw_right_and_ped() { Sensor_ClearTriggeredFlag(&sensor5); }
Action action_clear_triggers_bw_south_to_little() { Sensor_ClearTriggeredFlag(&sensor2); }
Action action_clear_triggers_bw_south_to_bw_and_little_ped() { Sensor_ClearTriggeredFlag(&sensor6); }

/**
 * Broadway Right Turn && Pedestrian State Triggers
 */
bool trigger_bw_right_and_ped_to_little() { return sensor2.triggered; }
bool trigger_bw_right_and_ped_to_bw_and_little_ped() { return !sensor2.triggered && sensor6.triggered; }
// UNUSED
bool trigger_bw_right_and_ped_to_bw() { return !sensor2.triggered && !sensor6.triggered && (sensor3.triggered || sensor4.triggered || (sensor0.triggered && !sensor1.triggered)); }
bool trigger_bw_right_and_ped_to_bw_south() { return sensor1.triggered && !sensor2.triggered && !sensor3.triggered && !sensor4.triggered && !sensor6.triggered; }
/** Actions */
Action action_clear_triggers_bw_right_and_ped_to_little() { Sensor_ClearTriggeredFlag(&sensor2); }
Action action_clear_triggers_bw_right_and_ped_to_bw_and_little_ped() { Sensor_ClearTriggeredFlag(&sensor6); }
Action action_clear_triggers_bw_right_and_ped_to_bw_south() { Sensor_ClearTriggeredFlag(&sensor1); }

/**
 * Little Street State Triggers
 */
bool trigger_little_to_bw_and_little_ped() { return sensor6.triggered; }
bool trigger_little_to_bw() { return !sensor6.triggered && (sensor3.triggered || sensor4.triggered || (sensor0.triggered && !sensor1.triggered)); }
bool trigger_little_to_bw_south() { return sensor1.triggered && !sensor3.triggered && !sensor4.triggered && !sensor6.triggered; }
bool trigger_little_to_bw_right_and_ped() { return !sensor0.triggered && !sensor3.triggered && !sensor4.triggered && sensor5.triggered && !sensor6.triggered; }
/** Actions */
Action action_clear_triggers_little_to_bw_and_little_ped() { Sensor_ClearTriggeredFlag(&sensor6); }
Action action_clear_triggers_little_to_bw_south() { Sensor_ClearTriggeredFlag(&sensor1); }
Action action_clear_triggers_little_to_bw_right_and_ped() { Sensor_ClearTriggeredFlag(&sensor5); }
/**
 * Broadway and Little Street Pedestrian State Triggers
 */
bool trigger_bw_and_little_ped_to_bw_south() { return sensor1.triggered && !sensor3.triggered && !sensor4.triggered; }
bool trigger_bw_and_little_ped_to_bw_right_and_ped() { return !sensor0.triggered && !sensor3.triggered && !sensor4.triggered && sensor5.triggered; }
bool trigger_bw_and_little_ped_to_little() { return !sensor0.triggered && !sensor1.triggered && sensor2.triggered && !sensor3.triggered && !sensor4.triggered && !sensor5.triggered; }
/** Actions */
Action action_clear_triggers_bw_and_little_ped_to_bw_south() { Sensor_ClearTriggeredFlag(&sensor1); }
Action action_clear_triggers_bw_and_little_ped_to_bw_right_and_ped() { Sensor_ClearTriggeredFlag(&sensor5); }
Action action_clear_triggers_bw_and_little_ped_to_little() { Sensor_ClearTriggeredFlag(&sensor2); }

Action action_clear_triggers_bw() {
    
}