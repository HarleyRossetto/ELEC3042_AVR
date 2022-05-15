#if !defined(INTERSECTION_H)
#define INTERSECTION_H
#include "../types.h"
#include "trafficLight.h"
#include "../bool.h"

//typedef enum { IS_BROADWAY, IS_BROADWAY_SOUTHBOUND, IS_BROADWAY_TURN_AND_PED, IS_LITTLE_STREET, IS_BROADWAY_AND_LITTLE_STREET_PED } I_States;

typedef struct  {
    TrafficLightState little_street_state;
    TrafficLightState little_street_pedestrian_state;
    TrafficLightState broadway_south_turn_state;
    TrafficLightState broadway_south_straight_state;
    TrafficLightState broadway_north_state;
    TrafficLightState broadway_north_turn_state;
    TrafficLightState broadway_pedestrian_state;
} IntersectionLightState;

#define MAX_STATES 7
extern IntersectionLightState intersectionStates[MAX_STATES];
typedef IntersectionLightState (*IntersectionLightStateArrayPtr)[MAX_STATES];

Initialiser initialiseIntersectionStates();

void applyIntersectionState(IntersectionLightState*);
IntersectionLightState mixIntersectionStates(IntersectionLightState *, IntersectionLightState *, TrafficLightState mixPhase);

/**
 * Broadway State Triggers
 */
bool trigger_bw_to_bw();
bool trigger_bw_to_bw_south();
bool trigger_bw_to_bw_right_and_ped();
bool trigger_bw_to_little();
bool trigger_bw_to_bw_and_little_ped();

/**
 * Broadway Southbound State Triggers
 */
bool trigger_bw_south_to_bw_right_and_ped();
bool trigger_bw_south_to_little();
bool trigger_bw_south_to_bw_and_little_ped();
bool trigger_bw_south_to_bw();

/**
 * Broadway Right Turn & Pedestrian State Triggers
 */
bool trigger_bw_right_and_ped_to_little();
bool trigger_bw_right_and_ped_to_bw_and_little_ped();
bool trigger_bw_right_and_ped_to_bw();
bool trigger_bw_right_and_ped_to_bw_south();

/**
 * Little Street State Triggers
 */
bool trigger_little_to_bw_and_little_ped();
bool trigger_little_to_bw();
bool trigger_little_to_bw_south();
bool trigger_little_to_bw_right_and_ped();

/**
 * Broadway and Little Street Pedestrian State Triggers
 */
bool trigger_bw_and_little_ped_to_bw();
bool trigger_bw_and_little_ped_to_bw_south();
bool trigger_bw_and_little_ped_to_bw_right_and_ped();
bool trigger_bw_and_little_ped_to_little();
#endif // INTERSECTION_H
