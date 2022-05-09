#if !defined(TRAFFIC_LIGHT_H)
#define TRAFFIC_LIGHT_H
#include "light.h"

typedef enum { RED, YELLOW, GREEN} TrafficLightState;

typedef struct {
    Light red;
    Light yellow;
    Light green;
    TrafficLightState activeLight;
} TrafficLight;

void TrafficLight_Clear(Light *light);
void TrafficLight_Set(Light *light);
void TrafficLight_Toggle(Light *light);

void TrafficLight_Green(TrafficLight *light);
void TrafficLight_Yellow(TrafficLight *light);
void TrafficLight_Red(TrafficLight *light);
void TrafficLight_Hazard(TrafficLight *light);
void TrafficLight_Blank(TrafficLight *light);

TrafficLight TrafficLight_CreateInternal();
TrafficLight TrafficLight_CreateExternal();

#endif // TRAFFIC_LIGHT_H
