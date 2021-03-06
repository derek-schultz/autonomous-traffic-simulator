/*
 * ats.h - Autonomous Traffic Simulator - Parallel Programming Group Project
 * to simulate robotic cars in traffic conditions.
 *
 * Team Members:
 * Matt Hancock
 * Bryant Pong
 * Derek Schultz
 *
 * CSCI-4320
 * 2013-05-07
 *
 */

#ifndef _ATS_H_
#define _ATS_H_

//#define DEBUG

// Include the ROSS Library:
#include <ross.h>

/** DEFINES **/

// Number of cells in the grid (default is 64 x 64 grid):
#define MAP_WIDTH 512
#define MAP_HEIGHT 512

#define NUM_VP_X 32
#define NUM_VP_Y 32

#define GREEN_LIGHT_DURATION 20
#define LEFT_TURN_LIGHT_DURATION 10
#define CAR_ACCELERATION_DELAY 1.0
#define INITIAL_ARRIVAL_MEAN 100
#define MINIMUM_TRAVEL_TIME 60
#define TRAVEL_TIME_VARIATION 10
#define MAX_TRAVEL_DISTANCE 200
#define INTERSECTION_CROSSING_TIME 4

/** END DEFINES BLOCK **/

/*
 * GLOBALS AND CONSTANTS
 */

// VPs per PE?
extern tw_lpid g_vp_per_proc; // set in main

// LPs per PE?
extern tw_lpid g_cells_per_vp_x;
extern tw_lpid g_cells_per_vp_y;
extern tw_lpid g_cells_per_vp;

// Average service time?
extern tw_stime g_mean_service;

// QUESTION: lookahead?
extern tw_stime g_lookahead;

extern tw_stime g_full_cycle_duration;

// QUESTION: mult?
// Why are all these static?
extern tw_stime g_mult;

// Number of LPs per PE
extern unsigned int g_nlp_per_pe;

// TODO: figure out what this means
extern int g_traffic_start_events;

// TODO: figure out what this means
extern int g_optimistic_memory; 

// rate for timestamp exponential distribution
extern tw_stime g_mean;

// Holds the total cars initiated and completed for statistics
extern unsigned long long g_total_cars;
extern unsigned long long g_cars_finished;
extern unsigned long long g_total_time;
extern unsigned long long g_average_time;

extern tw_lpid num_cells_per_kp;
extern tw_lpid vp_per_proc;

/* END GLOBALS */

// Command line arguments
unsigned int autonomous;

/****************************************** ENUMS ****************************/

// Events enumeration:
enum event { LIGHT_CHANGE, CAR_ARRIVES,
             CAR_MOVES_FORWARD, CAR_ENTERS_INTERSECTION };

// Traffic lights enumeration:
enum light_color { RED, GREEN };

// Directions to determine which way traffic is permitted in an intersection:
enum intersection_direction { NORTH_SOUTH, NORTH_SOUTH_LEFT,
                              EAST_WEST,   EAST_WEST_LEFT };

// The different lanes a car can be in
enum intersection_position { WEST, WEST_LEFT, EAST, EAST_LEFT,
                             SOUTH, SOUTH_LEFT, NORTH, NORTH_LEFT };

// The different travel intentions a car can have
// NL indicates coming from the north, turning left, etc.
enum travel_direction { NL, NR, NS, EL, ER, ES, SL, SR, SS, WL, WR, WS };

/**************************************** END ENUMS BLOCK ********************/

/** STRUCTS **/

// Car representation:
typedef struct {
    // Variable to hold the start time of this car:
    int start_time;
    // Variable to hold the end time of this car:
    int end_time;
    // Variables to hold the destination's X and Y coordinates:
    int x_to_go;
    int y_to_go;
	
	int x_to_go_original;
    int y_to_go_original;

    enum intersection_position position;
    enum travel_direction intention;
    int queue_location;

    tw_lpid next_intersection;
    
	int has_turned;

    unsigned int id;
	
} car_type;

// Message repesentation:
typedef struct {
    // Enumeration for events:
    enum event event_type;
    
    // Struct to hold the car this message is referring to:
    car_type car;

    // We need to save light timing information so we can reverse it
    tw_stime saved_green_until;

    tw_stime saved_north_lead_car_will_enter;
    tw_stime saved_north_left_lead_car_will_enter;
    tw_stime saved_east_lead_car_will_enter;
    tw_stime saved_east_left_lead_car_will_enter;
    tw_stime saved_south_lead_car_will_enter;
    tw_stime saved_south_left_lead_car_will_enter;
    tw_stime saved_west_lead_car_will_enter;
    tw_stime saved_west_left_lead_car_will_enter;

    tw_stime saved_NS_lock, saved_NR_lock, saved_NL_lock;
    tw_stime saved_ES_lock, saved_ER_lock, saved_EL_lock;
    tw_stime saved_WS_lock, saved_WR_lock, saved_WL_lock;
    tw_stime saved_SS_lock, saved_SR_lock, saved_SL_lock;

} message_data;

// Representation of an intersection:
typedef struct {

    // Number of cars arrived at this intersection:
    int total_cars_arrived;
    // Number of cars finished at this intersection:
    int total_cars_finished;

	// Number of cars arriving in each direction:
	int num_cars_south;
	int num_cars_west;
	int num_cars_north;
	int num_cars_east;

    int num_cars_south_left;
    int num_cars_west_left;
    int num_cars_north_left;
    int num_cars_east_left;
    
    // Variables to hold the timing of the lights
    tw_stime north_south_green_until;
    tw_stime north_south_left_green_until;
    tw_stime east_west_green_until;
    tw_stime east_west_left_green_until;

    tw_stime north_lead_car_will_enter, north_left_lead_car_will_enter;
    tw_stime east_lead_car_will_enter, east_left_lead_car_will_enter;
    tw_stime south_lead_car_will_enter, south_left_lead_car_will_enter;
    tw_stime west_lead_car_will_enter, west_left_lead_car_will_enter;
    
    tw_stime NS_lock, NR_lock, NL_lock;
    tw_stime ES_lock, ER_lock, EL_lock;
    tw_stime WS_lock, WR_lock, WL_lock;
    tw_stime SS_lock, SR_lock, SL_lock;

    // Variable to hold the direction the lights are going:
    enum intersection_direction traffic_direction;
} intersection_state;

/** END STRUCTS BLOCK **/

/**********************************FUNCTION PROTOTYPES************************/

// With the old traffic lights

void traffic_light_intersection_startup(intersection_state*, tw_lp*);

void traffic_light_intersection_eventhandler(
    intersection_state*,
    tw_bf*,
    message_data*,
    tw_lp*
);

void traffic_light_intersection_reverse_eventhandler(
    intersection_state*,
    tw_bf*,
    message_data*,
    tw_lp*
);

// With communicating autonomous vehicles

int will_collide(enum travel_direction, enum travel_direction);

void autonomous_traffic_intersection_startup(intersection_state*, tw_lp*);

void autonomous_traffic_intersection_eventhandler(
    intersection_state*,
    tw_bf*,
    message_data*,
    tw_lp*
);

void autonomous_traffic_intersection_reverse_eventhandler(
    intersection_state*,
    tw_bf*,
    message_data*,
    tw_lp*
);

// Function to collection statistics for an intersection:
void intersection_statistics_collectstats(intersection_state*, tw_lp*);

// Mapping functions
tw_peid cell_mapping_lp_to_pe(tw_lpid lpid);
tw_lp* cell_mapping_to_lp(tw_lpid lpid);
tw_lpid cell_mapping_to_local_index(tw_lpid lpid);
void traffic_grid_mapping();
tw_lpid cell_compute_move(tw_lpid lpid, int direction);

/*******************************END FUNCTION PROTOTYPES***********************/

#endif /* _ATS_H_ */
