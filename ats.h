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

// Include the ROSS Library:
#include <ross.h>

/** DEFINES **/

// Number of cells in the grid (default is 64 x 64 grid):
#define MAP_WIDTH 64
#define MAP_HEIGHT 64

// Ask Jeremy what these are
#define NUM_VP_X 32
#define NUM_VP_Y 32

// Maximum number of cars allowed on a lane:
#define MAX_CARS 30

// Maximum number of lanes per direction:
#define MAX_LANES_PER_DIRECTION 3

#define GREEN_LIGHT_DURATION 20
#define LEFT_TURN_LIGHT_DURATION 10
#define CAR_ACCELERATION_DELAY 1.0

/** END DEFINES BLOCK **/

/*
 * GLOBALS AND CONSTANTS
 * Maybe temporary until I figure out what is going on!
 */

// VPs per PE?
tw_lpid g_vp_per_proc = 0; // set in main

// LPs per PE?
tw_lpid g_cells_per_vp_x = MAP_WIDTH / NUM_VP_X;
tw_lpid g_cells_per_vp_y = MAP_HEIGHT / NUM_VP_Y;
tw_lpid g_cells_per_vp = (MAP_WIDTH / NUM_VP_X) * (MAP_HEIGHT / NUM_VP_Y);

// Average service time?
tw_stime g_mean_service = 1.0;

// QUESTION: lookahead?
tw_stime g_lookahead = 20.0;

g_full_cycle_duration = 2*LEFT_TURN_LIGHT_DURATION + 2*GREEN_LIGHT_DURATION;

// QUESTION: mult?
// Why are all these static?
static tw_stime g_mult = 1.6;

// Number of LPs per PE
static unsigned int g_nlp_per_pe = 8;

// TODO: figure out what this means
static int g_traffic_start_events = 15;

// TODO: figure out what this means
static int g_optimistic_memory = 65536; // 64 KB

// rate for timestamp exponential distribution
static tw_stime g_mean = 1.0;

// Holds the total cars initiated and completed for statistics
static unsigned long long g_total_cars = 0;
static unsigned long long g_cars_finished = 0;
static unsigned long long g_total_time = 0;
static unsigned long long g_average_time = 0;

tw_lpid num_cells_per_kp = 0;
tw_lpid vp_per_proc = 0;

/* END GLOBALS */

// Command line arguments
unsigned int autonomous;

/****************************************** ENUMS ****************************/

// Events enumeration:
enum event { LIGHT_CHANGE, CAR_ARRIVES, CAR_DEPARTS };

// Traffic lights enumeration:
enum light_color { RED, GREEN };

// Directions to determine which way traffic is permitted in an intersection:
enum intersection_direction { NORTH_SOUTH, NORTH_SOUTH_LEFT,
                              EAST_WEST,   EAST_WEST_LEFT };
enum intersection_position { WEST, WEST_LEFT, EAST, EAST_LEFT,
                             SOUTH, SOUTH_LEFT, NORTH, NORTH_LEFT };
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

    intersection_position position;
    
	//int has_turned_yet;
	
	// Variable to hold the past intersection this car was in:
	//tw_lpid past_intersection;

} car_type;

// Message repesentation:
typedef struct {
    // Enumeration for events:
    enum events event_type;
    // Struct to hold the car this message is referring to:
    car_type car;
} message_data;

// Representation of a 3-lane intersection:
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

    int num_cars_total;

	// Number of cars leaving in each direction:
	int num_cars_out_south;
	int num_cars_out_west;
	int num_cars_out_north;
	int num_cars_out_east;
    
    // Describes whether a direction will get a green arrow:
    //int has_turning_arrow;

    // Variable to hold the time remaining on the intersection:
    int north_south_last_green;
    int north_south_green_until;
    int north_south_next_green;

    int north_south_left_last_green;
    int north_south_left_green_until;
    int north_south_left_next_green;

    int east_west_last_green;
    int east_west_green_until;
    int east_west_next_green;

    int east_west_left_last_green;
    int east_west_left_green_until;
    int east_west_left_next_green;

    // Variable to hold the total time that this light waits:
    int total_time;

    // Variable to hold the total time for a left-turn arrow:
    int left_total_time;

    // Variable to hold the direction the lights are going:
    enum intersection_directions traffic_direction;
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
