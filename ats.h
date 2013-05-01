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
tw_stime g_lookahead = 1.0;

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

tw_lpid num_cells_per_kp = 0;
tw_lpid vp_per_proc = 0;

/* END GLOBALS */

// Command line arguments
unsigned int autonomous;

/****************************************** ENUMS ****************************/

// Events enumeration:
enum events { LIGHT_CHANGE, CAR_ARRIVES };

// Traffic lights enumeration:
enum light_colors { RED, GREEN };

// Directions to determine which way traffic is permitted in an intersection:
enum intersection_directions { NORTH_SOUTH, EAST_WEST };
enum cardinal_directions { WEST, EAST, SOUTH, NORTH };

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
    // Enumeration to hold the current direction:
    //enum directions current_direction;
} car_type;

// Message repesentation:
typedef struct {
    // Enumeration for events:
    enum events event_type;
    // Struct to hold the car this message is referring to:
    car_type car;
} message_data;

// Representation of a lane:
typedef struct {

    // Number of cars in this lane:
    car_type cars[30];

    // Number of cars in this lane:
    int number_of_cars;

    // The traffic light color for this lane:
    enum light_colors light;

} lane_type;

// Representation of an intersection:
typedef struct {

    // Number of cars arrived at this intersection:
    int total_cars_arrived;
    // Number of cars finished at this intersection:
    int total_cars_finished;

    // Four arrays to represent the number of lanes in each 4-way intersection:
    lane_type north_lanes[MAX_LANES_PER_DIRECTION];
    lane_type south_lanes[MAX_LANES_PER_DIRECTION];
    lane_type west_lanes[MAX_LANES_PER_DIRECTION];
    lane_type east_lanes[MAX_LANES_PER_DIRECTION];

    // Number of lanes for each direction:
    int number_of_north_lanes;
    int number_of_south_lanes;
    int number_of_west_lanes;
    int number_of_east_lanes;
    
    // Describes whether a direction will get a green arrow:
    int has_green_arrow;

    // Variable to hold the time remaining on the intersection:
    int time_remaining;

    // Variable to hold the total time that this light waits:
    int total_time;

    // Variable to hold the total time for a left-turn arrow:
    int left_total_time;

    // Variable to hold the direction the lights are going:
    enum intersection_directions traffic_direction;
} intersection_state;

/** END STRUCTS BLOCK **/

/**********************************FUNCTION PROTOTYPES************************/

// Initialization of an intersection:
void intersection_startup(intersection_state*, tw_lp*);

// Event handler for an intersection:
void intersection_eventhandler(intersection_state*, tw_bf*, message_data*, tw_lp*);

// Reverse event handler for an intersection:
void intersection_reverse_eventhandler(intersection_state*, tw_bf*, message_data*, tw_lp*);
// Temp
void intersection_reverse_eventhandler(intersection_state* s, tw_bf* b, message_data* d, tw_lp* l) { }

// Function to collection statistics for an intersection:
void intersection_statistics_collectstats(intersection_state*, tw_lp*);
// Temp
void intersection_statistics_collectstats(intersection_state* s, tw_lp* l) { }

// Mapping functions
tw_peid cell_mapping_lp_to_pe(tw_lpid lpid);
tw_lp* cell_mapping_to_lp(tw_lpid lpid);
tw_lpid cell_mapping_to_local_index(tw_lpid lpid);
void traffic_grid_mapping();
tw_lpid cell_compute_move(tw_lpid lpid, int direction);

/*******************************END FUNCTION PROTOTYPES***********************/

#endif /* _ATS_H_ */