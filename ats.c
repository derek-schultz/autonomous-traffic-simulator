/*
 * ats.c - Autonomous Traffic Simulator - Parallel Programming Group Project
 * to simulate robotic cars in traffic conditions.
 *
 * Team Members:
 * Bryant Pong
 * Matt Hancock
 * Derek Schultz
 *
 * CSCI-4320
 * 4/27/13
 * Last Updated: 4/27/13 - 6:44 PM
 */

// Include the ROSS Library:
#include <ross.h>

/** DEFINES **/

// Number of cells in the grid (default is 64 x 64 grid):
#define MAP_WIDTH 64
#define MAP_HEIGHT 64

// Maximum number of cars allowed on a lane:
#define MAX_CARS 30

// Maximum number of lanes per direction:
#define MAX_LANES_PER_DIRECTION 3

/** END DEFINES BLOCK **/

/****************************************** ENUMS *******************************/

// Events enumeration:
enum events { LIGHT_CHANGE, CAR_ARRIVES };

// Traffic lights enumeration:
enum light_colors { RED, GREEN };

/**************************************** END ENUMS BLOCK ***********************/

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
	int north_has_green_arrow;
	int south_has_green_arrow;
	int west_has_green_arrow;
	int east_has_green_arrow;
} intersection_state;

/** END STRUCTS BLOCK **/

/**********************************FUNCTION PROTOTYPES************************/

// Initialization of an intersection:
void intersection_startup(intersection_state*, tw_lp*);

// Event handler for an intersection:
void intersection_eventhandler(intersection_state*, tw_bf*, message_data*, tw_lp*);

// Reverse event handler for an intersection:
void intersection_reverse_eventhandler(intersection_state*, tw_bf*, message_data*,
										tw_lp*);

// Function to collection statistics for an intersection:
void intersection_statistics_collectstats(intersection_state*, tw_lp *);

/*******************************END FUNCTION PROTOTYPES***********************/

// Main Function:
int main(int argc, char* argv[]) {

	return 0;
} /** END FUNCTION main **/
