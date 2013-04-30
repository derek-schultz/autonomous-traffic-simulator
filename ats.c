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

// Directions to determine which way traffic is permitted in an intersection:
enum directions { NORTH_SOUTH, EAST_WEST };

// To determine the orientation of the car
enum orientation {NORTH_LEFT, NORTH_CENTER, NORTH_RIGHT, SOUTH_LEFT, SOUTH_CENTER, SOUTH_RIGHT,
					EAST_LEFT, EAST_CENTER, EAST_RIGHT, WEST_LEFT, WEST_CENTER, WEST_RIGHT}

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
	// Enumeration to hold the current direction:
	enum orientation current_orientation;
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
	enum directions traffic_direction;
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

// Event handler for an intersection:
void intersection_eventhandler(intersection_state* SV, tw_bf* CV, 
								message_data* M, tw_lp* LP) {

	// Time warp starting time:
	tw_stime ts = 0.0;
	
	// Current event:
	tw_event* current_event = NULL;

	// New message data:
	message_data* new_message = NULL;

	// Unknown time warp bit field:
	*(int* ) CV = (int) 0;
	
	// Handle the events defined in the "events" enumeration:
	switch(M->event_type) {

		// Handle the LIGHT_CHANGE event IF AND ONLY IF the time remaining on this intersection == 0:
		case LIGHT_CHANGE:
			
			// TIME EXPIRED!

			// Check if the traffic is permitted north-south (green on north and south lights):
			if(SV->traffic_direction == NORTH_SOUTH) {
				
				// Check if the left turning light is on (if any):
				if(SV->has_green_arrow) {
					// Check if this left-turn is GREEN:
					if(SV->north_lanes[0].light == GREEN) {
						// Turn the left-turn arrow to RED; turn on lanes to green:
						SV->north_lanes[0].light = SV->south_lanes[0].light = RED;
						
						int i;
						for(i = 1; i < number_of_north_lanes; i++) {	
							SV->north_lanes[i].light = GREEN;
						} 

						for(i = 1; i < number_of_south_lanes; i++) {
							SV->south_lanes[i].light = GREEN;
						}

						// Set the remaining time to total time:
						SV->time_remaining = SV->total_time;
					} else {
						// The left turn is RED; changing direction to EAST WEST:

						// Change permitted direction to east-west (green on east and west lights):
						SV->traffic_direction = EAST_WEST;

						// Turn off all the north and south lights to RED:
						int i;
						// North Lanes:
						for(i = 0; i < SV->number_of_north_lanes; i++) {
							SV->north_lanes[i].light = RED;
						}
	
						// South Lanes:
						for(i = 0; i < SV->number_of_south_lanes; i++) {
							SV->south_lanes[i].light = RED;
						}

						// Left-turning lanes exist:
						SV->east_lanes[0].light = GREEN;
						SV->west_lanes[0].light = GREEN;

						// Set the left total time to the total time:
						SV->time_remaining = SV->left_total_time;
					} 
				} else {
					// This intersection does not have any green left arrows; just change direction:

					// Change permitted direction to east-west (green on east and west lights):
					SV->traffic_direction = EAST_WEST;

					int i;

					// North Lanes:
					for(i = 0; i < SV->number_of_north_lanes; i++) {
						SV->north_lanes[i].light = RED;
					}
	
					// South Lanes:
					for(i = 0; i < SV->number_of_south_lanes; i++) {
						SV->south_lanes[i].light = RED;
					}
				
					// Change the East-West lanes to GREEN:
					for(i = 0; i < number_of_east_lanes; i++) {	
						SV->east_lanes[i].light = GREEN;
					} 

					for(i = 1; i < number_of_west_lanes; i++) {
						SV->west_lanes[i].light = GREEN;
					}
					SV->time_remaining = SV->total_time;
				}
			} else if(SV->traffic_direction == EAST_WEST) {
				
				// Check if the left turning light is on (if any):
				if(SV->has_green_arrow) {
					// Check if this left-turn is GREEN:
					if(SV->east_lanes[0].light == GREEN) {
						// Turn the left-turn arrow to RED; turn on lanes to green:
						SV->east_lanes[0].light = SV->west_lanes[0].light = RED;
						
						int i;
						for(i = 1; i < number_of_east_lanes; i++) {
							SV->east_lanes[i].light = GREEN;
						}
						
						for(i = 1; i < number_of_west_lanes; i++) {
							SV->west_lanes[i].light = GREEN;
						}
						
						// Set the remaining time to total time:
						SV->time_remaining = SV->total_time;
					} else {
						// The left turn is RED; changing direction to NORTH SOUTH:
						
						// Change permitted direction to north-south (green on north and south lights):
						SV->traffic_direction = NORTH_SOUTH;
						
						// Turn off all the north and south lights to RED:
						int i;
						// East Lanes:
						for(i = 0; i < SV->number_of_esat_lanes; i++) {
							SV->north_lanes[i].light = RED;
						}
						
						// West Lanes:
						for(i = 0; i < SV->number_of_west_lanes; i++) {
							SV->south_lanes[i].light = RED;
						}
						
						// Left-turning lanes exist:
						SV->north_lanes[0].light = GREEN;
						SV->south_lanes[0].light = GREEN;
						
						// Set the left total time to the total time:
						SV->time_remaining = SV->left_total_time;
					}
				} else {
					// This intersection does not have any green left arrows; just change direction:
					
					// Change permitted direction to north-south (green on north and south lights):
					SV->traffic_direction = NORTH_SOUTH;
					
					int i;
					
					// East Lanes:
					for(i = 0; i < SV->number_of_east_lanes; i++) {
						SV->east_lanes[i].light = RED;
					}
					
					// West Lanes:
					for(i = 0; i < SV->number_of_west_lanes; i++) {
						SV->west_lanes[i].light = RED;
					}
					
					// Change the North-South lanes to GREEN:
					for(i = 0; i < number_of_north_lanes; i++) {
						SV->north_lanes[i].light = GREEN;
					}
					
					for(i = 1; i < number_of_south_lanes; i++) {
						SV->south_lanes[i].light = GREEN;
					}
					SV->time_remaining = SV->total_time;
				}
			}
			
			ts = tw_rand_exponential(lp->rng, MEAN_SERVICE);
			current_event = tw_event_new(lp->gid, ts, lp);
			new_message = (Msg_Data *)tw_event_data(CurEvent);
			new_message->car.x_to_go = M->car.x_to_go;
			new_message->car.y_to_go = M->car.y_to_go;
			new_message->car.start_time = M->car.start_time;
			new_message->car.end_time = M->car.end_time;
			new_message->event_type = CAR_ARRIVES;
			//printf("send ari ");
			tw_event_send(current_event);
			
			break;
			
		case CAR_ARRIVES:
			switch(M->car_type.orientation) {
				case NORTH_LEFT:
					break;
					
				case NORTH_CENTER:
					break;
					
				case NORTH_RIGHT:
					break;
					
				case SOUTH_LEFT:
					break;
					
				case SOUTH_CENTER:
					break;
					
				case SOUTH_RIGHT:
					break;
					
				case EAST_LEFT:
					break;
					
				case EAST_CENTER:
					break;
					
				case EAST_RIGHT:
					break;
					
				case WEST_LEFT:
					break;
					
				case WEST_CENTER:
					break;
					
				case WEST_RIgHT:
					break;
					
			}
			break;

	}
} /** END FUNCTION intersection_eventhandler **/


// Main Function:
int main(int argc, char* argv[]) {

	return 0;
} /** END FUNCTION main **/
