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

// Ask Jeremy what these are
#define NUM_VP_X 32
#define NUM_VP_Y 32

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
	enum directions traffic_direction;
} intersection_state;

/** END STRUCTS BLOCK **/

/**********************************FUNCTION PROTOTYPES************************/

// Initialization of an intersection:
void intersection_startup(intersection_state*, tw_LP*);

// Event handler for an intersection:
void intersection_eventhandler(intersection_state*, tw_bf*, message_data*, tw_LP*);

// Reverse event handler for an intersection:
void intersection_reverse_eventhandler(intersection_state*, tw_bf*, message_data*, tw_LP*);

// Function to collection statistics for an intersection:
void intersection_statistics_collectstats(intersection_state*, tw_LP *);

// Mapping functions
tw_peid cell_mapping_lp_to_pe(tw_lpid lpid);
tw_lp* cell_mapping_to_lp(tw_lpid lpid);
tw_lpid cell_mapping_to_local_index(tw_lpid lpid);
void traffic_grid_mapping();
tw_lpid cell_compute_move(tw_lpid lpid, int direction);

/*******************************END FUNCTION PROTOTYPES***********************/

// Function roles:
tw_lptype mylps[] = {
    {
        (init_f) intersection_startup,
        (event_f) intersection_eventhandler,
        (revent_f) intersection_reverse_eventhandler,
        (final_f) intersection_statistics_collectstats,
        (map_f) cell_mapping_lp_to_pe,
        sizeof(intersection_state)
    },
    { 0 },
};

//Command Line Arguments
extern unsigned int autonomous;

const tw_optdef model_opts[] = {
    TWOPT_GROUP("Traffic Model"),
    TWOPT_UINT("autonomous", autonomous, "1 for autonomous vehicles, 0 for traditional intersections"),
    TWOPT_END(),
};

// Main Function:
int main(int argc, char* argv[]) {

    tw_init(&argc, &argv);



    tw_run();
    tw_end();

    return 0;
} /** END FUNCTION main **/

tw_peid cell_mapping_lp_to_pe(tw_lpid lpid) {
    long lp_x = lpid % MAP_WIDTH;
    long lp_y = lpid / MAP_WIDTH;
    long vp_num_x = lp_x / g_cells_per_vp_x;
    long vp_num_y = lp_y / g_cells_per_vp_y;
    long vp_num = vp_num_x + (vp_num_y * NUM_VP_X);  
    tw_peid peid = vp_num/g_vp_per_proc;  
    return peid;
}

tw_lp* cell_mapping_to_lp(tw_lpid lpid) {
    tw_lpid lp_x = lpid % MAP_WIDTH; //lpid -> (lp_x,lp_y)
    tw_lpid lp_y = lpid / MAP_WIDTH;
    tw_lpid vp_index_x = lp_x % g_cells_per_vp_x;
    tw_lpid vp_index_y = lp_y % g_cells_per_vp_y;
    tw_lpid vp_index = vp_index_x + (vp_index_y * (g_cells_per_vp_x));
    tw_lpid vp_num_x = lp_x / g_cells_per_vp_x;
    tw_lpid vp_num_y = lp_y / g_cells_per_vp_y;
    tw_lpid vp_num = vp_num_x + (vp_num_y * NUM_VP_X);
    vp_num = vp_num % g_vp_per_proc;
    tw_lpid index = vp_index + vp_num*g_cells_per_vp;

#ifdef ROSS_runtime_check  
    if (index >= g_tw_nlp)
        tw_error(TW_LOC, "index (%llu) beyond g_tw_nlp (%llu) range \n", index, g_tw_nlp);
#endif /* ROSS_runtime_check */

    return g_tw_lp[index];
}

tw_lpid cell_mapping_to_local_index(tw_lpid lpid) {
    tw_lpid lp_x = lpid % MAP_WIDTH; //lpid -> (lp_x,lp_y)
    tw_lpid lp_y = lpid / MAP_WIDTH;
    tw_lpid vp_index_x = lp_x % g_cells_per_vp_x;
    tw_lpid vp_index_y = lp_y % g_cells_per_vp_y;
    tw_lpid vp_index = vp_index_x + (vp_index_y * (g_cells_per_vp_x));
    tw_lpid vp_num_x = lp_x/g_cells_per_vp_x;
    tw_lpid vp_num_y = lp_y/g_cells_per_vp_y;
    tw_lpid vp_num = vp_num_x + (vp_num_y * NUM_VP_X);  
    vp_num = vp_num % g_vp_per_proc;
    tw_lpid index = vp_index + vp_num * g_cells_per_vp;

    if (index >= g_tw_nlp)
        tw_error(TW_LOC, "index (%llu) beyond g_tw_nlp (%llu) range \n", index, g_tw_nlp);

    return index;
}

void traffic_grid_mapping() {
    tw_lpid x, y;
    tw_lpid lpid, kpid;
    tw_lpid num_cells_per_kp, vp_per_proc;
    tw_lpid local_lp_count;

    num_cells_per_kp = (MAP_WIDTH * MAP_HEIGHT) / (NUM_VP_X * NUM_VP_Y);
    vp_per_proc = (NUM_VP_X * NUM_VP_Y) / ((tw_nnodes() * g_tw_npe)) ;
    g_tw_nlp = nlp_per_pe;
    g_tw_nkp = vp_per_proc;

    local_lp_count = 0;
    for (y = 0; y < MAP_HEIGHT; y++)
    {
        for (x = 0; x < MAP_WIDTH; x++)
        {
            lpid = (x + (y * MAP_WIDTH));
            if (g_tw_mynode == cell_mapping_lp_to_pe(lpid)) {
                kpid = local_lp_count / num_cells_per_kp;
                local_lp_count++; // MUST COME AFTER!! DO NOT PRE-INCREMENT ELSE KPID is WRONG!!

                if (kpid >= g_tw_nkp)
                    tw_error(TW_LOC, "Attempting to mapping a KPid (%llu) for Global LPid %llu that is beyond g_tw_nkp (%llu)\n",
                    kpid, lpid, g_tw_nkp );

                tw_lp_onpe(cell_mapping_to_local_index(lpid), g_tw_pe[0], lpid);
                if (g_tw_kp[kpid] == NULL)
                    tw_kp_onpe(kpid, g_tw_pe[0]);
                tw_lp_onkp(g_tw_lp[cell_mapping_to_local_index(lpid)], g_tw_kp[kpid]);
                tw_lp_settype(cell_mapping_to_local_index(lpid), &mylps[0]);
            }
        }
    }
}

// Event handler for an intersection:
void intersection_eventhandler(intersection_state* SV, tw_bf* CV, message_data* M, tw_LP* LP) {

    // Time warp starting time:
    tw_stime ts = 0.0;
    
    // Current event:
    tw_event* current_event = NULL;

    // New message data:
    message_data* new_message = NULL;

    // Unknown time warp bit field:
    *(int* ) CV = (int) 0;
    
    
    // Subtract one from the remaining time until it is 0
    
    if(SV->time_remaining == 0)
    {
        M->event_type = LIGHT_CHANGE;
    }
    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {
            
        SV->time_remaining--;
            
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
            
            ts = tw_rand_exponential(LP->rng, MEAN_SERVICE);
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (Msg_Data *)tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
            // change event to a car arriving now that light has changed
            new_message->event_type = CAR_ARRIVES;
            //printf("send ari ");
            tw_event_send(current_event);
            
            break;
            
        case CAR_ARRIVES:
            
            LP->gid = Cell_ComputeMove(LP->gid, 0);
            ts = tw_rand_exponential(LP->rng, MEAN_SERVICE);
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (Msg_Data *) tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
            new_message->event_type = CAR_ARRIVES;
            tw_event_send(current_event);
            
            break;

    }
} /** END FUNCTION intersection_eventhandler **/
