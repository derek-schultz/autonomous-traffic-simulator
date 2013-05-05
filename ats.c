/*
 * ats.c - Autonomous Traffic Simulator - Parallel Programming Group Project
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

#include "ats.h"

// Function roles:
tw_lptype traffic_light_lps[] = {
    {
        (init_f) intersection_startup,
        (event_f) traffic_light_intersection_eventhandler,
        (revent_f) intersection_reverse_eventhandler,
        (final_f) intersection_statistics_collectstats,
        (map_f) cell_mapping_lp_to_pe,
        sizeof(intersection_state)
    },
    { 0 },
};

tw_lptype autonomous_traffic_lps[] = {
    {
        (init_f) intersection_startup,
        (event_f) autonomous_traffic_intersection_eventhandler,
        (revent_f) intersection_reverse_eventhandler,
        (final_f) intersection_statistics_collectstats,
        (map_f) cell_mapping_lp_to_pe,
        sizeof(intersection_state)
    },
    { 0 },
};


//Command Line Arguments
const tw_optdef model_opts[] = {
    TWOPT_GROUP("Traffic Model"),
    TWOPT_UINT("autonomous", autonomous, "1 for autonomous vehicles, 0 for traditional intersections"),
    TWOPT_END(),
};

// Main Function:
int main(int argc, char* argv[]) {
    // QUESTION: I don't know what this is or why it is set so low
    g_tw_ts_end = 30; // ROSS default is 100000.0
    g_tw_gvt_interval = 512; // ROSS default is 16
    g_tw_mblock = 8; // ROSS default to 16
    
    // QUESTION: Jeremy says this is necessary if compiled with MEMORY queues
    g_tw_memory_nqueues = 1;

    tw_opt_add(model_opts);
    tw_init(&argc, &argv);

    if (g_lookahead > 1.0)
        tw_error(TW_LOC, "Lookahead must be less than 1.0\n"); // QUESTION: why?

    // Reset mean based on lookahead. QUESTION: why?
    g_mean = g_mean - g_lookahead;

    // QUESTION: this was 1 before...
    g_tw_memory_nqueues = 16; // give at least 16 memory queue event

    // Set lookahead
    g_tw_lookahead = g_lookahead;

    // TODO: clean this up?
    g_nlp_per_pe = (MAP_WIDTH * MAP_HEIGHT) / (tw_nnodes() * g_tw_npe);
    g_tw_events_per_pe = (g_mult * g_nlp_per_pe * g_traffic_start_events) + g_optimistic_memory;
    num_cells_per_kp = (MAP_WIDTH * MAP_HEIGHT) / (NUM_VP_X * NUM_VP_Y);
    vp_per_proc = (NUM_VP_X * NUM_VP_Y) / ((tw_nnodes() * g_tw_npe)) ;
    g_vp_per_proc = vp_per_proc;
    g_tw_nlp = g_nlp_per_pe;
    g_tw_nkp = vp_per_proc;

    g_tw_mapping = CUSTOM;
    g_tw_custom_initial_mapping = &traffic_grid_mapping;
    g_tw_custom_lp_global_to_local_map = &cell_mapping_to_lp;

    tw_define_lps(g_nlp_per_pe, sizeof(message_data), 0);

    int i;
    for(i = 0; i < g_tw_nlp; i++)
    {
        if (autonomous)
            tw_lp_settype(i, &autonomous_traffic_lps[0]);
        else
            tw_lp_settype(i, &traffic_light_lps[0]);
    }

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
    tw_peid peid = vp_num / g_vp_per_proc;
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
    vp_per_proc = (NUM_VP_X * NUM_VP_Y) / ((tw_nnodes() * g_tw_npe));
    g_tw_nlp = g_nlp_per_pe;
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
                tw_lp_settype(cell_mapping_to_local_index(lpid), &traffic_lps[0]);
            }
        }
    }
}

tw_lpid cell_compute_move(tw_lpid lpid, int direction)
{
    tw_lpid lpid_x, lpid_y;
    tw_lpid n_x, n_y;
    tw_lpid dest_lpid;

    lpid_y = lpid / MAP_WIDTH;
    lpid_x = lpid - (lpid_y * MAP_WIDTH);

    switch(direction)
    {
    case WEST:
        // X-going west
        n_x = ((lpid_x - 1) + MAP_WIDTH) % MAP_WIDTH;
        n_y = lpid_y;
        break;

    case EAST:
        // X-going east
        n_x = (lpid_x + 1) % MAP_WIDTH;
        n_y = lpid_y;
        break;

    case SOUTH:
        // Y-going south
        n_x = lpid_x;
        n_y = ((lpid_y - 1) + MAP_HEIGHT) % MAP_HEIGHT;
        break;

    case NORTH:
        // Y-going north
        n_x = lpid_x;
        n_y = (lpid_y + 1) % MAP_HEIGHT;
        break;

    default:
        tw_error(TW_LOC, "Bad direction value \n");
    }

    dest_lpid = (tw_lpid) (n_x + (n_y * MAP_WIDTH));
    // printf("ComputeMove: Src LP %llu (%d, %d), Dir %u, Dest LP %llu (%d, %d)\n", lpid, lpid_x, lpid_y, direction, dest_lpid, n_x, n_y);
    return dest_lpid;
}

void intersection_startup(intersection_state* SV, tw_lp* LP) {
    tw_stime ts;
    tw_event* current_event;
    message_data* new_message;

    int i;
    for(i = 0; i < g_traffic_start_events; i++) {
        // Arrival time
        ts = tw_rand_exponential(LP->rng, g_mean_service);

        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);

        new_message->event_type = CAR_ARRIVES;
        new_message->car.x_to_go = rand() % 200 - 99; // TODO: what is this?
        new_message->car.y_to_go = rand() % 200 - 99;
        new_message->car.start_time = 0;

        tw_event_send(current_event);
    }
}

// Event handler for an intersection:
void traffic_light_intersection_eventhandler(intersection_state* SV, tw_bf* CV, message_data* M, tw_lp* LP) {

    // Time warp starting time:
    tw_stime ts = 0.0;
    
    // Current event:
    tw_event* current_event = NULL;

    // New message data:
    message_data* new_message = NULL;

    // Unknown time warp bit field:
    *(int*) CV = (int) 0;
    
    
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
                        for(i = 1; i < SV->number_of_north_lanes; i++) {
                            SV->north_lanes[i].light = GREEN;
                        } 

                        for(i = 1; i < SV->number_of_south_lanes; i++) {
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
                    for(i = 0; i < SV->number_of_east_lanes; i++) {
                        SV->east_lanes[i].light = GREEN;
                    } 

                    for(i = 1; i < SV->number_of_west_lanes; i++) {
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
                        for(i = 1; i < SV->number_of_east_lanes; i++) {
                            SV->east_lanes[i].light = GREEN;
                        }
                        
                        for(i = 1; i < SV->number_of_west_lanes; i++) {
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
                        for(i = 0; i < SV->number_of_east_lanes; i++) {
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
                    for(i = 0; i < SV->number_of_north_lanes; i++) {
                        SV->north_lanes[i].light = GREEN;
                    }
                    
                    for(i = 1; i < SV->number_of_south_lanes; i++) {
                        SV->south_lanes[i].light = GREEN;
                    }
                    SV->time_remaining = SV->total_time;
                }
            }
            
            ts = tw_rand_exponential(LP->rng, g_mean_service);
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (message_data*)tw_event_data(current_event);
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
            
			// follows the y path first
			if(M->car.y_to_go > 0) {
				M->car.y_to_go--;
				LP->gid = cell_compute_move(LP->gid, NORTH);
			}
			else if(M->car.y_to_go < 0) {
				M->car.y_to_go++;
				LP->gid = cell_compute_move(LP->gid, SOUTH);
			}
			// once y is 0, follows x path
			else if(M->car.x_to_go > 0) {
				M->car.x_to_go--;
				LP->gid = cell_compute_move(LP->gid, EAST);
			}
			else if(M->car.x_to_go < 0) {
				M->car.x_to_go++;
				LP->gid = cell_compute_move(LP->gid, WEST);
			}

            ts = tw_rand_exponential(LP->rng, g_mean_service);
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (message_data*) tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
            new_message->event_type = CAR_ARRIVES;
            tw_event_send(current_event);
            
            break;

    }
} /** END FUNCTION intersection_eventhandler **/

/*
 * Determines whether cars a and b will colide if they travel through the
 * intersection at the same time.
 * NL indicates that the car is coming from the north and turning left
 * ES indicates that the car is coming from the east and going stratight
 * etc...
 */
int will_collide(travel_directions a, travel_directions b) {
    if (a == NL && (b == SS || b == SR || b == ES || b == EL || b == WS || b == WL))
        return 1;
    if (a == NR && (b == SL || b == ES))
        return 1;
    if (a == NS && (b == SL || b == ES || b == EL || b == WS || b == WR || b == WL))
        return 1;
    if (a == EL && (b == SS || b == SL || b == NS || b == NL || b == WR || b == WS))
        return 1;
    if (a == ER && (b == SS || b == WL))
        return 1;
    if (a == ES && (b == SS || b == SL || b == NL || b == NR || b == NS || b == WL))
        return 1;
    if (a == SL && (b == ES || b == EL || b == NR || b == NS || b == WL || b == WS))
        return 1;
    if (a == SR && (b == WS || b == NL))
        return 1;
    if (a == SS && (b == NL || b == ES || b == EL || b == ER || b == WS || b == WL))
        return 1;
    if (a == WL && (b == NS || b == NL || b == SS || b == SL || b == ES || b == ER))
        return 1;
    if (a == WR && (b == EL || b == NS))
        return 1;
    if (a == WS && (b == SS || b == SL || b == SR || b == EL || b == NS || b == NL))
        return 1;
    return 0;
}

void autonomous_traffic_intersection_eventhandler(intersection_state* SV, tw_bf* CV, message_data* M, tw_lp* LP) {
    // Time warp starting time:
    tw_stime ts = 0.0;
    
    // Current event:
    tw_event* current_event = NULL;

    // New message data:
    message_data* new_message = NULL;

    // Unknown time warp bit field:
    *(int*) CV = (int) 0;
    
    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {

    case CARS_GO:
        int i;
        car_type cars[4 * MAX_LANES_PER_DIRECTION][4 * MAX_LANES_PER_DIRECTION];
        for (i = 0; i < MAX_LANES_PER_DIRECTION; i++) {
            SV->north_lanes[i]
        }
                    
    case CAR_ARRIVES:
            
        // follows the y path first
        if(M->car.y_to_go > 0) {
            M->car.y_to_go--;
            LP->gid = cell_compute_move(LP->gid, NORTH);
        }
        else if(M->car.y_to_go < 0) {
            M->car.y_to_go++;
            LP->gid = cell_compute_move(LP->gid, SOUTH);
        }
        // once y is 0, follows x path
        else if(M->car.x_to_go > 0) {
            M->car.x_to_go--;
            LP->gid = cell_compute_move(LP->gid, EAST);
        }
        else if(M->car.x_to_go < 0) {
            M->car.x_to_go++;
            LP->gid = cell_compute_move(LP->gid, WEST);
        }

        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*) tw_event_data(current_event);
        new_message->car.x_to_go = M->car.x_to_go;
        new_message->car.y_to_go = M->car.y_to_go;
        new_message->car.start_time = M->car.start_time;
        new_message->car.end_time = M->car.end_time;
        new_message->event_type = CAR_ARRIVES;
        tw_event_send(current_event);
        
        break;
    }
}
