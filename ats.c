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
tw_lptype traffic_lps[] = {
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
const tw_optdef model_opts[] = {
    TWOPT_GROUP("Traffic Model"),
    TWOPT_UINT("autonomous", autonomous, "1 for autonomous vehicles, 0 for traditional intersections"),
    TWOPT_END(),
};

// Main Function:
int main(int argc, char* argv[]) {
    // QUESTION: I don't know what this is or why it is set so low
    g_tw_ts_end = 30; // ROSS default is 100000.0
    g_tw_gvt_interval = 16; // ROSS default is 16

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
        tw_lp_settype(i, &traffic_lps[0]);

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
		new_message->car.x_to_go_original = new_message->car.x_to_go;
		new_message->car.y_to_go_original = new_message->car.y_to_go;
        new_message->car.start_time = tw_clock_now(LP->pe);
		new_message->car.has_turned_yet = 0;
		
        tw_event_send(current_event);
    }
}

// Event handler for an intersection:
void intersection_eventhandler(intersection_state* SV, tw_bf* CV, message_data* M, tw_lp* LP) {

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
            
        //SV->time_remaining--;
            
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
            
			/*
			int i;
			for(i=0; i<SV->time_remaining; i++) {
				
			}*/

			// If the traffic direction is NORTH-SOUTH:
			if(SV->traffic_direction == NORTH_SOUTH) {
				
				// Check if the NORTH-SOUTH direction has an arrow:
				if(SV->has_green_arrow) {
					
					// Check if the NORTH-SOUTH left turns are green:
					if(SV->north_lanes[0].light == GREEN) {
						
						// Only select cars from the left lane in the NORTH-SOUTH:
						int i;
						int numberOfCarsDeployed = 0;

						// North lane - RELEASE ALL 30 CARS:
						for(i = 0; i < 30; i++) {
							
							// If we hit the number of cars in this lane, break:
							if(i >= SV->north_lanes[0].number_of_cars) {
								break;
							}

							// Schedule a CAR_ARRIVES event for each car's next intersection:
							ts = tw_rand_exponential(LP->rng, g_mean_service);
							current_event = tw_event_new(SV->north_lanes[0].cars[i].next_intersection, ts, LP);
							new_message = (message_data*) tw_event_data(current_event);
							new_message->car.x_to_go = M->car.x_to_go;
            				new_message->car.y_to_go = M->car.y_to_go;
            				new_message->car.start_time = M->car.start_time;
            				new_message->car.end_time = M->car.end_time;
							new_message->car.has_turned_yet = M->car.has_turned_yet;
            				// change event to a car arriving now that light has changed
            				new_message->event_type = CAR_ARRIVES;
            				tw_event_send(current_event);

							numberOfCarsDeployed++;
						} 

						// Decrement the number of cars in the lane:
						SV->north_lanes[0].number_of_cars -= numberOfCarsDeployed;
					}
				}
			}
			
			// WRONG!!!!
            ts = tw_rand_exponential(LP->rng, g_mean_service);
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (message_data*)tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
			new_message->car.has_turned_yet = M->car.has_turned_yet;
            // change event to a car arriving now that light has changed
            new_message->event_type = CAR_ARRIVES;
            //printf("send ari ");
            tw_event_send(current_event);
            // END WRONG

            break;
            
        case CAR_ARRIVES:
            
			SV->total_cars_arrived++;
			// follows the y path first
			if(M->car.y_to_go == 0 && M->car.x_to_go == 0) {
				M->car.end_time = tw_clock_now(LP->pe);
				SV->total_cars_finished++;
				g_total_time += (M->car.end_time - M->car.start_time);
				printf("Car finished with x: %d and y:%d with time: %d\n", M->car.x_to_go_original,
					   M->car.y_to_go_original, (M->car.end_time - M->car.start_time));
				break;
			}
			if(M->car.y_to_go > 0) {
				SV->south_lanes[1].cars[SV->south_lanes[1].number_of_cars] = M->car;
				SV->south_lanes[1].number_of_cars++;
				//M->car.y_to_go--;
				//LP->gid = cell_compute_move(LP->gid, NORTH);
				
				// Calculate the next intersection in the NORTH direction:
				M->car.next_intersection = cell_compute_move(LP->gid, NORTH);
			}
			else if(M->car.y_to_go < 0) {
				SV->north_lanes[1].cars[SV->north_lanes[1].number_of_cars] = M->car;
				SV->north_lanes[1].number_of_cars++;
				//M->car.y_to_go++;
				//LP->gid = cell_compute_move(LP->gid, SOUTH);

				// Calculate the next intersection in the SOUTH direction:
				M->car.next_intersection = cell_compute_move(LP->gid, SOUTH);
			}
			else if(M->car.y_to_go == 0) {
				if(M->car.has_turned_yet) {
					if(M->car.x_to_go > 0) {
						SV->west_lanes[1].cars[SV->west_lanes[1].number_of_cars] = M->car;
						SV->west_lanes[1].number_of_cars++;

						// Calculate the next intersection in the EAST direction:
						M->car.next_intersection = cell_compute_move(LP->gid, EAST);
					}
					else if(M->car.x_to_go < 0) {
						SV->east_lanes[1].cars[SV->east_lanes[1].number_of_cars] = M->car;
						SV->east_lanes[1].number_of_cars++;
					
						// Calculate the next intersection in the WEST direction:
						M->car.next_intersection = cell_compute_move(LP->gid, WEST);
					}
				}
				else if(M->car.y_to_go_original > 0) {
					M->car.has_turned_yet = 1;
					SV->south_lanes[2].cars[SV->south_lanes[2].number_of_cars] = M->car;
					SV->south_lanes[2].number_of_cars++;

					// Calculate the next intersection in the NORTH direction:
					M->car.next_intersection = cell_compute_move(LP->gid, NORTH);
				}
				else if(M->car.y_to_go_original < 0) {
					M->car.has_turned_yet = 1;
					SV->north_lanes[0].cars[SV->north_lanes[0].number_of_cars] = M->car;
					SV->north_lanes[0].number_of_cars++;

					// Calculate the next intersection in the SOUTH direction:
					M->car.next_intersection = cell_compute_move(LP->gid, SOUTH);
				}
				
			}
            break;

    }
} /** END FUNCTION intersection_eventhandler **/

// Reverse Intersection Event Handler that is called when a Time Warp is initiated:
void intersection_reverse_eventhandler(intersection_state* SV, tw_bf* CV, message_data* M, tw_lp* LP) {

    // Unknown time warp bit field:
    *(int*) CV = (int) 0;

    // If the time_remaining is 0, execute a light change:
    if(SV->time_remaining == 0) {
        M->event_type = LIGHT_CHANGE;
    }

    // Handle the events defined in the "events" enumeration, but in reverse:
    switch(M->event_type) {

        // Increment the amount of time remaining:
        SV->time_remaining++;

        // Handle the LIGHT_CHANGE Event in reverse:
        case LIGHT_CHANGE:

            // Check if the traffic is permitted north-south:
            if(SV->traffic_direction == NORTH_SOUTH) {

                // Check if there are green arrows:
                if(SV->has_green_arrow) {
                    // Check if this left-turn is GREEN:
                    if(SV->north_lanes[0].light == GREEN) {

                        /* 
                         * Turn the NORTH/SOUTH left-turn arrow to RED; 
                         * switch all EAST-WEST lights from red to green:
                         */
                         SV->north_lanes[0].light = SV->south_lanes[0].light = RED;

                         // Turn the EAST-WEST straight lights from red to green:
                         int i;
                         for(i = 1; i < SV->number_of_west_lanes; i++) {
                            SV->west_lanes[i].light = GREEN;
                         }

                         for(i = 1; i < SV->number_of_east_lanes; i++) {
                            SV->east_lanes[i].light = GREEN;
                         }

                         // Change the permitted direction to EAST-WEST:
                         SV->traffic_direction = EAST_WEST;
                         
                         // Reset the traffic light timer:
						SV->time_remaining = SV->total_time;
                    } else {
                        /*
                         * The left turn is RED; turn off the NORTH-SOUTH straight lights
                         * and turn on the left-turn light GREEN:
                         */

                        // Turn on the left-turn lights to green:
                         SV->north_lanes[0].light = SV->south_lanes[0].light = GREEN;

                         // Turn off the NORTH-SOUTH straight lights to red:
                         int i;

                         // North Lanes:
                         for(i = 1; i < SV->number_of_north_lanes; i++) {
                            SV->north_lanes[i].light = RED;
                         }

                         // South Lanes:
                         for(i = 1; i < SV->number_of_south_lanes; i++) {
                            SV->south_lanes[i].light = RED;
                         }

                         // Reset the total time to the left total time:
                         SV->time_remaining = SV->left_total_time;

                    }
                } else {
                    /*
                     * This intersection does not have any green left arrows; just
                     * change directions.
                     */

                     // Change permitted direction to EAST-WEST:
                     SV->traffic_direction = EAST_WEST;

                     int i;

                     // Turn off the NORTH-SOUTH lanes:

                     // Northbound Lights:
                     for(i = 0; i < SV->number_of_north_lanes; i++) {
                        SV->north_lanes[i].light = RED;
                     }

                     // Southbound Lights:
                     for(i = 0; i < SV->number_of_south_lanes; i++) {
                        SV->south_lanes[i].light = RED;
                     }

                     // Turn on the EAST-WEST lanes:

                     // Eastbound Lights:
                     for(i = 0; i < SV->number_of_east_lanes; i++) {
                        SV->east_lanes[i].light = GREEN;
                     }

                     // Westbound Lights:
                     for(i = 0; i < SV->number_of_west_lanes; i++) {
                        SV->west_lanes[i].light = GREEN;
                     }

                     // Reset the time to the total time:
                     SV->time_remaining = SV->total_time;
                }
            } else if(SV->traffic_direction == EAST_WEST) {
                // Traffic is moving EAST-WEST:

                // Check if East-West bound lanes have a left-turn arrow:
                if(SV->has_green_arrow) {
                    // Check if this left-turn is GREEN:
                    if(SV->east_lanes[0].light == GREEN) {

                        /*
                         * This light is green; turn the left-turn arrow to RED
                         * and turn on the NORTH-SOUTH lanes to GREEN:
                         */
                         SV->east_lanes[0].light = SV->west_lanes[0].light = RED;

                         // Switch directions to NORTH-SOUTH:
                         SV->traffic_direction = NORTH_SOUTH;

                         // Turn on the NORTH-SOUTH lanes to GREEN:
                         int i;

                         for(i = 1; i < SV->number_of_north_lanes; i++) {
                            SV->north_lanes[i].light = GREEN;
                         }

                         for(i = 1; i < SV->number_of_south_lanes; i++) {
                            SV->south_lanes[i].light = GREEN;
                         }

                         // Reset the total time:
                         SV->time_remaining = SV->total_time;
                    } else {
                        /*
                         * The left turn is RED; turn off EAST-WEST straight lanes 
                         * and turn on the EAST-WEST left turn arrows:
                         */
                         SV->east_lanes[0].light = SV->west_lanes[0].light = GREEN;

                         int i;

                         // East Lanes:
                         for(i = 1; i < SV->number_of_east_lanes; i++) {
                            SV->east_lanes[0].light = RED;
                         }

                         // West Lanes:
                         for(i = 1; i < SV->number_of_west_lanes; i++) {
                            SV->west_lanes[0].light = RED;
                         }

                         // Reset the total time left to the left total time:
                         SV->time_remaining = SV->left_total_time;
                    }
                } else {
                    /*
                     * This intersection does not have any green left-arrows; just
                     * change directions to NORTH-SOUTH and turn off all EAST-WEST lights:
                     */
                     SV->traffic_direction = NORTH_SOUTH;

                     int i;

                     // East and West Lanes:
                     for(i = 0; i < SV->number_of_east_lanes; i++) {
                        SV->east_lanes[i].light = RED;
                     }
                     for(i = 0; i < SV->number_of_west_lanes; i++) {
                        SV->west_lanes[i].light = RED;
                     }

                     // Turn on all lights North and South:
                     for(i = 0; i < SV->number_of_north_lanes; i++) {
                        SV->north_lanes[i].light = GREEN;
                     }
                     for(i = 0; i < SV->number_of_south_lanes; i++) {
                        SV->south_lanes[i].light = GREEN;
                     }

                     // Reset the total amount of time:
                     SV->time_remaining = SV->total_time;
                }
            }

            // Reverse the event:
            tw_rand_reverse_unif(LP->rng);
            break;

        case CAR_ARRIVES:
            // Handle the case when a car arrives:
			
			SV->total_cars_arrived--;
            // Reverse follow the y path first:
            if(M->car.y_to_go > 0) {
                M->car.y_to_go++;
                LP->gid = cell_compute_move(LP->gid, SOUTH);
            } else if(M->car.y_to_go < 0) {
                M->car.y_to_go--;
                LP->gid = cell_compute_move(LP->gid, NORTH);
            }

            // Reverse once y is 0, follows x path:
            else if(M->car.x_to_go > 0) {
                M->car.x_to_go++;
                LP->gid = cell_compute_move(LP->gid, WEST);
            } else if(M->car.x_to_go < 0) {
                M->car.x_to_go--;
                LP->gid = cell_compute_move(LP->gid, EAST);
            }
			else {
				M->car.end_time = tw_clock_now(LP->pe);
				SV->total_cars_finished--;
				break;
			}
			
            // Reverse the event:
            tw_rand_reverse_unif(LP->rng);

            break;
    }

} /** END FUNCTION intersection_reverse_eventhandler **/

void intersection_statistics_collectstats(intersection_state* SV, tw_lp* LP) {
	g_total_cars += SV->total_cars_arrived;
	g_cars_finished += SV->total_cars_finished;
	g_average_time = g_total_time/g_cars_finished;
}
