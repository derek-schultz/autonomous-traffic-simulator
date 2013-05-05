#include "ats.h"

void traffic_light_intersection_startup(intersection_state* SV, tw_lp* LP) {
    tw_stime ts;
    tw_event* current_event;
    message_data* new_message;

    // Initialize the number of cars arriving into the intersection:
    SV->num_cars_in_south = 0;
    SV->num_cars_in_west = 0;
    SV->num_cars_in_north = 0;
    SV->num_cars_in_east = 0;

    // Initialize the number of cars leaving the intersection
    SV->num_cars_out_south = 0;
    SV->num_cars_out_west = 0;
    SV->num_cars_out_north = 0;
    SV->num_cars_out_east = 0;

    // Initialize the total number of cars arrived:
    SV->total_cars_arrived = 0;
    SV->total_cars_finished = 0;

    // Initialize total_time and remaining_time:
    SV->total_time = SV->time_remaining = 10;
    
    int i;
    for(i = 0; i < g_traffic_start_events; i++) {
        // Arrival time
        ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;

        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);

        new_message->event_type = CAR_ARRIVES;
        new_message->car.x_to_go = rand() % 200 - 99; // TODO: what is this?
        new_message->car.y_to_go = rand() % 200 - 99;
        new_message->car.x_to_go_original = new_message->car.x_to_go;
        new_message->car.y_to_go_original = new_message->car.y_to_go;
        new_message->car.start_time = tw_clock_now(LP->pe);
        //new_message->car.has_turned_yet = 0;
        
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
    
    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {
            
        // Handle the LIGHT_CHANGE event IF AND ONLY IF the time remaining on this intersection == 0:
        case LIGHT_CHANGE:
            
            // TIME EXPIRED!
        
            // Check if the traffic is permitted north-south (green on north and south lights):
            if(SV->traffic_direction == NORTH_SOUTH) {
                
                // Switch permitted traffic to EAST_WEST:
                SV->traffic_direction = EAST_WEST;
            } else {

                // Traffic was permitted east-west; switch permitted traffic to NORTH_SOUTH:
                SV->traffic_direction = NORTH_SOUTH;
            }

            // Reset the total time on the light to the original time:
            SV->time_remaining = SV->total_time;
            break;
            
        // Handle the case where the car arrives at an intersection:
        case CAR_ARRIVES:

            // Car reached its destination:
            if(M->car.y_to_go == 0 && M->car.x_to_go == 0) {
                M->car.end_time = tw_clock_now(LP->pe);
                SV->total_cars_finished++;
                g_total_time += (M->car.end_time - M->car.start_time);
                printf("Car finished with x: %d and y:%d with time: %d\n", M->car.x_to_go_original,
                       M->car.y_to_go_original, (M->car.end_time - M->car.start_time));
                break;
            }
       
            // Increment the total number of cars in this intersection:
            SV->total_cars_arrived++;

            // follows the y path first

            // The car is too far south; have the car head up north:
            if(M->car.y_to_go > 0) {

                // Add a car in the south lane:
                SV->num_cars_in_south++;                

                // Calculate the next intersection in the NORTH direction:
                M->car.next_intersection = cell_compute_move(LP->gid, NORTH);

                // Decrement the distance to travel up north:
                M->car.y_to_go--;
            }
            else if(M->car.y_to_go < 0) {

                // Add a car in the north lane:
                SV->num_cars_in_north++;

                // Calculate the next intersection in the SOUTH direction:
                M->car.next_intersection = cell_compute_move(LP->gid, SOUTH);

                // Decrement the distance to travel down south:
                M->car.y_to_go++;
            }
            else if(M->car.x_to_go > 0) {

                // Add a car in the west lane:
                SV->num_cars_in_west++;

                // Calculate the next intersection in the EAST direction:
                M->car.next_intersection = cell_compute_move(LP->gid, EAST);

                // Decrement the distance to travel east:
                M->car.x_to_go--;
            }
            else if(M->car.x_to_go < 0) {

                // Add a car in the east lane:
                SV->num_cars_in_east++;
                    
                // Calculate the next intersection in the WEST direction:
                M->car.next_intersection = cell_compute_move(LP->gid, WEST);
                
                // Decrement the distance to travel west:
                M->car.x_to_go++;
            }

            // Schedule a departure event:
            ts = tw_rand_exponential(LP->rng, g_mean_service);
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (message_data *) tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.x_to_go_original = M->car.x_to_go_original;
            new_message->car.y_to_go_original = M->car.y_to_go_original;
            new_message->car.next_intersection = M->car.next_intersection;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
            new_message->event_type = CAR_DEPARTS;
            tw_event_send(current_event);

            break;

        // Handle the case in which a car departs a traffic light:
        case CAR_DEPARTS:
            
            SV->time_remaining--;
            
            // going in the north direction
            if(M->car.y_to_go > 0) {
                if(SV->traffic_direction == NORTH_SOUTH) {
                    SV->num_cars_out_north++;
                    ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                    current_event = tw_event_new(LP->gid, ts, LP);
                    new_message = (message_data *) tw_event_data(current_event);
                    new_message->car.x_to_go = M->car.x_to_go;
                    new_message->car.y_to_go = M->car.y_to_go;
                    new_message->car.x_to_go_original = M->car.x_to_go_original;
                    new_message->car.y_to_go_original = M->car.y_to_go_original;
                    new_message->car.next_intersection = M->car.next_intersection;
                    new_message->car.start_time = M->car.start_time;
                    new_message->car.end_time = M->car.end_time;
                    new_message->event_type = CAR_ARRIVES;
                    tw_event_send(current_event);
                }
            }
            // going in the south direction
            else if(M->car.y_to_go < 0) {
                if(SV->traffic_direction == NORTH_SOUTH) {
                    SV->num_cars_out_south++;
                    ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                    current_event = tw_event_new(LP->gid, ts, LP);
                    new_message = (message_data *) tw_event_data(current_event);
                    new_message->car.x_to_go = M->car.x_to_go;
                    new_message->car.y_to_go = M->car.y_to_go;
                    new_message->car.x_to_go_original = M->car.x_to_go_original;
                    new_message->car.y_to_go_original = M->car.y_to_go_original;
                    new_message->car.next_intersection = M->car.next_intersection;
                    new_message->car.start_time = M->car.start_time;
                    new_message->car.end_time = M->car.end_time;
                    new_message->event_type = CAR_ARRIVES;
                    tw_event_send(current_event);
                }
            // needs to turn now
            } else if(M->car.y_to_go == 0 && M->car.x_to_go == M->car.x_to_go_original) {
                if(SV->traffic_direction == NORTH_SOUTH) {
                    if(M->car.y_to_go_original > 0 && M->car.x_to_go > 0) {
                        SV->num_cars_out_east++;
                        ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                        current_event = tw_event_new(LP->gid, ts, LP);
                        new_message = (message_data *) tw_event_data(current_event);
                        new_message->car.x_to_go = M->car.x_to_go;
                        new_message->car.y_to_go = M->car.y_to_go;
                        new_message->car.x_to_go_original = M->car.x_to_go_original;
                        new_message->car.y_to_go_original = M->car.y_to_go_original;
                        new_message->car.next_intersection = M->car.next_intersection;
                        new_message->car.start_time = M->car.start_time;
                        new_message->car.end_time = M->car.end_time;
                        new_message->event_type = CAR_ARRIVES;
                        tw_event_send(current_event);
                    }
                    else if(M->car.y_to_go_original > 0 && M->car.x_to_go < 0) {
                        if(SV->num_cars_in_north == 0) {
                            SV->num_cars_out_west++;
                            ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                            current_event = tw_event_new(LP->gid, ts, LP);
                            new_message = (message_data *) tw_event_data(current_event);
                            new_message->car.x_to_go = M->car.x_to_go;
                            new_message->car.y_to_go = M->car.y_to_go;
                            new_message->car.x_to_go_original = M->car.x_to_go_original;
                            new_message->car.y_to_go_original = M->car.y_to_go_original;
                            new_message->car.next_intersection = M->car.next_intersection;
                            new_message->car.start_time = M->car.start_time;
                            new_message->car.end_time = M->car.end_time;
                            new_message->event_type = CAR_ARRIVES;
                            tw_event_send(current_event);
                        } else {
                            ts = 0;
                            current_event = tw_event_new(LP->gid, ts, LP);
                            new_message = (message_data *) tw_event_data(current_event);
                            new_message->car.x_to_go = M->car.x_to_go;
                            new_message->car.y_to_go = M->car.y_to_go;
                            new_message->car.x_to_go_original = M->car.x_to_go_original;
                            new_message->car.y_to_go_original = M->car.y_to_go_original;
                            new_message->car.next_intersection = M->car.next_intersection;
                            new_message->car.start_time = M->car.start_time;
                            new_message->car.end_time = M->car.end_time;
                            new_message->event_type = CAR_DEPARTS;
                        }
                    }
                    else if(M->car.y_to_go_original < 0 && M->car.x_to_go > 0) {
                        if(SV->num_cars_in_south == 0) {
                            SV->num_cars_out_east++;
                            ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                            current_event = tw_event_new(LP->gid, ts, LP);
                            new_message = (message_data *) tw_event_data(current_event);
                            new_message->car.x_to_go = M->car.x_to_go;
                            new_message->car.y_to_go = M->car.y_to_go;
                            new_message->car.x_to_go_original = M->car.x_to_go_original;
                            new_message->car.y_to_go_original = M->car.y_to_go_original;
                            new_message->car.next_intersection = M->car.next_intersection;
                            new_message->car.start_time = M->car.start_time;
                            new_message->car.end_time = M->car.end_time;
                            new_message->event_type = CAR_ARRIVES;
                            tw_event_send(current_event);
                        } else {
                            ts = 0;
                            current_event = tw_event_new(LP->gid, ts, LP);
                            new_message = (message_data *) tw_event_data(current_event);
                            new_message->car.x_to_go = M->car.x_to_go;
                            new_message->car.y_to_go = M->car.y_to_go;
                            new_message->car.x_to_go_original = M->car.x_to_go_original;
                            new_message->car.y_to_go_original = M->car.y_to_go_original;
                            new_message->car.next_intersection = M->car.next_intersection;
                            new_message->car.start_time = M->car.start_time;
                            new_message->car.end_time = M->car.end_time;
                            new_message->event_type = CAR_DEPARTS;
                        }
                    }
                    else if(M->car.y_to_go_original < 0 && M->car.x_to_go < 0) {
                        SV->num_cars_out_west++;
                        ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                        current_event = tw_event_new(LP->gid, ts, LP);
                        new_message = (message_data *) tw_event_data(current_event);
                        new_message->car.x_to_go = M->car.x_to_go;
                        new_message->car.y_to_go = M->car.y_to_go;
                        new_message->car.x_to_go_original = M->car.x_to_go_original;
                        new_message->car.y_to_go_original = M->car.y_to_go_original;
                        new_message->car.next_intersection = M->car.next_intersection;
                        new_message->car.start_time = M->car.start_time;
                        new_message->car.end_time = M->car.end_time;
                        new_message->event_type = CAR_ARRIVES;
                        tw_event_send(current_event);
                    }
                }
                else if(M->car.x_to_go > 0) {
                    if(SV->traffic_direction == EAST_WEST) {
                        SV->num_cars_out_east++;
                        ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                        current_event = tw_event_new(LP->gid, ts, LP);
                        new_message = (message_data *) tw_event_data(current_event);
                        new_message->car.x_to_go = M->car.x_to_go;
                        new_message->car.y_to_go = M->car.y_to_go;
                        new_message->car.x_to_go_original = M->car.x_to_go_original;
                        new_message->car.y_to_go_original = M->car.y_to_go_original;
                        new_message->car.next_intersection = M->car.next_intersection;
                        new_message->car.start_time = M->car.start_time;
                        new_message->car.end_time = M->car.end_time;
                        new_message->event_type = CAR_ARRIVES;
                        tw_event_send(current_event);
                    }
                }
                else if(M->car.x_to_go < 0) {
                    if(SV->traffic_direction == EAST_WEST) {
                        SV->num_cars_out_west++;
                        ts = tw_rand_exponential(LP->rng, g_mean_service) + 20;
                        current_event = tw_event_new(LP->gid, ts, LP);
                        new_message = (message_data *) tw_event_data(current_event);
                        new_message->car.x_to_go = M->car.x_to_go;
                        new_message->car.y_to_go = M->car.y_to_go;
                        new_message->car.x_to_go_original = M->car.x_to_go_original;
                        new_message->car.y_to_go_original = M->car.y_to_go_original;
                        new_message->car.next_intersection = M->car.next_intersection;
                        new_message->car.start_time = M->car.start_time;
                        new_message->car.end_time = M->car.end_time;
                        new_message->event_type = CAR_ARRIVES;
                        tw_event_send(current_event);
                    }
                }
            }
            
            if(SV->time_remaining == 0) {
                ts = 0;
                current_event = tw_event_new(LP->gid, ts, LP);
                new_message = (message_data *) tw_event_data(current_event);
                new_message->event_type = LIGHT_CHANGE;
            }
            
            break;

    }
} /** END FUNCTION intersection_eventhandler **/

// Reverse Intersection Event Handler that is called when a Time Warp is initiated:
void traffic_light_intersection_reverse_eventhandler(
        intersection_state* SV,
        tw_bf* CV,
        message_data* M,
        tw_lp* LP) {

    // Unknown time warp bit field:
    *(int*) CV = (int) 0;

    // Handle the events defined in the "events" enumeration, but in reverse:
    switch(M->event_type) {
            
            // Handle the LIGHT_CHANGE event IF AND ONLY IF the time remaining on this intersection == 0:
        case LIGHT_CHANGE:
            
            // TIME EXPIRED!
            
            // Check if the traffic is permitted north-south (green on north and south lights):
            if(SV->traffic_direction == NORTH_SOUTH) {
                
                // Switch permitted traffic to EAST_WEST:
                SV->traffic_direction = EAST_WEST;
            } else {
                
                // Traffic was permitted east-west; switch permitted traffic to NORTH_SOUTH:
                SV->traffic_direction = NORTH_SOUTH;
            }
            
            // Reset the total time on the light to the original time:
            SV->time_remaining = SV->total_time;
            break;
        
        case CAR_ARRIVES:
            
            // Increment the total number of cars in this intersection:
            SV->total_cars_arrived--;
            
            // follows the y path first
            
            // The car is too far south; have the car head up north:
            if(M->car.y_to_go > 0) {
                
                // Add a car in the south lane:
                SV->num_cars_in_south--;
            }
            else if(M->car.y_to_go < 0) {
                
                // Add a car in the north lane:
                SV->num_cars_in_north--;
            }
            else if(M->car.x_to_go > 0) {
                //SV->west_lanes[1].cars[SV->west_lanes[1].number_of_cars] = M->car;
                //SV->west_lanes[1].number_of_cars++;
                
                // Add a car in the west lane:
                SV->num_cars_in_west--;
            }
            else if(M->car.x_to_go < 0) {
                //SV->east_lanes[1].cars[SV->east_lanes[1].number_of_cars] = M->car;
                //SV->east_lanes[1].number_of_cars++;
                
                // Add a car in the east lane:
                SV->num_cars_in_east--;
            }
            
            break;
        
        case CAR_DEPARTS:
            
            SV->time_remaining++;
            
            // going in the north direction
            if(M->car.y_to_go > 0) {
                if(SV->traffic_direction == NORTH_SOUTH) {
                    SV->num_cars_out_north--;
                }
            }
            // going in the south direction
            else if(M->car.y_to_go < 0) {
                if(SV->traffic_direction == NORTH_SOUTH) {
                    SV->num_cars_out_south--;
                }
                // needs to turn now
            } else if(M->car.y_to_go == 0 && M->car.x_to_go == M->car.x_to_go_original) {
                if(SV->traffic_direction == NORTH_SOUTH) {
                    if(M->car.y_to_go_original > 0 && M->car.x_to_go > 0) {
                        SV->num_cars_out_east--;
                    }
                    else if(M->car.y_to_go_original > 0 && M->car.x_to_go < 0) {
                        if(SV->num_cars_in_north == 0) {
                            SV->num_cars_out_west--;
                        }
                    }
                    else if(M->car.y_to_go_original < 0 && M->car.x_to_go > 0) {
                        if(SV->num_cars_in_south == 0) {
                            SV->num_cars_out_east--;
                        }
                    }
                    else if(M->car.y_to_go_original < 0 && M->car.x_to_go < 0) {
                        SV->num_cars_out_west--;
                    }
                }
                else if(M->car.x_to_go > 0) {
                    if(SV->traffic_direction == EAST_WEST) {
                        SV->num_cars_out_east--;
                    }
                }
                else if(M->car.x_to_go < 0) {
                    if(SV->traffic_direction == EAST_WEST) {
                        SV->num_cars_out_west--;
                    }
                }
            }
            
            if(SV->time_remaining == 0) {
                SV->time_remaining++;
            }
            
            break;
    }
    
    tw_rand_reverse_unif(LP->rng);

} /** END FUNCTION intersection_reverse_eventhandler **/
