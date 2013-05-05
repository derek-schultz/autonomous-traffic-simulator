#include "ats.h"

void traffic_light_intersection_startup(intersection_state* SV, tw_lp* LP) {
    tw_stime ts = 0;
    tw_event* current_event;
    message_data* new_message;

    // Initialize the total number of cars arrived:
    SV->total_cars_arrived = 0;
    SV->total_cars_finished = 0;

    // Initialize the number of cars arriving into the intersection:
    SV->num_cars_south = 0;
    SV->num_cars_west = 0;
    SV->num_cars_north = 0;
    SV->num_cars_east = 0;

    SV->num_cars_south_left = 0;
    SV->num_cars_west_left = 0;
    SV->num_cars_north_left = 0;
    SV->num_cars_east_left = 0;

    SV->north_south_green_until = -1;
    SV->north_south_left_green_until = -1;
    SV->east_west_green_until = -1;
    SV->east_west_left_green_until = -1;

    // Initialize a random direction to be green
    SV->traffic_direction = tw_rand_ulong(LP->rng, NORTH_SOUTH, EAST_WEST_LEFT);

    // Schedule the first light change
    current_event = tw_event_new(LP->gid, ts, LP);
    new_message = (message_data*)tw_event_data(current_event);
    new_message->event_type = LIGHT_CHANGE;
    tw_event_send(current_event);

    // Put cars on the road
    int i;
    for(i = 0; i < g_traffic_start_events; i++) {
        // Arrival time
        ts = tw_rand_exponential(LP->rng, INITIAL_ARRIVAL_MEAN);

        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);

        new_message->event_type = CAR_ARRIVES;

        assign_rand_dest:
        new_message->car.x_to_go = tw_rand_integer(LP->rng, -MAX_TRAVEL_DISTANCE, MAX_TRAVEL_DISTANCE);
        new_message->car.y_to_go = tw_rand_integer(LP->rng, -MAX_TRAVEL_DISTANCE, MAX_TRAVEL_DISTANCE);
        new_message->car.x_to_go_original = new_message->car.x_to_go;
        new_message->car.y_to_go_original = new_message->car.y_to_go;
        if (new_message->car.y_to_go == 0)
            new_message->car.has_turned = 1;
        else
            new_message->car.has_turned = 0;
        new_message->car.start_time = tw_now(LP);

        tw_event_send(current_event);
    }
}

// Event handler for an intersection:
void traffic_light_intersection_eventhandler(intersection_state* SV, tw_bf* CV,
                                             message_data* M, tw_lp* LP) {

    // Time warp starting time:
    tw_stime ts = 0.0;

    // Save a copy
    tw_stime saved_green_until;

    tw_stime light_green_until;
    
    // Current event:
    tw_event* current_event = NULL;

    // New message data:
    message_data* new_message = NULL;

    // Unknown time warp bit field:
    *(int*) CV = (int) 0;

    tw_lpid next_intersection;
    tw_stime queue_wait_time;
    
    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {
            
    // Handle the LIGHT_CHANGE event IF AND ONLY IF the time remaining on this intersection == 0:
    case LIGHT_CHANGE:
        
        // TIME EXPIRED!
    
        // Check if the traffic is permitted north-south (green on north and south lights):
        if (SV->traffic_direction == NORTH_SOUTH_LEFT) {

            // Traffic was permitted NORTH_SOUTH_LEFT; switch permitted traffic to NORTH_SOUTH:
            SV->traffic_direction = NORTH_SOUTH;

            // Update the timers on the lights
            SV->north_south_green_until = tw_now(LP) + LEFT_TURN_LIGHT_DURATION;
            saved_green_until = SV->north_south_green_until;

            // Schedule the next light change
            ts = GREEN_LIGHT_DURATION;

        }
        else if(SV->traffic_direction == NORTH_SOUTH) {
            
            // Switch permitted traffic to EAST_WEST_LEFT:
            SV->traffic_direction = EAST_WEST_LEFT;

            // Update the timers on the lights
            SV->east_west_left_green_until = tw_now(LP) + LEFT_TURN_LIGHT_DURATION;
            saved_green_until = SV->east_west_left_green_until;

            // Schedule the next light change
            ts = LEFT_TURN_LIGHT_DURATION;

        } else if (SV->traffic_direction == EAST_WEST_LEFT) {

            // Switch permitted traffic to EAST_WEST:
            SV->traffic_direction = EAST_WEST;

            // Update the timers on the lights
            SV->east_west_green_until = tw_now(LP) + LEFT_TURN_LIGHT_DURATION;
            saved_green_until = SV->east_west_green_until;

            // Schedule the next light change
            ts = GREEN_LIGHT_DURATION;

        } else if (SV->traffic_direction == EAST_WEST) {

            // Switch permitted traffic to NORTH_SOUTH_LEFT:
            SV->traffic_direction = NORTH_SOUTH_LEFT;

            // Update the timers on the lights
            SV->north_south_left_green_until = tw_now(LP) + LEFT_TURN_LIGHT_DURATION;
            saved_green_until = SV->north_south_left_green_until;

            // Schedule the next light change
            ts = LEFT_TURN_LIGHT_DURATION;

        }

        // Send the next light change event
        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);
        new_message->event_type = LIGHT_CHANGE;
        new_message->saved_green_until = saved_green_until;
        tw_event_send(current_event);

        break;
        
    // Handle the case where the car arrives at an intersection:
    case CAR_ARRIVES:


        // Car reached its destination:
        if(M->car.y_to_go == 0 && M->car.x_to_go == 0) {
            M->car.end_time = tw_now(LP);
            SV->total_cars_finished++;
            g_total_time += (M->car.end_time - M->car.start_time);
            printf("Car finished with x: %d and y: %d with time: %d\n", M->car.x_to_go_original,
                   M->car.y_to_go_original, (M->car.end_time - M->car.start_time));
            break;
        }
   
        // Increment the total number of cars in this intersection:
        SV->total_cars_arrived++;

        // follows the y path first

        // The car is too far south; have the car head up north:
        if(M->car.y_to_go > 0) {

            // Add a car in the south lane:
            SV->num_cars_south++;
            M->car.position = SOUTH;

            // Calculate the next intersection in the NORTH direction:
            next_intersection = cell_compute_move(LP->gid, NORTH);

            // Decrement the distance to travel up north:
            M->car.y_to_go--;
        }
        else if(M->car.y_to_go < 0) {

            // Add a car in the north lane:
            SV->num_cars_north++;
            M->car.position = NORTH;

            // Calculate the next intersection in the SOUTH direction:
            next_intersection = cell_compute_move(LP->gid, SOUTH);

            // Decrement the distance to travel down south:
            M->car.y_to_go++;
        }
        else if(M->car.y_to_go == 0) {

            if(M->car.has_turned) {

                if(M->car.x_to_go > 0) {

                    // Add a car in the west lane:
                    SV->num_cars_west++;
                    M->car.position = WEST;

                    // Calculate the next intersection in the EAST direction:
                    next_intersection = cell_compute_move(LP->gid, EAST);

                    // Decrement the distance to travel east:
                    M->car.x_to_go--;
                }
                else if(M->car.x_to_go < 0) {

                    // Add a car in the east lane:
                    SV->num_cars_east++;
                    M->car.position = EAST;
                        
                    // Calculate the next intersection in the WEST direction:
                    next_intersection = cell_compute_move(LP->gid, WEST);
                    
                    // Decrement the distance to travel west:
                    M->car.x_to_go++;
                }

            }

            else {

                M->car.has_turned = 1;

                if(M->car.x_to_go > 0) {

                    if(M->car.y_to_go_original > 0) {
                        SV->num_cars_south++;
                        M->car.position = SOUTH;
                    } else {
                        SV->num_cars_north_left++;
                        M->car.position = NORTH_LEFT;
                    }

                    // Calculate the next intersection in the EAST direction:
                    next_intersection = cell_compute_move(LP->gid, EAST);

                    // Decrement the distance to travel east:
                    M->car.x_to_go--;
                }
                else if(M->car.x_to_go < 0) {

                    if(M->car.y_to_go_original > 0) {
                        SV->num_cars_south_left++;
                        M->car.position = SOUTH_LEFT;
                    } else {
                        SV->num_cars_north++;
                        M->car.position = NORTH;
                    }

                    // Calculate the next intersection in the WEST direction:
                    next_intersection = cell_compute_move(LP->gid, WEST);
                    
                    // Decrement the distance to travel west:
                    M->car.x_to_go++;
                }

            }
        }

        switch (M->car.position) {

        case NORTH:
            queue_wait_time = SV->num_cars_north * CAR_ACCELERATION_DELAY;
            light_green_until = SV->north_south_green_until;
            break;
        case NORTH_LEFT:
            queue_wait_time = SV->num_cars_north_left * CAR_ACCELERATION_DELAY;
            light_green_until = SV->north_south_left_green_until;
            break;
        case EAST:
            queue_wait_time = SV->num_cars_east * CAR_ACCELERATION_DELAY;
            light_green_until = SV->east_west_green_until;
            break;
        case EAST_LEFT:
            queue_wait_time = SV->num_cars_east_left * CAR_ACCELERATION_DELAY;
            light_green_until = SV->east_west_left_green_until;
            break;
        case SOUTH:
            queue_wait_time = SV->num_cars_south * CAR_ACCELERATION_DELAY;
            light_green_until = SV->north_south_green_until;
            break;
        case SOUTH_LEFT:
            queue_wait_time = SV->num_cars_south_left * CAR_ACCELERATION_DELAY;
            light_green_until = SV->north_south_left_green_until;
            break;
        case WEST:
            queue_wait_time = SV->num_cars_west * CAR_ACCELERATION_DELAY;
            light_green_until = SV->east_west_green_until;
            break;
        case WEST_LEFT:
            queue_wait_time = SV->num_cars_west_left * CAR_ACCELERATION_DELAY;
            light_green_until = SV->east_west_left_green_until;
            break;

        }

        /* If the light is green and there aren't too many cars ahead
         * we can make it through right now! Schedule the next arrival */
        if (tw_now(LP) + queue_wait_time < light_green_until) {
            ts = tw_rand_exponential(LP->rng, TRAVEL_TIME_VARIATION)
                 + MINIMUM_TRAVEL_TIME;
        }

        /* If the light is red or there are too many cars ahead, we will
         * have to wait at least one cycle. */
        else {
            if (M->car.position == NORTH || M->car.position == SOUTH ||
                M->car.position == EAST  || M->car.position == WEST) {
                ts = tw_rand_exponential(LP->rng, TRAVEL_TIME_VARIATION)
                     + MINIMUM_TRAVEL_TIME
                     + (queue_wait_time / GREEN_LIGHT_DURATION)
                     * g_full_cycle_duration;
            } else {
                ts = tw_rand_exponential(LP->rng, TRAVEL_TIME_VARIATION)
                     + MINIMUM_TRAVEL_TIME
                     + (queue_wait_time / LEFT_TURN_LIGHT_DURATION)
                     * g_full_cycle_duration;
            }
        }

        current_event = tw_event_new(next_intersection, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);
        new_message->car.x_to_go = M->car.x_to_go;
        new_message->car.y_to_go = M->car.y_to_go;
        new_message->car.x_to_go_original = M->car.x_to_go_original;
        new_message->car.y_to_go_original = M->car.y_to_go_original;
        new_message->car.start_time = M->car.start_time;
        new_message->car.end_time = M->car.end_time;
        new_message->event_type = CAR_ARRIVES;
        tw_event_send(current_event);
                                
        break;

    }
} /** END FUNCTION intersection_eventhandler **/

// Reverse Intersection Event Handler that is called when a Time Warp is initiated:
void traffic_light_intersection_reverse_eventhandler(
        intersection_state* SV,
        tw_bf* CV,
        message_data* M,
        tw_lp* LP) {

    tw_lpid next_intersection;
    tw_stime queue_wait_time;

    // Unknown time warp bit field:
    *(int*) CV = (int) 0;

    // Handle the events defined in the "events" enumeration, but in reverse:
    switch(M->event_type) {
            
    case LIGHT_CHANGE:
                    
        // Check if the traffic is permitted north-south (green on north and south lights):
        if (SV->traffic_direction == NORTH_SOUTH_LEFT) {

            // Traffic was permitted NORTH_SOUTH_LEFT; switch permitted traffic to EAST_WEST:
            SV->traffic_direction = EAST_WEST;

            // Update the timers on the lights
            SV->north_south_green_until = M->saved_green_until;

        }
        else if(SV->traffic_direction == NORTH_SOUTH) {
            
            // Switch permitted traffic to NORTH_SOUTH_LEFT:
            SV->traffic_direction = NORTH_SOUTH_LEFT;

            // Update the timers on the lights
            SV->north_south_left_green_until = M->saved_green_until;

        } else if (SV->traffic_direction == EAST_WEST_LEFT) {

            // Switch permitted traffic to NORTH_SOUTH:
            SV->traffic_direction = NORTH_SOUTH;

            // Update the timers on the lights
            SV->north_south_green_until = M->saved_green_until;

        } else if (SV->traffic_direction == EAST_WEST) {

            // Switch permitted traffic to EAST_WEST_LEFT:
            SV->traffic_direction = EAST_WEST_LEFT;

            // Update the timers on the lights
            SV->east_west_left_green_until = M->saved_green_until;

        }

        break;
    
    case CAR_ARRIVES:

        // Car reached its destination:
        if(M->car.y_to_go == 0 && M->car.x_to_go == 0) {
            SV->total_cars_finished--;
            g_total_time -= (M->car.end_time - M->car.start_time);
            printf("Car unfinished!\n");
            break;
        }
   
        // Increment the total number of cars in this intersection:
        SV->total_cars_arrived--;

        // follows the y path first

        // The car is too far south; have the car head up north:
        if(M->car.y_to_go > 0) {
            SV->num_cars_south--;
            M->car.y_to_go++;
        }
        else if(M->car.y_to_go < 0) {
            SV->num_cars_north--;
            M->car.y_to_go--;
        }
        else if(M->car.y_to_go == 0) {

            //TODO: figure out how to reverse has_turned

            if(M->car.has_turned) {

                if(M->car.x_to_go > 0) {
                    SV->num_cars_west--;
                    M->car.x_to_go++;
                }
                else if(M->car.x_to_go < 0) {
                    SV->num_cars_east--;
                    M->car.x_to_go--;
                }

            }

            else {

                if(M->car.x_to_go > 0) {

                    if(M->car.y_to_go_original > 0) {
                        SV->num_cars_south--;
                    } else {
                        SV->num_cars_north_left--;
                    }

                    M->car.x_to_go++;
                }
                else if(M->car.x_to_go < 0) {

                    if(M->car.y_to_go_original > 0) {
                        SV->num_cars_south_left--;
                    } else {
                        SV->num_cars_north--;
                    }

                    M->car.x_to_go--;
                }

            }
        }

        tw_rand_reverse_unif(LP->rng);

        break;

    }
    

} /** END FUNCTION intersection_reverse_eventhandler **/
