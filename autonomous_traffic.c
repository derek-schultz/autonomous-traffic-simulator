#include "ats.h"

/*
 * Determines whether cars a and b will colide if they travel through the
 * intersection at the same time.
 * NL indicates that the car is coming from the north and turning left
 * ES indicates that the car is coming from the east and going stratight
 * etc...
 */
int will_collide(enum travel_direction a, enum travel_direction b) {
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

void autonomous_traffic_intersection_startup(intersection_state* SV,
                                             tw_lp* LP) {
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

void autonomous_traffic_intersection_eventhandler(
        intersection_state* SV,
        tw_bf* CV,
        message_data* M,
        tw_lp* LP) {

    // Time warp starting time:
    tw_stime ts = 0.0;
    
    // Current event:
    tw_event* current_event = NULL;

    // New message data:
    message_data* new_message = NULL;

    // Unknown time warp bit field:
    *(int*)CV = (int)0;

    tw_lpid next_intersection;
    tw_stime queue_wait_time;

    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {

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

        // The car is too far south; have the car head up north:
        if(M->car.y_to_go > 0) {

            // Add a car in the south lane:
            SV->num_cars_south++;
            M->car.position = SOUTH;
            M->car.intention = SS;
            M->car.queue_location = SV->num_cars_south;

            // Calculate the next intersection in the NORTH direction:
            next_intersection = cell_compute_move(LP->gid, NORTH);

            // Decrement the distance to travel up north:
            M->car.y_to_go--;
        }
        else if(M->car.y_to_go < 0) {

            // Add a car in the north lane:
            SV->num_cars_north++;
            M->car.position = NORTH;
            M->car.intention = NS;
            M->car.queue_location = SV->num_cars_north;

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
                    M->car.intention = WS;
                    M->car.queue_location = SV->num_cars_west;

                    // Calculate the next intersection in the EAST direction:
                    next_intersection = cell_compute_move(LP->gid, EAST);

                    // Decrement the distance to travel east:
                    M->car.x_to_go--;
                }
                else if(M->car.x_to_go < 0) {

                    // Add a car in the east lane:
                    SV->num_cars_east++;
                    M->car.position = EAST;
                    M->car.intention = ES;
                    M->car.queue_location = SV->num_cars_east;
                        
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
                        M->car.intention = SR;
                        M->car.queue_location = SV->num_cars_south;
                    } else {
                        SV->num_cars_north_left++;
                        M->car.position = NORTH_LEFT;
                        M->car.intention = NL;
                        M->car.queue_location = SV->num_cars_north_left;
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
                        M->car.intention = SL;
                        M->car.queue_location = SV->num_cars_south_left;
                    } else {
                        SV->num_cars_north++;
                        M->car.position = NORTH;
                        M->car.intention = NR;
                        M->car.queue_location = SV->num_cars_north;
                    }

                    // Calculate the next intersection in the WEST direction:
                    next_intersection = cell_compute_move(LP->gid, WEST);
                    
                    // Decrement the distance to travel west:
                    M->car.x_to_go++;
                }

            }
        }

        if (M->car.queue_location == 1) {
            // There is no one waiting! Move up now!
            ts = 0.0;
        } else {
            // Move up when the car at the front of the line enters the intersection
            ts = SV->lead_car_will_enter - tw_now(LP);
            // This awkward off-by-one can't be avoided
            M->car.queue_position--;
        }

        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);
        new_message->car.x_to_go = M->car.x_to_go;
        new_message->car.y_to_go = M->car.y_to_go;
        new_message->car.start_time = M->car.start_time;
        new_message->car.end_time = M->car.end_time;
        new_message->car.position = M->car.position;
        new_message->car.queue_location;
        new_message->event_type = CAR_MOVES_FORWARD;
        tw_event_send(current_event);

    case CAR_MOVES_FORWARD:
        M->car.queue_position--;

        if (M->car.queue_position == 0) {
            switch (M->car.intention) {

            case NL:
                if (SV->SS_LOCK > tw_now(LP)) {
                    departure_time = SV->SS_LOCK;
                }
                if (SV->SR_LOCK > tw_now(LP) && SV->SR_LOCK > departure_time) {
                    departure_time = SV->SR_LOCK;
                } 
                if (SV->ES_LOCK > tw_now(LP) && SV->ES_LOCK > departure_time) {
                    departure_time = SV->ES_LOCK;
                } 
                if (SV->EL_LOCK > tw_now(LP) && SV->EL_LOCK > departure_time) {
                    departure_time = SV->EL_LOCK;
                } 
                if (SV->WS_LOCK > tw_now(LP) && SV->WS_LOCK > departure_time) {
                    departure_time = SV->WS_LOCK;
                } 
                if (SV->WL_LOCK > tw_now(LP) && SV->WL_LOCK > departure_time) {
                    departure_time = SV->WL_LOCK;
                }

            case NR:
                if (SV->SL_LOCK > tw_now(LP)) {
                    departure_time = SV->SL_LOCK;
                }
                if (SV->ES_LOCK > tw_now(LP) && SV->ES_LOCK > departure_time) {
                    departure_time = SV->ES_LOCK;
                }
            
            case NS:
                if (SV->SL_LOCK > tw_now(LP)) {
                    departure_time = SV->SL_LOCK;
                }
                if (SV->ES_LOCK > tw_now(LP) && SV->ES_LOCK > departure_time) {
                    departure_time = SV->ES_LOCK;
                } 
                if (SV->EL_LOCK > tw_now(LP) && SV->EL_LOCK > departure_time) {
                    departure_time = SV->EL_LOCK;
                } 
                if (SV->WS_LOCK > tw_now(LP) && SV->WS_LOCK > departure_time) {
                    departure_time = SV->WS_LOCK;
                } 
                if (SV->WR_LOCK > tw_now(LP) && SV->WR_LOCK > departure_time) {
                    departure_time = SV->WR_LOCK;
                } 
                if (SV->WL_LOCK > tw_now(LP) && SV->WL_LOCK > departure_time) {
                    departure_time = SV->WL_LOCK;
                }
            
            case EL:
                if (SV->SS_LOCK > tw_now(LP)) {
                    departure_time = SV->SS_LOCK;
                }
                if (SV->SL_LOCK > tw_now(LP) && SV->SL_LOCK > departure_time) {
                    departure_time = SV->SL_LOCK;
                } 
                if (SV->NS_LOCK > tw_now(LP) && SV->NS_LOCK > departure_time) {
                    departure_time = SV->NS_LOCK;
                } 
                if (SV->NL_LOCK > tw_now(LP) && SV->NL_LOCK > departure_time) {
                    departure_time = SV->NL_LOCK;
                } 
                if (SV->WR_LOCK > tw_now(LP) && SV->WR_LOCK > departure_time) {
                    departure_time = SV->WR_LOCK;
                } 
                if (SV->WS_LOCK > tw_now(LP) && SV->WS_LOCK > departure_time) {
                    departure_time = SV->WS_LOCK;
                }
            
            case ER:
                if (SV->SS_LOCK > tw_now(LP)) {
                    departure_time = SV->SS_LOCK;
                }
                if (SV->WL_LOCK > tw_now(LP) && SV->WL_LOCK > departure_time) {
                    departure_time = SV->WL_LOCK;
                }
            
            case ES:
                if (SV->SS_LOCK > tw_now(LP)) {
                    departure_time = SV->SS_LOCK;
                }
                if (SV->SL_LOCK > tw_now(LP) && SV->SL_LOCK > departure_time) {
                    departure_time = SV->SL_LOCK;
                } 
                if (SV->NL_LOCK > tw_now(LP) && SV->NL_LOCK > departure_time) {
                    departure_time = SV->NL_LOCK;
                } 
                if (SV->NR_LOCK > tw_now(LP) && SV->NR_LOCK > departure_time) {
                    departure_time = SV->NR_LOCK;
                } 
                if (SV->NS_LOCK > tw_now(LP) && SV->NS_LOCK > departure_time) {
                    departure_time = SV->NS_LOCK;
                } 
                if (SV->WL_LOCK > tw_now(LP) && SV->WL_LOCK > departure_time) {
                    departure_time = SV->WL_LOCK;
                }
            
            case SL:
                if (SV->ES_LOCK > tw_now(LP)) {
                    departure_time = SV->ES_LOCK;
                }
                if (SV->EL_LOCK > tw_now(LP) && SV->EL_LOCK > departure_time) {
                    departure_time = SV->EL_LOCK;
                } 
                if (SV->NR_LOCK > tw_now(LP) && SV->NR_LOCK > departure_time) {
                    departure_time = SV->NR_LOCK;
                } 
                if (SV->NS_LOCK > tw_now(LP) && SV->NS_LOCK > departure_time) {
                    departure_time = SV->NS_LOCK;
                } 
                if (SV->WL_LOCK > tw_now(LP) && SV->WL_LOCK > departure_time) {
                    departure_time = SV->WL_LOCK;
                } 
                if (SV->WS_LOCK > tw_now(LP) && SV->WS_LOCK > departure_time) {
                    departure_time = SV->WS_LOCK;
                }
            
            case SR:
                if (SV->WS_LOCK > tw_now(LP)) {
                    departure_time = SV->WS_LOCK;
                }
                if (SV->NL_LOCK > tw_now(LP) && SV->NL_LOCK > departure_time) {
                    departure_time = SV->NL_LOCK;
                }
            
            case SS:
                if (SV->NL_LOCK > tw_now(LP)) {
                    departure_time = SV->NL_LOCK;
                }
                if (SV->ES_LOCK > tw_now(LP) && SV->ES_LOCK > departure_time) {
                    departure_time = SV->ES_LOCK;
                } 
                if (SV->EL_LOCK > tw_now(LP) && SV->EL_LOCK > departure_time) {
                    departure_time = SV->EL_LOCK;
                } 
                if (SV->ER_LOCK > tw_now(LP) && SV->ER_LOCK > departure_time) {
                    departure_time = SV->ER_LOCK;
                } 
                if (SV->WS_LOCK > tw_now(LP) && SV->WS_LOCK > departure_time) {
                    departure_time = SV->WS_LOCK;
                } 
                if (SV->WL_LOCK > tw_now(LP) && SV->WL_LOCK > departure_time) {
                    departure_time = SV->WL_LOCK;
                }
            
            case WL:
                if (SV->NS_LOCK > tw_now(LP)) {
                    departure_time = SV->NS_LOCK;
                }
                if (SV->NL_LOCK > tw_now(LP) && SV->NL_LOCK > departure_time) {
                    departure_time = SV->NL_LOCK;
                } 
                if (SV->SS_LOCK > tw_now(LP) && SV->SS_LOCK > departure_time) {
                    departure_time = SV->SS_LOCK;
                } 
                if (SV->SL_LOCK > tw_now(LP) && SV->SL_LOCK > departure_time) {
                    departure_time = SV->SL_LOCK;
                } 
                if (SV->ES_LOCK > tw_now(LP) && SV->ES_LOCK > departure_time) {
                    departure_time = SV->ES_LOCK;
                } 
                if (SV->ER_LOCK > tw_now(LP) && SV->ER_LOCK > departure_time) {
                    departure_time = SV->ER_LOCK;
                }
            
            case WR:
                if (SV->EL_LOCK > tw_now(LP)) {
                    departure_time = SV->EL_LOCK;
                }
                if (SV->NS_LOCK > tw_now(LP) && SV->NS_LOCK > departure_time) {
                    departure_time = SV->NS_LOCK;
                }
            
            case WS:
                if (SV->SS_LOCK > tw_now(LP)) {
                    departure_time = SV->SS_LOCK;
                }
                if (SV->SL_LOCK > tw_now(LP) && SV->SL_LOCK > departure_time) {
                    departure_time = SV->SL_LOCK;
                } 
                if (SV->SR_LOCK > tw_now(LP) && SV->SR_LOCK > departure_time) {
                    departure_time = SV->SR_LOCK;
                } 
                if (SV->EL_LOCK > tw_now(LP) && SV->EL_LOCK > departure_time) {
                    departure_time = SV->EL_LOCK;
                } 
                if (SV->NS_LOCK > tw_now(LP) && SV->NS_LOCK > departure_time) {
                    departure_time = SV->NS_LOCK;
                } 
                if (SV->NL_LOCK > tw_now(LP) && SV->NL_LOCK > departure_time) {
                    departure_time = SV->NL_LOCK;
                }
            }
        }


    case CARS_GO:
        int i;
        car_type cars[4][4];

        cars[0][0] = north car;

        int placed = 0;
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                if (cars[i][j] == 0) {
                    cars[i][j] = car;
                    placed = 1;
                }
                else if (will_colide(cars[i][j], car)) {
                    break;
                }
            }
            if (placed)
                break;
        }
                    
    case CAR_ARRIVES:
        
        // TODO: put car in intersection

        /* 
         * If there are no cars in the intersection, we need to schedule a
         * departure for this vehicle immediately. If there are already cars in
         * the intersection, organized departures are already cycling and the
         * car will be automatically dispatched when possible.
         */
        if (SV->num_cars_total == 0) {
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (message_data*)tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
            new_message->event_type = CARS_GO;
            tw_event_send(current_event);
        }

        break;
    }

}

void autonomous_traffic_intersection_reverse_eventhandler(
    intersection_state* SV,
    tw_bf* BF,
    message_data* M,
    tw_lp* LP
) {

}