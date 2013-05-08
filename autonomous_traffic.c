#include "ats.h"

static unsigned int car_counter = 0;

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

    // Put cars on the road
    int i;
    for(i = 0; i < g_traffic_start_events; i++) {
        // Arrival time
        ts = tw_rand_exponential(LP->rng, INITIAL_ARRIVAL_MEAN);

        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);

        new_message->event_type = CAR_ARRIVES;

        assign_rand_dest:
        new_message->car.x_to_go = tw_rand_integer(LP->rng,
                                                   -MAX_TRAVEL_DISTANCE,
                                                   MAX_TRAVEL_DISTANCE);
        new_message->car.y_to_go = tw_rand_integer(LP->rng,
                                                   -MAX_TRAVEL_DISTANCE,
                                                   MAX_TRAVEL_DISTANCE);
        new_message->car.x_to_go_original = new_message->car.x_to_go;
        new_message->car.y_to_go_original = new_message->car.y_to_go;

        new_message->car.id = car_counter;
        car_counter++;

        #ifdef DEBUG
            if (LP->gid == 0)
                printf("Putting car %d on the road at %f. Destination x: %d y: %d\n",
                        ts, new_message->car.id,
                        new_message->car.x_to_go, new_message->car.y_to_go);
        #endif

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

    tw_stime departure_time;
    tw_stime lead_car_enters;
    
    // Current event:
    tw_event* current_event = NULL;

    // New message data:
    message_data* new_message = NULL;

    // Unknown time warp bit field:
    *(int*)CV = (int)0;

    tw_stime queue_wait_time;

    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {

    case CAR_ARRIVES:

        // Car reached its destination:
        if(M->car.y_to_go == 0 && M->car.x_to_go == 0) {
            M->car.end_time = tw_now(LP);
            SV->total_cars_finished++;
            g_total_time += (M->car.end_time - M->car.start_time);
            #ifdef DEBUG
            printf("Autonomous car %d finished with x: %d and y: %d with time: %d\n",
                   M->car.id, M->car.x_to_go_original, M->car.y_to_go_original,
                   (M->car.end_time - M->car.start_time));
            #endif
            break;
        }

        #ifdef DEBUG
            if (LP->gid == 0) {

                printf("Autonomous car %d arrives at intersection %d with x: %d and y: %d and x original: %d and y original: %d with time: %f\n",
                       M->car.id, LP->gid,
                       M->car.x_to_go, M->car.y_to_go,
                       M->car.x_to_go_original, M->car.y_to_go_original,
                       (tw_now(LP) - M->car.start_time));
                printf("Queues: %d %d %d %d %d %d %d %d\n",
                       SV->num_cars_north, SV->num_cars_north_left,
                       SV->num_cars_east, SV->num_cars_east_left,
                       SV->num_cars_south, SV->num_cars_south_left,
                       SV->num_cars_west, SV->num_cars_west_left);

            }
        #endif

        // Increment the total number of cars in this intersection:
        SV->total_cars_arrived++;

        // The car is too far south; have the car head up north:
        if(M->car.y_to_go > 0) {

            // Add a car in the south lane:
            SV->num_cars_south++;
            M->car.position = SOUTH;
            M->car.intention = SS;
            M->car.queue_location = SV->num_cars_south;

            lead_car_enters = SV->south_lead_car_will_enter;

            // Calculate the next intersection in the NORTH direction:
            M->car.next_intersection = cell_compute_move(LP->gid, NORTH);

            // Decrement the distance to travel up north:
            M->car.y_to_go--;
        }
        else if(M->car.y_to_go < 0) {

            // Add a car in the north lane:
            SV->num_cars_north++;
            M->car.position = NORTH;
            M->car.intention = NS;
            M->car.queue_location = SV->num_cars_north;

            lead_car_enters = SV->north_lead_car_will_enter;

            // Calculate the next intersection in the SOUTH direction:
            M->car.next_intersection = cell_compute_move(LP->gid, SOUTH);

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

                    lead_car_enters = SV->west_lead_car_will_enter;

                    // Calculate the next intersection in the EAST direction:
                    M->car.next_intersection = cell_compute_move(LP->gid,
                                                                 EAST);

                    // Decrement the distance to travel east:
                    M->car.x_to_go--;
                }
                else if(M->car.x_to_go < 0) {

                    // Add a car in the east lane:
                    SV->num_cars_east++;
                    M->car.position = EAST;
                    M->car.intention = ES;
                    M->car.queue_location = SV->num_cars_east;

                    lead_car_enters = SV->east_lead_car_will_enter;
                        
                    // Calculate the next intersection in the WEST direction:
                    M->car.next_intersection = cell_compute_move(LP->gid,
                                                                 WEST);
                    
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
                        lead_car_enters = SV->south_lead_car_will_enter;
                    } else {
                        SV->num_cars_north_left++;
                        M->car.position = NORTH_LEFT;
                        M->car.intention = NL;
                        M->car.queue_location = SV->num_cars_north_left;
                        lead_car_enters = SV->north_left_lead_car_will_enter;
                    }

                    // Calculate the next intersection in the EAST direction:
                    M->car.next_intersection = cell_compute_move(LP->gid,
                                                                 EAST);

                    // Decrement the distance to travel east:
                    M->car.x_to_go--;
                }
                else if(M->car.x_to_go < 0) {

                    if(M->car.y_to_go_original > 0) {
                        SV->num_cars_south_left++;
                        M->car.position = SOUTH_LEFT;
                        M->car.intention = SL;
                        M->car.queue_location = SV->num_cars_south_left;
                        lead_car_enters = SV->south_left_lead_car_will_enter;
                    } else {
                        SV->num_cars_north++;
                        M->car.position = NORTH;
                        M->car.intention = NR;
                        M->car.queue_location = SV->num_cars_north;
                        lead_car_enters = SV->north_lead_car_will_enter;
                    }

                    // Calculate the next intersection in the WEST direction:
                    M->car.next_intersection = cell_compute_move(LP->gid, WEST);
                    
                    // Decrement the distance to travel west:
                    M->car.x_to_go++;
                }

            }
        }

        M->car.queue_location--;
        if (M->car.queue_location == 0) {
            // There is no one waiting! Move up now!
            ts = 0.0;

        } else {
            // Move up when the car at the front enters the intersection
            ts = lead_car_enters - tw_now(LP);
        }

		if(ts < 0)
			ts = 0;
        #ifdef DEBUG
        if (LP->gid == 0)
            printf("The lead car will enter %f. Autonomous car %d headed %d is in queue position %d and will leave in %f sec with x: %d and y: %d and x original: %d and y original: %d with time: %f\n",
               lead_car_enters, M->car.id, M->car.position, M->car.queue_location, ts,
               M->car.x_to_go, M->car.y_to_go,
               M->car.x_to_go_original, M->car.y_to_go_original,
               (tw_now(LP) - M->car.start_time));
        #endif

        current_event = tw_event_new(LP->gid, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);
        new_message->car.x_to_go = M->car.x_to_go;
        new_message->car.y_to_go = M->car.y_to_go;
        new_message->car.x_to_go_original = M->car.x_to_go_original;
        new_message->car.y_to_go_original = M->car.y_to_go_original;
        new_message->car.start_time = M->car.start_time;
        new_message->car.end_time = M->car.end_time;
        new_message->car.position = M->car.position;
        new_message->car.intention = M->car.intention;
        new_message->car.has_turned = M->car.has_turned;
        new_message->car.next_intersection = M->car.next_intersection;
        new_message->car.queue_location = M->car.queue_location;
        new_message->car.id = M->car.id;
        new_message->event_type = CAR_MOVES_FORWARD;
		new_message->saved_north_lead_car_will_enter = M->saved_north_lead_car_will_enter;
		new_message->saved_north_left_lead_car_will_enter = M->saved_north_left_lead_car_will_enter;
		new_message->saved_east_lead_car_will_enter = M->saved_east_lead_car_will_enter;
		new_message->saved_east_left_lead_car_will_enter = M->saved_east_left_lead_car_will_enter;
		new_message->saved_south_lead_car_will_enter = M->saved_south_lead_car_will_enter;
		new_message->saved_south_left_lead_car_will_enter = M->saved_south_left_lead_car_will_enter;
		new_message->saved_west_lead_car_will_enter = M->saved_west_lead_car_will_enter;
		new_message->saved_west_left_lead_car_will_enter = M->saved_west_left_lead_car_will_enter;
        tw_event_send(current_event);

        break;

    case CAR_MOVES_FORWARD:

        M->car.queue_location--;

        #ifdef DEBUG
        if (LP->gid == 0)

            printf("Autonomous car %d moves forward with x: %d and y: %d and x original: %d and y original: %d with time: %f and queue: %d\n",
                   M->car.id, M->car.x_to_go, M->car.y_to_go,
                   M->car.x_to_go_original, M->car.y_to_go_original,
                   (tw_now(LP) - M->car.start_time), M->car.queue_location);
        #endif

        // If car is at the front of the line:
        if (M->car.queue_location == -1) {

            #ifdef DEBUG
            if (LP->gid == 0)

                printf("Autonomous car %d is first in line with x: %d and y: %d and x original: %d and y original: %d with time: %f\n",
                   M->car.id, M->car.x_to_go, M->car.y_to_go,
                   M->car.x_to_go_original, M->car.y_to_go_original,
                   (tw_now(LP) - M->car.start_time));
            #endif


            departure_time = tw_now(LP);

            switch (M->car.intention) {

            case NL:
                if (SV->SS_lock > tw_now(LP)) {
                    departure_time = SV->SS_lock;

                    M->saved_north_left_lead_car_will_enter = 
                                        SV->north_left_lead_car_will_enter;

                    SV->north_left_lead_car_will_enter = departure_time;
                }
                if (SV->SR_lock > tw_now(LP) && SV->SR_lock > departure_time) {
                    departure_time = SV->SR_lock;

                    M->saved_north_left_lead_car_will_enter = 
                                        SV->north_left_lead_car_will_enter;

                    SV->north_left_lead_car_will_enter = departure_time;
                } 
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    departure_time = SV->ES_lock;

                    M->saved_north_left_lead_car_will_enter = 
                                        SV->north_left_lead_car_will_enter;

                    SV->north_left_lead_car_will_enter = departure_time;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    departure_time = SV->EL_lock;

                    M->saved_north_left_lead_car_will_enter = 
                                        SV->north_left_lead_car_will_enter;

                    SV->north_left_lead_car_will_enter = departure_time;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    departure_time = SV->WS_lock;

                    M->saved_north_left_lead_car_will_enter = 
                                        SV->north_left_lead_car_will_enter;

                    SV->north_left_lead_car_will_enter = departure_time;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    departure_time = SV->WL_lock;

                    M->saved_north_left_lead_car_will_enter = 
                                        SV->north_left_lead_car_will_enter;

                    SV->north_left_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_north_left_lead_car_will_enter = 
                                        SV->north_left_lead_car_will_enter;
                    SV->north_left_lead_car_will_enter = departure_time;
                }
                break;
                
            case NR:
                if (SV->SL_lock > tw_now(LP)) {
                    departure_time = SV->SL_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                }
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    departure_time = SV->ES_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;
                    SV->north_lead_car_will_enter = departure_time;
                }
                break;
                
            case NS:
                if (SV->SL_lock > tw_now(LP)) {
                    departure_time = SV->SL_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                }
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    departure_time = SV->ES_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    departure_time = SV->EL_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    departure_time = SV->WS_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                } 
                if (SV->WR_lock > tw_now(LP) && SV->WR_lock > departure_time) {
                    departure_time = SV->WR_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    departure_time = SV->WL_lock;

                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;

                    SV->north_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_north_lead_car_will_enter = 
                                        SV->north_lead_car_will_enter;
                    SV->north_lead_car_will_enter = departure_time;
                }
                break;
                
            case EL:
                if (SV->SS_lock > tw_now(LP)) {
                    departure_time = SV->SS_lock;

                    M->saved_east_left_lead_car_will_enter = 
                                        SV->east_left_lead_car_will_enter;

                    SV->east_left_lead_car_will_enter = departure_time;
                }
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    departure_time = SV->SL_lock;

                    M->saved_east_left_lead_car_will_enter = 
                                        SV->east_left_lead_car_will_enter;

                    SV->east_left_lead_car_will_enter = departure_time;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    departure_time = SV->NS_lock;

                    M->saved_east_left_lead_car_will_enter = 
                                        SV->east_left_lead_car_will_enter;

                    SV->east_left_lead_car_will_enter = departure_time;
                } 
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    departure_time = SV->NL_lock;

                    M->saved_east_left_lead_car_will_enter = 
                                        SV->east_left_lead_car_will_enter;

                    SV->east_left_lead_car_will_enter = departure_time;
                } 
                if (SV->WR_lock > tw_now(LP) && SV->WR_lock > departure_time) {
                    departure_time = SV->WR_lock;

                    M->saved_east_left_lead_car_will_enter = 
                                        SV->east_left_lead_car_will_enter;

                    SV->east_left_lead_car_will_enter = departure_time;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    departure_time = SV->WS_lock;

                    M->saved_east_left_lead_car_will_enter = 
                                        SV->east_left_lead_car_will_enter;

                    SV->east_left_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_east_left_lead_car_will_enter = 
                                        SV->east_left_lead_car_will_enter;
                    SV->east_left_lead_car_will_enter = departure_time;
                }
                break;
                
            case ER:
                if (SV->SS_lock > tw_now(LP)) {
                    departure_time = SV->SS_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                }
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    departure_time = SV->WL_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;
                    SV->east_lead_car_will_enter = departure_time;
                }
                break;
                
            case ES:
                if (SV->SS_lock > tw_now(LP)) {
                    departure_time = SV->SS_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                }
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    departure_time = SV->SL_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                } 
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    departure_time = SV->NL_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                } 
                if (SV->NR_lock > tw_now(LP) && SV->NR_lock > departure_time) {
                    departure_time = SV->NR_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    departure_time = SV->NS_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    departure_time = SV->WL_lock;

                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;

                    SV->east_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_east_lead_car_will_enter = 
                                        SV->east_lead_car_will_enter;
                    SV->east_lead_car_will_enter = departure_time;
                }
                break;
                
            case SL:
                if (SV->ES_lock > tw_now(LP)) {
                    departure_time = SV->ES_lock;

                    M->saved_south_left_lead_car_will_enter = 
                                        SV->south_left_lead_car_will_enter;

                    SV->south_left_lead_car_will_enter = departure_time;
                }
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    departure_time = SV->EL_lock;

                    M->saved_south_left_lead_car_will_enter = 
                                        SV->south_left_lead_car_will_enter;

                    SV->south_left_lead_car_will_enter = departure_time;
                } 
                if (SV->NR_lock > tw_now(LP) && SV->NR_lock > departure_time) {
                    departure_time = SV->NR_lock;

                    M->saved_south_left_lead_car_will_enter = 
                                        SV->south_left_lead_car_will_enter;

                    SV->south_left_lead_car_will_enter = departure_time;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    departure_time = SV->NS_lock;

                    M->saved_south_left_lead_car_will_enter = 
                                        SV->south_left_lead_car_will_enter;

                    SV->south_left_lead_car_will_enter = departure_time;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    departure_time = SV->WL_lock;

                    M->saved_south_left_lead_car_will_enter = 
                                        SV->south_left_lead_car_will_enter;

                    SV->south_left_lead_car_will_enter = departure_time;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    departure_time = SV->WS_lock;

                    M->saved_south_left_lead_car_will_enter = 
                                        SV->south_left_lead_car_will_enter;

                    SV->south_left_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_south_left_lead_car_will_enter = 
                                        SV->south_left_lead_car_will_enter;
                    SV->south_left_lead_car_will_enter = departure_time;
                }
                break;
                
            case SR:
                if (SV->WS_lock > tw_now(LP)) {
                    departure_time = SV->WS_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                }
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    departure_time = SV->NL_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;
                    SV->south_lead_car_will_enter = departure_time;
                }
                break;
                
            case SS:
                if (SV->NL_lock > tw_now(LP)) {
                    departure_time = SV->NL_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                }
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    departure_time = SV->ES_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    departure_time = SV->EL_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                } 
                if (SV->ER_lock > tw_now(LP) && SV->ER_lock > departure_time) {
                    departure_time = SV->ER_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    departure_time = SV->WS_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    departure_time = SV->WL_lock;

                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;

                    SV->south_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_south_lead_car_will_enter = 
                                        SV->south_lead_car_will_enter;
                    SV->south_lead_car_will_enter = departure_time;
                }
                break;
                
            case WL:
                if (SV->NS_lock > tw_now(LP)) {
                    departure_time = SV->NS_lock;

                    M->saved_west_left_lead_car_will_enter = 
                                        SV->west_left_lead_car_will_enter;

                    SV->west_left_lead_car_will_enter = departure_time;
                }
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    departure_time = SV->NL_lock;

                    M->saved_west_left_lead_car_will_enter = 
                                        SV->west_left_lead_car_will_enter;

                    SV->west_left_lead_car_will_enter = departure_time;
                } 
                if (SV->SS_lock > tw_now(LP) && SV->SS_lock > departure_time) {
                    departure_time = SV->SS_lock;

                    M->saved_west_left_lead_car_will_enter = 
                                        SV->west_left_lead_car_will_enter;

                    SV->west_left_lead_car_will_enter = departure_time;
                } 
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    departure_time = SV->SL_lock;

                    M->saved_west_left_lead_car_will_enter = 
                                        SV->west_left_lead_car_will_enter;

                    SV->west_left_lead_car_will_enter = departure_time;
                } 
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    departure_time = SV->ES_lock;

                    M->saved_west_left_lead_car_will_enter = 
                                        SV->west_left_lead_car_will_enter;

                    SV->west_left_lead_car_will_enter = departure_time;
                } 
                if (SV->ER_lock > tw_now(LP) && SV->ER_lock > departure_time) {
                    departure_time = SV->ER_lock;

                    M->saved_west_left_lead_car_will_enter = 
                                        SV->west_left_lead_car_will_enter;

                    SV->west_left_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_west_left_lead_car_will_enter = 
                                        SV->west_left_lead_car_will_enter;
                    SV->west_left_lead_car_will_enter = departure_time;
                }
                break;
                
            case WR:
                if (SV->EL_lock > tw_now(LP)) {
                    departure_time = SV->EL_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                }
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    departure_time = SV->NS_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;
                    SV->west_lead_car_will_enter = departure_time;
                }
                break;
                
            case WS:
                if (SV->SS_lock > tw_now(LP)) {
                    departure_time = SV->SS_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                }
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    departure_time = SV->SL_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                } 
                if (SV->SR_lock > tw_now(LP) && SV->SR_lock > departure_time) {
                    departure_time = SV->SR_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    departure_time = SV->EL_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    departure_time = SV->NS_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                } 
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    departure_time = SV->NL_lock;

                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;

                    SV->west_lead_car_will_enter = departure_time;
                }
                if (departure_time == tw_now(LP)) {
                    M->saved_west_lead_car_will_enter = 
                                        SV->west_lead_car_will_enter;
                    SV->west_lead_car_will_enter = departure_time;
                }
                break;
            }

            ts = departure_time - tw_now(LP);

            #ifdef DEBUG
            if (LP->gid == 0)
                printf("Autonomous car %d scheduled to depart at %f\n",
                   M->car.id, departure_time);
            #endif
			
            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (message_data*)tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.x_to_go_original = M->car.x_to_go_original;
            new_message->car.y_to_go_original = M->car.y_to_go_original;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
            new_message->car.position = M->car.position;
            new_message->car.intention = M->car.intention;
            new_message->car.has_turned = M->car.has_turned;
            new_message->car.next_intersection = M->car.next_intersection;
            new_message->car.queue_location = M->car.queue_location;
            new_message->car.id = M->car.id;
            new_message->event_type = CAR_ENTERS_INTERSECTION;
            new_message->saved_north_lead_car_will_enter = M->saved_north_lead_car_will_enter;
			new_message->saved_north_left_lead_car_will_enter = M->saved_north_left_lead_car_will_enter;
			new_message->saved_east_lead_car_will_enter = M->saved_east_lead_car_will_enter;
			new_message->saved_east_left_lead_car_will_enter = M->saved_east_left_lead_car_will_enter;
			new_message->saved_south_lead_car_will_enter = M->saved_south_lead_car_will_enter;
			new_message->saved_south_left_lead_car_will_enter = M->saved_south_left_lead_car_will_enter;
			new_message->saved_west_lead_car_will_enter = M->saved_west_lead_car_will_enter;
			new_message->saved_west_left_lead_car_will_enter = M->saved_west_left_lead_car_will_enter;
            tw_event_send(current_event);
			//printf("111\n");
        }

        // If the car is not at the front of the line
        else {
			//printf("222\n");
            // Schedule the next move up
            switch(M->car.position) {

            case NORTH:
                ts = SV->north_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            case NORTH_LEFT:
                ts = SV->north_left_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            case SOUTH:
                ts = SV->south_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            case SOUTH_LEFT:
                ts = SV->south_left_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            case EAST:
                ts = SV->east_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            case EAST_LEFT:
                ts = SV->east_left_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            case WEST:
                ts = SV->west_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            case WEST_LEFT:
                ts = SV->west_left_lead_car_will_enter
                     + CAR_ACCELERATION_DELAY * M->car.queue_location;
                break;

            }

            current_event = tw_event_new(LP->gid, ts, LP);
            new_message = (message_data*)tw_event_data(current_event);
            new_message->car.x_to_go = M->car.x_to_go;
            new_message->car.y_to_go = M->car.y_to_go;
            new_message->car.x_to_go_original = M->car.x_to_go_original;
            new_message->car.y_to_go_original = M->car.y_to_go_original;
            new_message->car.start_time = M->car.start_time;
            new_message->car.end_time = M->car.end_time;
            new_message->car.position = M->car.position;
            new_message->car.intention = M->car.intention;
            new_message->car.has_turned = M->car.has_turned;
            new_message->car.next_intersection = M->car.next_intersection;
            new_message->car.queue_location = M->car.queue_location;
            new_message->car.id = M->car.id;
            new_message->event_type = CAR_MOVES_FORWARD;
			new_message->saved_north_lead_car_will_enter = M->saved_north_lead_car_will_enter;
			new_message->saved_north_left_lead_car_will_enter = M->saved_north_left_lead_car_will_enter;
			new_message->saved_east_lead_car_will_enter = M->saved_east_lead_car_will_enter;
			new_message->saved_east_left_lead_car_will_enter = M->saved_east_left_lead_car_will_enter;
			new_message->saved_south_lead_car_will_enter = M->saved_south_lead_car_will_enter;
			new_message->saved_south_left_lead_car_will_enter = M->saved_south_left_lead_car_will_enter;
			new_message->saved_west_lead_car_will_enter = M->saved_west_lead_car_will_enter;
			new_message->saved_west_left_lead_car_will_enter = M->saved_west_left_lead_car_will_enter;
            tw_event_send(current_event);
			
        }

        break;

    case CAR_ENTERS_INTERSECTION:

        #ifdef DEBUG
        if (LP->gid == 0)

            printf("Autonomous car %d enters intersection with x: %d and y: %d and x original: %d and y original: %d with time: %f\n",
                   M->car.id, M->car.x_to_go, M->car.y_to_go,
                   M->car.x_to_go_original, M->car.y_to_go_original,
                   (tw_now(LP) - M->car.start_time));
        #endif

        // apply lock
        switch(M->car.intention) {

        case NS:
            SV->NS_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_left--;
                        if (SV->num_cars_north < 0) SV->num_cars_north = 0;
            break;
        case NR:
            SV->NR_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_left--;
                        if (SV->num_cars_north < 0) SV->num_cars_north = 0;
            break;
        case NL:
            SV->NL_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_left--;
                        if (SV->num_cars_north_left < 0) SV->num_cars_north_left = 0;
            break;
        case ES:
            SV->ES_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_east--;
                        if (SV->num_cars_east < 0) SV->num_cars_east = 0;
            break;
        case ER:
            SV->ER_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_east--;
                        if (SV->num_cars_east < 0) SV->num_cars_east = 0;
            break;
        case EL:
            SV->EL_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_east_left--;
                        if (SV->num_cars_east_left < 0) SV->num_cars_east_left = 0;
            break;
        case SS:
            SV->SS_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_south--;
                        if (SV->num_cars_south < 0) SV->num_cars_south = 0;
            break;
        case SR:
            SV->SR_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_south--;
                        if (SV->num_cars_south < 0) SV->num_cars_south = 0;
            break;
        case SL:
            SV->SL_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_south_left--;
                        if (SV->num_cars_south_left < 0) SV->num_cars_south_left = 0;
            break;
        case WS:
            SV->WS_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_west--;
                        if (SV->num_cars_west < 0) SV->num_cars_west = 0;
            break;
        case WR:
            SV->WR_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_west--;
                        if (SV->num_cars_west < 0) SV->num_cars_west = 0;
            break;
        case WL:
            SV->WL_lock = tw_now(LP) + INTERSECTION_CROSSING_TIME;
            SV->num_cars_north_west_left--;
                        if (SV->num_cars_west_left < 0) SV->num_cars_west_left = 0;
            break;

        }

        // send car to next intersection
        ts = tw_rand_exponential(LP->rng, TRAVEL_TIME_VARIATION)
             + MINIMUM_TRAVEL_TIME;
        current_event = tw_event_new(M->car.next_intersection, ts, LP);
        new_message = (message_data*)tw_event_data(current_event);
        new_message->car.x_to_go = M->car.x_to_go;
        new_message->car.y_to_go = M->car.y_to_go;
        new_message->car.x_to_go_original = M->car.x_to_go_original;
        new_message->car.y_to_go_original = M->car.y_to_go_original;
        new_message->car.start_time = M->car.start_time;
        new_message->car.end_time = M->car.end_time;
        new_message->car.position = M->car.position;
        new_message->car.intention = M->car.intention;
        new_message->car.has_turned = M->car.has_turned;
        new_message->car.next_intersection = M->car.next_intersection;
        new_message->car.queue_location = M->car.queue_location;
        new_message->car.id = M->car.id;
        new_message->event_type = CAR_ARRIVES;
		new_message->saved_north_lead_car_will_enter = M->saved_north_lead_car_will_enter;
		new_message->saved_north_left_lead_car_will_enter = M->saved_north_left_lead_car_will_enter;
		new_message->saved_east_lead_car_will_enter = M->saved_east_lead_car_will_enter;
		new_message->saved_east_left_lead_car_will_enter = M->saved_east_left_lead_car_will_enter;
		new_message->saved_south_lead_car_will_enter = M->saved_south_lead_car_will_enter;
		new_message->saved_south_left_lead_car_will_enter = M->saved_south_left_lead_car_will_enter;
		new_message->saved_west_lead_car_will_enter = M->saved_west_lead_car_will_enter;
		new_message->saved_west_left_lead_car_will_enter = M->saved_west_left_lead_car_will_enter;
        tw_event_send(current_event);

        break;

    }

}

void autonomous_traffic_intersection_reverse_eventhandler(
    intersection_state* SV,
    tw_bf* CV,
    message_data* M,
    tw_lp* LP
) {
    // Time warp starting time:
    tw_stime ts = 0.0;

    tw_stime departure_time;
    tw_stime lead_car_enters;
    
    // Current event:
    tw_event* current_event = NULL;

    // New message data:
    message_data* new_message = NULL;

    // Unknown time warp bit field:
    *(int*)CV = (int)0;

    tw_stime queue_wait_time;

    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {

    case CAR_ARRIVES:
        // Car reached its destination:
        if(M->car.y_to_go == 0 && M->car.x_to_go == 0) {
            SV->total_cars_finished--;
            g_total_time -= (M->car.end_time - M->car.start_time);
            break;
        }
        
        // Increment the total number of cars in this intersection:
        SV->total_cars_arrived--;

        // The car is too far south; have the car head up north:
        if(M->car.y_to_go > 0) {

            // Remove car from the south lane:
            SV->num_cars_south--;
                        if (SV->num_cars_south < 0) SV->num_cars_south = 0;
        }
        else if(M->car.y_to_go < 0) {

            // Add a car in the north lane:
            SV->num_cars_north_left--;
                        if (SV->num_cars_north < 0) SV->num_cars_north = 0;
        }
        else if(M->car.y_to_go == 0) {

            if(M->car.has_turned) {

                if(M->car.x_to_go > 0) {

                    // Add a car in the west lane:
                    SV->num_cars_north_west--;
                        if (SV->num_cars_west < 0) SV->num_cars_west = 0;

                }
                else if(M->car.x_to_go < 0) {

                    // Add a car in the east lane:
                    SV->num_cars_north_east--;
                        if (SV->num_cars_east < 0) SV->num_cars_east = 0;

                }

            }

            else {

                if(M->car.x_to_go > 0) {

                    if(M->car.y_to_go_original > 0) {
                        SV->num_cars_south--;
                        if (SV->num_cars_south < 0) SV->num_cars_south = 0;
                    } else {
                        SV->num_cars_north_left--;
                        if (SV->num_cars_north_left < 0) SV->num_cars_north_left = 0;
                    }

                }
                else if(M->car.x_to_go < 0) {

                    if(M->car.y_to_go_original > 0) {
                        SV->num_cars_south_left--;
                        if (SV->num_cars_south_left < 0) SV->num_cars_south_left = 0;
                    } else {
                        SV->num_cars_north_left--;
                        if (SV->num_cars_north < 0) SV->num_cars_north = 0;
                    }

                }
            }

        }

        break;

    case CAR_MOVES_FORWARD:
        M->car.queue_location--;

        // If car is at the front of the line:
        if (M->car.queue_location == 0) {
            switch (M->car.intention) {

            case NL:
                if (SV->SS_lock > tw_now(LP)) {
                    SV->north_left_lead_car_will_enter = 
                                       M->saved_north_left_lead_car_will_enter;
                }
                if (SV->SR_lock > tw_now(LP) && SV->SR_lock > departure_time) {
                    SV->north_left_lead_car_will_enter = 
                                       M->saved_north_left_lead_car_will_enter;
                } 
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    SV->north_left_lead_car_will_enter = 
                                       M->saved_north_left_lead_car_will_enter;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    SV->north_left_lead_car_will_enter = 
                                       M->saved_north_left_lead_car_will_enter;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    SV->north_left_lead_car_will_enter = 
                                       M->saved_north_left_lead_car_will_enter;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    SV->north_left_lead_car_will_enter = 
                                       M->saved_north_left_lead_car_will_enter;
                }
                break;
                
            case NR:
                if (SV->SL_lock > tw_now(LP)) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                }
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                }
                break;
                
            case NS:
                if (SV->SL_lock > tw_now(LP)) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                }
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                } 
                if (SV->WR_lock > tw_now(LP) && SV->WR_lock > departure_time) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    SV->north_lead_car_will_enter = 
                                       M->saved_north_lead_car_will_enter;
                }
                break;
                
            case EL:
                if (SV->SS_lock > tw_now(LP)) {
                    SV->east_left_lead_car_will_enter = 
                                       M->saved_east_left_lead_car_will_enter;
                }
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    SV->east_left_lead_car_will_enter = 
                                       M->saved_east_left_lead_car_will_enter;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    SV->east_left_lead_car_will_enter = 
                                       M->saved_east_left_lead_car_will_enter;
                } 
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    SV->east_left_lead_car_will_enter = 
                                       M->saved_east_left_lead_car_will_enter;
                } 
                if (SV->WR_lock > tw_now(LP) && SV->WR_lock > departure_time) {
                    SV->east_left_lead_car_will_enter = 
                                       M->saved_east_left_lead_car_will_enter;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    SV->east_left_lead_car_will_enter = 
                                       M->saved_east_left_lead_car_will_enter;
                }
                break;
                
            case ER:
                if (SV->SS_lock > tw_now(LP)) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                }
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                }
                break;
                
            case ES:
                if (SV->SS_lock > tw_now(LP)) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                }
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                } 
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                } 
                if (SV->NR_lock > tw_now(LP) && SV->NR_lock > departure_time) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    SV->east_lead_car_will_enter = 
                                       M->saved_east_lead_car_will_enter;
                }
                break;
                
            case SL:
                if (SV->ES_lock > tw_now(LP)) {
                    SV->south_left_lead_car_will_enter = 
                                       M->saved_south_left_lead_car_will_enter;
                }
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    SV->south_left_lead_car_will_enter = 
                                       M->saved_south_left_lead_car_will_enter;
                } 
                if (SV->NR_lock > tw_now(LP) && SV->NR_lock > departure_time) {
                    SV->south_left_lead_car_will_enter = 
                                       M->saved_south_left_lead_car_will_enter;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    SV->south_left_lead_car_will_enter = 
                                       M->saved_south_left_lead_car_will_enter;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    SV->south_left_lead_car_will_enter = 
                                       M->saved_south_left_lead_car_will_enter;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    SV->south_left_lead_car_will_enter = 
                                       M->saved_south_left_lead_car_will_enter;
                }
                break;
                
            case SR:
                if (SV->WS_lock > tw_now(LP)) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                }
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                }
                break;
                
            case SS:
                if (SV->NL_lock > tw_now(LP)) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                }
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                } 
                if (SV->ER_lock > tw_now(LP) && SV->ER_lock > departure_time) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                } 
                if (SV->WS_lock > tw_now(LP) && SV->WS_lock > departure_time) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                } 
                if (SV->WL_lock > tw_now(LP) && SV->WL_lock > departure_time) {
                    SV->south_lead_car_will_enter = 
                                       M->saved_south_lead_car_will_enter;
                }
                break;
                
            case WL:
                if (SV->NS_lock > tw_now(LP)) {
                    SV->west_left_lead_car_will_enter = 
                                       M->saved_west_left_lead_car_will_enter;
                }
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    SV->west_left_lead_car_will_enter = 
                                       M->saved_west_left_lead_car_will_enter;
                } 
                if (SV->SS_lock > tw_now(LP) && SV->SS_lock > departure_time) {
                    SV->west_left_lead_car_will_enter = 
                                       M->saved_west_left_lead_car_will_enter;
                } 
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    SV->west_left_lead_car_will_enter = 
                                       M->saved_west_left_lead_car_will_enter;
                } 
                if (SV->ES_lock > tw_now(LP) && SV->ES_lock > departure_time) {
                    SV->west_left_lead_car_will_enter = 
                                       M->saved_west_left_lead_car_will_enter;
                } 
                if (SV->ER_lock > tw_now(LP) && SV->ER_lock > departure_time) {
                    SV->west_left_lead_car_will_enter = 
                                       M->saved_west_left_lead_car_will_enter;
                }
                break;
                
            case WR:
                if (SV->EL_lock > tw_now(LP)) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                }
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                }
                break;
                
            case WS:
                if (SV->SS_lock > tw_now(LP)) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                }
                if (SV->SL_lock > tw_now(LP) && SV->SL_lock > departure_time) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                } 
                if (SV->SR_lock > tw_now(LP) && SV->SR_lock > departure_time) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                } 
                if (SV->EL_lock > tw_now(LP) && SV->EL_lock > departure_time) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                } 
                if (SV->NS_lock > tw_now(LP) && SV->NS_lock > departure_time) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                } 
                if (SV->NL_lock > tw_now(LP) && SV->NL_lock > departure_time) {
                    SV->west_lead_car_will_enter = 
                                       M->saved_west_lead_car_will_enter;
                }
                break;
            }

        }

        break;

    case CAR_ENTERS_INTERSECTION:
        // apply lock
        switch(M->car.intention) {

        case NS:
            SV->NS_lock = M->saved_NS_lock;
            break;
        case NR:
            SV->NR_lock = M->saved_NR_lock;
            break;
        case NL:
            SV->NL_lock = M->saved_NL_lock;
            break;
        case ES:
            SV->ES_lock = M->saved_ES_lock;
            break;
        case ER:
            SV->ER_lock = M->saved_ER_lock;
            break;
        case EL:
            SV->EL_lock = M->saved_EL_lock;
            break;
        case SS:
            SV->SS_lock = M->saved_SS_lock;
            break;
        case SR:
            SV->SR_lock = M->saved_SR_lock;
            break;
        case SL:
            SV->SL_lock = M->saved_SL_lock;
            break;
        case WS:
            SV->WS_lock = M->saved_WS_lock;
            break;
        case WR:
            SV->WR_lock = M->saved_WR_lock;
            break;
        case WL:
            SV->WL_lock = M->saved_WL_lock;
            break;

        }
			
		tw_rand_reverse_unif(LP->rng);

        break;

    }

}