#include "ats.h"

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

void autonomous_traffic_intersection_startup(intersection_state* SV,
                                             tw_lp* LP) {
    tw_stime ts;
    tw_event* current_event;
    message_data* new_message;

    // Initialize the number of cars arriving into the intersection:
    SV->num_cars_in_south = 0;
    SV->num_cars_in_west = 0;
    SV->num_cars_in_north = 0;
    SV->num_cars_in_east = 0;

    SV->num_cars_total = 0;

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
        ts = tw_rand_exponential(LP->rng, g_mean_service) + 20; //TODO 20?

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
    *(int*) CV = (int) 0;
    
    // Handle the events defined in the "events" enumeration:
    switch(M->event_type) {

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
