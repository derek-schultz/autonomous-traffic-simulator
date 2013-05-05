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
        car_type cars[4 * MAX_LANES_PER_DIRECTION]
                     [4 * MAX_LANES_PER_DIRECTION];
                     
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
