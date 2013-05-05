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

// VPs per PE?
tw_lpid g_vp_per_proc = 0; // set in main

// LPs per PE?
tw_lpid g_cells_per_vp_x = MAP_WIDTH / NUM_VP_X;
tw_lpid g_cells_per_vp_y = MAP_HEIGHT / NUM_VP_Y;
tw_lpid g_cells_per_vp = (MAP_WIDTH / NUM_VP_X) * (MAP_HEIGHT / NUM_VP_Y);

// Average service time?
tw_stime g_mean_service = 1.0;

// QUESTION: lookahead?
tw_stime g_lookahead = 1.0;

tw_stime g_full_cycle_duration = 2*LEFT_TURN_LIGHT_DURATION + 2*GREEN_LIGHT_DURATION;

// QUESTION: mult?
// Why are all these static?
tw_stime g_mult = 1.6;

// Number of LPs per PE
unsigned int g_nlp_per_pe = 8;

// TODO: figure out what this means
int g_traffic_start_events = 15;

// TODO: figure out what this means
int g_optimistic_memory = 65536; // 64 KB

// rate for timestamp exponential distribution
tw_stime g_mean = 1.0;

// Holds the total cars initiated and completed for statistics
unsigned long long g_total_cars = 0;
unsigned long long g_cars_finished = 0;
unsigned long long g_total_time = 0;
unsigned long long g_average_time = 0;

tw_lpid num_cells_per_kp = 0;
tw_lpid vp_per_proc = 0;

// Function roles:
tw_lptype traffic_light_lps[] = {
    {
        (init_f) traffic_light_intersection_startup,
        (event_f) traffic_light_intersection_eventhandler,
        (revent_f) traffic_light_intersection_reverse_eventhandler,
        (final_f) intersection_statistics_collectstats,
        (map_f) cell_mapping_lp_to_pe,
        sizeof(intersection_state)
    },
    { 0 },
};

tw_lptype autonomous_traffic_lps[] = {
    {
        (init_f) autonomous_traffic_intersection_startup,
        (event_f) autonomous_traffic_intersection_eventhandler,
        (revent_f) autonomous_traffic_intersection_reverse_eventhandler,
        (final_f) intersection_statistics_collectstats,
        (map_f) cell_mapping_lp_to_pe,
        sizeof(intersection_state)
    },
    { 0 },
};

//Command Line Arguments
const tw_optdef model_opts[] = {
    TWOPT_GROUP("Traffic Model"),
    TWOPT_UINT("autonomous", autonomous,
               "1 for autonomous vehicles, 0 for traditional intersections"),
    TWOPT_END(),
};

// Main Function:
int main(int argc, char* argv[]) {
    // QUESTION: I don't know what this is or why it is set so low
    g_tw_ts_end = 10000; // ROSS default is 100000.0
    g_tw_gvt_interval = 512; // ROSS default is 16
    g_tw_mblock = 8; // ROSS default to 16
    
    tw_opt_add(model_opts);
    tw_init(&argc, &argv);

    if (g_lookahead > 1.0)
        tw_error(TW_LOC, "Lookahead must be less than 1.0\n"); // QUESTION: why

    // Reset mean based on lookahead. QUESTION: why?
    g_mean = g_mean - g_lookahead;

    // Set lookahead
    g_tw_lookahead = g_lookahead;

    // TODO: clean this up?
    g_nlp_per_pe = (MAP_WIDTH * MAP_HEIGHT) / (tw_nnodes() * g_tw_npe);
    g_tw_events_per_pe = (g_mult * g_nlp_per_pe * g_traffic_start_events) 
                         + g_optimistic_memory;
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

	printf("Number of cars finised %lld\n", g_cars_finished);
	printf("Number of cars that arrived %lld\n", g_total_cars);
	printf("Average travel time %llu\n", g_average_time);
	
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
        tw_error(TW_LOC, "index (%llu) beyond g_tw_nlp (%llu) range \n", index,
                 g_tw_nlp);
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
        tw_error(TW_LOC, "index (%llu) beyond g_tw_nlp (%llu) range \n", index,
                 g_tw_nlp);

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
                
                // MUST COME AFTER!! DO NOT PRE-INCREMENT ELSE KPID is WRONG!!
                local_lp_count++;

                if (kpid >= g_tw_nkp)
                    tw_error(TW_LOC, "Attempting to mapping a KPid (%llu) for \
                        Global LPid %llu that is beyond g_tw_nkp (%llu)\n",
                        kpid, lpid, g_tw_nkp );

                tw_lp_onpe(cell_mapping_to_local_index(lpid), g_tw_pe[0],
                           lpid);

                if (g_tw_kp[kpid] == NULL)
                    tw_kp_onpe(kpid, g_tw_pe[0]);

                tw_lp_onkp(g_tw_lp[cell_mapping_to_local_index(lpid)],
                           g_tw_kp[kpid]);
                if (autonomous)
                    tw_lp_settype(cell_mapping_to_local_index(lpid),
                                  &autonomous_traffic_lps[0]);
                else
                    tw_lp_settype(cell_mapping_to_local_index(lpid),
                                  &traffic_light_lps[0]);
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


void intersection_statistics_collectstats(intersection_state* SV, tw_lp* LP) {
	g_total_cars += SV->total_cars_arrived;
	g_cars_finished += SV->total_cars_finished;
	if(g_cars_finished > 0)
		g_average_time = g_total_time/g_cars_finished;
}
