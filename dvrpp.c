/******************************************************************************\
* Distance vector routing protocol with reverse path poisoning.                *
\******************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "routing-simulator.h"

// Message format to send between nodes.

typedef struct {
} data_t;

// State format.
typedef struct {
  node_t curr;
  node_t next_hop[MAX_NODES];
  cost_t dist[MAX_NODES][MAX_NODES];
} state_t;

// Handler for the node to allocate and initialize its state.
void *init_state() {
  state_t *state = (state_t *)calloc(1, sizeof(state_t));
  state->curr = get_current_node();
  for(node_t nodex = get_first_node(); nodex <= get_last_node(); nodex = get_next_node(nodex)) {
    for(node_t nodey = get_first_node(); nodey <= get_last_node(); nodey = get_next_node(nodey)) {
      state->dist[nodex][nodey] = COST_INFINITY;
      if (nodex == nodey) state->dist[nodex][nodey] = 0;
    }
    state->next_hop[nodex] = -1;
  } 
  return state;
}

/*
Bellman-Ford algorithm:
  - Finds least cost neighbor to reach a certain node
    and sets the route to the node with selected neighbor as next hop.
  W/Path Poisining:
    - If a path through a neighbor is defined the cost of the path backwards
      is defined as infinite, thus poisining it,
*/
void bellford(state_t *state) {
  int chng = 0;
  for(node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
    if (node != state->curr) {
      cost_t min = COST_INFINITY;
      node_t chneigh = -1;
      for (node_t neigh = get_first_node(); neigh <= get_last_node(); neigh = get_next_node(neigh)){
        cost_t cv;
        if ((cv = get_link_cost(neigh)) != COST_INFINITY && neigh != state->curr) {
          if (COST_ADD(cv, state->dist[neigh][node]) < min) {
            min = COST_ADD(cv, state->dist[neigh][node]);
            chneigh = neigh;
          }
        }
      }
      if(min != state->dist[state->curr][node]) {
        state->dist[state->curr][node] = min;
        state->next_hop[node] = chneigh;
        set_route(node, chneigh, min);
        chng = 1;
      }
    }
  }
  if (chng) {
    message_t msg;
    cost_t tempdv[MAX_NODES];
    for (node_t neigh = get_first_node(); neigh <= get_last_node(); neigh = get_next_node(neigh)){
      if (get_link_cost(neigh) != COST_INFINITY && neigh != state->curr) {
        memcpy(tempdv, state->dist[state->curr], sizeof(state->dist[state->curr]));
        for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
          if(state->next_hop[node] == neigh) {
            tempdv[node] = COST_INFINITY;
          }
        }
        msg.data = (void*)tempdv;
        msg.size = sizeof(msg.data);
        send_message(neigh, msg);
      }
    }
  }
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
  state_t *state = (state_t*)get_state();
  //Algoritmo Bellmand-Ford
  bellford(state);
}

// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
  state_t *state = (state_t*)get_state();
  cost_t *uplist =  (cost_t*)message.data;
  memcpy(state->dist[sender], uplist, message.size);
  bellford(state);
}
