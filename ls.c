/******************************************************************************\
* Link state routing protocol.                                                 *
\******************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "routing-simulator.h"

typedef struct {
  cost_t link_cost[MAX_NODES];
  int version;
} link_state_t;

// Message format to send between nodes.
typedef struct {
  link_state_t ls[MAX_NODES];
} data_t;

// State format.
typedef struct {
  link_state_t ls[MAX_NODES];
} state_t;

// Handler for the node to allocate and initialize its state.
void *init_state() {
  state_t *state = (state_t *)calloc(1, sizeof(state_t));
  for(node_t nodex = get_first_node(); nodex <= get_last_node(); nodex = get_next_node(nodex)) {
    for(node_t nodey = get_first_node(); nodey <= get_last_node(); nodey = get_next_node(nodey)) {
      state->ls[nodex].version = 0;
      state->ls[nodex].link_cost[nodey] = COST_INFINITY;
      if (nodex == nodey) {  
        state->ls[nodex].link_cost[nodey] = 0;
      } 
    }
  }  
  return state;
}

//Node_t swap function for Bubble Sort
void swap(node_t* xp, node_t* yp) {
    node_t temp = *xp;
    *xp = *yp;
    *yp = temp;
}
 
// A function to implement bubble sort
void bubbleSort(state_t *state, node_t queue[], int n, cost_t dist[MAX_NODES]) {
    int i, j;
    for(i = 0; i < n - 1; i++)
        // Last i elements are already in place
        for (j = 0; j < n - i - 1; j++)
            if (dist[queue[j]] < dist[queue[j + 1]])
                swap(&queue[j], &queue[j + 1]);
}


/*
Dijkstra algorithm:
  - Choses node based on the smallest distance available in queue.
  - Sets route based on previous next hop (If node is a neighbor and doesn't have next hop, it himself is the next hop).
  - Calculates all distances for neighbors of chosen node.
*/
void dijkstra(state_t *state) {
  int size = 0;
  node_t queue[MAX_NODES];
  node_t next_hop[MAX_NODES];
  cost_t dist[MAX_NODES];
  node_t current = get_current_node(); 
  for(node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
    queue[size] = node;
    next_hop[size] = -1;
    dist[node] = get_link_cost(node);
    size++;
  }
  bubbleSort(state, queue, size, dist);
  size--;
  while(size > 0) {
    bubbleSort(state, queue, size, dist);
    node_t chosen = queue[size-1];
    size--;
    node_t nhop = next_hop[chosen];
    if (next_hop[chosen] == -1 && get_link_cost(chosen) != COST_INFINITY) {
      nhop = chosen; 
    }
    if (nhop == -1) break;;
    set_route(chosen, nhop, dist[chosen]);
    for(int i = 0; i < size; i++) {
      if(state->ls[chosen].link_cost[queue[i]] != COST_INFINITY) {
        cost_t alt = COST_ADD(dist[chosen], state->ls[chosen].link_cost[queue[i]]);
        if (alt < dist[queue[i]]) {
          dist[queue[i]] = alt;
          next_hop[queue[i]] = nhop;
        }
      }
    }
  }
  for(node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
    if(next_hop[node] == -1 && dist[node] == COST_INFINITY) {
      if(node != current) {
        set_route(node, -1, COST_INFINITY);
      }
    }
  }
}

// Function to send the current node state to it's neighbors
void send_state(state_t *state) {
  message_t msg;
  msg.size = sizeof(data_t);
  msg.data = malloc(msg.size);
  data_t *data = (data_t*)msg.data;
  memcpy(data->ls, state->ls, sizeof(state->ls));
  for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)){
    if(get_link_cost(node) != COST_INFINITY && node != get_current_node()) {
      send_message(node, msg);
    }
  }
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
  state_t *state = (state_t*)get_state();
  state->ls[get_current_node()].link_cost[neighbor] = new_cost;
  state->ls[get_current_node()].version = state->ls[get_current_node()].version + 1;
  dijkstra(state);
  send_state(state);

}


// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
  state_t *state = (state_t*)get_state();
  data_t *data = (data_t*)message.data;
  int chng = 0;
  for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)){
    if (state->ls[node].version < data->ls[node].version) {
      memcpy(&state->ls[node], &data->ls[node], sizeof(link_state_t));
      chng = 1;
    }
  }
  if (chng) {
    dijkstra(state);
    send_state(state);
  }
}


