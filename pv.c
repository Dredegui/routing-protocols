/******************************************************************************\
* Path vector routing protocol.                                                *
\******************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "routing-simulator.h"

// Message format to send between nodes.

typedef struct {
  cost_t distancev[MAX_NODES];
  node_t pathVector[MAX_NODES][MAX_NODES];
} data_t;

// State format.
typedef struct {
  node_t curr;
  cost_t dist[MAX_NODES][MAX_NODES];
  node_t path_vector[MAX_NODES][MAX_NODES][MAX_NODES]; 
} state_t;

// Handler for the node to allocate and initialize its state.
void *init_state() {
  state_t *state = (state_t *)calloc(1, sizeof(state_t));
  state->curr = get_current_node();
  for(node_t nodex = get_first_node(); nodex <= get_last_node(); nodex = get_next_node(nodex)) {
    for(node_t nodey = get_first_node(); nodey <= get_last_node(); nodey = get_next_node(nodey)) {
      state->dist[nodex][nodey] = COST_INFINITY;
      state->path_vector[nodex][nodey][0] = -1;
      if (nodex == nodey) {
        state->dist[nodex][nodey] = 0;
        if (nodex == state->curr) {
          state->path_vector[nodex][nodey][0] = nodex;
          state->path_vector[nodex][nodey][1] = -1;
        }
      }
    }
  }  
  return state;
}

/*
Prints the given Path Vector
*/
void printPath(node_t *temp) {
  int i = 0;
  printf("PATH START:\n");
  while (temp[i] != -1 && i < 10)
  {
    printf("node%d: %d ", i, temp[i]);
    i++;
  }
  printf("end node: %d", temp[i]);
  printf("PATH END:\n"); 
}

/*
Checks if Path is valid:
  - Iterates the path vector checking if the current node is
  already inside it.
  - If it is returns Invalid.
  - Else returns valid.
*/
int valPath(node_t *temp) {
  printf("[valPath]\n");
  node_t curr = get_current_node();
  int i = 0;  
  while(temp[i] != -1 && temp && i < MAX_NODES) {
    printf("[valPath] temp[i]: %d, i: %d\n", temp[i], i);
    if(temp[i] == curr) return 0;
    i++;
  }
  return 1;
}

/*
Updates distance vector and path vector.
Sets route.
*/
void upPath(state_t *state, node_t neigh, node_t node, cost_t min) {
  printf("[upPath]\n");
  state->dist[state->curr][node] = min;
  set_route(node, neigh, min);
  node_t *path_vector = state->path_vector[state->curr][node];
  if (neigh == -1) {
    path_vector[0] = -1;
    return;
  }
  node_t *temp = (state->path_vector)[neigh][node];
  // Possible problem next is null is path is singular
  int i = 0;
  path_vector[i] = state->curr;
  while (path_vector[i] != -1 && i < MAX_NODES) {
    printf("Path node: %d, value: %d\n", i, path_vector[i]);
    path_vector[i+1] = temp[i];
    i++;
  }
  printf("i: %d: i-1:%d\n",path_vector[i], path_vector[i-1]);
}

/*
Bellman-Ford algorithm:
  - Finds least cost neighbor to reach a certain node
    and sets the route to the node with selected neighbor as next hop.
  W/Path Vector:
    - If the path isn't valid even if the cost is lower 
      doesn't acknowledge that route in the algorithm. 
*/
void bellford(state_t *state) {
  int chng = 0;
  for(node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)) {
    if (node != state->curr) {
      cost_t min = COST_INFINITY;
      printf("min: %d\n", min);
      node_t chneigh = -1;
      for (node_t neigh = get_first_node(); neigh <= get_last_node(); neigh = get_next_node(neigh)){
        cost_t cv;
        if ((cv = get_link_cost(neigh)) != COST_INFINITY && neigh != state->curr) {
          printf("curr: %d | neighbor: %d | c(x, v): %d | Dv[y]:%d\n", get_current_node(), neigh, cv, state->dist[neigh][node]);
          // printPath(state->path_vector[neigh][node]);
          if (COST_ADD(cv, state->dist[neigh][node]) < min && valPath(state->path_vector[neigh][node])) {
            min = COST_ADD(cv, state->dist[neigh][node]);
            chneigh = neigh;
          }
        }
      }
      if(min != state->dist[state->curr][node]) {
        upPath(state, chneigh, node, min);
        chng = 1;
      }
    }
  }
  if (chng) {
    message_t msg;
    msg.data = malloc(sizeof(data_t));
    data_t *data = (data_t*)msg.data;
    memcpy(data->distancev, state->dist[state->curr], sizeof(state->dist[state->curr]));
    memcpy(data->pathVector, state->path_vector[state->curr], sizeof(state->path_vector[state->curr]));
    printPath(data->pathVector[0]);
    msg.size = sizeof(data_t);
    for (node_t node = get_first_node(); node <= get_last_node(); node = get_next_node(node)){
      if(get_link_cost(node) != COST_INFINITY && node != state->curr) {
        send_message(node, msg);
      }
    }
    printf("Sent Msg\n");
  }
}

// Notify a node that a neighboring link has changed cost.
void notify_link_change(node_t neighbor, cost_t new_cost) {
  state_t *state = (state_t*)get_state();
  printf("new cost: %d\n", new_cost);
  //Algoritmo Bellmand-Ford
  bellford(state);
}

// Receive a message sent by a neighboring node.
void notify_receive_message(node_t sender, message_t message) {
  printf("Received Msg\n");
  state_t *state = (state_t*)get_state();
  data_t *data =  (data_t*)message.data;
  printPath(data->pathVector[0]);
  memcpy(state->dist[sender], data->distancev, sizeof(state->dist[sender]));
  memcpy(state->path_vector[sender], data->pathVector, sizeof(state->path_vector[sender]));
  bellford(state);
}
