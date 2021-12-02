#include "../include/swarm_server.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    SwarmServer swarm;
    swarm.InitiateMap();
}
