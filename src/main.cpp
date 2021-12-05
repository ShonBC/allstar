#include "../include/swarm_server.h"
#include <vector>

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");

    std::vector<std::vector<double>> input = {
        {1,1},
        {3,4},
        {5,5},
        {4,8}
    };
    SwarmServer swarm;
    // swarm.InitiateMap();
    swarm.AssignGoals(input);
}
