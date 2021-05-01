#pragma once

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <vector>
#include <cmath>
using namespace std;
//Create node class
class node {
  public:
    int nodeID;
    float Xposition;
    float Yposition;
    float desirability;
    float normalizedDesirability;
    float accumulatedND;
};

//Create ant class
class ant {
public:
    //HARD CODED
    int path[20][2];
    float totalDistanceTraveled;
};


//Generate random number between 2 floats
float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

//Generate random nodes
std::vector<node> create_node_list(float gridlengthX, float gridlengthY, int number_of_nodes)
{
srand(time(NULL));

float x, y;
std::vector<node> list_of_nodes;

for (int i = 0; i<number_of_nodes; i++)
{
node new_node;
x = RandomFloat(gridlengthX, -gridlengthX);
y = RandomFloat(gridlengthY, -gridlengthY);

new_node.nodeID = i;
new_node.Xposition = x;
new_node.Yposition = y;

list_of_nodes.push_back(new_node);
}

return list_of_nodes;
}

//Find distance between two nodes
float distance(node a, node b)
{
float d = sqrt( pow((a.Xposition-b.Xposition),2) + pow((a.Yposition-b.Yposition),2) );

return d;
}

//Find total attractiveness
float findDesirability(float distance, float distancePower, float pheromoneStrength, float pheromonePower)
{
float des = pow(1 / distance, distancePower) * pow(pheromoneStrength, pheromonePower);

return des;
}