#include <iostream>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>
using namespace std;

#include "Ant_Colony_Setup.h"

using namespace std;


int main()
{
//simulation parameters
float dstPower = 4;
float pheromonePower = 1;
float pheromoneIntensity = 10;
float initialPheromoneIntensity = 10;
float evaporationRate = 0.3f;
int numAnts = 20;
int totalAnts = 500;
int numberOfRuns = totalAnts/numAnts;

//Define node parameters
float gridlengthX = 10.0;
float gridlengthY = 10.0;
int number_of_nodes = 20;

//create list of nodes
vector<node> node_list = create_node_list(gridlengthX, gridlengthY, number_of_nodes);

//Test Node list
// for (int i = 0; i<node_list.size(); i++)
// {
//     std::cout << "X coordinate: " << node_list[i].Xposition << "  "
//     << "Y coordinate: " << node_list[i].Yposition << "  "
//     << "ID: " << node_list[i].nodeID << '\n';
// }

//Find weighting matrices

//Create array of distances
float distanceArray[number_of_nodes][number_of_nodes];

//Create array of pheromone strengths
float pheromoneArray[number_of_nodes][number_of_nodes];

//populate matrices
for (int i = 0; i<node_list.size(); i++)
{
    for (int j = 0; j<node_list.size(); j++)
    {   
        distanceArray[i][j] = distance(node_list[i],node_list[j]);
        pheromoneArray[i][j] = initialPheromoneIntensity;
        // cout << distanceArray[i][j] << " ";
    }
    // cout << "\n";
}

//create vector of ants

//run the simulation
for (int x = 0; x<numberOfRuns; x++)
{   

    //create ants
    std::vector<ant> vecAnts;
    for (int y = 0; y<numAnts; y++)
    {   
        ant antY;
        antY.totalDistanceTraveled = 0;
        vecAnts.push_back(antY);
    }





    
    //iterate over numAnts
    for (int y = 0; y<numAnts; y++)
    {   
        //starting ID for ant
        //int startingID = rand() % node_list.size();
        int startingID = 0;

        //create vector of remaining nodes to visit
        vector<node> remainingNodeList = node_list;
        node firstNode = remainingNodeList[startingID];
        node currentNode = firstNode;

        //get first node ID and erase it from potentials
        //try 1
        vector<node>::iterator it = remainingNodeList.begin() + firstNode.nodeID;
        
        remainingNodeList.erase(it);
        for (int i = 0; i<remainingNodeList.size(); i++)
        {
            cout << remainingNodeList[i].nodeID;
        }
        cout << "\n";
        cout <<startingID;
        cout << "\n";

        //try 2
        //vector<node>::iterator it = node_list.begin() + startingID;
        //remainingNodeList.erase(it);
        //cout << remainingNodeList.size() << std::endl;

        //define temporary desirability vector
        //vector<float> tempDesirability;

        // cout << "wtf";


        //iterate over all points
        
        for (int i = 0; i<number_of_nodes; i++)
        {
            //cout << remainingNodeList.size();
            for (int j = 0; j<remainingNodeList.size(); j++)
            {   

                node potentialNode = remainingNodeList[j];

                if (currentNode.nodeID == potentialNode.nodeID)
                {
                //set desirability
                remainingNodeList[j].desirability = 0;
                } else {
                    //Find distance to next nodes
                    float dst = distanceArray[currentNode.nodeID][potentialNode.nodeID];

                    //Find pheromone strenght to next nodes
                    float pheromoneStrength = pheromoneArray[currentNode.nodeID][potentialNode.nodeID];

                    //set desirability
                    remainingNodeList[j].desirability = findDesirability(dst, dstPower, pheromoneStrength, pheromonePower);
                    //cout << remainingNodeList.size();
                }
                //cout << remainingNodeList[j].desirability << "  " << distanceArray[currentNode.nodeID][j] << "  " << pheromoneArray[currentNode.nodeID][j] << "  " << currentNode.nodeID << "  " << potentialNode.nodeID << "\n";

            }

            //calculate total desirability
            float totalDesirability = 0;

            for (int j = 0; j<remainingNodeList.size(); j++)
            {
                totalDesirability += remainingNodeList[j].desirability;
            }
            //cout << totalDesirability << "\n";
            float accumulation = 0;
            //calculate normalized desirability & accumulated normalized desirability
            for (int j = 0; j<remainingNodeList.size(); j++)
            {
                remainingNodeList[j].normalizedDesirability = remainingNodeList[j].desirability / totalDesirability;
                accumulation += remainingNodeList[j].normalizedDesirability;
                remainingNodeList[j].accumulatedND = accumulation;
            }

            double randNum = RandomFloat(0, 1);
            node chosenNode;
            //cout << remainingNodeList.size();
            //Find the chosenNode
            int j = 0;
            while (remainingNodeList[j].accumulatedND < randNum)
            {
                j++;
            }
            chosenNode = remainingNodeList[j];

            //cout << totalDesirability << "\n";

            //update path
            vecAnts[y].path[i][0] = currentNode.nodeID;
            vecAnts[y].path[i][1] = chosenNode.nodeID;

            //update distance travelled
            vecAnts[y].totalDistanceTraveled += distanceArray[currentNode.nodeID][chosenNode.nodeID];

            //change current node
            currentNode = chosenNode;

            //remove chosenNode from remainingNodeList
            vector<node>::iterator it = remainingNodeList.begin() + j;
            remainingNodeList.erase(it);

            //cout << remainingNodeList.size() << " remaining node list size\n";
            //cout << i << " i iterator\n";
            cout << remainingNodeList.size();
        }

        //cout << number_of_nodes << "number of nodes\n";
        //Travel to beginning
        //may need to change to (number_of_nodes - 1)
        vecAnts[y].path[number_of_nodes - 1][0] = currentNode.nodeID;
        vecAnts[y].path[number_of_nodes - 1][1] = firstNode.nodeID;
        vecAnts[y].totalDistanceTraveled += distanceArray[currentNode.nodeID][firstNode.nodeID];

        currentNode = firstNode;
    }


    //find best trail
    float min_dst = vecAnts[0].totalDistanceTraveled;
    auto bestPath = vecAnts[0].path;
    for (int y = 1; y<numAnts; y++)
    {
        if (min_dst > vecAnts[y].totalDistanceTraveled)
        {
        min_dst = vecAnts[y].totalDistanceTraveled;
        bestPath = vecAnts[y].path;
        }
    }

    //adjust pheromone trails
    for (int y = 0; y<node_list.size(); y++)
    {
        for (int z = 0; z<node_list.size(); z++)
        {   
            //Evaporate
            pheromoneArray[y][z] -= evaporationRate;
        }

        //Strengthen best trail
        //cout << bestPath[y] [0] << "  " << bestPath[y] [1] << '\n';
        pheromoneArray[ bestPath[y] [0] ]     [ bestPath[y] [1] ] += pheromoneIntensity;
        pheromoneArray[ bestPath[y] [1] ]     [ bestPath[y] [0] ] += pheromoneIntensity;
    }
}
std::cout.flush();
for (int x = 0; x<node_list.size(); x++)
{
    for (int y = 0; y<node_list.size(); y++)
    {
        cout << pheromoneArray[x][y] << "  ";
    }
    cout << '\n';
}
return 0;
}