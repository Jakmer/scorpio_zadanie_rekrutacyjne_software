#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "autonomy_simulator/SetGoal.h"
#include "autonomy_simulator/RoverPose.h"
#include <vector>
#include <queue>
#include <list>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <std_msgs/UInt8.h>

struct Vertice
{
  int x;
  int y;
  int height;

  Vertice(int x, int y, int height) : x(x), y(y), height(height) {}
};

class Graph
{
public:
    Graph(std::vector<int8_t> &mapList)
    {
        adjacencyList = createGraph(mapList);
    }
    ~Graph()
    {}

    std::vector<std::list<Vertice>> adjacencyList;
    void printGraph() const {
        for (size_t i = 0; i < adjacencyList.size(); ++i) {
            const Vertice& v = vertices[i];
            std::cout << "Vertice (" << v.x << ", " << v.y << ", " << v.height << "): ";
            for (const Vertice& neighbor : adjacencyList[i]) {
                std::cout << "(" << neighbor.x << ", " << neighbor.y << ", " << neighbor.height << ") ";
            }
            std::cout << std::endl;
        }
    }

    std::vector<std_msgs::UInt8> getShortestPath(const autonomy_simulator::RoverPose &pose, const autonomy_simulator::SetGoal &goal)
    {
        int start_x = pose.x;
        int start_y = pose.y;
        int direction = pose.orientation;
    
        int end_x = goal.x;
        int end_y = goal.y;

        return dijkstra(start_x, start_y, direction, end_x, end_y);
    }

private:

std::vector<std_msgs::UInt8> dijkstra(int start_x, int start_y, int direction, int end_x, int end_y)
{
    int width = 50;
    int height = 50;
    int startIndex = start_x + start_y * width;
    int endIndex = end_x + end_y * width;

    std::vector<int8_t> distances(vertices.size(), std::numeric_limits<int8_t>::max());
    std::vector<bool> visited(vertices.size(), false);
    std::vector<int> previous(vertices.size(), -1);
    std::vector<int> orientations(vertices.size() -1);
    std::priority_queue<std::pair<int8_t, int>, std::vector<std::pair<int8_t, int>>, std::greater<std::pair<int8_t, int>>> pq;

    distances[startIndex] = 0;
    orientations[startIndex] = 0;
    pq.push(std::make_pair(0, startIndex));

    while (!pq.empty()) {
        int currentIndex = pq.top().second;
        pq.pop();

        if (visited[currentIndex]) {
            continue;
        }
        visited[currentIndex] = true;

        for (const Vertice& neighbor : adjacencyList[currentIndex]) {
            int neighborIndex = neighbor.x + neighbor.y * width;

            int edgeWeight = 1;

            int moveDirection = getMoveDirection(currentIndex, neighborIndex, orientations[currentIndex]);

            if (moveDirection == -1) {
                continue;
            }

            if (moveDirection != orientations[currentIndex]) {
                edgeWeight = 2;
            }

            if (distances[currentIndex] + edgeWeight < distances[neighborIndex]) {
                distances[neighborIndex] = distances[currentIndex] + edgeWeight;
                previous[neighborIndex] = currentIndex; 
                pq.push(std::make_pair(distances[neighborIndex], neighborIndex));
                orientations[neighborIndex] = moveDirection;
            }
        }
    }

    std::vector<std_msgs::UInt8> moves;

    return moves;
}

int getMoveDirection(int fromIndex, int toIndex, int currentDirection) {

    int from_x = fromIndex % 50;
    int from_y = fromIndex / 50;
    int to_x = toIndex % 50;
    int to_y = toIndex / 50;

    if(currentDirection%2 == 0)
    {
        if(from_x == to_x)
        {
           return currentDirection; 
        }
        else if(from_x < to_x)
        {
            return 1;
        }
        else {
          return 0;
        }
        // TO BE FINISHED 
    }
    else {
    
    }

    return -1;
}

    std::vector<std::list<Vertice>> createGraph(std::vector<int8_t> &mapList)
    {
        int width = 50;
        int height = 50;

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                Vertice v(j, i, mapList[j + i * width]);
                vertices.push_back(v);
                adjacencyList.emplace_back();
                std::cout<<std::setw(4)<<v.height;
            }
            std::cout<<std::endl;
        }

        // Create edges
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int currentIndex = j + i * width;

                if (i > 0) {
                    int neighborIndex = j + (i - 1) * width;
                    
                    if(abs(vertices[neighborIndex].height - vertices[currentIndex].height) <= 10)
                    {
                        adjacencyList[currentIndex].push_back(vertices[neighborIndex]);
                    }
                }
                if (i < height - 1) {
                    int neighborIndex = j + (i + 1) * width;

                    if(abs(vertices[neighborIndex].height - vertices[currentIndex].height) <= 10)
                    {
                        adjacencyList[currentIndex].push_back(vertices[neighborIndex]);
                    }
                }
                if (j > 0) {
                    int neighborIndex = (j - 1) + i * width;

                    if(abs(vertices[neighborIndex].height - vertices[currentIndex].height) <= 10)
                    {
                        adjacencyList[currentIndex].push_back(vertices[neighborIndex]);
                    }
                }
                if (j < width - 1) {
                    int neighborIndex = (j + 1) + i * width;

                    if(abs(vertices[neighborIndex].height - vertices[currentIndex].height) <= 10)
                    {
                        adjacencyList[currentIndex].push_back(vertices[neighborIndex]);
                    }
                }
            }
        }

        return adjacencyList;        
    }

    std::vector<Vertice> vertices;
};

#endif // GRAPH_HPP
