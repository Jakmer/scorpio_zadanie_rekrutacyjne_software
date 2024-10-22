#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <list>
#include <cstdint>
#include <cmath>
#include <iostream>

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

private:
    std::vector<std::list<Vertice>> createGraph(std::vector<int8_t> &mapList)
    {
        int width = 50;
        int height = 50;

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                Vertice v(j, i, mapList[j + i * width]);
                vertices.push_back(v);
                adjacencyList.emplace_back();
            }
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
