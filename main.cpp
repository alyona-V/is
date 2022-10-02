#include <iostream>
#include <list>
#include <set>
#include "structs.h"
#include "utils.h"
#include <chrono>
#include <string>
#include <cmath>
#include <fstream>

class BFS //breadth-first-search
{
public:
    Result find_path(Node start, Node goal, Map grid)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        int steps = 0;
        start.g = 0;
        std::list<Node> OPEN;
        OPEN.push_back(start);
        std::set<Node> CLOSED;
        CLOSED.insert(start);
        bool pathfound = false;
        while(!OPEN.empty() && !pathfound)
        {
           Node current = OPEN.front();
           OPEN.pop_front();
           steps++;
           auto neighbors = grid.get_neighbors(current);
           for(auto n:neighbors) {
               if (CLOSED.find(n) == CLOSED.end())
               {
                   n.g = current.g + 1;
                   n.parent = &(*CLOSED.find(current));
                   OPEN.push_back(n);
                   CLOSED.insert(n);
                   if(n == goal) {
                       result.path = reconstruct_path(n);
                       result.cost = n.g;
                       pathfound = true;
                       break;
                    }
                }
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = (std::chrono::high_resolution_clock::now() - time_now).count()/1e+9;
        return result;
    }
    std::list<Node> reconstruct_path(Node n)
    {
        std::list<Node> path;
        while(n.parent != nullptr)
        {
            path.push_front(n);
            n = *n.parent;
        }
        path.push_front(n);
        return path;
    }
};

class AStar
{
public:
    Result find_path(Node start, Node goal, Map grid, std::string metrictype="Octile", int connections=8, double hweight=1)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        Result result;
        int steps = 0;
        start.g = 0;
        start.h = count_h_value(start, goal, metrictype);
        start.f = start.g+start.h*hweight;
        std::list<Node> OPEN;
        OPEN.push_back(start);
        std::set<Node> CLOSED;
        CLOSED.insert(start);
        bool pathfound = false;
        while(!OPEN.empty() && !pathfound) {
            OPEN.sort([](const Node &a, const Node &b){
                if (a.f == b.f)
                    return a<b;
                return a.f <b.f;
            });
            Node current = OPEN.front();
            OPEN.pop_front();
            steps++;
            auto neighbors = grid.get_neighbors(current, connections);
            for (auto n: neighbors) {
                Node ff = *(CLOSED.find(n));
                if (n.i != current.i && n.j != current.j)
                    n.g = current.g + 1.4142;
                else
                    n.g = current.g + 1;
                n.h = count_h_value(n, goal, metrictype);
                n.f = n.g + hweight*n.h;
                if (CLOSED.find(n) == CLOSED.end()) {
                    n.parent = &(*CLOSED.find(current));
                    OPEN.push_back(n);
                    CLOSED.erase(ff);
                    CLOSED.insert(n);}
                if (n == goal) {
                    result.path = reconstruct_path(n);
                    result.cost = n.g;
                    pathfound = true;
                    break;
            }
            }
        }
        result.steps = steps;
        result.nodes_created = CLOSED.size();
        result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::high_resolution_clock::now() - time_now).count();
        return result;
    }
    static double count_h_value(Node current, Node goal, std::string metrictype = "Octile") {
        double dx = abs(current.i - goal.i);
        double dy = abs(current.j - goal.j);
        if (metrictype=="Octile"){
            double min = dx;
            if (min>dy){
                min = dy;
            }
            current.h = abs(dx - dy) + 1.44 * min;
        }
        else if(metrictype=="Euclidean"){
            current.h = sqrt(pow(dx, 2) + pow(dy, 2));
        }
        else if(metrictype=="Manhattan"){
            current.h = dx + dy;
        }
        return current.h;
            }
    std::list<Node> reconstruct_path(Node current)
    {
        std::list<Node>path;
        while(current.parent!=nullptr){
            path.push_front(current);
            current = *current.parent;
        }
        path.push_front(current);
        return path;
    }
        };

int main(int argc, char* argv[])
{
    for(int i=0; i<argc; i++)
       std::cout<<argv[i]<<"\n";
    if(argc<2)
    {
        std::cout << "Name of the input XML file is not specified."<<std::endl;
        return 1;
    }
    Loader loader;
    loader.load_instance(argv[1]);
    Result result;
    if(loader.algorithm == "BFS")
    {
        BFS bfs;
        result = bfs.find_path(loader.start, loader.goal, loader.grid);
    }
    else
    {
        if(loader.algorithm == "Dijkstra")
            loader.hweight = 0;
        AStar astar;
        result = astar.find_path(loader.start, loader.goal, loader.grid, loader.metrictype, loader.connections, loader.hweight);
        std::cout<<loader.metrictype<<" "<<loader.connections<<"\n";
    }
    //loader.grid.print(result.path);
    std::ofstream file;
    file.open("C:\\Users\\advor\\CLionProjects\\ic\\1.txt", std::ios::app);
    file<<"Instance 6 "<<"\n"<<loader.algorithm<<" "<<loader.hweight<<" "<<loader.metrictype<<" "<<loader.connections<<"\nCost: "<<result.cost<<"\nRuntime: "<<result.runtime
                               <<"\nSteps: "<<result.steps<<"\nNodes created: "<<result.nodes_created<<"\n"<<std::endl;
    file.close();
    std::cout<<"Cost: "<<result.cost<<"\nRuntime: "<<result.runtime
             <<"\nSteps: "<<result.steps<<"\nNodes created: "<<result.nodes_created<<std::endl;

    return 0;
}