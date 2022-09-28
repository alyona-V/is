#include <iostream>
#include <list>
#include <set>
#include "structs.h"
#include "utils.h"
#include <chrono>
#include <string>
#include <cmath>
#include <algorithm>

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
{ ClosedList CLOSED;
    OpenList OPEN;
public:
    Result find_path(Node start, Node goal, Map grid, std::string metrictype="Euclidian", int connections=8, double hweight=1)
    {
       auto time_now = std::chrono::high_resolution_clock::now();
       Result result;
        start.g = 0;
        start.f = 0;
        OPEN.addNode(start);
        while (true) {
            auto current = OPEN.getMin();
            CLOSED.addClose(current);
            OPEN.popMin();
            auto neig = grid.get_neighbors(current);
            for (auto b: neig) {

                if (!CLOSED.inClose(b.i, b.j)) {
                    b.h = count_h_value(b, goal, "Euclidean");
                    if((abs((b.i-current.i)==1)) && (abs((b.j-current.j)==1)))
                    {
                        b.g = current.g + sqrt(2);
                    }
                    b.g = current.g + 1;
                    b.f = b.g + hweight * b.h;
                    b.parent = CLOSED.getPointer(current.i, current.j);
                    OPEN.addNode(b);}}
                    if (current.i == goal.i and current.j == goal.j) {
                        CLOSED.addClose(current);
                        result.path = reconstruct_path(current);
                        result.cost = current.g;
                        break;
                    }
                }
                result.nodes_created = CLOSED.getSize() + OPEN.getSize();
                result.steps = CLOSED.getSize();
                result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(
                        std::chrono::high_resolution_clock::now() - time_now).count();
                return result;
            }
            static double count_h_value(Node current, Node goal, std::string metrictype = "Euclidean") {
                double dx = abs(goal.i - current.i);
                double dy = abs(goal.j - current.j);
                if (metrictype=="Octile"){ // добавили диагональную метрику
                    double min = dx;
                    if (min>dy){
                        min = dy;
                    }
                    current.h = abs(dx - dy) + 1.44 * min;
                }
                else if(metrictype=="Euclidean"){ // добавили евклидовскую(?) метрику
                    current.h = sqrt(dx*dx+dy*dy);
                }
                else if(metrictype=="Chebyshev"){ // добавили метрику Чебышева(или у, я не знаю)
                    double max = dx;
                    if(dy>max){
                        max = dy;
                    }
                    current.h = max;
                }
                else{ // это метрика Манхеттена
                    current.h = dx + dy;
                }
                return current.h;
            }
    std::list<Node> reconstruct_path(Node current)
    {
        std::list<Node>path;
        while(current.parent!=nullptr){
            path.push_front(current); // добавляем каждого соседа в начало пути
            current = *current.parent; // сосед - указатель на родителя соседа
        }
        path.push_front(current); // добавляем соседа без родителя в начало пути
        return path; // возвращаем путь
    }
        };

int main(int argc, char* argv[]) //argc - argumnet counter, argv - argument values
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
    BFS bfs;
    result = bfs.find_path(loader.start, loader.goal, loader.grid);
    std::cout<<"BFS";
    std::cout<<"Cost: "<<result.cost<<"\nRuntime: "<<result.runtime
             <<"\nSteps: "<<result.steps<<"\nNodes created: "<<result.nodes_created<<std::endl;
    if(loader.algorithm == "Dijkstra")
            loader.hweight = 0;
    AStar astar;
    result = astar.find_path(loader.start, loader.goal, loader.grid, loader.metrictype, loader.connections, loader.hweight);


    //loader.grid.print(result.path);
    std::cout<<"a*";
    std::cout<<"Cost: "<<result.cost<<"\nRuntime: "<<result.runtime
    <<"\nSteps: "<<result.steps<<"\nNodes created: "<<result.nodes_created<<std::endl;
    return 0;
}