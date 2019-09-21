#include "path_planning/dijkstra.h"

Dijkstra::Dijkstra(){

}

Dijkstra::~Dijkstra(){

}

void Dijkstra::solve(){
    auto curr_vtx(boost::vertex(flatIdx(source_.first,source_.second), *graph_));
    (*graph_)[curr_vtx].total_dist = .0;
    auto target_vtx(boost::vertex(flatIdx(target_.first,target_.second), *graph_));
    auto next(curr_vtx);
    std::queue<Vertex > unvisited_set;
    while((*graph_)[target_vtx].state != Visited){

        if((*graph_)[curr_vtx].state == Visited){
            curr_vtx = unvisited_set.front();
            unvisited_set.pop();
            continue;
        }

        (*graph_)[curr_vtx].state = Visited;

        auto curr_vtx_idx(flatIdx((*graph_)[curr_vtx].x,(*graph_)[curr_vtx].y));

        auto out_edges(boost::out_edges(curr_vtx, *graph_));

        auto e(out_edges.first);
        auto end_e(out_edges.second);

        auto opt_dist(std::numeric_limits<double>::max());
        Vertex v;
        for(;e != end_e; e++){
            v = boost::target(*e, *graph_);
            if((*graph_)[v].state == Occupied)
                continue;

            if((*graph_)[v].total_dist > (*graph_)[curr_vtx].total_dist + (*graph_)[*e].dist){
                (*graph_)[v].total_dist = (*graph_)[curr_vtx].total_dist + (*graph_)[*e].dist;
                (*graph_)[v].prev_idx = curr_vtx_idx;
            }

            unvisited_set.push(v);

            if((*graph_)[v].total_dist < opt_dist &&
                    (*graph_)[v].state == Unvisited){
                opt_dist = (*graph_)[v].total_dist;
                next = v;
            }
        }

        curr_vtx = next;
        boost::this_thread::sleep_for(boost::chrono::milliseconds{delay_});
//        std::cout << "target_vtx : ("<< (*graph_)[target_vtx].x << ","<< (*graph_)[target_vtx].y << ") ; " << flatIdx(dest.first,dest.second) << std::endl;
//        std::cout << "Current : (" << (*graph_)[curr_vtx].x << "," << (*graph_)[v].y << ")" << std::endl;
    }

}

void Dijkstra::reinit(){
    finished_ = false;
    for(size_t i(0); i < boost::num_vertices(*graph_); i++){
        auto v = boost::vertex(i, *graph_);
        if((*graph_)[v].state != Occupied)
            (*graph_)[v].state = Unvisited;
        (*graph_)[v].prev_idx = -1;
        (*graph_)[v].total_dist = std::numeric_limits<double>::max();
    }
}
