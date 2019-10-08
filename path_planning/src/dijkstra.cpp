#include "path_planning/dijkstra.h"

Dijkstra::Dijkstra(){

}

Dijkstra::~Dijkstra(){

}

void Dijkstra::solve(){
    reinit();
    while(!finished_){
        solvePerStep();
    }

}

void Dijkstra::solvePerStep(){
    if((*graph_)[curr_vtx_].state == Visited){
        curr_vtx_ = unvisited_set_.front();
        unvisited_set_.pop();
        return;
    }

    (*graph_)[curr_vtx_].state = Visited;

    auto curr_vtx_idx(flatIdx((*graph_)[curr_vtx_].x, (*graph_)[curr_vtx_].y));

    auto out_edges(boost::out_edges(curr_vtx_, *graph_));

    auto e(out_edges.first);
    auto end_e(out_edges.second);

    auto opt_dist(std::numeric_limits<double>::max());
    Vertex v;
    for(;e != end_e; e++){
        v = boost::target(*e, *graph_);
        if((*graph_)[v].state == Occupied)
            continue;

        if((*graph_)[v].total_dist > (*graph_)[curr_vtx_].total_dist + (*graph_)[*e].dist){
            (*graph_)[v].total_dist = (*graph_)[curr_vtx_].total_dist + (*graph_)[*e].dist;
            (*graph_)[v].prev_idx = curr_vtx_idx;
        }

        unvisited_set_.push(v);

        if((*graph_)[v].total_dist < opt_dist &&
                (*graph_)[v].state == Unvisited){
            opt_dist = (*graph_)[v].total_dist;
            next_ = v;
        }
    }

    curr_vtx_ = next_;
    boost::this_thread::sleep_for(boost::chrono::milliseconds{delay_});

    if((*graph_)[target_vtx_].state == Visited){
        boost::mutex::scoped_lock lk(finished_mtx_);
        finished_ = true;
    }

//    std::cout << "target_vtx : ("<< (*graph_)[target_vtx].x << ","<< (*graph_)[target_vtx].y << ") ; " << flatIdx(dest.first,dest.s) << std::endl;
//    std::cout << "Current : (" << (*graph_)[curr_vtx].x << "," << (*graph_)[v].y << ")" << std::endl;
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

    curr_vtx_ = boost::vertex(flatIdx(getX(source_), getY(source_)), *graph_);
    (*graph_)[curr_vtx_].total_dist = .0;
    target_vtx_ = boost::vertex(flatIdx(getX(target_), getY(target_)), *graph_);
    next_ = curr_vtx_;

    // clear the queue
    std::queue<Vertex > empty;
    std::swap(unvisited_set_, empty);
}
