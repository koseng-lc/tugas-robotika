#include "path_planning/solver.h"

Solver::Solver()
    : graph_(new Graph(CELL_ROWS * CELL_COLS))
    , ogm_(new OGM(CELL_COLS, CELL_ROWS))
    , finished_(false){

    initGraph();

}

Solver::~Solver(){

}

void Solver::initGraph(){
    std::cout << "Initializing Graph..." << std::endl;
    constexpr auto sqrt_2 = std::sqrt(2);
    for(int y(0); y < CELL_ROWS; y++){
        for(int x(0); x < CELL_COLS; x++){
//            Vertex curr = boost::add_vertex(VertexProp{x,y,Unvisited}, *graph_);
            Vertex curr = boost::vertex(flatIdx(x,y),*graph_);
            (*graph_)[curr].x = x;
            (*graph_)[curr].y = y;
            (*graph_)[curr].state = Unvisited;
            (*graph_)[curr].prev_idx = -1;
            (*graph_)[curr].total_dist = std::numeric_limits<double>::max();

            if(x+1 < CELL_COLS && y-1 >= 0){
//                Vertex nb1 = boost::add_vertex(VertexProp{x+1,y-1,Unvisited}, *graph_);
                Vertex nb1 = boost::vertex(flatIdx(x+1,y-1), *graph_);
                boost::add_edge(curr, nb1, EdgeProp{sqrt_2}, *graph_);
            }


            if(x+1 < CELL_COLS){
//                Vertex nb2 = boost::add_vertex(VertexProp{x+1,y,Unvisited}, *graph_);
                Vertex nb2 = boost::vertex(flatIdx(x+1,y), *graph_);
                boost::add_edge(curr, nb2, EdgeProp{1.0}, *graph_);
            }


            if(x+1 < CELL_COLS && y+1 < CELL_ROWS){
//                Vertex nb3 = boost::add_vertex(VertexProp{x+1,y+1,Unvisited}, *graph_);
                Vertex nb3 = boost::vertex(flatIdx(x+1,y+1), *graph_);
                boost::add_edge(curr, nb3, EdgeProp{sqrt_2}, *graph_);
            }


            if(y+1 < CELL_ROWS){
//                Vertex nb4 = boost::add_vertex(VertexProp{x,y+1,Unvisited}, *graph_);
                Vertex nb4 = boost::vertex(flatIdx(x,y+1), *graph_);
                boost::add_edge(curr, nb4, EdgeProp{1.0}, *graph_);
            }

//            auto e = boost::edge(curr, nb1, *graph_);

        }
    }
    std::cout << " > Num of vertices : " << boost::num_vertices(*graph_) << std::endl;
    std::cout << " > Num of edges : " << boost::num_edges(*graph_) << std::endl;
}
