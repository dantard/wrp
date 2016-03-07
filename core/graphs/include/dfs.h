//#ifndef _DFS_H_
//#define _DFS_H_
//#include "graphs_common.h"
//#include <boost/graph/depth_first_search.hpp>

//struct DFSVisitor : default_dfs_visitor {
//    DFSVisitor(std::vector<EdgeDescriptor> & ed) : edges(ed){

//    }

//    void discover_vertex(vertex_descriptor u, const graph_t & g){

//    }
//    void tree_edge(EdgeDescriptor e, const graph_t & g) {

//        if (edges.size()> 0){
//            int last_target = target(edges[edges.size()-1], g);
//            int new_source  = source(e, g);

//            if (last_target != new_source){
//                int count = edges.size();
//                for (int i = count - 1; i >=0; i--){
//                    EdgeDescriptor fwd_edge = edges[i];
//                    EdgeDescriptor backw_edge(target(fwd_edge,g), source(fwd_edge,g), NULL);
//                    edges.push_back(backw_edge);
//                    if (new_source == source(fwd_edge,g)){
//                        break;
//                    }
//                }
//            }
//        }
//        edges.push_back(e);
//    }

//    void examine_edge(EdgeDescriptor e, const graph_t & g) const {
//    }
//    void finish_edge(EdgeDescriptor e, const graph_t & g) const {
//    }
//    void back_edge(EdgeDescriptor e, const graph_t & g) const {
//    }
//    void forward_or_cross_edge(EdgeDescriptor e, const graph_t & g) const {
//    }

//private:
//    std::vector<EdgeDescriptor> & edges;

//};


//struct BFSVisitor : default_bfs_visitor {
//    BFSVisitor(std::vector<EdgeDescriptor> & ed) : edges(ed){

//    }

//    void discover_vertex(vertex_descriptor u, const graph_t & g){
//        std::cerr << "disc " << u << std::endl;
//    }
//    void tree_edge(EdgeDescriptor e, const graph_t & g) {
//        std::cerr << "tree " << e << std::endl;
//        if (edges.size()> 0){
//            int last_target = target(edges[edges.size()-1], g);
//            int new_source  = source(e, g);

//            if (last_target != new_source){
//                int count = edges.size();
//                for (int i = count - 1; i >=0; i--){
//                    EdgeDescriptor fwd_edge = edges[i];
//                    EdgeDescriptor backw_edge(target(fwd_edge,g), source(fwd_edge,g), NULL);
//                    edges.push_back(backw_edge);
//                    if (new_source == source(fwd_edge,g)){
//                        break;
//                    }
//                }
//            }
//        }
//        edges.push_back(e);

//    }
//    void examine_edge(EdgeDescriptor e, const graph_t & g) const {
//        std::cerr << "tree " << e << std::endl;
//    }
//    void finish_edge(EdgeDescriptor e, const graph_t & g) const {
//    }
//    void back_edge(EdgeDescriptor e, const graph_t & g) const {
//    }
//    void forward_or_cross_edge(EdgeDescriptor e, const graph_t & g) const {
//    }

//private:
//    std::vector<EdgeDescriptor> & edges;

//};

//int dfs_get_path(Matrix & m, int start_vertex, std::vector<vertex_descriptor> & path);
//int bfs_get_path(Matrix & m, int start_vertex, std::vector<vertex_descriptor> & path);
//int bfs_get_path(graph_t g, int start_vertex, std::vector<vertex_descriptor> & path);
//int graph_trim_path(const int num_of_vertices, const unsigned int mask, std::vector<vertex_descriptor> & path);

//#endif
