//#include <stdio.h>

//#include "include/dfs.h"
//#include "include/graphs_common.h"
//#include "../../utils/bits.h"




//int dfs_get_path(Matrix & m, int start_vertex, std::vector<vertex_descriptor> & path){
//    graph_t g = graph_matrix_to_graph(m);
//    std::vector<EdgeDescriptor> edges;

//    DFSVisitor dfs(edges);
//    depth_first_search(g, visitor(dfs).root_vertex(start_vertex));

//    path.clear();
//    path.push_back(start_vertex);
//    for (int j = 0; j < edges.size() ; j++){
//        path.push_back(target(edges[j],g));
//    }

//    if (path.size() == 1){
//        return 1;
//    }

//    /* check if the path actully exists */
//    for (int j = 0; j < path.size()-1 ; j++){
//        if (m(path[j], path[j+1])<1e-6 && m(path[j+1], path[j])<1e-6){
//            path.clear();
//            return 1;
//        }
//    }
//    return 0;

//}


//int dfs_get_path(graph_t g, int start_vertex, std::vector<vertex_descriptor> & path){
//    std::vector<EdgeDescriptor> edges;

//    DFSVisitor dfs(edges);
//    depth_first_search(g, visitor(dfs).root_vertex(start_vertex));

//    path.clear();
//    path.push_back(start_vertex);
//    for (int j = 0; j < edges.size() ; j++){
//        path.push_back(target(edges[j],g));
//    }

//    if (path.size() == 1){
//        return 1;
//    }

//    //    /* check if the path actully exists */
//    //    for (int j = 0; j < path.size()-1 ; j++){
//    //        if (m(path[j], path[j+1])<1e-6 && m(path[j+1], path[j])<1e-6){
//    //            path.clear();
//    //            return 1;
//    //        }
//    //    }
//    return 0;

//}




//int bfs_get_path(Matrix & m, int start_vertex, std::vector<vertex_descriptor> & path){
//    graph_t g = graph_matrix_to_graph(m);
//    std::vector<EdgeDescriptor> edges;

//    BFSVisitor bfs(edges);
//    breadth_first_search(g, vertex(start_vertex, g), visitor(bfs));

//    path.clear();
//    path.push_back(start_vertex);
//    for (int j = 0; j < edges.size() ; j++){
//        path.push_back(target(edges[j],g));
//    }

//}



//int graph_trim_path(const int num_of_vertices, const unsigned int mask, std::vector<vertex_descriptor> & path){
//    /* For every node of the net*/
//    for (int i = 0; i< num_of_vertices; i++){

//        /* Look for intervals of the same node: [0 1 2 3 2 1 4 5] means that the
//         * token goes for an excursion to node 2 and 3. We want to eliminate
//         * that part of the path if there are nodes involved in the BC */

//        int begin = -1, end = -1;
//        for (int j = 0; j< path.size(); j++){
//            if (begin == -1){
//                if (i == path[j]){
//                    begin = j;
//                }
//            }else{
//                if (i == path[j]){
//                    end = j;
//                }
//            }
//        }
//        if (begin!=-1 && end != -1){
//            int keep = 0;
//            for (int j = begin+1; j< end; j++){
//                keep |= CHECK_BIT(mask, path[j]);
//            }
//            if (!keep){
//                for (int j = begin+1; j<= end; j++){
//                    path[j] = -1;
//                }
//            }
//        }
//    }

//    /* Eliminate the repetitions at the end of the path including
//     * only the nodes strictly necessary to touch those indicated
//     * by the mask
//     */
//    unsigned int number = 0;
//    for (int i = 0; i< path.size(); i++){
//        if (path[i]==-1){
//            continue;
//        }
//        unsigned int current = number & mask;
//        if(current != mask){
//            SET_BIT(number, path[i]);
//        }else{
//            path[i] = -1;
//        }
//    }

//    /* Eliminate the steps with -1 values */
//    int i = 0;
//    while (i < path.size()){
//        if (path[i] == -1){
//            path.erase(path.begin()+i);
//        }else{
//            i++;
//        }
//    }
//}


