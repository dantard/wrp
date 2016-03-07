//#include <stdio.h>
//#include <boost/graph/dijkstra_shortest_paths.hpp>
//#include <boost/graph/prim_minimum_spanning_tree.hpp>
//#include <boost/graph/kruskal_min_spanning_tree.hpp>
//#include <boost/graph/adjacency_list.hpp>

//#include "include/graphs_common.h"

///* ALL functions return either  1. a values (if it makes sense); in case of error, the value -1 --OR--  2. the error code (0 is no error)  */

//Matrix graph_predecessor_map_to_adjacency_matrix(std::vector<vertex_descriptor> & p){
//    Matrix m(p.size(),p.size());
//    for (int i = 0; i < p.size(); i++){
//        if (i != p[i]){
//            m(i,p[i])=1;
//            m(p[i],i)=1;
//        }
//    }
//    return m;
//}

//void graph_matrix_multiply(Matrix &m1, Matrix &m2){
//    for (int i = 0; i< m1.size1(); i++){
//        for (int j = 0; j< m1.size2(); j++){
//            m2(i,j) = m1(i,j) * m2(i,j);
//        }
//    }
//}

//void graph_matrix_multiply(Matrix &m1, Matrix &m2, Matrix & m3){
//    m3.resize(m1.size1(), m2.size2());
//    for (int i = 0; i< m1.size1(); i++){
//        for (int j = 0; j< m1.size2(); j++){
//            m3(i,j) = m1(i,j) * m2(i,j);
//        }
//    }
//}

//graph_t graph_matrix_to_graph(Matrix & m){

//    std::vector<Edge> edges;
//    std::vector<double> weights;

//    for (int i = 0; i< m.size1(); i++){
//        for (int j = 0; j< m.size2(); j++){
//            if (fabs(m(i,j)) > 1e-6){
//                edges.push_back(Edge(i, j));
//                weights.push_back(m(i,j));
//                //  std::cerr << "Pushing back " << i <<"," << j << " with weight " << m(i,j) << std::endl;
//            }
//        }
//    }
//    graph_t g(&edges[0], (&edges[0]) + edges.size(), &weights[0], m.size1());
//    return g;
//}

//int graph_dijkstra_get_path(std::vector<vertex_descriptor> & p , int start, int goal, std::vector<vertex_descriptor> & path){
//    graph_traits< graph_t >::vertex_descriptor current = goal;
//    path.clear();
//    while(current!=start) {
//        path.push_back(current);
//        if (current == p[current]){
//            return 1;
//        }
//        current=p[current];
//    }
//    std::reverse(path.begin(),path.end());
//    return 0;
//}

//int graph_dijkstra_get_next(std::vector<vertex_descriptor> & p, int start, int goal){
//    if (start == goal){
//        return start;
//    }
//    std::vector<vertex_descriptor> path;
//    int err = graph_dijkstra_get_path(p, start, goal, path);
//    if (!err){
//        return path[0];
//    }else{
//        return -1;
//    }
//}

//double graph_dijkstra_get_value(std::vector<double> & weights, int dest){
//    return weights[dest];
//}

//int graph_dijkstra_compute(Matrix & m, int src, std::vector<vertex_descriptor> & p, std::vector<double> & w){
//    graph_t g = graph_matrix_to_graph(m);
//    vertex_descriptor s = vertex(src, g);
//    p.resize(num_vertices(g));
//    w.resize(num_vertices(g));
//    try{
//        dijkstra_shortest_paths(g, s,
//                                predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
//                                distance_map(boost::make_iterator_property_map(w.begin(), get(boost::vertex_index, g))));
//        return 0;
//    }catch(...){
//        return 1;
//    }
//}

//int graph_prim_compute(Matrix & m, std::vector<vertex_descriptor> & p){
//    graph_t g = graph_matrix_to_graph(m);
//    p.resize(num_vertices(g));

//    try{
//        /* compute the tree*/
//        prim_minimum_spanning_tree(g, &p[0]);

//        /* check if the tree is valid (if more than one vertex has not parent, is not valid) */
//        int sum = 0;
//        for (int i = 0; i< p.size(); i++){
//            sum += (p[i] == i ? 1 : 0);
//        }
//        return sum > 1;
//    }catch(...){
//        return 0;
//    }
//}

//int graph_kruskal_compute(Matrix & m, std::vector<vertex_descriptor> & p){
//    graph_t g = graph_matrix_to_graph(m);
//    std::vector < EdgeDescriptor > spanning_tree;

//    p.resize(m.size1());

//    try{
//        /* compute the tree*/
//        kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));
//        if (spanning_tree.size() == m.size1() - 1){
//            for (int i = 0; i< m.size1(); i++){
//                p[i] = i;
//            }
//            for (std::vector < EdgeDescriptor >::iterator ei = spanning_tree.begin();
//                 ei != spanning_tree.end(); ++ei) {
//                std::cerr << *ei <<  p.size() <<std::endl;


//                int i = source(*ei, g);
//                int j = target(*ei, g);

//                //std::cerr << i << " " << j << " "<<  p[i] << " " << p[j] << std::endl;

//                if (p[i] == i){
//                    p[i] = j;
//                }else if (p[j] == j){
//                    p[j] = i;
//                }else{
//                    assert(0);
//                }
//            }
//            return 0;
//        }
//        return 1;
//    }catch(...){
//        return 0;
//    }
//}

///* Returns the size of the spanning tree or -1 if error */
//int graph_kruskal_compute(Matrix & m, Matrix & out){
//    graph_t g = graph_matrix_to_graph(m);

//    out.resize(m.size1(),m.size2(), false);
//    out.reset();

//    std::vector < EdgeDescriptor > spanning_tree;
//    property_map < graph_t, edge_weight_t >::type weight = get(edge_weight, g);

//    try{
//        /* compute the tree*/
//        kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));

//        for (std::vector < EdgeDescriptor >::iterator ei = spanning_tree.begin();
//             ei != spanning_tree.end(); ++ei) {
//            int i = source(*ei, g);
//            int j = target(*ei, g);
//            out(i,j) = weight[*ei];
//            out(j,i) = weight[*ei];

//            //  std::cout << source(*ei, g) << " <--> " << target(*ei, g)
//            //  << " with weight of " << weight[*ei]
//            //  << std::endl;

//        }
//        return spanning_tree.size();

//    }catch(...){
//        return -1;
//    }
//}



//int graph_is_network_connected(Matrix & m){
//    std::vector<vertex_descriptor> p;
//    return !graph_prim_compute(m, p);
//}

//void graph_print_graph(std::vector<vertex_descriptor> & p, std::string s){
//    std::cerr << s << " Size:" << p.size() <<std::endl;
//    for (std::size_t i = 0; i != p.size(); ++i){
//        if (p[i] != i){
//            std::cerr << "parent[" << i << "] = " << p[i] << std::endl;
//        }else{
//            std::cerr << "parent[" << i << "] = no parent" << std::endl;
//        }
//    }
//}

//std::vector<double> graph_create_unary_weights(std::vector<vertex_descriptor> & p){
//    std::vector<double> weigths;
//    for (int i = 0; i< p.size(); i++){
//        p.push_back(1.0D);
//    }
//    return weigths;
//}

//graph_t graph_predecessor_map_to_graph(std::vector<vertex_descriptor> & p, std::vector<double> weights){
//    std::vector<Edge> edges;
//    for (int i = 0; i< p.size(); i++){
//        if (i!=p[i]){
//            edges.push_back(Edge(i, p[i]));
//        }
//    }
//    graph_t g(&edges[0], (&edges[0]) + edges.size(), &weights[0], p.size());
//    return g;
//}


//void graph_matrix_log(Matrix &m, Matrix &out){
//    out.resize(m.size1(),m.size2());
//    for (int i = 0; i< m.size1(); i++){
//        for (int j = 0; j< m.size2(); j++){
//            if (m(i,j) > 1e-6){
//                out(i,j) = log(m(i,j));
//            }else{
//                out(i,j) = m(i,j);
//            }
//        }
//    }
//}

//void graph_matrix_math_op(Matrix &m, Matrix &out, double (*op)(double)){
//    out.resize(m.size1(),m.size2());
//    for (int i = 0; i< m.size1(); i++){
//        for (int j = 0; j< m.size2(); j++){
//            if (m(i,j) > 1e-6){
//                out(i,j) = op(m(i,j));
//            }else{
//                out(i,j) = m(i,j);
//            }
//        }
//    }
//}


//void graph_print_matrix(Matrix & m, std::string name){
//    std::cerr << name << ":" << std::endl;
//    for (int i = 0; i< m.size1(); i++){
//        for (int j = 0; j< m.size2(); j++){
//            fprintf(stderr,"%3.2f\t", m(i,j));
//        }
//        fprintf(stderr,"\n");
//    }
//    fprintf(stderr,"\n");
//}
//void graph_print_path(std::vector<vertex_descriptor> & p, std::string s, int src){
//    std::cerr << s;
//    if (src!=-1){
//        std::cerr << ": " << src;
//    }
//    for (int i = 0; i< p.size() ; i++){
//        std::cerr << " -> " << p[i];
//    }
//    std::cerr << std::endl;

//}
//void graph_print_weight(std::vector<double> & p, std::string s, int from){
//    std::cerr << s << ": " << std::endl;
//    for (int i = 0; i< p.size() ; i++){
//        std::cerr << "w["<< from <<" -> " << i <<"] = " << (double)p[i] << std::endl;
//    }

//}

//int graph_prim_compute(Matrix &m, Matrix &out){
//    graph_t g = graph_matrix_to_graph(m);
//    std::vector<vertex_descriptor> p;
//    graph_prim_compute(m,p);
//    out = graph_predecessor_map_to_adjacency_matrix(p);
//    graph_matrix_multiply(m, out);
//}

//int graph_prob_compute(Matrix & m, Matrix & next, Matrix & a0) {
//    Matrix a1(m.size1(),m.size2());

//    a0.resize(m.size1(),m.size2());
//    next.resize(m.size1(),m.size2());

//    int i, j, k, size = m.size1();
//    for (i = 0; i < size; i++) {
//        for (j = 0; j < size; j++) {
//            next(i,j) = j;
//            if (i == j) {
//                a0(i,j) = 1000.0;
//            } else {
//                a0(i,j) = m(i,j)*10.0;
//            }
//        }
//    }

//    for (k = 0; k < size; k++) {
//        for (i = 0; i < size; i++) {
//            for (j = 0; j < size; j++) {
//                if (a0(i,k) * a0(k,k) * a0(k,j) / 1000000.0 > a0(i,j)) {
//                    next(i,j) = next(i,k);
//                    a1(i,j) = a0(i,k) * a0(k,k) * a0(k,j) / 1000000.0;
//                } else {
//                    a1(i,j) = a0(i,j);
//                }
//            }
//        }
//        a0 = a1;
//    }
//    return 0;
//}

//int graph_prob_get_path(Matrix & next, Matrix & a0, int src, int dest, std::vector<vertex_descriptor> & path) {
//    if (a0(src,dest) > 1e-6){
//        int res = src;
//        path.clear();
//        while (res != dest) {
//            res = next(res,dest);
//            path.push_back(res);
//        }
//        return 0;
//    }else{
//        return 1;
//    }
//}

//int graph_prob_get_next(Matrix & next, Matrix & a0, int src, int dest) {
//    if (a0(src,dest) > 1e-6){
//        return next(src,dest);
//    }else{
//        return -1;
//    }
//}

///* returns first hop (if exist) or -1 if not. If path is passed, the function returns the whole path */
//double graph_prob_get_value(Matrix & a0, int src, int dest) {
//    return a0(src,dest)/10.0;
//}


//void graph_matrix_simmetrize_max(Matrix & m, Matrix & m2){
//    m2.resize(m.size1(),m.size2());
//    m2.clear();
//    for (int i = 0; i< m.size1(); i++){
//        for (int j = 0; j< m.size2(); j++){
//            m2(i,j) = m(i,j) > m(j,i) ? m(i,j): m(j,i);
//        }
//    }
//}

//void graph_matrix_simmetrize_min(Matrix & m, Matrix & m2){
//    m2.resize(m.size1(),m.size2());
//    m2.clear();
//    for (int i = 0; i< m.size1(); i++){
//        for (int j = 0; j< m.size2(); j++){
//            m2(i,j) = m(i,j) < m(j,i) ? m(i,j): m(j,i);
//        }
//    }

//}

//void graph_matrix_upper_triangle(Matrix & m, Matrix & m2){
//    m2.resize(m.size1(),m.size2());
//    m2.clear();
//    for (int i = 0; i< m.size1(); i++){
//        for (int j = i; j< m.size2(); j++){
//            m2(i,j) = m(i,j);
//        }
//    }
//}

//void graph_matrix_lower_triangle(Matrix & m, Matrix & m1){
//    Matrix m2(m.size1(),m.size2());
//    for (int i = 0; i< m.size1(); i++){
//        for (int j = i; j< m.size2(); j++){
//            m2(j,i) = m(j,i);
//        }
//    }
//    m1 = m2;
//}
//void test(Matrix & m, Matrix & m2){
//    Matrix m1(5,5);
//    graph_print_matrix(m1,"m1-1");
//    m = m1;
//    m1(0,0) = 1001;
//    graph_print_matrix(m1,"m1-2");
//    graph_print_matrix(m,"m");
//}


//void graph_get_neighbors(Matrix & m, int vertex, std::vector<vertex_descriptor> & neighbors){
//    neighbors.clear();
//    for (int i = 0; i<m.size2(); i++){
//        if (m(vertex, i) > 1e-6) {
//            neighbors.push_back(i);
//        }
//    }
//}
//int graph_is_network_connected(int node_id, Matrix & LQM, std::vector<char> MASK){

//    static std::vector<double> w;
//    static std::vector<vertex_descriptor> v;

//    int disc;
//    graph_dijkstra_compute(LQM,node_id,v,w);
//    for (int i = 0; i < LQM.size1(); i++){
//        if (MASK[i] != 2 && w[i]> 1e3){
//            disc = 1;
//            break;
//        }
//    }
//    return !disc;
//}
