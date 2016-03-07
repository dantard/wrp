#ifndef GRAPHS_COMMON_H_
#define GRAPHS_COMMON_H_

#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <stack>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/depth_first_search.hpp>


#define CHECK_BIT(array, bit) ((array >> bit) & 1)
#define SET_BIT(array, bit)   (array |= (1 << bit))
#define CLEAR_BIT(array, bit) (array &= ~(1 << bit))

using namespace boost;
typedef adjacency_list < vecS, vecS, directedS,
property<vertex_distance_t, double>, property < edge_weight_t, double > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef std::pair<int, int> Edge;
typedef std::vector<vertex_descriptor> vdesc;
typedef graph_traits<graph_t>::edge_descriptor EdgeDescriptor;

class BitArray : public std::vector<int> {
public:
    BitArray(int n): std::vector<int>(n,0) {

    }
    void print(std::string text){
        printf("%s\n", text.c_str());
        for (int i = 0; i< size() ; i++){
            printf("%s", at(i)?"X":"0");
        }
        printf("\n");
    }
    void clear(){}

    int count(){
        int sum_of_elems = 0;
        for (int i = 0; i< size(); i++){
            sum_of_elems += at(i);
        }
        return sum_of_elems;
    }
    void set(int i){
        at(i) = 1;
    }
    void unset(int i){
        at(i) = 0;
    }
    void reset(){
        for (int i = 0; i< size(); i++){
            at(i) = 0;
        }
    }
};
class Stack : public std::stack<int> {
public:
    int pop(){
        int val = top();
        std::stack<int>::pop();
        return val;
    }
};

class NodeSet : public std::vector<vertex_descriptor> {
public:
    void print(std::string name = "Nodes", int src = -1){
        std::cerr << name;
        if (src!=-1){
            std::cerr << ": " << src;
        }
        for (int i = 0; i< size() ; i++){
            std::cerr << " " << at(i);
        }
        std::cerr << std::endl;
    }

    vertex_descriptor rightmost(){
        return at(size()-1);
    }

};

class Path : public NodeSet {
public:
    void trim(const int num_of_vertices, const unsigned int mask){
        /* For every node of the net*/
        for (int i = 0; i< num_of_vertices; i++){

            /* Look for intervals of the same node: [0 1 2 3 2 1 4 5] means that the
             * token goes for an excursion to node 2 and 3. We want to eliminate
             * that part of the path if there are nodes involved in the BC */

            int begin = -1, end = -1;
            for (int j = 0; j< size(); j++){
                if (begin == -1){
                    if (i == at(j)){
                        begin = j;
                    }
                }else{
                    if (i == at(j)){
                        end = j;
                    }
                }
            }
            if (begin!=-1 && end != -1){
                int keep = 0;
                for (int j = begin+1; j< end; j++){
                    keep |= CHECK_BIT(mask, at(j));
                }
                if (!keep){
                    for (int j = begin+1; j<= end; j++){
                        at(j) = -1;
                    }
                }
            }
        }

        /* Eliminate the repetitions at the end of the path including
         * only the nodes strictly necessary to touch those indicated
         * by the mask
         */
        unsigned int number = 0;
        for (int i = 0; i< size(); i++){
            if (at(i)==-1){
                continue;
            }
            unsigned int current = number & mask;
            if(current != mask){
                SET_BIT(number, at(i));
            }else{
                at(i) = -1;
            }
        }

        /* Eliminate the steps with -1 values */
        int i = 0;
        while (i < size()){
            if (at(i) == -1){
                erase(begin()+i);
            }else{
                i++;
            }
        }
    }
};


class Matrix:  public boost::numeric::ublas::matrix<double> {
public :
    Matrix(int i, int j): matrix<double>(i,j){
        clear();
    }
    Matrix(): matrix<double>(){
        clear();
    }

    Matrix & randomize(){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (i!=j){
                    at_element(i,j) = double(rand()) / double(RAND_MAX);
                }else{
                    at_element(i,j) = 0;
                }
            }
        }
        return *this;
    }

    int serialize(char * matrix){
        int cnt = 0;
        for (int i = 0; i< size(); i++){
            for (int j = 0; j< size(); j++){
                matrix[cnt++] = at_element(i,j);
            }
        }
        return cnt;
    }

    int serialize_no_diagonal(char * matrix){
        int cnt = 0;
        for (int i = 0; i< size(); i++){
            for (int j = 0; j< size(); j++){
                if (i!=j){
                    matrix[cnt++] = at_element(i,j);
                }
            }
        }
        return cnt;
    }
    int deserialize(char * matrix, int size){
        resize(size,size);
        deserialize(matrix);
    }
    int deserialize(char * matrix){
        reset();
        int cnt = 0;
        for (int i = 0; i< size(); i++){
            for (int j = 0; j< size(); j++){
                at_element(i,j) = matrix[cnt++];
            }
        }
        return cnt;
    }

    bool initialized(){
        return size() > 0;
    }

    int deserialize_no_diagonal(char * matrix){
        reset();
        int cnt = 0;
        for (int i = 0; i< size(); i++){
            for (int j = 0; j< size(); j++){
                if (i!=j){
                    at_element(i,j) = matrix[cnt++];
                }
            }
        }
        return cnt;
    }

    int serialize_sparse(char * matrix){
        int cnt = 2, this_row;
        for (int i = 0; i< size(); i++){
            this_row = 0;
            matrix[cnt++] = 0x20 + i;
            for (int j = 0; j< size(); j++){
                if (!is_zero(i,j)){
                    matrix[cnt++] = j;
                    matrix[cnt++] = at_element(i,j);
                    this_row = 1;
                }
            }
            cnt = this_row > 0 ? cnt : cnt - 1;
        }
        (*((short *) matrix)) = (short) cnt;
        return cnt;
    }
    void deserialize_sparse(char * matrix, int size){
        resize(size, size);
        deserialize_sparse(matrix);
    }

    void deserialize_sparse(char * matrix){
        int cnt = 2, row = 0;
        short size = (*((short *) matrix));

        reset();

        while (cnt < size - 1){
            if (matrix[cnt] >= 0x20){
                row = matrix[cnt++] - 0x20;
            }
            at_element(row, matrix[cnt++]) = matrix[cnt++];
        }
    }

    int serialize_sparse_2(char * matrix){
        int cnt = 2, this_row;
        for (int i = 0; i< size(); i++){
            this_row = 0;
            matrix[cnt++] = -i;
            for (int j = i; j< size(); j++){
                if (!(is_zero(i,j) && is_zero(j,i))){
                    matrix[cnt++] = j;
                    matrix[cnt++] = at_element(i,j);
                    matrix[cnt++] = at_element(j,i);
                    this_row = 1;
                }
            }
            cnt = this_row > 0 ? cnt : cnt - 1;
        }
        (*((short *) matrix)) = (short) cnt;
        return cnt;
    }

    void deserialize_sparse_2(char * matrix){
        int cnt = 2, row = 0;
        short size = (*((short *) matrix));

        reset();
        while (cnt < size - 2){
            if (matrix[cnt] < 0 || cnt == 2){
                row = -matrix[cnt++];
            }
            int col = matrix[cnt++];
            at_element(row, col) = matrix[cnt++];
            at_element(col, row) = matrix[cnt++];
        }
    }


    bool is_zero(int i, int j){
        return (fabs(at_element(i,j))< 1e-6);
    }

    int sparse_count(){
        int count = 0;
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (!is_zero(i,j)){
                    count++;
                }
            }
        }
        return count;
    }

    Matrix & select_column(int column){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (j != column){
                    at_element(i,j) = 0;
                }
            }
        }
        return *this;
    }

    void replace_column(Matrix & m, int column){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (j == column){
                    at_element(i,j) =  m(i,j);
                }
            }
        }
    }

    void replace_non_zero(Matrix & m){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (!m.is_zero(i,j)){
                    at_element(i,j) =  m(i,j);
                }
            }
        }
    }

    void select_row(int row){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (i != row){
                    at_element(i,j) = 0;
                }
            }
        }
    }

    Matrix & subtract(Matrix & m){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                this->at_element(i,j) =this->at_element(i,j) - m(i,j);
            }
        }
        return *this;
    }

//    Matrix & or_operand(Matrix & m){
//        for (int i = 0; i< size1(); i++){
//            for (int j = 0; j< size2(); j++){

//                this->at_element(i,j) = m
//            }
//        }
//        return *this;
//    }


    Matrix & mult(Matrix & m){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                this->at_element(i,j) =this->at_element(i,j)*m(i,j);
            }
        }
        return *this;
    }
    bool compare(const Matrix & b){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (at_element(i,j)!=b(i,j)){
                    return false;
                }
            }
        }
        return true;
    }
    Matrix & mult (const double b){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (fabs(this->at_element(i,j))>1e-6){
                    this->at_element(i,j) =this->at_element(i,j)*b;
                }
            }
        }
        return *this;
    }

    Matrix & cut (const double b){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (this->at_element(i,j) > b){
                    this->at_element(i,j) = b;
                }
            }
        }
        return *this;
    }

    Matrix & lower_cut(const double b){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (this->at_element(i,j) < b){
                    this->at_element(i,j) = 0;
                }
            }
        }
        return *this;
    }
    Matrix & add (const Matrix & m){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
               this->at_element(i,j) =this->at_element(i,j)+ m(i,j);
            }
        }
        return *this;
    }

    Matrix & add (const double b){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (this->at_element(i,j)!=0){
                    this->at_element(i,j) =this->at_element(i,j)+b;
                }
            }
        }
        return *this;
    }
    Matrix & overwrite (Matrix & m){
        this->resize(m.size1(),m.size2());
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                this->at_element(i,j) =m(i,j);
            }
        }
        return *this;
    }

    Matrix clone (){
        Matrix m1(size1(),size2());
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                m1(i,j) =at_element(i,j);
            }
        }
        return m1;
    }
    Matrix & clone2(){
        Matrix * m1 = new Matrix(size1(),size2());
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                m1->at_element(i,j) =at_element(i,j);
            }
        }
        return *m1;
    }
    Matrix & log_e (){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){

                if (this->at_element(i,j)>1e-6){
                    if (this->at_element(i,j)==1.0){
                        fprintf(stderr,"*** WARNING: m(%d,%d)==1.0, logarithm will make the link disappear. Use 0.999 instead.\n", i, j);
                    }
                    this->at_element(i,j) = log(this->at_element(i,j));
                }
            }
        }
        return *this;
    }


    Matrix & reset(){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                this->at_element(i,j) = 0;
            }
        }
        return *this;
    }
    Matrix & lower(){
        for (int i = 0; i< size1(); i++){
            for (int j = i; j< size2(); j++){
                this->at_element(i,j) = 0;
            }
        }
        return *this;
    }

    Matrix & to_adjacency(){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (fabs(this->at_element(i,j)) > 1e-6){
                    this->at_element(i,j) = 1;
                }else{
                    this->at_element(i,j) = 0;
                }
            }
        }
        return *this;
    }

    Matrix & transpose(Matrix & m){
        m.resize(size1(), size2());
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                m(i,j) = this->at_element(j,i);
            }
        }
        return *this;
    }

    Matrix & inverse(double num){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (fabs(at_element(i,j))>1e-6) {
                    at_element(i,j) = num/at_element(i,j);
                }
            }
        }
        return *this;
    }

    Matrix & inverse(){
        return inverse(1.0);
    }

    Matrix & symmetrize_to_min_non_zero(){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                double valij = fabs(at_element(i,j));
                double valji = fabs(at_element(j,i));
                if (valij > 1e-6 && valji > 1e-6) {
                    at_element(i,j) = valij < valji? valij : valji;
                }else if(valji > 1e-6){
                    at_element(i,j) = valji;
                }
            }
        }
        return *this;
    }

    Matrix & symmetrize_to_min(){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                double valij = fabs(at_element(i,j));
                double valji = fabs(at_element(j,i));
                if (valij > 1e-6 && valji > 1e-6) {
                    at_element(i,j) = valij < valji? valij : valji;
                }else{
                    at_element(i,j) = 0;
                }


//                    else if(valji > 1e-6){
//                    at_element(i,j) = valji;
//                }
            }
        }
        return *this;
    }


    Matrix & symmetrize_to_max(){
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (at_element(j,i) > at_element(i,j)) {
                    at_element(i,j) = at_element(j,i);
                }
            }
        }
        return *this;
    }
    Matrix & print(std::string name = "matrix"){
        std::cerr << name << ":" << std::endl;
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                fprintf(stderr,"%-3.2f\t", at_element(i,j));
            }
            fprintf(stderr,"\n");
        }
        fprintf(stderr,"\n");
        return *this;
    }
    Matrix & print_undirected_graph(std::string name = "matrix"){
        std::cerr << name << ":" << std::endl;
        for (int i = 0; i< size1(); i++){
            for (int j = i; j< size2(); j++){
                if  (fabs(at_element(i,j)) > 1e-6){
                    fprintf(stderr,"%d - %d (%f)\n",i,j, at_element(i,j));
                }
            }
        }
        fprintf(stderr,"\n");
        return *this;
    }


    Matrix & write_graphviz(std::string name = "matrix.out", bool undirected = true){
        std::ofstream outf (name);

        outf << (undirected ? "graph G{" : "digraph G{") << std::endl;
        for (int i = 0; i< size1(); i++){
            outf << i << "[label=" << i <<"];" << std::endl;
            int start = undirected ? i : 0;
            for (int j = start; j< size2(); j++){
                if (fabs(at_element(i,j))>1e-6){
                    outf << i << (undirected ? " -- " : " -> ") << j << "[label="<< std::setprecision(2) << at_element(i,j) <<"];" << std::endl;
                }
            }
        }
        outf << "}";
        outf.close();
        return *this;
    }

    int write_graphviz_and_show(std::string name = "matrix.out", bool undirected = true){
        std::ostringstream oss;
        write_graphviz(name, undirected);
        oss << "dot -Tps -o /tmp/tmp.ps " << name << " && gnome-open /tmp/tmp.ps";
        return system (oss.str().c_str());
    }

    int write_graphviz_compare_and_show(Matrix & m, std::string name = "matrix.out", bool undirected = true){
        std::ostringstream oss;
        write_graphviz_compare(m,name,undirected);
        oss << "dot -Tps -o /tmp/tmp.ps " << name << " && gnome-open /tmp/tmp.ps";
        return system (oss.str().c_str());
    }

    Matrix & write_graphviz_compare(Matrix & m, std::string name = "matrix", bool undirected = true){
        std::ofstream outf (name);

        outf << (undirected ? "graph G{" : "digraph G{") << std::endl;
        for (int i = 0; i< size1(); i++){
            outf << i << "[label=" << i <<"];" << std::endl;
            int start = undirected ? i : 0;
            for (int j = start; j< size2(); j++){
                if (fabs(at_element(i,j)) >1e-6 && fabs(m(i,j))>1e-6){
                    outf << i << (undirected ? " -- " : " -> ") << j << "[label="<< std::setprecision(2) << at_element(i,j) <<", color = red];" << std::endl;
                }else if (fabs(at_element(i,j)) >1e-6){
                    outf << i << (undirected ? " -- " : " -> ") << j << "[label="<< std::setprecision(2) << at_element(i,j) <<", color = green];" << std::endl;
                }else if (fabs(m(i,j)) > 1e-6){
                    outf << i << (undirected ? " -- " : " -> ") << j << "[label="<< std::setprecision(2) << at_element(i,j) <<", color = blue];" << std::endl;
                }
            }
        }
        outf << "}";
        outf.close();
        return *this;
    }

    Matrix & print_as_int(std::string name = "matrix"){
        std::cerr << name << ":" << std::endl;
        fprintf(stderr,"     ");
        for (int j = 0; j< size2(); j++){
            fprintf(stderr,"%-3d ", j);
        }
        fprintf(stderr,"\n    ");
        for (int j = 0; j< size2(); j++){
            fprintf(stderr,"----");
        }
        fprintf(stderr,"\n");
        for (int i = 0; i< size1(); i++){
            fprintf(stderr,"%-3d| ", i);
            for (int j = 0; j< size2(); j++){
                fprintf(stderr,"%-3d ", (int) at_element(i,j));
            }
            fprintf(stderr,"\n");
        }
        fprintf(stderr,"\n");
        return *this;
    }

    /* Objectisation */

    graph_t matrix_to_graph(){
        std::vector<Edge> edges;
        std::vector<double> weights;
        for (int i = 0; i< size1(); i++){
            for (int j = 0; j< size2(); j++){
                if (fabs(at_element(i,j)) > 1e-6){
                    edges.push_back(Edge(i, j));
                    weights.push_back(at_element(i,j));
                    //  std::cerr << "Pushing back " << i <<"," << j << " with weight " << m(i,j) << std::endl;
                }
            }
        }
        graph_t g(&edges[0], (&edges[0]) + edges.size(), &weights[0], size1());
        return g;
    }


    /* Returns the size of the spanning tree or -1 if error */
    int kruskal(){

        for (int i = 0; i< size1(); i++){
            for (int j = i; j< size2(); j++){
                if (at_element(i,j)!=at_element(j,i)){
                    fprintf(stderr,"*** ERROR: trying to execute kruskal with non-symmetric matrix\n");
                    return -1;
                }
            }
        }

        graph_t g = lower().matrix_to_graph();
        reset();

        std::vector < EdgeDescriptor > spanning_tree;
        property_map < graph_t, edge_weight_t >::type weight = get(edge_weight, g);

        try{
            /* compute the tree*/
            kruskal_minimum_spanning_tree(g, std::back_inserter(spanning_tree));

            for (std::vector < EdgeDescriptor >::iterator ei = spanning_tree.begin();
                 ei != spanning_tree.end(); ++ei) {
                int i = source(*ei, g);
                int j = target(*ei, g);
                at_element(i,j) = weight[*ei];
                at_element(j,i) = weight[*ei];

                //  std::cout << source(*ei, g) << " <--> " << target(*ei, g)
                //  << " with weight of " << weight[*ei]
                //  << std::endl;

            }
            return spanning_tree.size();

        }catch(...){
            return -1;
        }
    }

    int maximumSpanningTree(){
        Matrix orig = clone();
        mult(-1);
        int res = kruskal();
        to_adjacency();
        mult(orig);
        return res;
    }


    int minimumSpanningTree(){
        return kruskal();
    }

    int maximumProbabilityTree(){
        Matrix orig = clone();
        log_e().mult(-1);
        int res = kruskal();
        to_adjacency();
        mult(orig);
        return res;
    }

    /* Dijkstra */
    Path p;
    std::vector<double> w;
    int start;

    int dijkstra_compute(int src){
        graph_t g = matrix_to_graph();
        vertex_descriptor s = vertex(src, g);
        p.resize(num_vertices(g));
        w.resize(num_vertices(g));
        start = src;

        try{
            dijkstra_shortest_paths(g, s,
                                    predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
                                    distance_map(boost::make_iterator_property_map(w.begin(), get(boost::vertex_index, g))));
            return 0;
        }catch(...){
            return 1;
        }
    }
    std::vector<double> & dijkstra_get_weights(){
        return w;
    }
    double dijkstra_get_weight(int i){
        return w[i];
    }

    int dijkstra_get_path(int goal, Path & path){

        graph_traits< graph_t >::vertex_descriptor current = goal;
        path.clear();
        while(current!=start) {
            path.push_back(current);
            if (current == p[current]){
                return 1;
            }
            current=p[current];
        }
        std::reverse(path.begin(),path.end());
        return 0;
    }

    int dijkstra_get_path(int src, int goal, Path & path){
        dijkstra_compute(src);
        return dijkstra_get_path(goal,path);
    }

    int dijkstra_get_next(int src, int goal){
        dijkstra_compute(src);
        return dijkstra_get_next(goal);
    }

    int dijkstra_get_next(int goal){
        if (start == goal){
            return start;
        }
        Path path;
        int err = dijkstra_get_path(goal, path);
        if (!err){
            return path[0];
        }else{
            return -1;
        }
    }

    void get_neighbors(int vertex, NodeSet & neighbors){
        neighbors.clear();
        for (int i = 0; i<size2(); i++){
            if (at_element(i, vertex) > 1e-6) {
                neighbors.push_back(i);
            }
        }
    }

    int is_connected(int node_id, std::vector<char> mask_vector, int ignore_value){
        int disc = 0;
        dijkstra_compute(node_id);
        for (int i = 0; i < mask_vector.size(); i++){
            if (mask_vector[i] != ignore_value && w[i]> 1e3){
                disc = 1;
                break;
            }
        }
        return !disc;
    }


    struct DFSVisitor : default_dfs_visitor {
        DFSVisitor(std::vector<EdgeDescriptor> & ed) : edges(ed){

        }

        void discover_vertex(vertex_descriptor u, const graph_t & g){

        }
        void tree_edge(EdgeDescriptor e, const graph_t & g) {

            if (edges.size()> 0){
                int last_target = target(edges[edges.size()-1], g);
                int new_source  = source(e, g);

                if (last_target != new_source){
                    int count = edges.size();
                    for (int i = count - 1; i >=0; i--){
                        EdgeDescriptor fwd_edge = edges[i];
                        EdgeDescriptor backw_edge(target(fwd_edge,g), source(fwd_edge,g), NULL);
                        edges.push_back(backw_edge);
                        if (new_source == source(fwd_edge,g)){
                            break;
                        }
                    }
                }
            }
            edges.push_back(e);
        }

        void examine_edge(EdgeDescriptor e, const graph_t & g) const {
        }
        void finish_edge(EdgeDescriptor e, const graph_t & g) const {
        }
        void back_edge(EdgeDescriptor e, const graph_t & g) const {
        }
        void forward_or_cross_edge(EdgeDescriptor e, const graph_t & g) const {
        }

    private:
        std::vector<EdgeDescriptor> & edges;

    };

    int dfs_get_path(int start_vertex, Path & path){
        graph_t g = matrix_to_graph();
        std::vector<EdgeDescriptor> edges;

        DFSVisitor dfs(edges);
        depth_first_search(g, visitor(dfs).root_vertex(start_vertex));

        path.clear();
        path.push_back(start_vertex);
        for (int j = 0; j < edges.size() ; j++){
            path.push_back(target(edges[j],g));
        }

        if (path.size() == 1){
            return 1;
        }

        /* check if the path actully exists */
        for (int j = 0; j < path.size()-1 ; j++){
            if (at_element(path[j], path[j+1])<1e-6 && at_element(path[j+1], path[j])<1e-6){
                path.clear();
                return 1;
            }
        }
        return 0;
    }


    void leafs(NodeSet & leafs){
        leafs.clear();
        for (int i = 0; i< size(); i++){
            graph_t g = matrix_to_graph();
            graph_t::adjacency_iterator ibegin, iend, tmp;
            int sum = 0;
            for (boost::tie(ibegin, iend) = boost::adjacent_vertices(i, g); ibegin != iend; ++ibegin)
            {
                tmp = ibegin;
                sum++;
            }
            if (sum == 1){
                leafs.push_back(i);
            }
        }
    }


    int size() const {
        return size1();
    }


    double get_cost(Path & p){
        double cost = 0;
        for (int k = 0; k < p.size()-1; k++){
            int i = p[k];
            int j = p[k+1];
            cost += at_element(i,j);
        }
        return cost;
    }

    double get_probability(Path & p){
        double prob = 1.0;
        for (int k = 0; k < p.size()-1; k++){
            int i = p[k];
            int j = p[k+1];
            prob *= at_element(i,j);
        }
        return prob;
    }


    void DFS(int x, Path & path){

        path.clear();
        std::stack<int> s;
        bool visited[size()];

        for(int i = 0; i < size(); i++){
            visited[i] = false;
        }

        s.push(x);

        while(!s.empty()){

            int k = s.top();
            path.push_back(k);
            s.pop();

            visited[k] = true;

            int sum = 0;
            for (int i = 0 ; i< size(); i++){
                sum+=visited[i]?1:0;
            }

            if (sum == size()) {
                break;
            }

            for (int i = size()-1; i >= 0 ; i--){
                if (at_element(k, i) > 1e-6 && !visited[i]) {
                    s.push(k);
                    s.push(i);
                }
            }
        }
    }
};

class Algorithm {

};

class DijkstraShortestPath : public Algorithm {
    Path p;
    std::vector<double> w;
    int start;

    int compute(Matrix & m, int src){
        graph_t g = m.matrix_to_graph();
        vertex_descriptor s = vertex(src, g);
        p.resize(num_vertices(g));
        w.resize(num_vertices(g));
        start = src;

        try{
            dijkstra_shortest_paths(g, s,
                                    predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
                                    distance_map(boost::make_iterator_property_map(w.begin(), get(boost::vertex_index, g))));
            return 0;
        }catch(...){
            return 1;
        }
    }
public:

    DijkstraShortestPath(Matrix & m, int src){
        compute(m, src);
        start = src;
    }

    std::vector<double> & get_weights(){
        return w;
    }
    double get_weight(int i){
        return w[i];
    }

    int get_path(int goal, Path & path){

        graph_traits< graph_t >::vertex_descriptor current = goal;
        path.clear();
        while(current!=start) {
            path.push_back(current);
            if (current == p[current]){
                return 1;
            }
            current=p[current];
        }
        path.push_back(start);
        std::reverse(path.begin(),path.end());
        return 0;
    }

    int dijkstra_get_next(int goal){
        if (start == goal){
            return start;
        }
        Path path;
        int err = get_path(goal, path);
        if (!err && path.size() > 1){
            return path[1];
        }else{
            return -1;
        }
    }
};

class ProbabilityShortestPath : public Algorithm {

    Matrix a0, a1;
    boost::numeric::ublas::matrix<int> next;
public:
    ProbabilityShortestPath(Matrix & m){
        compute(m);
    }

    ProbabilityShortestPath(Matrix & m, double max){
        compute(m, max);
    }

    ProbabilityShortestPath(){
    }

    void print(){
        a0.print("A0");
    }
    int compute(Matrix & m) {
        compute(m, 1.0);
    }

    int compute(Matrix & m, double max) {
        int i, j, k, size = m.size1();

        a0.resize(size,size);
        a1.resize(size,size);
        next.resize(size,size);

        for (i = 0; i < size; i++) {
            for (j = 0; j < size; j++) {
                next(i,j) = j;
                if (i == j) {
                    a0(i,j) = 1.0;
                } else {
                    a0(i,j) = m(i,j)/max;
                }
            }
        }

        for (k = 0; k < size; k++) {
            for (i = 0; i < size; i++) {
                for (j = 0; j < size; j++) {
                    if (a0(i,k) * a0(k,k) * a0(k,j) > a0(i,j)) {
                        next(i,j) = next(i,k);
                        a1(i,j) = a0(i,k) * a0(k,k) * a0(k,j);
                    } else {
                        a1(i,j) = a0(i,j);
                    }
                }
            }
            a0 = a1;
        }
        return 0;
    }

    int get_path(int src, int dest, Path & path) {
        if (a0(src,dest) > 1e-6){
            int res = src;
            path.clear();
            path.push_back(src);
            while (res != dest) {
                res = next(res,dest);
                path.push_back(res);
            }
            return 0;
        }else{
            return 1;
        }
    }

    int get_next(int src, int dest) {
        if (a0(src,dest) > 1e-6){
            return next(src,dest);
        }else{
            return -1;
        }
    }

    double get_prob(int src, int dest) {
        return a0(src, dest);
    }
};



class ProbabilityTreeVisitor : public Algorithm{
    ProbabilityShortestPath m1;
    int size;
    NodeSet ns;

    class Link{
    public:
        double prob;
        int hop;
        Link(int hop, double prob){
            this->hop = hop;
            this->prob = prob;
        }
        bool operator == (const Link &i) {
            return (i.hop ==this->hop);
        }
        bool operator< (const Link &i) const {
            return (i.prob > this->prob);
        }
    };


public:
    ProbabilityTreeVisitor(Matrix & m): m1(m){
        size = m.size();
        m.leafs(ns);
    }
    ProbabilityTreeVisitor(){

    }

    void set(Matrix & m){
        m1 = m;
        size = m.size();
        m.leafs(ns);
    }

    int get_path2(int x, Path & path){
        Stack s;
        BitArray visited(size);

        path.clear();
        s.push(x);

        while(visited.count() < size) {

            if (s.size() == 0){
                return visited.count();
            }

            int k = s.pop();
            path.push_back(k);

            if (visited[k]){
                continue;
            }

            visited.set(k);
            std::vector<Link> links;

            for (int i = 0; i< ns.size(); i++){
                int hop = m1.get_next(k, ns[i]);
                double prob = m1.get_prob(k, ns[i]);

                if (!visited[hop]){
                    Link m(hop, prob);
                    std::vector<Link>::iterator it = std::find(links.begin(), links.end(), m);
                    if (it !=links.end()){
                        if ((*it).prob > m.prob){
                            (*it).prob = m.prob;
                        }
                    }else{
                        links.push_back(m);
                    }
                }
            }

            if (links.size() > 0){
                std::sort(links.begin(), links.end());
                for (int i = 0; i < links.size(); i++){
                    int next = links[i].hop;
                    s.push(k);
                    s.push(next);
                }
            }
        }
        return visited.count();
    }


    void get_path(int x, Path & path) {

        Stack s;
        BitArray visited(size);
        BitArray pushed(size);

        path.clear();

        s.push(x);
        pushed.set(x);
        visited.set(x);

        while(!s.empty()){

            int k = s.pop();
            path.push_back(k);
            pushed.set(k);

            if (visited.count() == size) {
                while (!s.empty()) {
                    if (pushed.count() == size){
                        break;
                    }
                    k = s.pop();
                    path.push_back(k);
                    pushed.set(k);
                }
                break;
            }

            while (true){
                int next = -1;
                double min = 1e6;
                for (int i = 0; i< ns.size(); i++){
                    int hop = m1.get_next(k, ns[i]);
                    double prob = m1.get_prob(k, ns[i]);
                    if (!visited[hop] &&  prob < min ){
                        next = hop;
                        min = prob;
                    }
                }
                if (next != -1){
                    s.push(k);
                    s.push(next);
                    visited.set(next);
                }else{
                    break;
                }
            }
        }
    }
};

class DeepFirstVisitor : public Algorithm{
    Matrix & m1;
    int size;
    NodeSet ns;
public:
    DeepFirstVisitor(Matrix & m): m1(m){
        size = m.size();
    }

    void get_path(int x, Path & path){

        path.clear();
        Stack s;
        BitArray visited(size);
        BitArray pushed(size);

        s.push(x);
        visited.set(x);
        pushed.set(x);

        while(!s.empty()){

            int k = s.pop();
            path.push_back(k);
            pushed.set(k);

            if (visited.count() == size) {
                while (!s.empty()) {
                    if (pushed.count() == size){
                        break;
                    }
                    k = s.pop();
                    path.push_back(k);
                    pushed.set(k);
                }
                break;
            }

            for (int i = size-1; i >= 0 ; i--){
                if (m1.at_element(k, i) > 1e-6 && !visited[i]) {
                    s.push(k);
                    s.push(i);
                    visited.set(i);
                }
            }
        }
    }

    void get_path2(int x, Path & path){
        Stack s;
        BitArray visited(size);

        path.clear();
        s.push(x);

        while(visited.count() < size) {

            int k = s.pop();
            path.push_back(k);

            if (visited[k]){
                continue;
            }

            visited.set(k);

            for (int i = 0; i < size ; i++){
                if (m1.at_element(k, i) > 1e-6 && !visited[i]) {
                    s.push(k);
                    s.push(i);
                }
            }
        }
    }


};


#endif
