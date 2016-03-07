#include <queue>
#include <string.h>
extern "C" {
#include "../../../utils/timespec_utils.h"
}
class SignalAverage {
    std::vector<double> values;
    int max_length;

    SignalAverage(int max_length){
        this->max_length = max_length;
    }

    void addValue(double val){
        values.push_back(val);
        if (values.size() > max_length){
            values.erase(values.begin());
        }
    }

    double getValue(){
        double sum =0;
        if (values.size() > 0){
            for (int i=0;i<values.size(); i++){
                sum+=values[i];
            }
            return sum / double(values.size());
        }else{
            return 0.0;
        }
    }

    void reset(){
        values.clear();
    }
};

class PDR{

    std::vector<unsigned char> v,v2;
    int previous_frame_id, idx, idx2, ticks, node_id, max_length;
    int last_id;
    unsigned long long previous_frame_ts;

public:
    PDR(int max_length, int id){
        this->node_id = id;
        this->max_length = max_length;
        last_id = 0;
        previous_frame_id = 0;
    }

    PDR(){
        max_length = 100;
        last_id = 0;
    }

    void setNodeId(int id){
        this->node_id = id;
    }
    void reset(){
        v.clear();
    }

    double get(){
        int sum_of_elems = 0;
        for (int i = 0; i<v.size(); i++) {
            sum_of_elems += v[i];
        }

        if (v.size() == 0){
            return 0.0;
        }

        double perc = 100.0*double(sum_of_elems)/double(v.size());
        long long elapsed = timespec_timestamp() - previous_frame_ts;

        //TODO:::PARAM
        double x1 = 1.5, x2 = 2.5;
        if (elapsed > x2*1e6){
            perc = 0;
        }else if (elapsed > x1*1e6){
            double x = double(elapsed)/1e6;
            perc *= -1.0/(x2 - x1)*(x - x1) + 1.0;
        }
//        perc = perc > 1e-3? perc : 0.0;
//        fprintf(stderr,"%f %d\n",perc, (int) v.size());

        return perc;
    }

    double getRaw(){
        int sum_of_elems = 0;
        for (int i = 0; i<v.size(); i++) {
            sum_of_elems += v[i];
        }
        return 100.0*double(sum_of_elems)/double(v.size());
    }

    void newIncomingFrame(int send_id){

        for (int i = 0; i< send_id - previous_frame_id; i++){
            v.push_back(0);
            if (v.size() > max_length){
                v.erase(v.begin());
            }
        }

        if (v.size() > 0){
            v.at( v.size() - 1) = 1;
        }

        previous_frame_id = send_id;
        previous_frame_ts = timespec_timestamp();
    }

    void newOutgoingFrame(){
        v2.push_back(1);
        if (v2.size() > max_length){
            v2.erase(v2.begin());
        }
    }
    void newTimedoutFrame(){
        if (v2.size() > 0){
            v2[v2.size() - 1] = 0;
        }
    }

    double get_tx_pdr(){
        int sum = 0;
        for (int i = 0; i< v2.size(); i++){
            sum += v2[i];
        }
        if (v2.size()>0){
            return 100.0*double(sum)/double(v2.size());
        }else{
            return 0;
        }
    }
};

