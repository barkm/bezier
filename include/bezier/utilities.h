#ifndef BEZIER_UTILITIES_H
#define BEZIER_UTILITIES_H

#include <vector>

namespace bezier {
    using std::vector;

    /**
     * Partitions data points at joints. Each partition includes the joint.
     * @tparam T
     * @param data_points
     * @param joints
     * @return
     */
    template <typename T>
    vector<vector<T>> partition_data(const vector<T> & data_points,
                                const vector<int> & joints){
        vector<vector<T>> partitioned_data_points;
        vector<T> subset;
        if(joints.empty()){
            return {data_points};
        }

        for (int i = 0; i < joints.size(); ++i) {
            subset.clear();
            if(i == 0){
                subset.resize(joints[i]+1);
                std::copy(data_points.begin(), data_points.begin()+joints[i]+1, subset.begin());
            }
            else{
                subset.resize(joints[i]-joints[i-1]);
                std::copy(data_points.begin()+joints[i-1]+1, data_points.begin()+joints[i]+1, subset.begin());
            }
            partitioned_data_points.push_back(subset);
        }
        subset.clear();
        subset.resize(data_points.size()-*(joints.end()-1)-1);
        std::copy(data_points.begin()+*(joints.end()-1)+1, data_points.end(), subset.begin());
        partitioned_data_points.push_back(subset);

        return partitioned_data_points;
    }
}

#endif //BEZIER_UTILITIES_H
