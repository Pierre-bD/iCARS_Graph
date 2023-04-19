#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/


#ifndef LOC_GRAPH_HPP_
#define LOC_GRAPH_HPP_

namespace ic_graph 
{

class loc_graph
{
private:
gtsam::ISAM2Params isam2Params;
isam2Params.relinearizeTreshold = 0.01; //define proper value
isam2Params.relinearizeSkip = 1;
std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixedLagSmoother;
std::shared_ptr<gtsam::NonlinearFactorGraph> factorGraph;


public:



    loc_graph(/* args */);
    ~loc_graph();
};

loc_graph::loc_graph(/* args */)
{
}

loc_graph::~loc_graph()
{
}






}







#endif