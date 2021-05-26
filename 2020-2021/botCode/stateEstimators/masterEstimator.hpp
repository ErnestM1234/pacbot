#ifndef MASTER_ESTIMATOR_H
#define MASTER_ESTIMATOR_H

#include "../robotState/robotStateHistory.hpp"
#include <random>

class StateEstimator {
  public:
    typedef std::function<RobotState(RobotStateHistory, bool)> StateEstimatorF;
    StateEstimatorF f;
    std::unordered_set<SD> past_sds_using;
    std::unordered_set<SD> sds_using;
    std::unordered_set<SD> sds_estimating;
    std::default_random_engine generator;
    StateEstimator(StateEstimatorF f, std::unordered_set<SD> past_sds_using,
                   std::unordered_set<SD> sds_using,
                   std::unordered_set<SD> sds_estimating);

    void apply(std::shared_ptr<RobotStateHistory> state_history,
               bool with_uncertianity = false);
};

#endif