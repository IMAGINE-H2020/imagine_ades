#include "../../include/motion_containers/dmplib_motion.h"

#include <iostream>
#include <cstdarg>

namespace ades {

DMPLibMotion::DMPLibMotion(const string &name) :
    MotionContainer(name)
{
    setMotionType(MotionType::DMP);

    dmp.reset(new DMP::UMITSMP());
}

Trajectory DMPLibMotion::run(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps)
{
    // Assumes zero velocity at start
    DMP::Vec<DMP::DMPState> start_dmplib;
    start_dmplib.push_back(DMP::DMPState(start.x, 0));
    start_dmplib.push_back(DMP::DMPState(start.y, 0));
    start_dmplib.push_back(DMP::DMPState(start.z, 0));
    start_dmplib.push_back(DMP::DMPState(start.qw, 0));
    start_dmplib.push_back(DMP::DMPState(start.qx, 0));
    start_dmplib.push_back(DMP::DMPState(start.qy, 0));
    start_dmplib.push_back(DMP::DMPState(start.qz, 0));

    DMP::DVec goal_dmplib(std::vector<double>{goal.x, goal.y, goal.z, goal.qw, goal.qx, goal.qy, goal.qz});

    dmp->prepareExecution(goal_dmplib, start_dmplib, 1, 1);

    DMP::DVec timestamps = DMP::SampledTrajectoryV2::generateTimestamps(0, 1.0, 1.0 / (-0.5 + num_timestamps));
    DMP::SampledTrajectoryV2 result_dmplib = dmp->calculateTrajectory(timestamps, goal_dmplib, start_dmplib, 1.0, 1.0);

    Trajectory result;
    for (auto& d : result_dmplib.getPositionData())
    {
        result[d.first] = TrajectoryPoint(d.second[0], d.second[1], d.second[2], d.second[3], d.second[4], d.second[5], d.second[6]);
    }
    return result;
}

void DMPLibMotion::train(const std::vector<Trajectory> &trajectories)
{
    DMP::Vec<DMP::SampledTrajectoryV2> trainingTrajectories;

    for (auto& traj : trajectories)
    {
        std::map<double, DMP::DVec> traj_dmplib;
        for (auto& e : traj)
        {
            traj_dmplib[e.first] = DMP::DVec(std::vector<double>{e.second.x, e.second.y, e.second.z, e.second.qw, e.second.qx, e.second.qy, e.second.qz});
        }

        DMP::SampledTrajectoryV2 t(traj_dmplib);
        t = DMP::SampledTrajectoryV2::normalizeTimestamps(t, 0, 1);

        trainingTrajectories.push_back(t);
    }

    dmp->learnFromTrajectories(trainingTrajectories);
}

Trajectory DMPLibMotion::simulate(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps)
{
    return run(start, goal, num_timestamps);
}

}
