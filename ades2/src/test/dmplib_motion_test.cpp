#include "../../include/motion_containers/dmplib_motion.h"

#include <iostream>
#include <fstream>

ades::Trajectory loadTrajectory(const std::string& filename)
{
    ades::Trajectory trajectory;

    std::ifstream f(filename);
    if (!f.is_open())
    {
        return trajectory;
    }

    double t, x, y, z, qw, qx, qy, qz;
    char comma;
    while (true)
    {
        f >> t >> comma >> x >> comma >> y >> comma >> z >> comma >> qw >> comma >> qx >> comma >> qy >> comma >> qz;
        if (f.fail() || f.eof())
        {
            break;
        }

        trajectory[t] = ades::TrajectoryPoint(x, y, z, qw, qx, qy, qz);
    }

    return trajectory;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: ./dmplib_motion_test CSV_FILENAME" << std::endl;
        exit(0);
    }

    std::vector<ades::Trajectory> trajectories;
    trajectories.push_back(loadTrajectory(argv[1]));

    ades::DMPLibMotion m("test_motion");
    m.train(trajectories);

    ades::TrajectoryPoint start = trajectories.at(0).begin()->second;
    ades::TrajectoryPoint goal = trajectories.at(0).rbegin()->second;

    ades::Trajectory t_real = m.run(start, goal, 100);
    ades::Trajectory t_sim = m.simulate(start, goal, 100);
}
