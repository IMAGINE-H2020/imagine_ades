#include "../motion_container.h"

#include <dmp/representation/dmp/umitsmp.h>

namespace ades {

class DMPLibMotion : public MotionContainer
{
public:
    DMPLibMotion(const std::string& name);

    Trajectory run(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps) override;
    void train(const std::vector<Trajectory> &trajectories) override;
    Trajectory simulate(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps) override;

protected:
    DMP::UMITSMPPtr dmp;
};

}
