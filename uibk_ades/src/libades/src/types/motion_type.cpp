#include "../../include/types/motion_type.h"

namespace ades {

    std::string typeToString(enum MotionType mt)
    {
        int mt_ = static_cast<int>(mt);
        const std::string types[motionTypeNb] ={"DMP", "Trajectory", "Cyclic", "VisualServoing", "PID", "Unscrewing"};
        if((mt_ < 0) || (mt_ > motionTypeNb))
        {
            return "Unknown";
        }
        else
        {
            return types[mt_];
        }
    }
}
