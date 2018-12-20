//
// Created by Philipp Zech on 9/24/18.
//

#ifndef ADES2_MOTIONCONTAINER_H
#define ADES2_MOTIONCONTAINER_H

#pragma once

#include <string>
#include <vector>
#include <variant>
#include <map>
#include "effect_model.h"

using ull = unsigned long long;

using namespace std;

namespace ades {

    class TrajectoryPoint
    {
    public:
        TrajectoryPoint() {}
        TrajectoryPoint(double x, double y, double z, double qw, double qx, double qy, double qz) :
            x(x), y(y), z(z), qw(qw), qx(qx), qy(qy), qz(qz)
        {
        }

        double x = 0;
        double y = 0;
        double z = 0;

        double qw = 0;
        double qx = 0;
        double qy = 0;
        double qz = 0;
    };

    typedef std::map<double, TrajectoryPoint> Trajectory;

    enum class MotionType {DMP, CPG, TRAJ};

    class MotionContainer {
    private:
        string name_;

        ull id_;

        EffectModel effectModel_;

        MotionType type_;

        map<string, string> parameters_;

    public:
        explicit MotionContainer(string name) noexcept;

        MotionContainer(string name, MotionType type) noexcept;

        ~MotionContainer() noexcept = default;

        MotionContainer(MotionContainer&&) = default;

        MotionContainer& operator=(MotionContainer&&) = default;

        MotionContainer(const MotionContainer&) = default;

        MotionContainer& operator=(const MotionContainer&) = default;

        string getName() const noexcept;

        ull getId() const noexcept;

        void setMotionType(MotionType type) noexcept;

        MotionType getMotionType() const noexcept;

        EffectModel* getEffectModel() noexcept;

        bool addParameter(map<string, string> parameters) noexcept;

        vector<bool> deleteParameter(vector<string>) noexcept;

        map<string, string> getParameters() noexcept;

        virtual Trajectory run(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps);

        virtual void train(const std::vector<Trajectory> &trajectories);

        virtual Trajectory simulate(const ades::TrajectoryPoint& start, const ades::TrajectoryPoint& goal, int num_timestamps);

    };

};

#endif //ADES2_MOTIONCONTAINER_H
