// Copyright (c) 2024 - Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    FiveBar.h

    This implements 5-Bar Parallel SCARA Kinematics
    Two motors at (-L0, 0) and (L0, 0)
    Each motor has an arm of length L1
    The end effector is connected to these arms by two links of length L2
*/

#include "Kinematics.h"
#include "Cartesian.h"

namespace Kinematics {
    class FiveBar : public Cartesian {
    public:
        FiveBar(const char* name) : Cartesian(name) {}

        FiveBar(const FiveBar&)            = delete;
        FiveBar(FiveBar&&)                 = delete;
        FiveBar& operator=(const FiveBar&) = delete;
        FiveBar& operator=(FiveBar&&)      = delete;

        // Kinematic Interface
        void init() override;
        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis) override;
        bool transform_cartesian_to_motors(float* motors, float* cartesian) override;
        bool invalid_line(float* cartesian) override;
        void constrain_jog(float* target, plan_line_data_t* pl_data, float* position) override;
        void init_position() override;

        // Configuration handlers:
        void group(Configuration::HandlerBase& handler) override;

        ~FiveBar() {}

    private:
        float _L0 = 50.0;
        float _L1 = 100.0;
        float _L2 = 100.0;
        float _kinematic_segment_len_mm = 1.0;

        // Pre-calculated constants
        float _L1_sq_minus_L2_sq = 0.0f;
        float _two_L1            = 0.0f;
        float _L2_sq             = 0.0f;

        float _last_motor_pos[MAX_N_AXIS] = { 0 };
    };
}  // namespace Kinematics
