// Copyright (c) 2024 - Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "FiveBar.h"
#include "Machine/MachineConfig.h"
#include "Limit.h"
#include <cmath>

namespace Kinematics {
    void FiveBar::group(Configuration::HandlerBase& handler) {
        handler.item("L0", _L0, 0.0, 1000.0);
        handler.item("L1", _L1, 0.0, 1000.0);
        handler.item("L2", _L2, 0.0, 1000.0);
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm, 0.05, 20.0);
    }

    void FiveBar::init() {
        log_info("Kinematic system: " << name());
        _L1_sq_minus_L2_sq = _L1 * _L1 - _L2 * _L2;
        _two_L1            = 2.0f * _L1;
        _L2_sq             = _L2 * _L2;
        init_position();
    }

    bool FiveBar::invalid_line(float* cartesian) {
        float motors[MAX_N_AXIS];
        if (!transform_cartesian_to_motors(motors, cartesian)) {
            limit_error();
            return true;
        }
        return false;
    }

    void FiveBar::constrain_jog(float* target, plan_line_data_t* pl_data, float* position) {
        float motors[MAX_N_AXIS];
        if (transform_cartesian_to_motors(motors, target)) {
            return;
        }
        // If unreachable, don't move
        copyAxes(target, position);
    }

    bool FiveBar::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        axis_t n_axis = Axes::_numberAxis;

        float seg_target[MAX_N_AXIS];
        float feed_rate = pl_data->feed_rate;

        float motors[MAX_N_AXIS];
        if (!transform_cartesian_to_motors(motors, target)) {
            log_warn("Kinematics error. Target unreachable (" << target[0] << "," << target[1] << ")");
            return false;
        }

        float d[MAX_N_AXIS];
        copyAxes(d, target, n_axis);
        subtractAxes(d, position, n_axis);

        uint32_t segment_count = ceil(vector_length(d, 2) / _kinematic_segment_len_mm);
        if (segment_count == 0) {
            segment_count = 1;
        }

        float segment_dist = vector_length(d, n_axis) / segment_count;

        copyAxes(seg_target, position, n_axis);
        float delta_d[MAX_N_AXIS];
        copyArray(delta_d, d, n_axis);
        multiplyArray(delta_d, 1.0f / segment_count, n_axis);

        for (uint32_t segment = 1; segment <= segment_count; segment++) {
            if (sys.abort()) {
                return true;
            }

            addAxes(seg_target, delta_d, n_axis);

            if (!transform_cartesian_to_motors(motors, seg_target)) {
                log_error("Kinematic error motors");
                return false;
            }

            if (!pl_data->motion.rapidMotion) {
                float delta_distance = vector_distance(motors, _last_motor_pos, n_axis);
                pl_data->feed_rate   = (feed_rate * delta_distance / segment_dist);
            }

            if (!mc_move_motors(motors, pl_data)) {
                return false;
            }

            copyAxes(_last_motor_pos, motors, n_axis);
        }
        return true;
    }

    bool FiveBar::transform_cartesian_to_motors(float* motors, float* cartesian) {
        float x = cartesian[X_AXIS];
        float y = cartesian[Y_AXIS];

        float x_plus_L0  = x + _L0;
        float x_minus_L0 = x - _L0;
        float y_sq       = y * y;
        float K_sq       = x_plus_L0 * x_plus_L0 + y_sq;
        float S_sq       = x_minus_L0 * x_minus_L0 + y_sq;
        float K          = sqrtf(K_sq);
        float S          = sqrtf(S_sq);

        if (K == 0 || S == 0) {
            return false;
        }

        float gamma_1_num = _L1_sq_minus_L2_sq + K_sq;
        float gamma_1_den = _two_L1 * K;
        float gamma_1_arg = gamma_1_num / gamma_1_den;
        if (gamma_1_arg < -1.0f || gamma_1_arg > 1.0f) {
            return false;
        }
        float gamma_1 = acosf(gamma_1_arg);

        float gamma_2_num = _L1_sq_minus_L2_sq + S_sq;
        float gamma_2_den = _two_L1 * S;
        float gamma_2_arg = gamma_2_num / gamma_2_den;
        if (gamma_2_arg < -1.0f || gamma_2_arg > 1.0f) {
            return false;
        }
        float gamma_2 = acosf(gamma_2_arg);

        float beta_1 = atan2f(y, x + _L0);
        float beta_2 = atan2f(y, x - _L0);

        motors[X_AXIS] = (beta_1 + gamma_1) * (180.0f / M_PI);
        motors[Y_AXIS] = (beta_2 - gamma_2) * (180.0f / M_PI);

        // Copy other axes
        for (axis_t axis = Z_AXIS; axis < Axes::_numberAxis; axis++) {
            motors[axis] = cartesian[axis];
        }

        return true;
    }

    void FiveBar::motors_to_cartesian(float* cartesian, float* motors, axis_t n_axis) {
        float theta1 = motors[X_AXIS] * (M_PI / 180.0f);
        float theta2 = motors[Y_AXIS] * (M_PI / 180.0f);

        // Elbow positions
        float p1x = -_L0 + _L1 * cosf(theta1);
        float p1y = _L1 * sinf(theta1);
        float p2x = _L0 + _L1 * cosf(theta2);
        float p2y = _L1 * sinf(theta2);

        // Intersection of two circles centered at (p1x, p1y) and (p2x, p2y) with radius L2
        float dx = p2x - p1x;
        float dy = p2y - p1y;
        float d2 = dx * dx + dy * dy;
        float d  = sqrtf(d2);

        if (d > 2.0f * _L2 + 0.1f || d == 0 || _L2 == 0) {
            // No intersection
            copyAxes(cartesian, motors);
            return;
        }

        float a = d / 2.0f;  // Since both radii are L2, the intersection is at the midpoint of the line between centers + some offset
        float h = sqrtf(_L2_sq - a * a);

        float x0 = p1x + a * dx / d;
        float y0 = p1y + a * dy / d;

        // There are two intersections, we want the one with larger Y (assuming the robot works in positive Y)
        // For a 5-bar parallel robot, the links usually point "away" from the base.
        cartesian[X_AXIS] = x0 - h * dy / d;
        cartesian[Y_AXIS] = y0 + h * dx / d;

        // Copy other axes
        for (axis_t axis = Z_AXIS; axis < n_axis; axis++) {
            cartesian[axis] = motors[axis];
        }
    }

    void FiveBar::init_position() {
        auto n_axis = Axes::_numberAxis;
        for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
            float mpos = 0.0f;
            auto  axes = config->_axes;
            if (axes && axes->_axis[axis] && axes->_axis[axis]->_homing) {
                mpos = axes->_axis[axis]->_homing->_mpos;
            }
            set_steps(axis, motor_pos_to_steps(mpos, axis));
        }
        // Update the motor range limits
        float min_mpos[MAX_N_AXIS];
        float max_mpos[MAX_N_AXIS];
        for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
            min_mpos[axis] = limitsMinPosition(axis);
            max_mpos[axis] = limitsMaxPosition(axis);
        }
        transform_cartesian_to_motors(_min_motor_pos, min_mpos);
        transform_cartesian_to_motors(_max_motor_pos, max_mpos);
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<FiveBar> registration("five_bar");
    }
}
