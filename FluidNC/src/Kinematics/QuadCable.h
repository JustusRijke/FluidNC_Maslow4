#pragma once

#include "Cartesian.h"

namespace Kinematics {
    class QuadCable : public Cartesian {
    public:
        enum Corner { TopLeft = 0, TopRight = 1, BottomLeft = 2, BottomRight = 3 };

        QuadCable(const char* name) : Cartesian(name) {}

        QuadCable(const QuadCable&)            = delete;
        QuadCable(QuadCable&&)                 = delete;
        QuadCable& operator=(const QuadCable&) = delete;
        QuadCable& operator=(QuadCable&&)      = delete;

        void init() override;

        float calculate_corners();

        // bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        // void motors_to_cartesian(float* cartesian, float* motors, int n_axis) override;

        // Configuration handlers:
        void group(Configuration::HandlerBase& handler) override;

    protected:
        ~QuadCable() {}

    private:
        float _d_bl_tl;  // Distance from bottom left to top left (Left side)
        float _d_bl_br;  // Distance from bottom left to bottom right (Bottom side)
        float _d_br_tr;  // Distance from bottom right to top right (Right side)
        float _d_tl_tr;  // Distance from top left to top right (Top side)
        float _d_bl_tr;  // Distance from bottom left to top right (Diagonal BL-TR)
        float _d_br_tl;  // Distance from bottom right to top left (Diagonal BR-TL)

        struct Point {
            float x, y;
        };
        Point _anchors[4];
    };
}  // namespace Kinematics