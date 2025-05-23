#include "QuadCable.h"
#include <cmath>

namespace Kinematics {

    void QuadCable::init() {
        Cartesian::init();

        float d_br_tl_expected = calculate_corners();
        if (std::abs(_d_br_tl - d_br_tl_expected) > 0.01f) {
            log_config_error("Diagonal BR-TL " << _d_br_tl << "mm does not match calculated " << d_br_tl_expected << "mm (difference is "
                                               << std::abs(_d_br_tl - d_br_tl_expected) << "mm)");
        } else
            log_info("Anchor points: BL(" << _anchors[BottomLeft].x << ", " << _anchors[BottomLeft].y
                                          << "), "
                                             "BR("
                                          << _anchors[BottomRight].x << ", " << _anchors[BottomRight].y
                                          << "), "
                                             "TL("
                                          << _anchors[TopLeft].x << ", " << _anchors[TopLeft].y
                                          << "), "
                                             "TR("
                                          << _anchors[TopRight].x << ", " << _anchors[TopRight].y << ")");
    }

    /**
     * Calculates the corner coordinates and returns the BR‑TL diagonal length.
     * Assumes input distances are positive and form a valid convex quad.
     */
    float QuadCable::calculate_corners() {
        // 1. Fix BL and BR on the X‑axis
        _anchors[BottomLeft]  = { 0.0f, 0.0f };
        _anchors[BottomRight] = { _d_bl_br, 0.0f };

        // 2. Compute TR from circle–circle intersection
        const float x_tr   = (_d_bl_br * _d_bl_br + _d_bl_tr * _d_bl_tr - _d_br_tr * _d_br_tr) / (2.0f * _d_bl_br);
        const float y_tr   = std::sqrt(_d_bl_tr * _d_bl_tr - x_tr * x_tr);
        _anchors[TopRight] = { x_tr, y_tr };

        // 3. Compute TL from circle–circle intersection
        const float a = (_d_bl_tl * _d_bl_tl - _d_tl_tr * _d_tl_tr + _d_bl_tr * _d_bl_tr) / (2.0f * _d_bl_tr);
        const float h = std::sqrt(_d_bl_tl * _d_bl_tl - a * a);

        _anchors[TopLeft].x = (a * _anchors[TopRight].x - h * _anchors[TopRight].y) / _d_bl_tr;
        _anchors[TopLeft].y = (a * _anchors[TopRight].y + h * _anchors[TopRight].x) / _d_bl_tr;

        // 4. Return the BR‑TL diagonal length
        const float dx = _anchors[TopLeft].x - _anchors[BottomRight].x;
        const float dy = _anchors[TopLeft].y - _anchors[BottomRight].y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void QuadCable::group(Configuration::HandlerBase& handler) {
        handler.item("d_bl_tl", _d_bl_tl, 100.0f, 10000.0f);
        handler.item("d_bl_br", _d_bl_br, 100.0f, 10000.0f);
        handler.item("d_br_tr", _d_br_tr, 100.0f, 10000.0f);
        handler.item("d_tl_tr", _d_tl_tr, 100.0f, 10000.0f);
        handler.item("d_bl_tr", _d_bl_tr, 100.0f, 10000.0f);
        handler.item("d_br_tl", _d_br_tl, 100.0f, 10000.0f);
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<QuadCable> registration("QuadCable");
    }

}  // namespace Kinematics