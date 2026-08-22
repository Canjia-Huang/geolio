//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_OBJECT_H
#define GEOLIO_BASE_OBJECT_H

#include <geolio/common/utils.h>
#include <geolio/common/parse_filepath.h>
#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/third_party/imgui/imgui.h>

namespace geolio::geobox
{
    /**
     * @brief Base class for all renderable GeoBox objects.
     */
    class BaseObject {
    public:
        /**
         * @brief Constructs an object from its source file path.
         * @param[in] filepath Path to the source asset or mesh file.
         */
        explicit BaseObject(
            const std::string& filepath
            ) : filepath_(filepath),
                name_(get_filename(filepath)),
                unique_id_(generate_random_string(22))
        {}

        /**
         * @brief Destroys the object.
         */
        virtual ~BaseObject() = default;

        /**
         * @brief Returns the display name of the object.
         * @return Reference to the object name.
         */
        [[nodiscard]] const std::string& name() const { return name_; }

        /**
         * @brief Returns whether the object is currently visible.
         * @return true if the object should be rendered; false otherwise.
         */
        [[nodiscard]] bool visible() const { return visible_; }

        /**
         * @brief Sets the visibility state of the object.
         * @param[in] visible New visibility flag.
         */
        void set_visible(const bool visible) { visible_ = visible; }

        /**
         * @brief Draws the object-specific property widgets in the GUI.
         */
        virtual void draw_object_properties() = 0;

        /**
         * @brief Renders the object in the scene.
         * @param[in] lighting Whether scene lighting should be enabled.
         */
        virtual void draw_scene(bool lighting) = 0;

        /**
         * @brief Retrieves the object's axis-aligned bounding box.
         * @param[out] xyzmin Array of minimum coordinates in x, y, z order.
         * @param[out] xyzmax Array of maximum coordinates in x, y, z order.
         */
        virtual void get_bbox(double* xyzmin, double* xyzmax) const = 0;

    protected:
        const std::string filepath_;
        const std::string name_;

        const std::string unique_id_;

        bool visible_ = true;
    };
}

#endif //GEOLIO_BASE_OBJECT_H
