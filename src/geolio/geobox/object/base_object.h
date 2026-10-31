//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_OBJECT_H
#define GEOLIO_BASE_OBJECT_H

#include <geolio/common/utils.h>
#include <geolio/common/parse_filepath.h>
#include <geogram/basic/geometry.h>

#include <utility>

namespace geolio::geobox
{
    /**
     * @brief Base class for all renderable GeoBox objects.
     */
    class BaseObject {
    public:
        /**
         * @brief Constructs an object from its source file path.
         * @param[in] name object name.
         * @param[in] filepath Path to the source asset.
         */
        BaseObject(std::string name, std::string filepath)
            : name_(std::move(name)),
              filepath_(std::move(filepath)),
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
         * @brief Returns the path of the file the object was read from.
         * @return The path, or an empty string when the object was not read
         *         from a file (e.g. the result of an operation).
         */
        [[nodiscard]] const std::string& filepath() const { return filepath_; }

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
         * @brief Saves the data of the object to @p filename.
         * @details Written in the format the file extension asks for. The
         *          default does nothing: a class whose data cannot be written
         *          leaves the object as it is.
         * @param[in] filename Path of the file to write.
         * @return true when the file was written.
         */
        virtual bool save(const std::string& filename) {
            static_cast<void>(filename);
            return false;
        }

        /**
         * @brief Retrieves the object's axis-aligned bounding box.
         * @param[out] xyzmin Array of minimum coordinates in x, y, z order.
         * @param[out] xyzmax Array of maximum coordinates in x, y, z order.
         */
        virtual void get_bbox(double* xyzmin, double* xyzmax) const = 0;

        /**
         * @brief Whether the data of the object can be rendered.
         * @details False while check_validity() reports data that the drawing
         *          passes cannot read. Such an object is left out of the scene
         *          (see GeoBoxApplication::draw_scene()), and the camera is not
         *          framed on it: reading its data would make the renderer access
         *          memory outside of it, which crashes the application.
         * @return true when draw_scene() can run.
         */
        [[nodiscard]] virtual bool renderable() const { return true; }

        /**
         * @brief Checks whether the data of the object can be rendered.
         * @details Called every time an object is created, before it is drawn:
         *          data that cannot be rendered (element indices out of range,
         *          coordinates that are not finite, ...) makes the drawing
         *          passes read outside the object's data, which crashes the
         *          application. A concrete class calls it at the end of its
         *          constructor, where its own overrides are dispatched, and
         *          keeps what it found so that draw_validity_dialog() can
         *          report it.
         * @return true when the data of the object can be rendered.
         */
        virtual bool check_validity() { return true; }

        /**
         * @brief Warns the user about the invalid data check_validity() found.
         * @details Drawn once per frame by the main application, on top of
         *          everything else, while the object still has something to
         *          report: the dialog cannot be dismissed without choosing what
         *          to do with the invalid data (see MeshObject).
         *          Defaults to nothing to report.
         */
        virtual void draw_validity_dialog() {}

    protected:
        const std::string name_;
        const std::string filepath_;

        const std::string unique_id_;

        bool visible_ = true;

        /**
         * @brief Color of the section titles in the object property panels.
         * @details Kept here so that every object class uses the same style and
         *          the color is defined in a single place. MeshObject uses it
         *          for its "Painting" title (see draw_object_properties()).
         */
        GEO::vec4f title_color_ = GEO::vec4f(0.05f, 0.18f, 0.50f, 1.0f);
    };
}

#endif //GEOLIO_BASE_OBJECT_H
