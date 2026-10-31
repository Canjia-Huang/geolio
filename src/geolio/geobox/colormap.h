//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_COLORMAP_H
#define GEOLIO_COLORMAP_H
#include <string>
#include <vector>
#include <geogram/basic/numeric.h>
#include <geogram_gfx/third_party/glad/glad.h>

namespace geolio::geobox
{
    /**
     * @brief Name of the random colormap GeoBox appends to the GeoGram ones.
     * @details Every texel of that colormap is a random color, which makes it
     *          suitable to display an attribute with few distinct values, one
     *          color each: setting the min/max fields of MeshObject to the
     *          values of the attribute spreads them over the texels of the
     *          colormap, see MeshObject::colormap_range_min().
     */
    inline constexpr const char* RANDOM_COLORMAP_NAME = "random";

    /**
     * @brief Name of the colormap that hides the values it colors by making
     *        them transparent.
     * @details Its alpha channel is 0 for the lower half of the colormap. The
     *          attribute display discards the fragments colored with such a
     *          texel instead of drawing them black (see MeshObject::draw_scene()).
     */
    inline constexpr const char* TRANSPARENT_COLORMAP_NAME = "transparent";

    /**
     * @brief Stores a generated OpenGL colormap texture and its display name.
     */
    struct ColormapInfo {
        /**
         * @brief Constructs an empty colormap entry with invalid texture ids.
         */
        ColormapInfo() : texture(0), nearest_texture(0) {
        }

        /**
         * @brief OpenGL texture object id for the colormap.
         * @details This is the smoothed texture: linear filtering and mipmaps
         *          interpolate between its texels.
         */
        GLuint texture;

        /**
         * @brief GL_NEAREST-filtered variant of @c texture, or 0 when there is
         *        none.
         * @details Sampled when the attribute is displayed with hard color
         *          boundaries (see MeshObject::smooth_colormap_): nearest
         *          filtering gives every texel of the colormap a flat color
         *          band, whereas @c texture blends the neighboring texels into
         *          a gradient. The random colormap is already built with
         *          GL_NEAREST, so its two texture ids are the same.
         */
        GLuint nearest_texture;

        /**
         * @brief Human-readable name of the colormap.
         */
        std::string name;
    };

    /**
     * @brief Builds the GL_NEAREST-filtered variant of a colormap texture.
     * @details The texels of \p texture are read back and copied into a new
     *          texture whose minification and magnification filters are both
     *          GL_NEAREST, so that sampling it yields one flat color per
     *          colormap entry instead of interpolating between entries. No
     *          mipmaps are generated: GL_NEAREST only samples level 0.
     * @param[in] texture Smoothed colormap texture to copy.
     * @return The nearest-filtered texture, or 0 when \p texture is 0 (no
     *         OpenGL context, as in the unit tests).
     */
    GLuint create_nearest_colormap_texture(GLuint texture);

    /**
     * @brief Builds a single OpenGL texture from an XPM colormap definition.
     * @details The texture is stored smoothed (linear filtering and mipmaps);
     *          its GL_NEAREST-filtered variant is built as well, so that the
     *          attribute can be displayed with hard color boundaries (see
     *          ColormapInfo::nearest_texture).
     * @param[in] name Display name associated with the colormap.
     * @param[in] xpm_data Pointer array describing the XPM image data.
     * @param[out] colormaps Output list that receives the newly created texture entry.
     */
    void init_colormap(
        const std::string& name,
        const char* const* xpm_data,
        std::vector<ColormapInfo>& colormaps
    );

    /**
     * @brief The colormaps GeoBox offers, as one process-wide service.
     * @details The available colormaps do not depend on the object that draws
     *          them: they are a constant of the application, built once while
     *          the OpenGL context is current, and read by every object that
     *          displays a scalar attribute through a colormap (see
     *          MeshObject::colormap_texture()). Keeping them here is what lets
     *          MeshObject be constructed without being handed a colormap table,
     *          and what lets every object see the same table -- including the
     *          objects created by the sub-applications (see
     *          BaseApplicationRegistry), which do not have access to the
     *          application's own members.
     * @details The library owns the textures it builds: terminate() deletes
     *          them, and both calls must be made while the GL context that
     *          built them is current. This mirrors the image texture cache of
     *          the TEXTURE painting mode (see release_texture_cache()).
     * @details The table is empty until initialize() is called, so the unit
     *          tests -- which run without a GL context, and install a table of
     *          their own through set_colormaps() -- never touch OpenGL here.
     */
    class ColormapLibrary {
    public:
        /**
         * @brief Gets the process-wide library instance.
         */
        static ColormapLibrary& instance();

        /**
         * @brief Builds the GeoBox colormap table.
         * @details Offers a subset of the GeoGram colormaps built by
         *          SimpleApplication::init_colormaps() -- the 1st, 4th, 5th,
         *          6th and 10th of that list (french, rainbow, cei_60757,
         *          inferno, blue_red) -- followed by the colormaps GeoBox adds
         *          (turbo, transparent and grayscale, the 5th, 17th and 18th
         *          colormaps of Graphite) and by the random one. The table is
         *          rebuilt from scratch, so calling this again after
         *          terminate() is what a recreated GL context needs.
         * @details A current OpenGL context is required: every entry is
         *          uploaded as a texture, together with its GL_NEAREST-filtered
         *          variant (see ColormapInfo::nearest_texture).
         */
        void initialize();

        /**
         * @brief Deletes the textures the library owns and empties the table.
         * @details Must be called while the GL context that built the textures
         *          is still current.
         */
        void terminate();

        /**
         * @brief Tests whether the table is empty, i.e. whether neither
         *        initialize() nor set_colormaps() was called.
         */
        [[nodiscard]] bool empty() const {
            return colormaps_.empty();
        }

        /**
         * @brief Returns the colormap table.
         * @details The reference stays valid until the next initialize(),
         *          terminate() or set_colormaps(), so callers read the entries
         *          they need instead of holding on to the reference.
         */
        [[nodiscard]] const std::vector<ColormapInfo>& colormaps() const {
            return colormaps_;
        }

        /**
         * @brief Returns the number of colormaps in the table.
         */
        [[nodiscard]] GEO::index_t size() const {
            return static_cast<GEO::index_t>(colormaps_.size());
        }

        /**
         * @brief Returns one colormap of the table.
         * @param[in] index Index of the colormap in colormaps().
         * @return The colormap, or nullptr when @p index is out of range.
         */
        [[nodiscard]] const ColormapInfo* colormap(GEO::index_t index) const;

        /**
         * @brief Installs a colormap table built elsewhere.
         * @details Lets a caller (typically a test, or a tool that builds its
         *          own colormaps) replace the table without touching OpenGL:
         *          no texture is created, and the textures of the installed
         *          entries stay owned by the caller. The textures of the table
         *          being replaced are released first, as terminate() does.
         * @param[in] colormaps Table to install, in display order.
         */
        void set_colormaps(std::vector<ColormapInfo> colormaps);

    private:
        /**
         * @brief Builds a colormap from an XPM definition and takes ownership
         *        of the two textures it creates.
         */
        void add_xpm(const std::string& name, const char* const* xpm_data);

        /**
         * @brief Appends the colormap whose every texel is a random color.
         * @details Built with GL_NEAREST filtering, so that the individual
         *          random texels stay distinct instead of being blended into a
         *          gradient: the colormap is meant to give a distinct color to
         *          each of a few values.
         */
        void add_random();

        /**
         * @brief Deletes the textures the library owns, if any.
         */
        void release_textures();

        ColormapLibrary() = default;

        /** Colormaps offered by the library, in display order. */
        std::vector<ColormapInfo> colormaps_;

        /**
         * @brief Textures built by the library, deleted by terminate().
         * @details An entry's two textures are recorded here when the library
         *          created them, which is what tells them apart from the
         *          textures of a table installed by set_colormaps().
         */
        std::vector<GLuint> owned_textures_;
    };
}

#endif //GEOLIO_COLORMAP_H
