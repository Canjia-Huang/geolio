//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_COLORMAP_H
#define GEOLIO_COLORMAP_H
#include <cstddef>
#include <string>
#include <vector>
#include <geogram_gfx/third_party/glad/glad.h>
#include <geogram_gfx/gui/colormaps/french.xpm>
#include <geogram_gfx/gui/colormaps/black_white.xpm>
#include <geogram_gfx/gui/colormaps/viridis.xpm>
#include <geogram_gfx/gui/colormaps/rainbow.xpm>
#include <geogram_gfx/gui/colormaps/cei_60757.xpm>
#include <geogram_gfx/gui/colormaps/inferno.xpm>
#include <geogram_gfx/gui/colormaps/magma.xpm>
#include <geogram_gfx/gui/colormaps/parula.xpm>
#include <geogram_gfx/gui/colormaps/plasma.xpm>
#include <geogram_gfx/gui/colormaps/blue_red.xpm>

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
    inline GLuint create_nearest_colormap_texture(
        const GLuint texture
        ) {
        if (texture == 0)
            return 0;

        glBindTexture(GL_TEXTURE_2D, texture);
        GLint width = 0;
        GLint height = 0;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
        if (width <= 0 || height <= 0) {
            glBindTexture(GL_TEXTURE_2D, 0);
            return 0;
        }

        std::vector<GLubyte> texels(
            static_cast<std::size_t>(4) * width * height);
        glGetTexImage(
            GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, texels.data());

        GLuint result = 0;
        glGenTextures(1, &result);
        glBindTexture(GL_TEXTURE_2D, result);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, texels.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glBindTexture(GL_TEXTURE_2D, 0);
        return result;
    }

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
    inline void init_colormap(
        const std::string& name,
        const char** xpm_data,
        std::vector<ColormapInfo>& colormaps
        ) {
        colormaps.emplace_back();
        colormaps.rbegin()->name = name;
        glGenTextures(1, &colormaps.rbegin()->texture);
        glBindTexture(GL_TEXTURE_2D, colormaps.rbegin()->texture);
        GEO::glTexImage2Dxpm(xpm_data);
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(
            GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR
        );
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBindTexture(GL_TEXTURE_2D, 0);
        colormaps.rbegin()->nearest_texture =
            create_nearest_colormap_texture(colormaps.rbegin()->texture);
    }

    /**
     * @brief Initializes the built-in GeoBox colormap list.
     * @param[out] colormaps Vector to populate with the available colormap entries.
     */
    inline void init_colormaps(
        std::vector<ColormapInfo>& colormaps
        ) {
        init_colormap("french", french_xpm, colormaps);
        init_colormap("black_white", black_white_xpm, colormaps);
        init_colormap("viridis", viridis_xpm, colormaps);
        init_colormap("rainbow", rainbow_xpm, colormaps);
        init_colormap("cei_60757", cei_60757_xpm, colormaps);
        init_colormap("inferno", inferno_xpm, colormaps);
        init_colormap("magma", magma_xpm, colormaps);
        init_colormap("parula", parula_xpm, colormaps);
        init_colormap("plasma", plasma_xpm, colormaps);
        init_colormap("blue_red", blue_red_xpm, colormaps);
    }
}

#endif //GEOLIO_COLORMAP_H
