//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_COLORMAP_H
#define GEOLIO_COLORMAP_H
#include <string>
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
     * @brief Stores a generated OpenGL colormap texture and its display name.
     */
    struct ColormapInfo {
        /**
         * @brief Constructs an empty colormap entry with an invalid texture id.
         */
        ColormapInfo() : texture(0) {
        }

        /**
         * @brief OpenGL texture object id for the colormap.
         */
        GLuint texture;

        /**
         * @brief Human-readable name of the colormap.
         */
        std::string name;
    };

    /**
     * @brief Builds a single OpenGL texture from an XPM colormap definition.
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
