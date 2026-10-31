//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "colormap.h"

#include <cstddef>
#include <random>
#include <utility>
#include <vector>

#include <geogram_gfx/basic/GL.h>
// The GeoGram colormaps GeoBox offers (see ColormapLibrary::initialize()) and
// the ones GeoBox adds, as embedded XPM definitions. Only this translation unit
// includes them: they are data, not interface, so the header that declares the
// colormap table stays light.
#include <geogram_gfx/gui/colormaps/french.xpm>
#include <geogram_gfx/gui/colormaps/rainbow.xpm>
#include <geogram_gfx/gui/colormaps/cei_60757.xpm>
#include <geogram_gfx/gui/colormaps/inferno.xpm>
#include <geogram_gfx/gui/colormaps/blue_red.xpm>
#include "colormaps/turbo.xpm"
#include "colormaps/transparent.xpm"
#include "colormaps/grayscale.xpm"

namespace geolio::geobox
{
    GLuint create_nearest_colormap_texture(
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

    void init_colormap(
        const std::string& name,
        const char* const* xpm_data,
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

    ColormapLibrary& ColormapLibrary::instance(
        ) {
        static ColormapLibrary library;
        return library;
    }

    void ColormapLibrary::initialize(
        ) {
        // Rebuilding is what a caller needs after the GL context was lost, so
        // the table is emptied (and its textures released) first.
        terminate();

        // The subset of the GeoGram colormaps GeoBox offers: the 1st, 4th,
        // 5th, 6th and 10th of the list SimpleApplication::init_colormaps()
        // builds. They are named rather than indexed, so the subset still
        // selects the same colormaps if GeoGram reorders its table.
        //
        // A current GL context is needed from here on: every entry is uploaded
        // as a texture, and so is its GL_NEAREST-filtered variant.
        add_xpm("french", french_xpm);
        add_xpm("rainbow", rainbow_xpm);
        add_xpm("cei_60757", cei_60757_xpm);
        add_xpm("inferno", inferno_xpm);
        add_xpm("blue_red", blue_red_xpm);

        // Colormaps GeoBox adds to the GeoGram ones (see the XPM definitions
        // included at the top of this file). They are the 5th, 17th and 18th
        // colormaps of Graphite.
        add_xpm("turbo", turbo_xpm);
        add_xpm("transparent", transparent_xpm);
        add_xpm("grayscale", grayscale_xpm);

        add_random();
    }

    void ColormapLibrary::add_xpm(
        const std::string& name,
        const char* const* xpm_data
        ) {
        init_colormap(name, xpm_data, colormaps_);
        const ColormapInfo& colormap = colormaps_.back();
        owned_textures_.push_back(colormap.texture);
        if (colormap.nearest_texture != 0 &&
            colormap.nearest_texture != colormap.texture) {
            owned_textures_.push_back(colormap.nearest_texture);
        }
    }

    void ColormapLibrary::add_random(
        ) {
        constexpr GLsizei kSize = 256;
        std::vector<GLubyte> pixels(
            static_cast<std::size_t>(4) * kSize);
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<int> dist(0, 255);
        for (GLsizei i = 0; i < kSize; ++i) {
            pixels[4 * static_cast<std::size_t>(i) + 0] =
                static_cast<GLubyte>(dist(rng));
            pixels[4 * static_cast<std::size_t>(i) + 1] =
                static_cast<GLubyte>(dist(rng));
            pixels[4 * static_cast<std::size_t>(i) + 2] =
                static_cast<GLubyte>(dist(rng));
            pixels[4 * static_cast<std::size_t>(i) + 3] = 255;
        }

        ColormapInfo info;
        info.name = RANDOM_COLORMAP_NAME;
        glGenTextures(1, &info.texture);
        glBindTexture(GL_TEXTURE_2D, info.texture);
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, kSize, 1, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glBindTexture(GL_TEXTURE_2D, 0);
        owned_textures_.push_back(info.texture);

        // This texture already samples with GL_NEAREST: it is its own
        // hard-boundary variant. Interpolating the random texels would erase
        // the distinct color each value is meant to get.
        info.nearest_texture = info.texture;

        colormaps_.push_back(std::move(info));
    }

    const ColormapInfo* ColormapLibrary::colormap(
        const GEO::index_t index
        ) const {
        if (index >= colormaps_.size())
            return nullptr;
        return &colormaps_[index];
    }

    void ColormapLibrary::set_colormaps(
        std::vector<ColormapInfo> colormaps
        ) {
        // The entries being replaced may be the ones this library built, whose
        // textures it owns; the ones being installed are the caller's.
        release_textures();
        colormaps_ = std::move(colormaps);
    }

    void ColormapLibrary::terminate(
        ) {
        release_textures();
        colormaps_.clear();
    }

    void ColormapLibrary::release_textures(
        ) {
        for (const GLuint texture : owned_textures_) {
            if (texture != 0)
                glDeleteTextures(1, &texture);
        }
        owned_textures_.clear();
    }
}
