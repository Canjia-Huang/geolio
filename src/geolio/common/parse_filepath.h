//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/20.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_PARSE_FILEPATH_H
#define GEOLIO_PARSE_FILEPATH_H

#include <string>
#include <filesystem>

namespace geolio
{
    /**
     * @brief Extracts the file name without its extension from a path.
     * @details Parses the last path component and removes the trailing extension if present.
     * @param[in] path Input file path to inspect.
     * @return File name without the extension, or an empty string if no valid file name is found.
     */
    inline std::string get_filename(
        const std::string& path
        ) {
        const std::filesystem::path inPath(path);

        const std::string filename_w_ex = inPath.filename().string();
        if (const std::size_t pos = filename_w_ex.find_last_of('.');
            pos != std::string::npos)
            return filename_w_ex.substr(0, pos);
        return "";
    }

    /**
     * @brief Extracts the extension from a file path.
     * @details Uses the final path component and returns the substring after the last dot,
     *          without including the dot itself.
     * @param[in] path Input file path to inspect.
     * @return File extension without the leading dot, or an empty string if no extension exists.
     */
    inline std::string get_extension(
        const std::string& path
        ) {
        const std::filesystem::path inPath(path);

        const std::string filename_w_ex = inPath.filename().string();
        if (const std::size_t pos = filename_w_ex.find_last_of('.');
            pos != std::string::npos)
            return filename_w_ex.substr(pos + 1);
        return "";
    }

    /**
     * @brief Resolves an existing path to its canonical absolute path.
     * @details Converts the input path to an absolute path and normalizes it via the filesystem
     *          canonicalization process. If the path does not exist, an empty string is returned.
     * @param[in] path Source path to resolve.
     * @return Canonical absolute file path, or an empty string if the path is invalid or does not exist.
     */
    inline std::string get_absolute_file_path(
        const std::string& path
        ) {
        if (!std::filesystem::exists(path))
            return "";
        return std::filesystem::canonical(std::filesystem::absolute(path)).string();
    }

    /**
     * @brief Returns the parent directory of an existing file or directory.
     * @details Resolves the path to a canonical absolute form and appends a trailing slash to
     *          the parent directory string for convenient use in path concatenation.
     * @param[in] path Input file or directory path.
     * @return Parent directory path ending with '/', or an empty string if the path does not exist.
     */
    inline std::string get_parent_path(
        const std::string& path
        ) {
        if (!std::filesystem::exists(path))
            return "";
        const std::filesystem::path absolute_path = std::filesystem::canonical(std::filesystem::absolute(path));
        return absolute_path.parent_path().string() + "/";
    }
}

#endif //GEOLIO_PARSE_FILEPATH_H
