//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/11/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_ATTRIBUTE_MANAGER_APPLICATION_H
#define GEOLIO_ATTRIBUTE_MANAGER_APPLICATION_H
#include <optional>
#include <unordered_map>
#include <utility>
#include <string>
#include <geogram/basic/attributes.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geolio/geobox/application/base_application.h>
#include <geolio/geobox/object/mesh_object.h>
#include <geolio/common/log.h>

namespace geolio::geobox
{
    class AttributeManagerApplication : public BaseApplication {
    public:
        explicit AttributeManagerApplication(ApplicationObjects& objects)
            : BaseApplication("manager", objects)
        {
            menu_tooltip_ = "Attribute manage";
        }

    protected:
        void draw_window_contents() override {
            // The mesh picker: every mesh of the scene has attributes.
            draw_mesh_selector(
                selected_mesh_object_,
                [](const GEO::Mesh& mesh) {
                    return mesh.vertices.nb() > 0;
                });

            ImGui::Separator();

            const auto mesh_object = selected_mesh_object_.lock();
            if (mesh_object == nullptr) {
                ImGui::TextDisabled("Select a mesh to list its attributes.");
                return;
            }

            // Everything the mesh stores, the coordinates of its vertices
            // included: they are an attribute too, but they are the mesh itself
            // and cannot be removed (see AttributeInfo::geometry).
            const auto attributes = mesh_object->attribute_infos(true);
            if (attributes.empty()) {
                ImGui::TextDisabled(
                    "`%s` has no attribute.", mesh_object->name().c_str());
                keep_.clear();
                dimensions_.clear();
                return;
            }

            // The rows only record what the user wants: the attributes left
            // checked are kept, the dimensions typed next to them are the ones
            // to set. Both are applied by the button at the bottom, so that a
            // table read from the stores it changes is never edited under its
            // own feet (see apply_pending_changes()).
            std::size_t nb_removed = 0;
            std::size_t nb_resized = 0;

            // The columns fit what they show: the dimension holds two digits,
            // the check box is as wide as itself, and the name takes the
            // rest.
            const ImGuiStyle& style = ImGui::GetStyle();
            const float digit_width =
                ImGui::CalcTextSize("00").x + 2.0f * style.FramePadding.x;
            const float dim_width =
                digit_width + 2.0f * style.CellPadding.x;
            float type_width = ImGui::CalcTextSize("Type").x;
            for (const auto& attribute : attributes) {
                type_width = std::max(
                    type_width,
                    ImGui::CalcTextSize(
                        element_type_name(attribute).c_str()).x);
            }
            type_width += 2.0f * style.CellPadding.x;
            const float box_size = ImGui::GetFrameHeight();
            const float keep_width = box_size + 2.0f * style.CellPadding.x;

            constexpr ImGuiTableFlags table_flags =
                ImGuiTableFlags_BordersOuter |
                ImGuiTableFlags_BordersInnerV |
                ImGuiTableFlags_RowBg;
            if (ImGui::BeginTable("##attributes", 4, table_flags)) {
                ImGui::TableSetupColumn(
                    "Attribute", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn(
                    "Type", ImGuiTableColumnFlags_WidthFixed, type_width);
                ImGui::TableSetupColumn(
                    "Dim", ImGuiTableColumnFlags_WidthFixed, dim_width);
                // The last column is flush with the right border of the table:
                // it holds nothing but the check box of the row.
                ImGui::TableSetupColumn(
                    "##keep", ImGuiTableColumnFlags_WidthFixed |
                    ImGuiTableColumnFlags_NoHeaderLabel, keep_width);
                ImGui::TableHeadersRow();

                for (const auto& attribute : attributes) {
                    ImGui::TableNextRow();

                    // Two attributes can share a name on different stores
                    // (e.g. "vertices.weight" and "facets.weight"), and the
                    // full name is what tells them apart: it identifies the
                    // widgets of the row.
                    const std::string name = attribute.full_name();
                    ImGui::PushID(name.c_str());

                    // An attribute is kept unless its box was unchecked.
                    const bool kept =
                        keep_.try_emplace(name, true).first->second;
                    if (!kept)
                        ++nb_removed;

                    ImGui::TableSetColumnIndex(0);
                    if (kept) {
                        ImGui::TextUnformatted(name.c_str());
                    }
                    else {
                        // What is about to go is greyed out.
                        ImGui::TextDisabled("%s", name.c_str());
                    }

                    ImGui::TableSetColumnIndex(1);
                    if (kept)
                        ImGui::TextUnformatted(
                            element_type_name(attribute).c_str());
                    else
                        ImGui::TextDisabled(
                            "%s", element_type_name(attribute).c_str());

                    // The dimension is the number of components of an element:
                    // GeoGram changes it in place (see redim_attribute()), with
                    // the field of the row. The value stays pending until the
                    // button of the window is pressed.
                    ImGui::TableSetColumnIndex(2);
                    int dimension = static_cast<int>(attribute.dimension);
                    if (const auto pending = dimensions_.find(name);
                        pending != dimensions_.end()) {
                        dimension = static_cast<int>(pending->second);
                    }
                    const float available = ImGui::GetContentRegionAvail().x;
                    if (available > digit_width)
                        ImGui::SetCursorPosX(
                            ImGui::GetCursorPosX() +
                            0.5f * (available - digit_width));
                    ImGui::SetNextItemWidth(digit_width);
                    // The dimension of the coordinates is the dimension of
                    // the mesh itself (see GEO::MeshVertices::set_dimension(),
                    // which is this very resize): it holds the coordinates of
                    // a point, so a mesh has at least 2 of them.
                    const int min_dimension = attribute.geometry ? 2 : 1;
                    ImGui::DragInt(
                        "##dimension", &dimension, 0.1f, min_dimension, 64,
                        "%d", ImGuiSliderFlags_AlwaysClamp);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip(
                            attribute.geometry
                                ? "Number of coordinates of each vertex, i.e. "
                                  "the dimension of the mesh, applied by the "
                                  "button below."
                                : "Number of components of each element, "
                                  "applied by the button below.");
                    }
                    if (ImGui::IsItemDeactivatedAfterEdit()) {
                        if (dimension == static_cast<int>(attribute.dimension))
                            dimensions_.erase(name);
                        else
                            dimensions_[name] = GEO::index_t(dimension);
                    }
                    if (dimension != static_cast<int>(attribute.dimension))
                        ++nb_resized;

                    // The check box is right aligned in the last column, i.e.
                    // against the right border of the table.
                    ImGui::TableSetColumnIndex(3);
                    const float box_available =
                        ImGui::GetContentRegionAvail().x;
                    if (box_available > box_size) {
                        ImGui::SetCursorPosX(
                            ImGui::GetCursorPosX() +
                            box_available - box_size);
                    }
                    if (attribute.geometry) {
                        // The geometry is part of the mesh: its box says it is
                        // kept, and cannot be unchecked.
                        bool keep_geometry = true;
                        ImGui::BeginDisabled();
                        ImGui::Checkbox("##keep", &keep_geometry);
                        ImGui::EndDisabled();
                    }
                    else {
                        bool keep = kept;
                        if (ImGui::Checkbox("##keep", &keep))
                            keep_[name] = keep;
                    }

                    ImGui::PopID();
                }

                ImGui::EndTable();
            }

            ImGui::Spacing();

            // The rows only hold what the user changed: there is nothing to
            // reset while they still describe the mesh as it is.
            const bool modified = nb_removed != 0 || nb_resized != 0;
            ImGui::BeginDisabled(!modified);
            {
                if (ImGui::Button("Reset", ImVec2(-1.0f, 0.0f)))
                    reset_pending_changes();

                if (ImGui::Button("Perform", ImVec2(-1.0f, 0.0f)))
                    apply_pending_changes(*mesh_object, attributes);
            }
            ImGui::EndDisabled();
        }

        /**
         * @brief Drops what the rows asked for, i.e. goes back to the mesh as
         *        it is.
         * @details Nothing was applied to the mesh yet (see
         *          apply_pending_changes()): forgetting the pending state is
         *          enough for the table to describe the mesh again, with every
         *          attribute kept and every dimension unchanged.
         */
        void reset_pending_changes() {
            keep_.clear();
            dimensions_.clear();
        }

        /**
         * @brief Removes and resizes the attributes the rows asked for.
         * @details The removals go first: an attribute that is not kept is
         *          removed whatever its row says about its dimension. The
         *          pending state is cleared afterwards, so that the rows show
         *          the mesh as it is now.
         * @param[in,out] mesh_object Object whose mesh to change.
         * @param[in] attributes The attributes the table listed; the rows keyed
         *            by their full name.
         */
        void apply_pending_changes(
            MeshObject& mesh_object,
            const std::vector<MeshObject::AttributeInfo>& attributes
            ) {
            for (const auto& attribute : attributes) {
                const std::string name = attribute.full_name();

                // The geometry is part of the mesh: it can be resized, but
                // there is nothing to remove (see AttributeInfo::geometry).
                if (!attribute.geometry) {
                    if (const auto kept = keep_.find(name);
                        kept != keep_.end() && !kept->second) {
                        remove_attribute(mesh_object, attribute);
                        continue;
                    }
                }

                const auto dimension = dimensions_.find(name);
                if (dimension != dimensions_.end() &&
                    dimension->second != attribute.dimension) {
                    redim_attribute(mesh_object, attribute, dimension->second);
                }
            }
            keep_.clear();
            dimensions_.clear();
        }

        /**
         * @brief Name of the type of the elements of @p attribute, as C++
         *        spells it.
         * @details The attribute stores are created for the types registered
         *          with geo_register_attribute_type(), and GeoGram keeps the
         *          C++ name they were registered with; the mangled one it
         *          stores with the attribute is the fallback.
         * @param[in] attribute Attribute to name the type of.
         * @return "double", "vec3", "unsigned int", ...
         */
        static std::string element_type_name(
            const MeshObject::AttributeInfo& attribute
            ) {
            if (GEO::AttributeStore::element_typeid_name_is_known(
                    attribute.element_type)) {
                return GEO::AttributeStore::
                    element_type_name_by_element_typeid_name(
                        attribute.element_type);
            }
            return attribute.element_type;
        }

        /**
         * @brief Changes the number of components of an attribute.
         * @details GeoGram resizes the attribute in place (see
         *          GEO::AttributeStore::redim()): the values it already holds
         *          are kept, and the components it gains are set to the
         *          default value of its element type. The table shows the new
         *          dimension, and the object is told that its mesh changed,
         *          since the renderer uploads the attribute again.
         *          Resizing the coordinates of the vertices is how the
         *          dimension of the mesh is changed: GeoGram's
         *          GEO::MeshVertices::set_dimension() does the same thing.
         * @param[in,out] mesh_object Object whose mesh to change.
         * @param[in] attribute Attribute to resize.
         * @param[in] dimension New number of components; 1 for a scalar
         *            attribute.
         * @return true when the dimension changed.
         */
        static bool redim_attribute(
            MeshObject& mesh_object,
            const MeshObject::AttributeInfo& attribute,
            const GEO::index_t dimension
            ) {
            if (dimension == 0 || dimension == attribute.dimension)
                return false;

            GEO::AttributeStore* store =
                mesh_object.mesh()
                    .get_subelements_by_type(attribute.subelements)
                    .attributes()
                    .find_attribute_store(attribute.name);
            if (store == nullptr) {
                LOG::WARN(
                    "The attribute `{}` of `{}` is already gone.",
                    attribute.full_name(),
                    mesh_object.name());
                return false;
            }

            const GEO::index_t previous_dimension = store->dimension();
            store->redim(dimension);
            mesh_object.mesh_changed();

            LOG::INFO(
                "Changed the dimension of `{}` of `{}` from {} to {}.",
                attribute.full_name(),
                mesh_object.name(),
                previous_dimension,
                dimension);
            return true;
        }

        /**
         * @brief Removes an attribute from the mesh of an object.
         * @details The attribute store is destroyed, and the object is told
         *          that its mesh changed: it would otherwise keep drawing from
         *          the buffer objects of the removed attribute, and the
         *          attribute it paints would stay selected.
         * @param[in,out] mesh_object Object whose mesh to change.
         * @param[in] attribute Attribute to remove.
         * @return true when the attribute was removed.
         */
        static bool remove_attribute(
            MeshObject& mesh_object,
            const MeshObject::AttributeInfo& attribute
            ) {
            GEO::AttributesManager& attributes =
                mesh_object.mesh()
                    .get_subelements_by_type(attribute.subelements)
                    .attributes();
            if (attribute.geometry) {
                LOG::ERROR(
                    "`{}` holds the geometry of `{}`: it cannot be removed.",
                    attribute.full_name(),
                    mesh_object.name());
                return false;
            }
            if (!attributes.is_defined(attribute.name)) {
                LOG::WARN(
                    "The attribute `{}` of `{}` is already gone.",
                    attribute.full_name(),
                    mesh_object.name());
                return false;
            }

            attributes.delete_attribute_store(attribute.name);
            mesh_object.mesh_changed();

            LOG::INFO(
                "Removed the attribute `{}` of `{}`.",
                attribute.full_name(),
                mesh_object.name());
            return true;
        }

        /** The mesh object picked with draw_mesh_selector(). */
        std::weak_ptr<MeshObject> selected_mesh_object_;

        /**
         * @brief Whether the row of an attribute is kept, by full name.
         * @details An attribute that is not in there is kept: the rows start
         *          checked, and only a box the user unchecked is recorded.
         */
        std::unordered_map<std::string, bool> keep_;

        /**
         * @brief The dimensions typed in the rows, by full name.
         * @details A name that is not in there has its dimension unchanged; the
         *          values are applied when the button of the window is pressed
         *          (see apply_pending_changes()).
         */
        std::unordered_map<std::string, GEO::index_t> dimensions_;
    };
}

#endif //GEOLIO_ATTRIBUTE_MANAGER_APPLICATION_H
