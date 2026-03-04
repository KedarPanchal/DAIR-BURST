#ifndef BURST_WALL_SPACE_HPP
#define BURST_WALL_SPACE_HPP

#include <optional>
#include <initializer_list>
#include <ranges>
#include <memory>
#include <vector>
#include <iterator>
#include <variant>

#include <CGAL/approximated_offset_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include "numeric_types.hpp"
#include "geometric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "robot.hpp"

namespace BURST::geometry {
    
    // Internal namespace for implementation details of WallSpace, not intended for public API
    namespace detail {
        template <typename C, typename V>
        concept valid_wall_space_input_collection = std::ranges::sized_range<C> && std::same_as<std::remove_cv_t<std::ranges::range_value_t<C>>, V>;
    }
    /*
     * WallSpace represents the geometry of the walls in the environment. It is defined by a polygon.
     */
    class WallSpace : public Renderable {
    private:
        
        HoledPolygon2D wall_shape;
        graphics::PolygonOptions wall_render_options;

    protected: 
        /* 
         * Protected constructors since the public API is through the static create method
         * Abstracting this away to protected constructors allows subclassing WallSpace in a test environment without depending on the static create method and its constraints
         */
        WallSpace(const Polygon2D& shape) noexcept : wall_shape{shape} {}
        template <detail::valid_wall_space_input_collection<Polygon2D> C>
        WallSpace(const Polygon2D& shape, C holes) noexcept : wall_shape{shape} {
            for (const Polygon2D& hole : holes) {
                // Holes must be oriented clockwise, so reverse the orientation if it's not
                if (hole.orientation() != CGAL::CLOCKWISE) {
                    Polygon2D oriented_hole = hole;
                    oriented_hole.reverse_orientation();
                    this->wall_shape.add_hole(oriented_hole);
                } else {
                    this->wall_shape.add_hole(hole);
                }
            }
        }

        // Protected method since the public API depends on the robot
        // Abstracting this away to a protected method allows subclassing WallSpace in a test environment without depending on the Robot class
        std::unique_ptr<ConfigurationSpace> constructConfigurationSpace(const numeric::fscalar& robot_radius) const noexcept {
            // TODO: Find an actually good epsilon instead of this approximation
            const double EPSILON = 0.000001;

            // Compute the inset of the outer boundary of the wall polygon
            std::vector<CurvilinearPolygon2D> outer_inset_results;
            CGAL::approximated_inset_2(this->wall_shape.outer_boundary(), robot_radius, EPSILON, std::back_inserter(outer_inset_results));
            /*
             * If there are no polygons, then the wall is too small for the robot and no configuration space could be made
             * If there are multiple polygons, then there were regions too tight for the robot to fit in, and no configuration space could be made
             * In both cases, return nullptr
             */
            if (outer_inset_results.size() != 1) return nullptr;
            // Reverse the resulting polygon's rotation if it's not counterclockwise
            if (outer_inset_results.front().orientation() != CGAL::COUNTERCLOCKWISE) outer_inset_results.front().reverse_orientation();

            // Compute the outset of all of the holes within the wall polygon, since the holes need to be expanded by the robot radius as well
            std::vector<CurvilinearPolygon2D> hole_outset_results;
            for (const auto& hole : this->wall_shape.holes()) {
                // Holes are CW, so reverse the orientation to ensure it's a valid polygon for CGAL
                Polygon2D oriented_hole = hole;
                if (oriented_hole.orientation() != CGAL::COUNTERCLOCKWISE) oriented_hole.reverse_orientation();
                // approximated_offset_2 returns a holed polygon in case the input has holes, but since holes are only simple polygons, take the outer boundary of the result
                hole_outset_results.push_back(CGAL::approximated_offset_2(oriented_hole, robot_radius, EPSILON).outer_boundary());
            }

            // Ensure that the holes and boundary don't overlap after the inset and outset operations
            // Check if the holes overlap
            if (CGAL::do_intersect(hole_outset_results.begin(), hole_outset_results.end())) return nullptr;
            // Check if the holes overlap with the boundary
            // Create a vector to store all of the polygons for the intersection check
            std::vector<CurvilinearPolygon2D> all_polygons{outer_inset_results.front()};
            all_polygons.insert(all_polygons.end(), hole_outset_results.begin(), hole_outset_results.end());

            // TODO: Fix this to also include the holes
            return ConfigurationSpace::create(outer_inset_results.front());
        }

    public:
        template <detail::valid_wall_space_input_collection<Point2D> C>
        static std::optional<WallSpace> create(C points) noexcept {
            // Can't make a polygon with 2 or fewer points
            if (points.size() <= 2) return std::nullopt;
            // Check for self-intersection and overall simplicity of the polygon
            if (!CGAL::is_simple_2(points.begin(), points.end())) return std::nullopt;
            // If there's collinear points, the polygon is degenerate, so we can't create a wall geometry
            if (CGAL::orientation_2(points.begin(), points.end()) == CGAL::COLLINEAR) return std::nullopt;

            // Create the wall polygon from the input points
            Polygon2D wall_polygon{points.begin(), points.end()};
            // If the polygon is not oriented counterclockwise, reverse the orientation to ensure it's a valid polygon for CGAL
            if (wall_polygon.orientation() != CGAL::COUNTERCLOCKWISE) wall_polygon.reverse_orientation();

            return WallSpace{wall_polygon};
        }
        template <detail::valid_wall_space_input_collection<Point2D> C1, detail::valid_wall_space_input_collection<Polygon2D> C2>
        static std::optional<WallSpace> create(C1 points, C2 holes) noexcept {
            // Can't make a polygon with 2 or fewer points
            if (points.size() <= 2) return std::nullopt;
            // Create the wall polygon from the input points now for use in the hole validation checks
            Polygon2D wall_polygon{points.begin(), points.end()};
            // If the polygon is not oriented counterclockwise, reverse the orientation to ensure it's a valid polygon for CGAL
            if (wall_polygon.orientation() != CGAL::COUNTERCLOCKWISE) wall_polygon.reverse_orientation();
            
            // Check that each hole is clockwise-oriented and that no vertices lie on the boundary of the wall polygon
            // Store oriented holes in a vector
            std::vector<Polygon2D> oriented_holes;
            for (const Polygon2D& hole : holes) {
                // Holes must be oriented clcokwise, so reverse the orientation if not
                Polygon2D oriented_hole = hole;
                if (hole.orientation() != CGAL::CLOCKWISE) oriented_hole.reverse_orientation();
                // Check that no vertices of the hole lie on the boundary of the wall polygon
                for (const Point2D& vertex : oriented_hole.vertices()) {
                    if (wall_polygon.has_on_boundary(vertex)) return std::nullopt;
                }
                // Append to the vector of oriented holes for use in creating the holed Polygon
                oriented_holes.push_back(oriented_hole);
            }

            // Construct a holed polygon from the wall polygon and the holes
            HoledPolygon2D wall_shape{wall_polygon, oriented_holes.begin(), oriented_holes.end()};
            /*
             * Check if the resulting holed polygon is valid, which means:
             * The outer boundary is a simple polygon and counterclockwise-oriented
             * Each hole is a simple polygon and clockwise-oriented
             * Holes are inside the outer boundary and do not intersect with each other
             */
            if (!CGAL::is_valid_polygon_with_holes(wall_shape, LinearTraits{})) return std::nullopt;

            // If the resulting holed polygon is valid, construct the WallSpace
            return WallSpace{wall_polygon, oriented_holes};
        }

        // Template is not needed for any implementation, but is needed for Robot
        // Thus this can be ommitted when called and the template parameters can be inferred
        template <typename R, typename M>
        std::optional<std::monostate> generateConfigurationSpace(Robot<R, M>& robot) const noexcept {
            auto config_geometry = this->constructConfigurationSpace(robot.getRadius());
            if (!config_geometry) return std::nullopt; // Degenerate configuration geometry, can't set it for the robot
            robot.setConfigurationEnvironment(std::move(config_geometry));

            // monostate is used as a replacement for void while still keeping the return type as an optional to indicate failure
            return std::optional<std::monostate>{std::monostate{}}; 
        }

        void render(graphics::Scene& scene) const noexcept override {
            graphics::PolygonOptions graphics_options{};
            graphics_options.face_color = [](const Polygon2D& polygon, void* fh) noexcept {
                return graphics::Color(173, 216, 230);  // Light blue walls
            };
            graphics_options.colored_face = [](const Polygon2D& polygon, void* fh) noexcept {
                return true;
            };
            // TODO: This import (CGAL/draw_polygon_2.h) is broken, find a way to fix
            // CGAL::add_to_graphics_scene(this->wall_shape, scene, graphics_options);
        }

        friend class std::unique_ptr<WallSpace>;
    };

}

#endif 
