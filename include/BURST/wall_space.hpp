#ifndef BURST_WALL_SPACE_HPP
#define BURST_WALL_SPACE_HPP

#include <optional>
#include <initializer_list>
#include <memory>
#include <vector>
#include <iterator>
#include <variant>

#include <CGAL/approximated_offset_2.h>
#include <CGAL/General_polygon_set_2.h>

#include "numeric_types.hpp"
#include "geometric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "robot.hpp"

namespace BURST::geometry {
    
    /*
     * WallSpace represents the geometry of the walls in the environment. It is defined by a polygon.
     */
    class WallSpace : public Renderable {
    private:
        
        Polygon2D wall_shape;
        graphics::PolygonOptions wall_render_options;

    protected: 
        // Protected constructors since the public API is through the static create method
        // Abstracting this away to protected constructors allows subclassing WallSpace in a test environment without depending on the static create method and its constraints
        WallSpace(const Polygon2D& shape) noexcept : wall_shape{shape} {}
        WallSpace(Polygon2D&& shape) noexcept : wall_shape{std::move(shape)} {}

        // Protected method since the public API depends on the robot
        // Abstracting this away to a protected method allows subclassing WallSpace in a test environment without depending on the Robot class
        std::unique_ptr<ConfigurationSpace> constructConfigurationSpace(const numeric::fscalar& robot_radius) const noexcept {
            // TODO: Use an approximated_inset_2 algorithm to construct the Minkowski difference of the wall polygon and a disk of radius robot_radius
            // Do NOT under ANY CIRCUMSTANCE use the inset_2 algorithm, since this gives an exact result at the cost of going to a massive war against the type system
            // As tempting as it is, just find a nice epsilon and call it a day instead of going through template hell
            
            // Create a vector to store the outputted inset Minkowski difference polygons
            std::vector<CurvilinearPolygon2D> inset_results;
            // Compute the Minkowski difference
            // TODO: Find an actually good epsilon instead of this approximation
            CGAL::approximated_inset_2(this->wall_shape, robot_radius, 0.000001, std::back_inserter(inset_results));
            /*
             * If there are no polygons, then the wall is too small for the robot and no configuration space could be made
             * If there are multiple polygons, then there were regions too tight for the robot to fit in, and no configuration space could be made
             * In both cases, return nullptr
             */
            if (inset_results.size() != 1) return nullptr;

            // Reverse the resulting polygon's rotation if it's not counterclockwise
            if (inset_results.front().orientation() != CGAL::COUNTERCLOCKWISE) {
                inset_results.front().reverse_orientation();
            }
            return ConfigurationSpace::create(inset_results.front());
        }

    public:
        template <typename Iter>
        static std::optional<WallSpace> create(Iter begin, Iter end) noexcept {
            // Can't make a polygon with 2 or fewer points
            if (std::distance(begin, end) <= 2) return std::nullopt;

            // Check for self-intersection and overall simplicity of the polygon
            if (!CGAL::is_simple_2(begin, end)) return std::nullopt;

            // If there's collinear points, the polygon is degenerate, so we can't create a wall geometry
            if (CGAL::orientation_2(begin, end) == CGAL::COLLINEAR) return std::nullopt;

            Polygon2D wall_polygon{begin, end};
            return std::optional<WallSpace>{WallSpace{std::move(wall_polygon)}};
        }
        static std::optional<WallSpace> create(std::initializer_list<Point2D> points) noexcept {
            return WallSpace::create(points.begin(), points.end());
        }
        // Template is not needed for any implementation, but is needed for Robot
        // Thus this can be ommitted when called and the template parameters can be inferred
        template <typename R, typename M>
        std::optional<std::monostate> generateConfigurationGeometry(Robot<R, M>& robot) const noexcept {
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
