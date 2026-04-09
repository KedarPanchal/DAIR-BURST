#ifndef BURST_WALL_SPACE_HPP
#define BURST_WALL_SPACE_HPP

#include <optional>
#include <memory>
#include <iterator>

#include <CGAL/approximated_offset_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/draw_arrangement_2.h>

#include <boost/container/small_vector.hpp>
#include <boost/uuid/uuid.hpp>

#include "numeric_types.hpp"
#include "geometric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "robot.hpp"
#include "logging.hpp"

namespace BURST::geometry {
    
    // WallSpace represents the geometry of the walls in the environment. It is defined by a polygon.
    class WallSpace : public Renderable {
    private:
        using arrangement_t = CGAL::Arrangement_2<CurvedTraits>;
        using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, typename arrangement_t::Vertex_const_handle, typename arrangement_t::Halfedge_const_handle, typename arrangement_t::Face_const_handle>;

        HoledPolygon2D wall_shape;
        graphics_options_t graphics_options;

    protected: 
        // Protected constructors since the public API is through the static create method
        // Abstracting this away to protected constructors allows subclassing WallSpace in a test environment without depending on the static create method and its constraints
        WallSpace(const Polygon2D& shape) noexcept : Renderable{}, wall_shape{shape} {}
        WallSpace(const HoledPolygon2D& shape) noexcept : Renderable{}, wall_shape{shape} {}

        // Protected methods since the public API depends on the robot
        // Abstracting this away to a protected method allows subclassing WallSpace in a test environment without depending on the Robot class

        // Constructs a configuration space given a robot radius
        std::unique_ptr<ConfigurationSpace> constructConfigurationSpace(const numeric::fscalar& robot_radius) const {
            // TODO: Find an actually good epsilon instead of this approximation
            const double EPSILON = 0.000001;

            // Compute the inset of the outer boundary of the wall polygon
            boost::container::small_vector<CurvilinearPolygon2D, 1> outer_inset_results;
            CGAL::approximated_inset_2(this->wall_shape.outer_boundary(), robot_radius, EPSILON, std::back_inserter(outer_inset_results));

            // If there are no polygons, then the wall is too small for the robot and no configuration space could be made
            // If there are multiple polygons, then there were regions too tight for the robot to fit in, and no configuration space could be made
            // TODO: For the above case, check with Dr. Shell if that's something worth allowing in the final sim
            // In both cases, return nullptr
            if (outer_inset_results.size() != 1) {
                BURST_ERROR("Wall polygon is too small for the robot, no configuration space could be generated");
                return nullptr;
            }
            // Reverse the resulting polygon's rotation if it's not counterclockwise
            if (outer_inset_results.front().orientation() != CGAL::COUNTERCLOCKWISE) outer_inset_results.front().reverse_orientation();

            // Create a polygon set to store all Minkowski sum/difference results to create the configuration space
            std::unique_ptr<CurvilinearPolygonSet2D> config_polygon_set = std::make_unique<CurvilinearPolygonSet2D>();
            config_polygon_set->insert(outer_inset_results.front());

            // Compute the outset of all of the holes within the wall polygon, since the holes need to be expanded by the robot radius as well
            // No checks needed for the holes touching the wall since that's a realistic case for when an object is close to a wall
            for (const auto& hole : this->wall_shape.holes()) {
                // Holes are CW, so reverse the orientation to ensure it's a valid polygon for CGAL
                Polygon2D oriented_hole = hole;
                if (oriented_hole.orientation() != CGAL::COUNTERCLOCKWISE) oriented_hole.reverse_orientation();
 
                // Compute the difference between the resultant polygon set and outset hole
                // This is insulated against overlapping holes and border touching
                // This is possible if two holes or a hole and the wall are close enough that the space between is too small for the robot
                config_polygon_set->difference(CGAL::approximated_offset_2(oriented_hole, robot_radius, EPSILON));
            }
            
            // Create the configuration space from the resulting polygon set
            return ConfigurationSpace::create(std::move(config_polygon_set));
        }

    public:
        template <valid_geometric_collection<Point2D> C>
        static std::optional<WallSpace> create(C points) {
            auto wall_polygon_opt = construct_polygon(points);  
            // If nullopt, then the wall polygon was degenerate and we can't create a wall geometry
            return wall_polygon_opt.has_value() ? std::optional<WallSpace>{WallSpace{wall_polygon_opt.value()}} : std::nullopt;
        }
        template <valid_geometric_collection<Point2D> C1, valid_geometric_collection<Polygon2D> C2>
        static std::optional<WallSpace> create(C1 points, C2 holes) {
            auto wall_polygon_opt = construct_polygon(points);
            // Degenerate wall polygon, can't create a wall geometry
            if (!wall_polygon_opt) return std::nullopt; 

            // Create a holed polygon to hold the holes and outer boundary
            HoledPolygon2D wall_shape{wall_polygon_opt.value()};
            
            // Check that each hole is clockwise-oriented
            for (const Polygon2D& hole : holes) {
                // Degenerate hole polygon, can't create a wall geometry
                if (!hole.is_simple()) {
                    BURST_ERROR("Hole polygon is degenerate, can't create a wall geometry");
                    return std::nullopt;
                }
                // Holes must be oriented clockwise, so reverse the orientation if not
                Polygon2D oriented_hole = hole;
                if (hole.orientation() != CGAL::CLOCKWISE) oriented_hole.reverse_orientation();
                // Add the hole
                wall_shape.add_hole(oriented_hole);
            }

            // Check if the resulting holed polygon is valid, which means:
            // The outer boundary is a simple polygon and counterclockwise-oriented
            // Each hole is a simple polygon and clockwise-oriented
            // Holes are inside the outer boundary and do not intersect with each other
            // Return nullopt if any of these conditions are violated
            if (CGAL::is_valid_polygon_with_holes(wall_shape, LinearTraits{})) {
                return WallSpace{wall_shape};
            } else {
                BURST_ERROR("Resulting wall polygon with holes is invalid with one of: degenerate outer boundary, degenerate hole, hole not inside outer boundary, or holes intersecting each other. Can't create a wall geometry");
                return std::nullopt;
            }
        }
        // Inlined overloads for std::initializer_list since it doesn't satisfy the sized_range condition below C++23
        inline static std::optional<WallSpace> create(std::initializer_list<Point2D> points) {
            return create<std::initializer_list<Point2D>>(points);
        }
        template <valid_geometric_collection<Point2D> C>
        inline static std::optional<WallSpace> create(C points, std::initializer_list<Polygon2D> holes) {
            return create<C, std::initializer_list<Polygon2D>>(points, holes);
        }
        template <valid_geometric_collection<Polygon2D> C>
        inline static std::optional<WallSpace> create(std::initializer_list<Point2D> points, C holes) {
            return create<std::initializer_list<Point2D>, C>(points, holes);
        }
        inline static std::optional<WallSpace> create(std::initializer_list<Point2D> points, std::initializer_list<Polygon2D> holes) {
            return create<std::initializer_list<Point2D>>(points, std::initializer_list<Polygon2D>(holes));
        }

        // Template is not needed for any implementation, but is needed for Robot
        // Thus this can be ommitted when called and the template parameters can be inferred
        template <typename T, typename P, typename R, typename D>
        bool generateConfigurationSpace(Robot<T, P, R, D>& robot) const {
            auto config_geometry = this->constructConfigurationSpace(robot.getRadius());
            if (!config_geometry) return false; // Degenerate configuration geometry, can't set it for the robot
            robot.setConfigurationEnvironment(std::move(config_geometry));

            return true;
        }

        void render(graphics::Scene& scene) noexcept override {
            this->graphics_options.colored_face = [](const arrangement_t&, arrangement_t::Face_const_handle) -> bool {
                return true;
            };
            this->graphics_options.face_color = [hash= boost::uuids::hash_value(this->uuid())](const arrangement_t&, arrangement_t::Face_const_handle) -> graphics::Color {
                graphics::Color wall_color;
                size_t half_size = sizeof(decltype(hash)) / 2;
                double hue = static_cast<double>(hash % 360);
                double saturation = static_cast<double>(hash % 100) / 100.0;
                // Switch the upper and lower halves of the hash to get more variability in the value component of the HSV color
                double value = static_cast<double>(((hash >> half_size) | (hash << half_size)) % 100) / 100.0;
                return wall_color.set_hsv(hue, saturation, value);
            };
            
            LinearPolygonSet2D graphics_polygon_set{this->wall_shape};
            CGAL::add_to_graphics_scene(graphics_polygon_set.arrangement(), scene, graphics_options);
        }

        friend class std::unique_ptr<WallSpace>;
    };

}

#endif 
