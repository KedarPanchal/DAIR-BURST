#ifndef WALL_GEOMETRY_HPP
#define WALL_GEOMETRY_HPP

#include <optional>
#include <list>
#include <initializer_list>
#include <variant>

#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/intersections.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_arrangement_2.h>
#include <CGAL/offset_polygon_2.h>

#include "types.hpp"
#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "robot.hpp"

namespace BURST::geometry {

    // Forward declare WallGeometry for ConfigurationGeometryImpl
    class WallGeometry;

    namespace detail {
        // This class is intended to be used internally and not as an API
        // This is because ConfigurationGeometry should never be instantiated directly
        // Thus it is a private nested class of WallGeometry
        class ConfigurationGeometryImpl : public ConfigurationGeometry {
        private:
            CurvedPolygon2D configuration_shape;

            ConfigurationGeometryImpl(const CurvedPolygon2D& shape) noexcept : configuration_shape{shape} {}
            ConfigurationGeometryImpl(CurvedPolygon2D&& shape) noexcept : configuration_shape{std::move(shape)} {}

            template <typename Iter>
            static std::unique_ptr<ConfigurationGeometryImpl> create(Iter begin, Iter end) noexcept {
                CurvedPolygon2D configuration_polygon{begin, end};
                return std::unique_ptr<ConfigurationGeometryImpl>{new ConfigurationGeometryImpl{std::move(configuration_polygon)}};
            }

        public:
            curve_iterator curve_begin() const noexcept override {
                return this->configuration_shape.curves_begin();
            }
            curve_iterator curve_end() const noexcept override {
                return this->configuration_shape.curves_end();
            }

            winding_order orientation() const noexcept override {
                return this->configuration_shape.orientation();
            }

            void render(scene& scene) const noexcept override {
                // TODO: Add a z-offset to the configuration geometry rendering so that it's visible
                // Right now, we're getting lucky with how we ordered the rendering, but this could change as the scene gets more complex
                curved_polygon_options graphics_options = curved_polygon_options();
                graphics_options.face_color = [](const CurvedPolygonArrangement2D& polygon, CurvedPolygonArrangement2D::Face_const_handle fh) noexcept {
                    return color(138, 154, 91);  // Light green configuration space
                };
                graphics_options.colored_face = [](const CurvedPolygonArrangement2D& polygon, CurvedPolygonArrangement2D::Face_const_handle fh) noexcept {
                    return true;
                };
                // TODO: Fix such that a curved shape can be rendered
                CurvedPolygonArrangement2D graphics_arrangement;
                CGAL::insert(graphics_arrangement, this->configuration_shape.curves_begin(), this->configuration_shape.curves_end());
                CGAL::add_to_graphics_scene(graphics_arrangement, scene, graphics_options); 
            }

            friend class BURST::geometry::WallGeometry; // For access to private constructor
        };
    }

    /*
     * WallGeometry represents the geometry of the walls in the environment. It is defined by a polygon.
     */
    class WallGeometry : public Renderable {
    private:
        
        Polygon2D wall_shape;
        polygon_options wall_render_options;

    protected: 
        // Protected constructors since the public API is through the static create method
        // Abstracting this away to protected constructors allows subclassing WallGeometry in a test environment without depending on the static create method and its constraints
        WallGeometry(const Polygon2D& shape) noexcept : wall_shape{shape} {}
        WallGeometry(Polygon2D&& shape) noexcept : wall_shape{std::move(shape)} {}

        // Protected method since the public API depends on the robot
        // Abstracting this away to a protected method allows subclassing WallGeometry in a test environment without depending on the Robot class
        std::unique_ptr<ConfigurationGeometry> constructConfigurationGeometry(const rscalar& robot_radius) const noexcept {
            std::list<CurvedPolygon2D> inner_minkowski_polygons;
            auto inner_minkowski = CGAL::inset_polygon_2(this->wall_shape, robot_radius, ConicTraits(), std::back_inserter(inner_minkowski_polygons));
        }

    public:
        template <typename Iter>
        static std::optional<WallGeometry> create(Iter begin, Iter end) noexcept {
            // Can't make a polygon with 2 or fewer points
            if (std::distance(begin, end) <= 2) return std::nullopt;

            // Check for self-intersection and overall simplicity of the polygon
            if (!CGAL::is_simple_2(begin, end)) return std::nullopt;

            // If there's collinear points, the polygon is degenerate, so we can't create a wall geometry
            if (CGAL::orientation_2(begin, end) == CGAL::COLLINEAR) return std::nullopt;

            Polygon2D wall_polygon{begin, end};
            return std::optional<WallGeometry>{WallGeometry{std::move(wall_polygon)}};
        }
        static std::optional<WallGeometry> create(std::initializer_list<Point2D> points) noexcept {
            return WallGeometry::create(points.begin(), points.end());
        }
        // Template is not needed for any implementation, but is needed for Robot
        // Thus this can be ommitted when called and the template parameters can be inferred
        template <typename R, typename M>
        std::optional<std::monostate> generateConfigurationGeometry(Robot<R, M>& robot) const noexcept {
            auto config_geometry = this->constructConfigurationGeometry(robot.getRadius());
            if (!config_geometry) return std::nullopt; // Degenerate configuration geometry, can't set it for the robot
            robot.setConfigurationEnvironment(std::move(config_geometry));

            // monostate is used as a replacement for void while still keeping the return type as an optional to indicate failure
            return std::optional<std::monostate>{std::monostate{}}; 
        }

        void render(scene& scene) const noexcept override {
            polygon_options graphics_options = polygon_options();
            graphics_options.face_color = [](const Polygon2D& polygon, void* fh) noexcept {
                return color(173, 216, 230);  // Light blue walls
            };
            graphics_options.colored_face = [](const Polygon2D& polygon, void* fh) noexcept {
                return true;
            };

            CGAL::add_to_graphics_scene(this->wall_shape, scene, graphics_options);
        }

        friend class std::unique_ptr<WallGeometry>;
    };

}

#endif 
