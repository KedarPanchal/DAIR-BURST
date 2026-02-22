#ifndef WALL_GEOMETRY_HPP
#define WALL_GEOMETRY_HPP

#include <optional>
#include <initializer_list>
#include <vector>
#include <variant>

#include <CGAL/approximated_offset_2.h>
#include <CGAL/draw_polygon_2.h>

#include "numeric_types.hpp"
#include "geometric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "robot.hpp"

namespace BURST::geometry {
    
    // Forward declare WallGeometry for ConfigurationGeometryImpl
    class WallGeometry;

    namespace detail {
        // This class is intended to be used internally and not as an API
        // This is because ConfigurationGeometry should never be instantiated directly
        // This is why its constructors are private and only accessible by WallGeometry
        class ConfigurationGeometryImpl : public ConfigurationGeometry {
        private:
            Polygon2D configuration_shape;

            ConfigurationGeometryImpl(const Polygon2D& shape) noexcept : configuration_shape{shape} {}
            ConfigurationGeometryImpl(Polygon2D&& shape) noexcept : configuration_shape{std::move(shape)} {}

            template <typename VertexIter>
            static std::unique_ptr<ConfigurationGeometryImpl> create(VertexIter begin, VertexIter end) noexcept {
                Polygon2D configuration_polygon{begin, end};
                return std::unique_ptr<ConfigurationGeometryImpl>{new ConfigurationGeometryImpl{std::move(configuration_polygon)}};
            }

        public:
            edge_iterator edge_begin() const noexcept override {
                return this->configuration_shape.edges_begin();
            }
            edge_iterator edge_end() const noexcept override {
                return this->configuration_shape.edges_end();
            }
            vertex_iterator vertex_begin() const noexcept override {
                return this->configuration_shape.vertices_begin();
            }
            vertex_iterator vertex_end() const noexcept override {
                return this->configuration_shape.vertices_end();
            }

            winding_order orientation() const noexcept override {
                return this->configuration_shape.orientation();
            }

            void render(graphics::Scene& scene) const noexcept override {
                // TODO: Add a z-offset to the configuration geometry rendering so that it's visible
                // Right now, we're getting lucky with how we ordered the rendering, but this could change as the scene gets more complex
                graphics::PolygonOptions graphics_options{};
                graphics_options.face_color = [](const Polygon2D& polygon, void* fh) noexcept {
                    return graphics::Color(138, 154, 91);  // Light green configuration space
                };
                graphics_options.colored_face = [](const Polygon2D& polygon, void* fh) noexcept {
                    return true;
                };

                CGAL::add_to_graphics_scene(this->configuration_shape, scene, graphics_options); 
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
        graphics::PolygonOptions wall_render_options;

    protected: 
        // Protected constructors since the public API is through the static create method
        // Abstracting this away to protected constructors allows subclassing WallGeometry in a test environment without depending on the static create method and its constraints
        WallGeometry(const Polygon2D& shape) noexcept : wall_shape{shape} {}
        WallGeometry(Polygon2D&& shape) noexcept : wall_shape{std::move(shape)} {}

        // Protected method since the public API depends on the robot
        // Abstracting this away to a protected method allows subclassing WallGeometry in a test environment without depending on the Robot class
        std::unique_ptr<ConfigurationGeometry> constructConfigurationGeometry(const numeric::fscalar& robot_radius) const noexcept {
            // TODO: Use an approximated_inset_2 algorithm to construct the Minkowski difference of the wall polygon and a disk of radius robot_radius
            // Do NOT under ANY CIRCUMSTANCE use the inset_2 algorithm, since this gives an exact result at the cost of going to a massive war against the type system
            // As tempting as it is, just find a nice epsilon and call it a day instead of going through template hell
            
            // Create a vector to store the outputted inset Minkowski difference polygons
            std::vector<Polygon2D> inset_results;
            // Compute the Minkowski difference
            // TODO: Find an actually good epsilon instead of this approximation
            CGAL::approximated_inset_2(this->wall_shape, robot_radius, 0.000001, std::back_inserter(inset_results));
            /*
             * If there are no polygons, then the wall is too small for the robot and no configuration space could be made
             * If there are multiple polygons, then there were regions too tight for the robot to fit in, and no configuration space could be made
             * In both cases, return nullptr
             */
            if (inset_results.size() != 1) return nullptr;
            // Otherwise, create a pointer to a configuration space and return it
            return detail::ConfigurationGeometryImpl::create(inset_results.front().vertices_begin(), inset_results.front().vertices_end());
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

        void render(graphics::Scene& scene) const noexcept override {
            graphics::PolygonOptions graphics_options{};
            graphics_options.face_color = [](const Polygon2D& polygon, void* fh) noexcept {
                return graphics::Color(173, 216, 230);  // Light blue walls
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
