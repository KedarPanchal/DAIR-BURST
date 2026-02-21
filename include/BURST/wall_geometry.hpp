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
            ClosedCurve2D configuration_shape;
            
            template <typename Iter>
            ConfigurationGeometryImpl(Iter begin, Iter end) noexcept {
                this->configuration_shape = ClosedCurve2D{};
                CGAL::insert(this->configuration_shape, begin, end);
            }

            template <typename Iter>
            static std::unique_ptr<ConfigurationGeometryImpl> create(Iter begin, Iter end) noexcept {
                return std::unique_ptr<ConfigurationGeometryImpl>{new ConfigurationGeometryImpl{begin, end}};
            }

        public:
            
            vertex_iterator<ClosedCurve2D> vertex_begin() const noexcept override {
                return this->configuration_shape.vertices_begin();
            }
            vertex_iterator<ClosedCurve2D> vertex_end() const noexcept override {
                return this->configuration_shape.vertices_end();
            }
            edge_iterator<ClosedCurve2D> edge_begin() const noexcept override {
                return this->configuration_shape.edges_begin();
            }
            edge_iterator<ClosedCurve2D> edge_end() const noexcept override {
                return this->configuration_shape.edges_end();
            }
            halfedge_iterator<ClosedCurve2D> directed_edge_begin() const noexcept override {
                return this->configuration_shape.halfedges_begin();
            }
            halfedge_iterator<ClosedCurve2D> directed_edge_end() const noexcept override {
                return this->configuration_shape.halfedges_end();
            }

            operator ClosedCurve2D() const noexcept override {
                return this->configuration_shape;
            }

            void render(scene& scene) const noexcept override {
                // TODO: Add a z-offset to the configuration geometry rendering so that it's visible
                // Right now, we're getting lucky with how we ordered the rendering, but this could change as the scene gets more complex
                closed_curve_options graphics_options = closed_curve_options();
                graphics_options.face_color = [](const ClosedCurve2D& polygon, ClosedCurve2D::Face_const_handle fh) noexcept {
                    return color(138, 154, 91);  // Light green configuration space
                };
                graphics_options.colored_face = [](const ClosedCurve2D& polygon, ClosedCurve2D::Face_const_handle fh) noexcept {
                    return true;
                };
                graphics_options.edge_color = [](const ClosedCurve2D& polygon, ClosedCurve2D::Halfedge_const_handle eh) noexcept {
                    return color(0, 100, 0);  // Dark green edges for configuration space
                };
                graphics_options.colored_edge = [](const ClosedCurve2D& polygon, ClosedCurve2D::Halfedge_const_handle eh) noexcept {
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
        polygon_options wall_render_options;

    protected: 
        // Protected constructors since the public API is through the static create method
        // Abstracting this away to protected constructors allows subclassing WallGeometry in a test environment without depending on the static create method and its constraints
        WallGeometry(const Polygon2D& shape) noexcept : wall_shape{shape} {}
        WallGeometry(Polygon2D&& shape) noexcept : wall_shape{std::move(shape)} {}

        // Protected method since the public API depends on the robot
        // Abstracting this away to a protected method allows subclassing WallGeometry in a test environment without depending on the Robot class
        std::unique_ptr<ConfigurationGeometry> constructConfigurationGeometry(const rscalar& robot_radius) const noexcept {
            std::list<CGAL::General_polygon_2<ConicTraits>> inner_minkowski_polygons;
            auto inner_minkowski = CGAL::inset_polygon_2(this->wall_shape, robot_radius, ConicTraits(), std::back_inserter(inner_minkowski_polygons));
            
            /*
             * If the inset operation has no elements, it means the robot is too large for the wallgeometry, resulting in a degenerate configuration geonetry
             * If the inset operation has more than 1 element, it means portions of the wallgeometry are too tight for the robot, resulting in a degenerate configuration geometry
             */
            if (inner_minkowski_polygons.size() != 1) return nullptr; 

            return detail::ConfigurationGeometryImpl::create(inner_minkowski_polygons.front().curves_begin(), inner_minkowski_polygons.front().curves_end());
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
        static std::optional<WallGeometry> create(std::initializer_list<Point2D<RationalKernel>> points) noexcept {
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
            CGAL::add_to_graphics_scene(this->wall_shape, scene);
        }

        friend class std::unique_ptr<WallGeometry>;
    };

}

#endif 
