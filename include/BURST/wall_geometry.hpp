#ifndef WALL_GEOMETRY_HPP
#define WALL_GEOMETRY_HPP

#include <optional>
#include <initializer_list>
#include <variant>
#include <vector>

#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/intersections.h>
#include <CGAL/draw_polygon_2.h>

#include "types.hpp"
#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "robot.hpp"

namespace BURST::geometry {
    
    /*
     * WallGeometry represents the geometry of the walls in the environment. It is defined by a polygon.
     */
    class WallGeometry : public Renderable {
    private:
        // This class is intended to be used internally and not as an API
        // This is because ConfigurationGeometry should never be instantiated directly
        // Thus it is a private nested class of WallGeometry
        class ConfigurationGeometryImpl : public ConfigurationGeometry {
        private:
            Polygon_2 configuration_shape;

            ConfigurationGeometryImpl(const Polygon_2& shape) noexcept : configuration_shape{shape} {}
            ConfigurationGeometryImpl(Polygon_2&& shape) noexcept : configuration_shape{std::move(shape)} {}

            template <typename Iter>
            static std::unique_ptr<ConfigurationGeometryImpl> create(Iter begin, Iter end) noexcept {
                Polygon_2 configuration_polygon{begin, end};
                return std::unique_ptr<ConfigurationGeometryImpl>{new ConfigurationGeometryImpl{std::move(configuration_polygon)}};
            }

        public:
            std::optional<Segment_2> getEdge(Point_2 intersection_point) const noexcept override {
                for (auto edge_it = this->configuration_shape.edges_begin(); edge_it != this->configuration_shape.edges_end(); edge_it++) {
                    // TODO: For now, the first edge containing the point is returned
                    // This doesn't handle corners
                    // The paper doesn't handle corners, either, so let's deflect blame and procrastinate
                    if (edge_it->has_on(intersection_point)) {
                        return std::optional<Segment_2>{*edge_it};
                    }
                }
                return std::nullopt;
            }
            std::optional<Segment_2> getEdge(Segment_2 intersection_segment) const noexcept override {
                for (auto edge_it = this->configuration_shape.edges_begin(); edge_it != this->configuration_shape.edges_end(); edge_it++) {
                    // Check if both endpoints of the intersection segment are on the edge
                    // This means that the intersection is a subsegment of the edge
                    if (edge_it->has_on(intersection_segment.source()) && edge_it->has_on(intersection_segment.target())) {
                        return std::optional<Segment_2>{*edge_it};
                    }
                }
                // No edge :(
                return std::nullopt;
            }

            void render(scene& scene) const noexcept override {
                CGAL::add_to_graphics_scene(this->configuration_shape, scene); 
            }

            friend class WallGeometry; // For access to private constructor
        };

        Polygon_2 wall_shape;
        
        // Helper method to compute the intersection between two lines or rays or segments
        template <typename T1, typename T2>
        typename std::enable_if_t<(std::is_same_v<T1, Line_2> || std::is_same_v<T1, Segment_2>) && (std::is_same_v<T2, Line_2> || std::is_same_v<T2, Segment_2>), std::optional<Point_2>>
        computeIntersection(T1 linear1, T2 linear2) const noexcept {
            auto maybe_intersection = CGAL::intersection(linear1, linear2);
            
            // Only enable the below cases if both T1 and T2 are segments, since it's only possible to have a disconnect in the intersection in that case
            if constexpr (std::is_same_v<T1, Segment_2> && std::is_same_v<T2, Segment_2>) {
                if (!maybe_intersection) {
                    // No intersection, which happens when the segments are disconnected
                    // To remedy this, extend the segments in both directions and check for a new intersection
                    // The quick and dirty way to extend a segment is to just make a line from it and then check for intersections
                    Line_2 line1(linear1);
                    Line_2 line2(linear2);
                    return this->computeIntersection(line1, line2);
                }
            }
            if (const Point_2* intersection = std::get_if<Point_2>(&*maybe_intersection)) return std::optional<Point_2>{*intersection};
            else return std::nullopt; // The intersection is not a point, this shouldn't happen otherwise the polygon is degenerate
        }

    protected: 
        // Protected constructors since the public API is through the static create method
        // Abstracting this away to protected constructors allows subclassing WallGeometry in a test environment without depending on the static create method and its constraints
        WallGeometry(const Polygon_2& shape) noexcept : wall_shape{shape} {}
        WallGeometry(Polygon_2&& shape) noexcept : wall_shape{std::move(shape)} {}

        // Protected method since the public API depends on the robot
        // Abstracting this away to a protected method allows subclassing WallGeometry in a test environment without depending on the Robot class
        std::unique_ptr<ConfigurationGeometry> constructConfigurationGeometry(const fscalar& robot_radius) const noexcept {
            /*
             * Algorithm -- this may not work with holed polygons (figure this out later)
             * 1. For each edge of the wall_shape polygon, identify the orthogonal vector pointing towards the interior of the polygon
             *   a. The interior of the polygon can be determined by its winding order.
             *   b. A counterclockwise winding order means a relative counterclockwise rotation of the edge vector
             *   c. A clockwise winding order means a relative clockwise rotation of the edge vector
             *   d. Making these rotations 90 degrees will give the orthogonal vector
             * 2. Translate the edge by the robot's radius in the direction of the normalized orthogonal vector
             *   a. This can be achieved by multiplying the orthogonal vector by the robot's radius
             * 3. Identify points of intersection and construct a new polygon using these points
             */
            std::vector<Segment_2> translated_edges; // Hold the translated edges of the configuration geometry
            // Find the transformed segments for each edge of the wall polygon
            for (auto edge_it = this->wall_shape.edges_begin(); edge_it != this->wall_shape.edges_end(); edge_it++) {
                // Construct a perpendicular vector based on the shape's orientation (winding order)
                Vector_2 orthogonal = edge_it->to_vector().perpendicular(this->wall_shape.orientation()); 
                // Normalize the orthogonal vector
                orthogonal /= CGAL::sqrt(orthogonal.squared_length()); 
                // Scale the orthogonal vector by the robot's radius
                orthogonal = orthogonal * robot_radius;
                // Build a transformation from the orthogonal vector
                Transformation translate(CGAL::TRANSLATION, orthogonal); 
                // Shift the edge by the transformation
                translated_edges.push_back(edge_it->transform(translate));
            }

            // For each pair of edges, identify the intersection points
            // For this implementation, use a pseudo-circular queue-style implementation to wrap around the edges
            std::vector<Point_2> configuration_vertices; // Hold the vertices of the configuration geometry
            for (size_t i = 0; i < translated_edges.size(); i++) {
                Segment_2& current_edge = translated_edges.at(i);
                Segment_2& next_edge = translated_edges.at((i + 1) % translated_edges.size());
                // Attempt to compute the intersection between the current edge and the next edge
                auto maybe_intersection = this->computeIntersection(current_edge, next_edge);
                // No intersection, which occurs when the configuration geometry is degenerate but not necessarily disconnected (i.e. collinear edges)
                if (!maybe_intersection) return nullptr;
                // If the intersection point exists, check if it's outside the bounds of the original wall polygon
                // If this occurs, then the original wall geometry is too small for the robot
                if (this->wall_shape.bounded_side(*maybe_intersection) == CGAL::ON_UNBOUNDED_SIDE) return nullptr;
                // If the intersection point is valid, add it to the configuration vertices
                configuration_vertices.push_back(*maybe_intersection);
            }
            // Check if the vertices are more than 2, simple, and not collinear
            // Return nullopt if these conditions fail
            if (configuration_vertices.size() <= 2) return nullptr;
            if (!CGAL::is_simple_2(configuration_vertices.begin(), configuration_vertices.end())) return nullptr;
            if (CGAL::orientation_2(configuration_vertices.begin(), configuration_vertices.end()) == CGAL::COLLINEAR) return nullptr;
            // Generate a configuration geometry from the vertices and set it as the robot's configuration environment
            Polygon_2 config_polygon{configuration_vertices.begin(), configuration_vertices.end()};
            return WallGeometry::ConfigurationGeometryImpl::create(configuration_vertices.begin(), configuration_vertices.end());
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

            Polygon_2 wall_polygon{begin, end};
            return std::optional<WallGeometry>{WallGeometry{std::move(wall_polygon)}};
        }
        static std::optional<WallGeometry> create(std::initializer_list<Point_2> points) noexcept {
            return WallGeometry::create(points.begin(), points.end());
        }

        std::optional<std::monostate> generateConfigurationGeometry(Robot& robot) const noexcept {
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
