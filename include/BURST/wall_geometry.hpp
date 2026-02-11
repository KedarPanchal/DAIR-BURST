#ifndef WALL_GEOMETRY_HPP
#define WALL_GEOMETRY_HPP

#include <optional>
#include <variant>
#include <vector>

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/draw_polygon_2.h>

#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "robot.hpp"

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Vector_2 = CGAL::Vector_2<Kernel>;
using Transformation = CGAL::Aff_transformation_2<Kernel>;
using scene = CGAL::Graphics_scene;

namespace BURST::geometry {
    
    // This class is intended to be used internally and not as an API
    // This is because ConfigurationGeometry should never be instantiated directly
    namespace {
        class ConfigurationGeometryImpl : public ConfigurationGeometry {
        private:
            const Polygon_2 configuration_shape;

        public:
            template <typename Iter>
            ConfigurationGeometryImpl(Iter begin, Iter end) : configuration_shape{Polygon_2{begin, end}} {}

            std::optional<Segment_2> getEdge(Point_2 intersection_point) const override {
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

            void render(scene& scene) const override {
                CGAL::add_to_graphics_scene(this->configuration_shape, scene); 
            }
        };
    }

    /*
     * WallGeometry represents the geometry of the walls in the environment. It is defined by a polygon.
     */
    class WallGeometry : public Renderable {
    private:
        const Polygon_2 wall_shape;

    public:
        // In the future, if logging is added, add a warning if the polygon is not simple or is degenerate
        template <typename Iter>
        WallGeometry(Iter begin, Iter end): wall_shape{Polygon_2{begin, end}} {}

        std::optional<std::monostate> generateConfigurationGeometry(Robot& robot) const {
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
            
            if (!this->wall_shape.is_clockwise_oriented() || !this->wall_shape.is_counterclockwise_oriented()) {
                // The polygon is degenerate, so we can't generate a configuration geometry
                return std::nullopt;
            }
            
            std::vector<Segment_2> translated_edges; // Hold the translated edges of the configuration geometry
            // Find the transformed segments for each edge of the wall polygon
            for (auto edge_it = this->wall_shape.edges_begin(); edge_it != this->wall_shape.edges_end(); edge_it++) {
                // Construct a perpendicular vector based on the shape's orientation (winding order)
                Vector_2 orthogonal = edge_it->to_vector().perpendicular(this->wall_shape.orientation()); 
                // Normalize the orthogonal vector
                orthogonal /= CGAL::sqrt(orthogonal.squared_length()); 
                // Scale the orthogonal vector by the robot's radius
                orthogonal = orthogonal * robot.getRadius(); 
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

                auto maybe_intersection = CGAL::intersection(current_edge, next_edge);
                if (!maybe_intersection) {
                    // No intersection, this shouldn't happen otherwise the polygon is disconnected
                    // Panic and cry and return nullopt
                    return std::nullopt;
                }
                // The intersection can be a point or a segment in CGAL, but it should always be a point in this case
                if (const Point_2* intersection = std::get_if<Point_2>(&*maybe_intersection)) {
                    configuration_vertices.push_back(*intersection);
                } else {
                    // The intersection is not a point, this shouldn't happen otherwise the polygon is degenerate
                    // Panic and cry and return nullopt
                    return std::nullopt;
                }
            }
            
            // Generate a configuration geometry from the vertices and set it as the robot's configuration environment
            std::unique_ptr<ConfigurationGeometry> config_geometry = std::make_unique<ConfigurationGeometryImpl>(configuration_vertices.begin(), configuration_vertices.end());
            robot.setConfigurationEnvironment(std::move(config_geometry));

            // monostate is used as a replacement for void while still keeping the return type as an optional to indicate failure
            return std::optional<std::monostate>{std::monostate{}}; 
        }

        void render(scene& scene) const override {
            CGAL::add_to_graphics_scene(this->wall_shape, scene);
        }
    };

}

#endif 
