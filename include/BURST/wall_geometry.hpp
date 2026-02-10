#ifndef WALL_GEOMETRY_HPP
#define WALL_GEOMETRY_HPP

#include <initializer_list>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/centroid.h>

#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "robot.hpp"

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using scene = CGAL::Graphics_scene;

namespace BURST::geometry {
    
    // This class is intended to be used internally and not as an API
    // This is because ConfigurationGeometry should never be instantiated directly
    namespace {
        class ConfigurationGeometryImpl : public ConfigurationGeometry {
        private:
            const Polygon_2 configuration_shape;

        public:
            ConfigurationGeometryImpl(std::initializer_list<Point_2> edge_endpoints) : configuration_shape{Polygon_2(edge_endpoints.begin(), edge_endpoints.end())} {}

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
        WallGeometry(std::initializer_list<Point_2> edge_endpoints): wall_shape{Polygon_2(edge_endpoints.begin(), edge_endpoints.end())} {}
        void generateConfigurationGeometry(Robot& robot) const {
        }

        void render(scene& scene) const override {
            CGAL::add_to_graphics_scene(this->wall_shape, scene);
        }
    };

}

#endif 
