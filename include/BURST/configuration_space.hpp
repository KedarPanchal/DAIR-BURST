#ifndef BURST_CONFIGURATION_SPACE_HPP
#define BURST_CONFIGURATION_SPACE_HPP

#include <optional>
#include <variant>
#include <memory>
#include <iterator>
#include <functional>
#include <algorithm>

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Graphics_scene.h>

#include <boost/container/small_vector.hpp>

#include "numeric_types.hpp"
#include "geometric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"

namespace BURST::geometry {
    
    // Forward declare WallSpace for ConfigurationSpace
    class WallSpace;

    // ConfigurationSpace should never be instantiated directly
    // This is why its constructors are private and only accessible by WallSpace
    class ConfigurationSpace : public Renderable {
    private:
        CurvilinearPolygonSet2D configuration_shape;
        mutable std::optional<BoundingBox2D> bounding_box;

        ConfigurationSpace(const CurvilinearPolygonSet2D& shape) noexcept : configuration_shape{shape}, bounding_box{} {}

        static std::unique_ptr<ConfigurationSpace> create(const CurvilinearPolygonSet2D& shape) noexcept {
            return std::unique_ptr<ConfigurationSpace>{new ConfigurationSpace{shape}};
        }
        
        /*
         * Functions used to lazily compute bounding box and polygon set since these can be expensive to compute
         * These return by reference to modify the mutable optionals in place
         * They're distinguished from the const reference public API functions by taking a dummy std::monostate argument
         * This is only used for preventing overload conflicts and has no semantic meaning
         */
        // TODO: Maybe manually compute this
        BoundingBox2D& bbox(const std::monostate& flag) const noexcept {
            if (!this->bounding_box.has_value()) {
                // Create small vector with buffer of 1, since we expect most configuration spaces to be a single holed polygon
                boost::container::small_vector<CurvilinearPolygon2D, 1> polygons;
                this->configuration_shape.polygons_with_holes(std::back_inserter(polygons));
                this->bounding_box = BoundingBox2D{};
                for (const CurvilinearPolygon2D& polygon : polygons) *this->bounding_box += polygon.bbox();
            }
            return this->bounding_box.value();
        }

    public:
        const BoundingBox2D& bbox() const noexcept {
            return this->bbox(std::monostate{});
        }

        // TODO: Make this return the edge the point intersects with instead of just the point itself
        std::optional<Point2D> intersection(const Point2D& point) const noexcept {
            // Convert the point to the traits required for the intersection check
            auto converted_point = CurvedTraits::Point_2(point.x(), point.y());
            if (this->configuration_shape.oriented_side(converted_point) == CGAL::ON_ORIENTED_BOUNDARY) return std::optional<Point2D>{point};
            return std::nullopt;
        }
        
        template <valid_trajectory_type Trajectory, valid_path_type Path, std::output_iterator<Point2D> OutputIteratorCollection, typename SourceFunc = const Point2D&(Trajectory::*)() const, typename VectorizeFunc = Vector2D(Trajectory::*)() const>
        size_t intersection(
                const Trajectory& trajectory,
                std::back_insert_iterator<OutputIteratorCollection> intersection_points,
                SourceFunc source = &Trajectory::source,
                VectorizeFunc vectorize = &Trajectory::to_vector
            ) const noexcept {
            Point2D ray_source = std::invoke(source, trajectory);
            Vector2D ray_vector = std::invoke(vectorize, trajectory);

            // Identify the margin of the bounding box to determine an extreme magnitude for the ray to be clipped at
            numeric::fscalar margin = this->bbox().xmax() - this->bbox().xmin() + this->bbox().ymax() - this->bbox().ymin();
            // Compute the maximum distance between the ray source and an edge of the bounding box to guarantee the ray passes through the bounding box in its entirety
            numeric::fscalar displacement = std::max({
                numeric::abs(ray_source.x() - this->bbox().xmin()), 
                numeric::abs(ray_source.x() - this->bbox().xmax()),
                numeric::abs(ray_source.y() - this->bbox().ymin()),
                numeric::abs(ray_source.y() - this->bbox().ymax())
            });
            // Create a segment from the ray with the identified margin
            Path long_path{ray_source, ray_source + ray_vector * (margin + displacement)};

            // Get the arrangement of the ConfigurationSpace to insert the segment into for intersection checking
            auto arrangement = this->configuration_shape.arrangement();
            // Insert the long segment into the arrangement
            CGAL::insert(arrangement, long_path);
            // Convert the ray source to the traits required for the source containment check
            auto converted_source = CurvedTraits::Point_2(ray_source.x(), ray_source.y());
            // Track the number of intersections found
            size_t intersection_count = 0;

            /*
             * Iterate through the arrangement vertices to find the intersection point
             * Existing polygon vertices will have a degree of 2
             * Intersections will have a degree greater than that since the long segment will increase the number of edges incident to the vertex
             */
            for (auto vertex_it = arrangement.vertices_begin(); vertex_it != arrangement.vertices_end(); ++vertex_it) {
                if (vertex_it->point() == converted_source) continue; // Skip the source of the ray since that's not an intersection
                // Skip if edge or endpoint
                if (vertex_it->degree() <= 2) continue;

                // Convert the vertex point back to a Point2D and return it as the intersection point
                auto intersection_point = vertex_it->point();
                // Add the point to the output collection using the provided output iterator, given it is not the source of the ray
                Point2D to_add = Point2D{numeric::sqrt_to_fscalar(intersection_point.x()), numeric::sqrt_to_fscalar(intersection_point.y())};
                if (to_add != ray_source) {
                    intersection_points = to_add;
                    intersection_count++;
                }
            }
            return intersection_count; // Return the number of intersections found
        }

        void render(graphics::Scene& scene) const noexcept {
            // TODO: Figure out how to render a curvilinear polygon
        }

        friend class BURST::geometry::WallSpace; // For access to private constructor
    };
 }
#endif
