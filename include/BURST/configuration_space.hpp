#ifndef BURST_CONFIGURATION_SPACE_HPP
#define BURST_CONFIGURATION_SPACE_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Graphics_scene.h>

#include <optional>
#include <variant>

#include "numeric_types.hpp"
#include "geometric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"

namespace BURST::geometry {
    
    // Internal implementations not intended for public use
    namespace detail {
        // Type traits for validating whether a type can be a Path for the intersection function in ConfigurationSpace
        // This requires a 2-argument constructor that accepts start and end geometry::Point2Ds (like geometry::Segment2D)
        template <typename T, typename = void>
        struct is_valid_path_type : std::false_type {};
        template <typename T>
        struct is_valid_path_type<T, std::void_t<decltype(T{std::declval<geometry::Point2D>(), std::declval<geometry::Point2D>()})>> : std::true_type {};
    }

    // Forward declare WallSpace for ConfigurationSpace
    class WallSpace;

    // ConfigurationSpace should never be instantiated directly
    // This is why its constructors are private and only accessible by WallSpace
    class ConfigurationSpace : public Renderable {
    private:
        CurvilinearPolygon2D configuration_shape;
        mutable std::optional<BoundingBox2D> bounding_box;
        mutable std::optional<CurvilinearPolygonSet2D> polygon_set;

        ConfigurationSpace(const CurvilinearPolygon2D& shape) noexcept : configuration_shape{shape}, bounding_box{} {}
        ConfigurationSpace(CurvilinearPolygon2D&& shape) noexcept : configuration_shape{std::move(shape)}, bounding_box{} {}

        static std::unique_ptr<ConfigurationSpace> create(const CurvilinearPolygon2D& shape) noexcept {
            return std::unique_ptr<ConfigurationSpace>{new ConfigurationSpace{shape}};
        }
        
        /*
         * Functions used to lazily compute bounding box and polygon set since these can be expensive to compute
         * These return by reference to modify the mutable optionals in place
         * They're distinguished from the const reference public API functions by taking a dummy std::monostate argument
         * This is only used for preventing overload conflicts and has no semantic meaning
         */
        BoundingBox2D& bbox(const std::monostate& flag) const noexcept {
            if (!this->bounding_box.has_value()) {
                this->bounding_box = this->configuration_shape.bbox();
            }
            return this->bounding_box.value();
        }
        CurvilinearPolygonSet2D& set(const std::monostate& flag) const noexcept {
            if (!this->polygon_set.has_value()) {
                this->polygon_set = CurvilinearPolygonSet2D{};
                this->polygon_set->insert(this->configuration_shape);
            }
            return this->polygon_set.value();
        }

    public:
        const BoundingBox2D& bbox() const noexcept {
            return this->bbox(std::monostate{});
        }
        const CurvilinearPolygonSet2D& set() const noexcept {
            return this->set(std::monostate{});
        }

        winding_order orientation() const noexcept {
            return this->configuration_shape.orientation();
        }
        
        // TODO: Make this return the edge the point intersects with instead of just the point itself
        std::optional<Point2D> intersection(const Point2D& point) const noexcept {
            // Convert the point to the traits required for the intersection check
            auto converted_point = CurvedTraits::Point_2(point.x(), point.y());
            if (this->set().oriented_side(converted_point) == CGAL::ON_ORIENTED_BOUNDARY) return std::optional<Point2D>{point};
            return std::nullopt;
        }
        
        template <typename Trajectory, typename Path, typename OutputIteratorCollection, typename SourceFunc = const Point2D&(Trajectory::*)() const, typename VectorizeFunc = Vector2D(Trajectory::*)() const>
        size_t intersection(
                const Trajectory& trajectory,
                std::back_insert_iterator<OutputIteratorCollection> intersection_points,
                SourceFunc source = &Trajectory::source,
                VectorizeFunc vectorize = &Trajectory::to_vector
            ) const noexcept {
            static_assert(detail::is_valid_path_type<Path>::value, "The Path type for ConfigurationSpace::intersection must have a 2-argument constructor that accepts start and end geometry::Point2Ds");

            // Identify the margin of the bounding box to determine an extreme magnitude for the ray to be clipped at
            numeric::fscalar margin = this->bbox().xmax() - this->bbox().xmin() + this->bbox().ymax() - this->bbox().ymin();
            // Create a segment from the ray with the identified margin
            Path long_path{std::invoke(source, trajectory), std::invoke(source, trajectory) + std::invoke(vectorize, trajectory) * margin};

            // Get the arrangement of the configuration geometry to insert the segment into for intersection checking
            auto arrangement = this->set().arrangement();
            // Insert the long segment into the arrangement
            CGAL::insert(arrangement, long_path);
            // Convert the ray source to the traits required for the source containment check
            auto converted_source = CurvedTraits::Point_2(std::invoke(source, trajectory).x(), std::invoke(source, trajectory).y());
            // Count the number of intersections found
            size_t intersection_count = 0;
            /*
             * Iterate through the arrangement vertices to find the intersection point
             * Existing polygon vertices will have a degree of 2
             * Intersections will have a degree greater than that since the long segment will increase the number of edges incident to the vertex
             */
            for (auto vertex_it = arrangement.vertices_begin(); vertex_it != arrangement.vertices_end(); ++vertex_it) {
                if (vertex_it->point() == converted_source) continue; // Skip the source of the ray since that's not an intersection
                if (vertex_it->degree() > 2) {
                    // Convert the vertex point back to a Point2D and return it as the intersection point
                    auto intersection_point = vertex_it->point();
                    // Add the point to the output collection using the provided output iterator
                    intersection_points = Point2D{intersection_point.x(), intersection_point.y()};
                    // Increment the intersection count
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
