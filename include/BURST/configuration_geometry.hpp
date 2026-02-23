#ifndef BURST_CONFIGURATION_SPACE_HPP
#define BURST_CONFIGURATION_SPACE_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Graphics_scene.h>

#include "numeric_types.hpp"
#include "geometric_types.hpp"
#include "graphics_types.hpp"
#include "renderable.hpp"

namespace BURST::geometry {

    // Forward declare WallSpace for ConfigurationSpace
    class WallSpace;

    // This class is intended to be used internally and not as an API
    // This is because InternalConfigurationSpace should never be instantiated directly
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
        
        // Functions used to lazily compute bounding box and polygon set since these can be expensive to compute
        BoundingBox2D& bbox() const noexcept {
            if (!this->bounding_box.has_value()) {
                this->bounding_box = this->configuration_shape.bbox();
            }
            return this->bounding_box.value();
        }
        CurvilinearPolygonSet2D& set() const noexcept {
            if (!this->polygon_set.has_value()) {
                this->polygon_set = CurvilinearPolygonSet2D{};
                this->polygon_set->insert(this->configuration_shape);
            }
            return this->polygon_set.value();
        }

    public:
        winding_order orientation() const noexcept {
            return this->configuration_shape.orientation();
        }

        std::optional<Point2D> intersection(const Point2D& point) const noexcept {
            // Convert the point to the traits required for the intersection check
            auto converted_point = CurvedTraits::Point_2(point.x(), point.y());
            if (this->set().oriented_side(converted_point) == CGAL::ON_ORIENTED_BOUNDARY) return std::optional<Point2D>{point};
            return std::nullopt;
        }
        
        // TODO: Make this return an array of intersection points instead of just one since there can be multiple intersections with a curvilinear polygon
        template <typename Trajectory, typename Path>
        std::optional<Point2D> intersection(const Trajectory& trajectory) const noexcept {
            // Identify the margin of the bounding box to determine an extreme magnitude for the ray to be clipped at
            numeric::fscalar margin = this->bbox().xmax() - this->bbox().xmin() + this->bbox().ymax() - this->bbox().ymin();
            // Create a segment from the ray with the identified margin
            Segment2D long_segment{trajectory.source(), trajectory.source() + trajectory.to_vector() * margin};

            // Get the arrangement of the configuration geometry to insert the segment into for intersection checking
            auto arrangement = this->set().arrangement();
            // Insert the long segment into the arrangement
            CGAL::insert(arrangement, long_segment);
            // Convert the ray source to the traits required for the source containment check
            auto converted_source = CurvedTraits::Point_2(trajectory.source().x(), trajectory.source().y());
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
                    return std::optional<Point2D>{Point2D(intersection_point.x(), intersection_point.y())};
                }
            }
            return std::nullopt; // No intersection found
        }

        void render(graphics::Scene& scene) const noexcept {
            // TODO: Figure out how to render a curvilinear polygon
        }

        friend class BURST::geometry::WallSpace; // For access to private constructor
    };
 }
#endif
