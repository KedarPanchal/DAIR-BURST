#ifndef BURST_CONFIGURATION_SPACE_HPP
#define BURST_CONFIGURATION_SPACE_HPP

#include <optional>
#include <variant>
#include <memory>
#include <ranges>
#include <iterator>
#include <functional>
#include <algorithm>

#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/draw_arrangement_2.h>

#include <boost/container/small_vector.hpp>

#include "numeric.hpp"
#include "geometry.hpp"
#include "renderable.hpp"

namespace BURST::geometry {
    
    // Forward declare WallSpace for ConfigurationSpace
    class WallSpace;

    // ConfigurationSpace should never be instantiated directly
    // This is why its constructors are private and only accessible by WallSpace
    class ConfigurationSpace : public renderable::Renderable<CurvedTraits, CurvilinearPolygonSet2D::Dcel> {
    private:
        std::unique_ptr<CurvilinearPolygonSet2D> configuration_shape;
        mutable std::optional<BoundingBox2D> bounding_box;

        ConfigurationSpace(std::unique_ptr<CurvilinearPolygonSet2D>&& shape) noexcept : Renderable{}, configuration_shape{std::move(shape)}, bounding_box{} {}

        static std::unique_ptr<ConfigurationSpace> create(std::unique_ptr<CurvilinearPolygonSet2D>&& shape) noexcept {
            return std::unique_ptr<ConfigurationSpace>{new ConfigurationSpace{std::move(shape)}};
        }
        
        /*
         * Functions used to lazily compute bounding box and polygon set since these can be expensive to compute
         * These return by reference to modify the mutable optionals in place
         * They're distinguished from the const reference public API functions by taking a dummy std::monostate argument
         * This is only used for preventing overload conflicts and has no semantic meaning
         */
        BoundingBox2D& bbox(const std::monostate& flag) const noexcept {
            if (!this->bounding_box.has_value()) {
                // Create small vector with buffer of 1, since we expect most configuration spaces to be a single holed polygon
                boost::container::small_vector<HoledCurvilinearPolygon2D, 1> polygons;
                this->configuration_shape->polygons_with_holes(std::back_inserter(polygons));
                this->bounding_box = BoundingBox2D{};
                for (const HoledCurvilinearPolygon2D& polygon : polygons) *this->bounding_box += polygon.outer_boundary().bbox();
            }
            return this->bounding_box.value();
        }

    public:
        const BoundingBox2D& bbox() const noexcept {
            return this->bbox(std::monostate{});
        }
        auto& arrangement() const noexcept {
            return this->configuration_shape->arrangement();
        }
        
        bool onEdge(const Point2D& point) const noexcept {
            // Convert the point to the traits required for the intersection check
            auto converted_point = CurvedTraits::Point_2(point.x(), point.y());
            return this->configuration_shape->oriented_side(converted_point) == CGAL::ON_ORIENTED_BOUNDARY;
        }

        bool contains(const Point2D& point) const noexcept {
            // Convert the point to the traits required for the containment check
            auto converted_point = CurvedTraits::Point_2(point.x(), point.y());
            auto orientation = this->configuration_shape->oriented_side(converted_point);
            return orientation == CGAL::ON_ORIENTED_BOUNDARY || orientation == CGAL::ON_POSITIVE_SIDE;
        }

        std::optional<std::variant<MonotoneCurve2D, Point2D>> intersection(const Point2D& point) const noexcept {
            using arrangement_t = CurvilinearPolygonSet2D::Arrangement_2;
            using converted_point_t = arrangement_t::Point_2;
            using converted_ft = decltype(std::declval<converted_point_t>().x());

            // Attempt to find the point in the arrangement of the configuration space using a landmarks point location
            auto converted_point = convert_point<converted_point_t>(point);
            auto result = CGAL::Arr_naive_point_location<arrangement_t>{this->configuration_shape->arrangement()}.locate(converted_point);

            // Handle according to the type of the result
            // If the point is located on a face, then it's not an intersection since the point is not on the boundary of the configuration space
            if (std::holds_alternative<arrangement_t::Face_const_handle>(result)) return std::nullopt;
            // If the point is located on an edge, then it's an intersection with the configuration space boundary
            if (std::holds_alternative<arrangement_t::Halfedge_const_handle>(result)) {
                return std::get<arrangement_t::Halfedge_const_handle>(result)->curve();
            }
            // Otherwise, the point is located on a vertex, and it's an intersection with the configuration space boundary
            // In practice, this should be a very rare case since the robot would have to traverse exactly to a vertex, but handle it anyway
            return convert_point<Point2D, converted_point_t>(std::get<arrangement_t::Vertex_const_handle>(result)->point(), numeric::sqrt_to_fscalar<converted_ft>);
        }
        
        template <valid_trajectory_type Trajectory, valid_path_type Path, std::ranges::output_range<Point2D> OutputIteratorCollection, typename SourceFunc = const Point2D&(Trajectory::*)() const, typename VectorizeFunc = Vector2D(Trajectory::*)() const>
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
            CurvilinearPolygonSet2D::Arrangement_2 arrangement = this->configuration_shape->arrangement();
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
                Point2D to_add = convert_point<Point2D, decltype(intersection_point)>(intersection_point, numeric::sqrt_to_fscalar<decltype(intersection_point.x())>);
                // Add the point to the output collection using the provided output iterator, given it is not the source of the ray and it lies on the ray path
                if (to_add != ray_source && long_path.has_on(to_add)) {
                    intersection_points = to_add;
                    intersection_count++;
                }
            }
            return intersection_count; // Return the number of intersections found
        }
        
        void render(renderable::Scene& scene, const renderable::Color& color = renderable::Color{0, 0, 255}) override {
            if (this->configuration_shape == nullptr) return; // If the configuration shape is null, then there's nothing to render
                            
            // Just have CGAL render the edges of the configuration space with transparent faces
            graphics_options_t config_options;
            config_options.colored_edge = [](const arrangement_t&, const arrangement_t::Halfedge_const_handle&) -> bool {
                return true;
            };
            config_options.edge_color = [color](const arrangement_t&, arrangement_t::Halfedge_const_handle) -> renderable::Color {
                return color;
            };
            config_options.colored_face = [](const arrangement_t&, const arrangement_t::Face_const_handle& face) -> bool {
                return true;
            };
            config_options.face_color = [](const arrangement_t&, arrangement_t::Face_const_handle) -> renderable::Color {
                return renderable::Color{0, 0, 0, 0}; // Transparent faces
            };

            CGAL::add_to_graphics_scene(this->configuration_shape->arrangement(), scene, config_options);
        }

        friend class BURST::geometry::WallSpace; // For access to private constructor
    };
 }
#endif
