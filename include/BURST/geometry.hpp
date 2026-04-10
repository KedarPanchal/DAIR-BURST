#ifndef BURST_GEOMETRIC_TYPES_HPP
#define BURST_GEOMETRIC_TYPES_HPP

#include <concepts>
#include <initializer_list>
#include <numeric>
#include <optional>
#include <ranges>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/enum.h>
#include <CGAL/Aff_transformation_2.h>
#include <CGAL/draw_arrangement_2.h>

#include <boost/container/small_vector.hpp>

#include "kernel.hpp"
#include "numeric.hpp"
#include "logging.hpp"
#include "renderable.hpp"

namespace BURST::geometry {

    // -- GEOMETRIC TYPES ------------------------------------------------------
    
    // Primitive types
    using Point2D = Kernel::Point_2;
    using Segment2D = Kernel::Segment_2;
    using Line2D = Kernel::Line_2;
    using Ray2D = Kernel::Ray_2;
    using MonotoneCurve2D = CurvedTraits::X_monotone_curve_2;
    using Polygon2D = CGAL::Polygon_2<Kernel>;
    using HoledPolygon2D = CGAL::Polygon_with_holes_2<Kernel>;
    using CurvilinearPolygon2D = CurvedTraits::General_polygon_2;
    using HoledCurvilinearPolygon2D = CGAL::General_polygon_with_holes_2<CurvilinearPolygon2D>;
    using Vector2D = CGAL::Vector_2<Kernel>;
    
    // Composite/complex types
    using CurvilinearPolygonSet2D = CGAL::General_polygon_set_2<CurvedTraits>;
    using LinearPolygonSet2D = CGAL::General_polygon_set_2<LinearTraits>;
    using BoundingBox2D = CGAL::Bbox_2;
    using WindingOrder = CGAL::Orientation;

    // Iterator types
    using vertex_iterator = Polygon2D::Vertex_const_iterator;
    using edge_iterator = Polygon2D::Edge_const_iterator;

    // Transformation types
    using Transformation = CGAL::Aff_transformation_2<Kernel>;


    // -- GEOMETRIC CONCEPTS ---------------------------------------------------

    // Checks if a type is a valid path type, which can be constructed from a start and end point
    // It can either be a Segment2D or a curve type that's convertible to an X_monotone_curve_2
    template <typename T>
    concept valid_path_type = requires (geometry::Point2D start, geometry::Point2D end) {
        T{start, end};
    } && (std::same_as<Segment2D, T> || std::convertible_to<T, CurvedTraits::X_monotone_curve_2>);
    
    // Checks if a type is a valid trajectory type, which can be constructed from an origin point and a direction vector
    template <typename T>
    concept valid_trajectory_type = requires (geometry::Point2D origin, geometry::Vector2D direction) {
        T{origin, direction};
    };

    // Checks if a type is a valid geometric collection, which is a sized range of a specific geometric type or an initializer list of that geometric type
    template <typename C, typename G>
    concept valid_geometric_collection = std::ranges::sized_range<C> &&
        std::same_as<std::remove_cv_t<std::ranges::range_value_t<C>>, G> ||
        std::same_as<C, std::initializer_list<G>>;

    // Define mappings for types to their corresponding polygon set types for rendering
    template <typename P>
    struct set_type_v;
    template <>
    struct set_type_v<Polygon2D> {
        using type = LinearPolygonSet2D;
    };
    template<>
    struct set_type_v<LinearPolygonSet2D> {
        using type = LinearPolygonSet2D;
    };
    template <>
    struct set_type_v<CurvilinearPolygon2D> {
        using type = CurvilinearPolygonSet2D;
    };
    template<>
    struct set_type_v<CurvilinearPolygonSet2D> {
        using type = CurvilinearPolygonSet2D;
    };
    template <typename P>
    concept has_set_type = requires {
        typename set_type_v<P>::type;
    };

    // -- HELPER FUNCTIONS -----------------------------------------------------
    
    // Utility function to construct a polygon off of any collection of points
    template <typename C> requires valid_geometric_collection<C, Point2D>
    std::optional<Polygon2D> construct_polygon(const C& points, CGAL::Orientation expected_orientation = CGAL::COUNTERCLOCKWISE) {
        // Can't make a polygon with 2 or fewer points
        if (std::ranges::size(points) <= 2) {
            BURST_ERROR("Cannot construct a polygon with 2 or fewer points, collection is degenerate");
            return std::nullopt;
        }

        // Check for self-intersection, overall simplicity, and non-degeneracy of the polygon and return nullopt if any of these conditions are violated
        if (!CGAL::is_simple_2(std::ranges::begin(points), std::ranges::end(points), LinearTraits{})) {
            BURST_ERROR("Cannot construct a polygon from the given collection of points for one of the following reasons: the polygon is self-intersecting, not simple, or degenerate");
            return std::nullopt;
        }

        // Create the polygon from the input points and return it
        Polygon2D polygon{std::ranges::begin(points), std::ranges::end(points)};
        // If the polygon is not oriented as expected, reverse the orientation to ensure it's a valid polygon for CGAL
        if (polygon.orientation() != expected_orientation) polygon.reverse_orientation();

        return polygon;
    }
    inline std::optional<Polygon2D> construct_polygon(const std::initializer_list<Point2D>& points, CGAL::Orientation expected_orientation = CGAL::COUNTERCLOCKWISE) {
        return construct_polygon<std::initializer_list<Point2D>>(points, expected_orientation);
    }

    template <typename C> requires valid_geometric_collection<C, Point2D>
    std::optional<Point2D> average(const C& points) {
        // Can't compute the average of an empty collection
        if (std::ranges::size(points) == 0) {
            BURST_ERROR("Cannot compute the average of an empty collection of points");
            return std::nullopt;
        }

        Point2D sum = std::accumulate(points.begin(), points.end(), Point2D{0, 0}, [](const Point2D& acc, const Point2D& point) {
            return Point2D{acc.x() + point.x(), acc.y() + point.y()};
        });
        return Point2D{sum.x() / std::ranges::size(points), sum.y() / std::ranges::size(points)};
    }
    inline std::optional<Point2D> average(const std::initializer_list<Point2D>& points) {
        return average<std::initializer_list<Point2D>>(points);
    }

    template <typename T, typename F>
        requires requires(const F& from_point) {
            T{from_point.x(), from_point.y()};
        } 
    T convert_point(const F& from_point) {
        return T{from_point.x(), from_point.y()};
    }
    template <typename T, typename F, typename ConvertFn>
        requires requires(const F& from_point, const ConvertFn& convert_fn) {
            T{convert_fn(from_point.x()), convert_fn(from_point.y())};
        } 
    T convert_point(const F& from_point, const ConvertFn& convert_fn) {
        return T{convert_fn(from_point.x()), convert_fn(from_point.y())};
    }
 
    template <valid_path_type P>
    MonotoneCurve2D construct_curve(const P& path) {
        if constexpr (std::same_as<P, Segment2D>) {
            return CurvedTraits::X_monotone_curve_2{path.source(), path.target()};
        } else {
            return static_cast<CurvedTraits::X_monotone_curve_2>(path); 
        }
    }
       
    inline std::optional<CurvilinearPolygon2D> construct_circle(const numeric::fscalar& radius, const Point2D& center) {
        // Only construct circles with positive radius
        if (radius <= 0) {
            BURST_ERROR("Cannot construct a circle with non-positive radius");
            return std::nullopt;
        }
        CGAL::Circle_2<Kernel> circle = CGAL::Circle_2<Kernel>{center, radius * radius};

        boost::container::small_vector<CurvedTraits::X_monotone_curve_2, 2> semicircles{
            MonotoneCurve2D{circle, CurvedTraits::Point_2{center.x() - radius, center.y()}, CurvedTraits::Point_2{center.x() + radius, center.y()}, CGAL::COUNTERCLOCKWISE},
            MonotoneCurve2D{circle, CurvedTraits::Point_2{center.x() + radius, center.y()}, CurvedTraits::Point_2{center.x() - radius, center.y()}, CGAL::COUNTERCLOCKWISE}
        };
        
        return CurvilinearPolygon2D{semicircles.begin(), semicircles.end()};
    }

    inline Point2D midpoint(const Point2D& a, const Point2D& b) {
        return Point2D{(a.x() + b.x())/2, (a.y() + b.y())/2};
    }
    
    
    namespace detail {
        
    }
    template <typename V> requires has_set_type<V>
    inline std::unique_ptr<renderable::Renderable> renderable(const V& renderable, renderable::Scene& scene, const renderable::Color& color) {
        using set_t = typename set_type_v<V>::type;

        // Convert the renderable to its corresponding polygon set type if it isn't a set type already
        set_t renderable_set = [&renderable]() {
            if constexpr (std::same_as<V, set_t>) {
                return renderable;
            } else {
                return set_t{renderable};
            }
        }();

        // Create an anonymous renderable instance to render the polygon
        class RenderablePolygon : public renderable::Renderable {
        private:
            set_t polygon_set;
            renderable::Color color;
        public:
            RenderablePolygon(const set_t& set, const renderable::Color& color) : polygon_set{set}, color{color} {}
            renderable::Color defaultColor() const override {
                return this->color;
            }

            void render(renderable::Scene& scene, const renderable::Color& color) const override {
                using arrangement_t = typename set_t::Arrangement_2;
                using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, typename arrangement_t::Vertex_const_handle, typename arrangement_t::Halfedge_const_handle, typename arrangement_t::Face_const_handle>;

                graphics_options_t polygon_options;
                polygon_options.colored_face = [](const arrangement_t&, const typename arrangement_t::Face_const_handle&) -> bool {
                    return true; 
                };
                polygon_options.face_color = [color](const arrangement_t&, typename arrangement_t::Face_const_handle) -> renderable::Color {
                    return color;
                };

                CGAL::add_to_graphics_scene(this->polygon_set.arrangement(), scene, polygon_options);
            }
        };
        
        return std::make_unique<RenderablePolygon>(renderable_set, color);
    }
    
    // Allows CGAL geometric types to be wrapped in a renderable for rendering in the scene
    // The return value of this can be passed in render_all to render the geometric type in the scene with the specified color
    template <typename HP> 
        requires std::same_as<HP, HoledPolygon2D> || 
        std::same_as<HP, HoledCurvilinearPolygon2D>
    inline std::unique_ptr<renderable::Renderable> renderable(const HP& renderable, renderable::Scene& scene, const renderable::Color& color) {
        using set_t = typename set_type_v<typename HP::Polygon_2>::type;

        // Create an anonymous renderable instance to render the holed polygon
        class RenderableHoledPolygon : public renderable::Renderable {
        private:
            HP holed_polygon;
            renderable::Color color;
        public:
            RenderableHoledPolygon(const HP& polygon, const renderable::Color& color) : holed_polygon{polygon}, color{color} {}
            renderable::Color defaultColor() const override {
                return this->color;
            }
            
            void render(renderable::Scene& scene, const renderable::Color& color) const override {
                using arrangement_t = typename set_t::Arrangement_2;
                using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, typename arrangement_t::Vertex_const_handle, typename arrangement_t::Halfedge_const_handle, typename arrangement_t::Face_const_handle>;

                // Render the holes to be black
                graphics_options_t hole_options;
                hole_options.colored_face = [](const arrangement_t&, const typename arrangement_t::Face_const_handle&) -> bool {
                    return true; 
                };
                hole_options.face_color = [](const arrangement_t&, typename arrangement_t::Face_const_handle) -> renderable::Color {
                    return renderable::Color{0, 0, 0};
                };
                // Copy the holes since we need to reverse their orientation to render them correctly
                for (auto hole : this->holed_polygon.holes()) {
                    // Reverse since holes are stored clockwise
                    auto reversed_hole = hole;
                    reversed_hole.reverse_orientation(); 
                    set_t hole_set{reversed_hole};
                    CGAL::add_to_graphics_scene(hole_set.arrangement(), scene, hole_options);
                }

                // Render the outer boundary as a colored face
                graphics_options_t boundary_options;
                boundary_options.colored_face = [](const arrangement_t&, const typename arrangement_t::Face_const_handle&) -> bool {
                    return true; 
                };
                boundary_options.face_color = [color](const arrangement_t&, typename arrangement_t::Face_const_handle) -> renderable::Color {
                    return color;
                };
                
                set_t outer_boundary_set{this->holed_polygon.outer_boundary()};
                CGAL::add_to_graphics_scene(outer_boundary_set.arrangement(), scene, boundary_options);
            }
        };

        return std::make_unique<RenderableHoledPolygon>(renderable, color);
    }
}

#endif
