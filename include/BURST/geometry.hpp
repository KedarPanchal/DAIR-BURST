#ifndef BURST_GEOMETRIC_TYPES_HPP
#define BURST_GEOMETRIC_TYPES_HPP

#include <concepts>
#include <initializer_list>
#include <numeric>
#include <optional>
#include <ranges>
#include <type_traits>
#include <memory>

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

/**
 * @file geometry.hpp
 * @brief Core 2D geometric aliases, concepts, and helpers built on CGAL types.
 *
 * This header centralizes point/segment/polygon aliases for both linear and curvilinear geometry,
 * validates collections used to build polygons, and provides small utilities for construction,
 * conversion, and optional CGAL graphics integration.
 */

namespace BURST::geometry {

    /** @brief Point in a plane (`Kernel::Point_2`). */
    using Point2D = Kernel::Point_2;
    /** @brief Closed segment between two points. */
    using Segment2D = Kernel::Segment_2;
    /** @brief Infinite line. */
    using Line2D = Kernel::Line_2;
    /** @brief Ray: origin and direction to infinity. */
    using Ray2D = Kernel::Ray_2;
    /** @brief X-monotone curve in @ref CurvedTraits (segments and circular arcs). */
    using MonotoneCurve2D = CurvedTraits::X_monotone_curve_2;
    /** @brief Simple linear polygon with straight edges. */
    using Polygon2D = CGAL::Polygon_2<Kernel>;
    /** @brief Linear polygon with zero or more interior holes. */
    using HoledPolygon2D = CGAL::Polygon_with_holes_2<Kernel>;
    /** @brief Curvilinear simple polygon (arcs and segments). */
    using CurvilinearPolygon2D = CurvedTraits::General_polygon_2;
    /** @brief Curvilinear polygon with holes. */
    using HoledCurvilinearPolygon2D = CGAL::General_polygon_with_holes_2<CurvilinearPolygon2D>;
    /** @brief 2D displacement vector. */
    using Vector2D = CGAL::Vector_2<Kernel>;
    
    /** @brief Boolean combination of curvilinear polygons. */
    using CurvilinearPolygonSet2D = CGAL::General_polygon_set_2<CurvedTraits>;
    /** @brief Boolean combination of linear (segment) polygons. */
    using LinearPolygonSet2D = CGAL::General_polygon_set_2<LinearTraits>;
    /** @brief Axis-aligned bounding box (`CGAL::Bbox_2`). */
    using BoundingBox2D = CGAL::Bbox_2;
    /** @brief Polygon winding/orientation enum (`CGAL::Orientation`). */
    using WindingOrder = CGAL::Orientation;

    /** @brief Const iterator over vertices of a @ref Polygon2D. */
    using vertex_iterator = Polygon2D::Vertex_const_iterator;
    /** @brief Const iterator over edges of a @ref Polygon2D. */
    using edge_iterator = Polygon2D::Edge_const_iterator;

    /** @brief Affine transform in the plane (rotation, translation, scaling). */
    using Transformation = CGAL::Aff_transformation_2<Kernel>;


    /**
     * @brief Path type connectable by two endpoints and usable as an X-monotone curve when curved.
     *
     * Satisfied by @ref Segment2D or any type constructible from two @ref Point2D values and
     * convertible to `CurvedTraits::X_monotone_curve_2` for curvilinear use.
     */
    template <typename T>
    concept valid_path_type = requires (geometry::Point2D start, geometry::Point2D end) {
        T{start, end};
    } && (std::same_as<Segment2D, T> || std::convertible_to<T, CurvedTraits::X_monotone_curve_2>);
    
    /**
     * @brief Trajectory type defined by an origin and a direction vector (e.g. @ref Ray2D).
     *
     * Used by movement models and configuration-space ray intersection.
     */
    template <typename T>
    concept valid_trajectory_type = requires (geometry::Point2D origin, geometry::Vector2D direction) {
        T{origin, direction};
    };

    /**
     * @brief Sized range of `G`, or an `std::initializer_list<G>`, for polygon construction APIs.
     *
     * @tparam C Collection type.
     * @tparam G Element type (e.g. @ref Point2D or @ref Polygon2D).
     */
    template <typename C, typename G>
    concept valid_geometric_collection = std::ranges::sized_range<C> &&
        std::same_as<std::remove_cv_t<std::ranges::range_value_t<C>>, G> ||
        std::same_as<C, std::initializer_list<G>>;

    /**
     * @brief Maps a polygon or polygon-set type to the @ref General_polygon_set_2 specialization used for rendering.
     *
     * Specializations are provided for @ref Polygon2D, @ref CurvilinearPolygon2D, and their
     * corresponding set types. Used by @ref renderable overloads.
     *
     * @tparam P Polygon-like type.
     */
    template <typename P>
    struct set_type_v;
    /** @brief Linear polygons map to @ref LinearPolygonSet2D. */
    template <>
    struct set_type_v<Polygon2D> {
        using type = LinearPolygonSet2D;
    };
    /** @brief Identity mapping for @ref LinearPolygonSet2D. */
    template<>
    struct set_type_v<LinearPolygonSet2D> {
        using type = LinearPolygonSet2D;
    };
    /** @brief Curvilinear polygons map to @ref CurvilinearPolygonSet2D. */
    template <>
    struct set_type_v<CurvilinearPolygon2D> {
        using type = CurvilinearPolygonSet2D;
    };
    /** @brief Identity mapping for @ref CurvilinearPolygonSet2D. */
    template<>
    struct set_type_v<CurvilinearPolygonSet2D> {
        using type = CurvilinearPolygonSet2D;
    };
    /** @brief Whether @ref set_type_v is defined for `P`. */
    template <typename P>
    concept has_set_type = requires {
        typename set_type_v<P>::type;
    };

    /**
     * @brief Build a simple linear polygon from a collection of points.
     *
     * Requires at least three points, simplicity (no self-intersection), and reverses winding to match `expected_orientation` when needed.
     * On failure, logs via @ref BURST_ERROR and returns `std::nullopt`.
     *
     * @param points Collection of points to construct the polygon from.
     * @param expected_orientation Desired outer boundary orientation (typically counterclockwise).
     * @return The constructed polygon if successful, `std::nullopt` otherwise.
     */
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
    /** @copydoc construct_polygon */
    inline std::optional<Polygon2D> construct_polygon(const std::initializer_list<Point2D>& points, CGAL::Orientation expected_orientation = CGAL::COUNTERCLOCKWISE) {
        return construct_polygon<std::initializer_list<Point2D>>(points, expected_orientation);
    }

    /**
     * @brief Arithmetic mean of a non-empty point collection.
     * @return `std::nullopt` if the collection is empty.
     */
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
    /** @copydoc average */
    inline std::optional<Point2D> average(const std::initializer_list<Point2D>& points) {
        return average<std::initializer_list<Point2D>>(points);
    }

    /**
     * @brief Convert between point types by copying `x` and `y` coordinates.
     * @tparam T Target point type constructible from two coordinates.
     * @tparam F Source point type exposing `x()` and `y()`.
     * @return Converted point of type `T`.
     */
    template <typename T, typename F>
        requires requires(const F& from_point) {
            T{from_point.x(), from_point.y()};
        } 
    T convert_point(const F& from_point) {
        return T{from_point.x(), from_point.y()};
    }
    /**
     * @brief Convert coordinates with a per-coordinate mapping (e.g. square-root evaluation).
     * @tparam ConvertFn Callable applied to each coordinate.
     * @return Converted point of type `T`.
     */
    template <typename T, typename F, typename ConvertFn>
        requires requires(const F& from_point, const ConvertFn& convert_fn) {
            T{convert_fn(from_point.x()), convert_fn(from_point.y())};
        } 
    T convert_point(const F& from_point, const ConvertFn& convert_fn) {
        return T{convert_fn(from_point.x()), convert_fn(from_point.y())};
    }
 
    /**
     * @brief Turn a @ref valid_path_type into an @ref MonotoneCurve2D for curvilinear algorithms.
     *
     * Segments are promoted to X-monotone curves; other path types convert explicitly.
     *
     * @return X-monotone curve corresponding to `path`.
     */
    template <valid_path_type P>
    MonotoneCurve2D construct_curve(const P& path) {
        if constexpr (std::same_as<P, Segment2D>) {
            return CurvedTraits::X_monotone_curve_2{path.source(), path.target()};
        } else {
            return static_cast<CurvedTraits::X_monotone_curve_2>(path); 
        }
    }
       
    /**
     * @brief Curvilinear polygon approximating a full circle of given `radius` about `center`.
     *
     * @return `std::nullopt` if `radius` is non-positive.
     */
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

    /** @brief Euclidean midpoint of two points. */
    inline Point2D midpoint(const Point2D& a, const Point2D& b) {
        return Point2D{(a.x() + b.x())/2, (a.y() + b.y())/2};
    }
    
    /**
     * @brief Wrap a polygon or polygon set as a @ref renderable::Renderable filled with `color`.
     *
     * `V` may be a simple polygon type or a compatible `CGAL::General_polygon_set_2`; the appropriate set type
     * is selected via @ref set_type_v. The returned object draws filled faces in the CGAL scene.
     *
     * @tparam V Polygon-like type with a corresponding @ref set_type_v specialization.
     * @param renderable Polygon or polygon-set geometry to be wrapped.
     * @param scene Scene passed through for API symmetry; the returned object owns the geometry.
     * @param color Fill color used as the default render color.
     * @return Heap-allocated renderable wrapper for the given geometry.
     */
    template <typename V> requires has_set_type<V>
    std::unique_ptr<renderable::Renderable> renderable(const V& renderable, renderable::Scene& scene, const renderable::Color& color) {
        using set_t = typename set_type_v<V>::type;

        // Convert the renderable to its corresponding polygon set type if it isn't a set type already
        set_t renderable_set = [&renderable]() {
            if constexpr (std::same_as<V, set_t>) return renderable;
            else return set_t{renderable};
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
    
    /**
     * @brief Wrap a holed polygon for visualization: outer boundary in `color`, holes in black.
     *
     * Supports both linear and curvilinear holed polygons; hole orientation is adjusted for CGAL
     * drawing conventions.
     *
     * @tparam HP Either @ref HoledPolygon2D or @ref HoledCurvilinearPolygon2D.
     * @param renderable Holed polygon geometry to be wrapped.
     * @param scene Scene passed through for API symmetry; the returned object owns the geometry.
     * @param color Color used for the outer boundary region.
     * @return Heap-allocated renderable wrapper for the given holed polygon.
     */
    template <typename HP> 
        requires std::same_as<HP, HoledPolygon2D> || 
        std::same_as<HP, HoledCurvilinearPolygon2D>
    std::unique_ptr<renderable::Renderable> renderable(const HP& renderable, renderable::Scene& scene, const renderable::Color& color) {
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

    /**
     * @brief Wrap one or more segments/curves as an arrangement renderable (edges only).
     *
     * This overload accepts a parameter pack of either all @ref Segment2D or all @ref MonotoneCurve2D.
     * The geometry is inserted into an appropriate `CGAL::Arrangement_2` and rendered with colored edges
     * (faces are not colored).
     *
     * @tparam V All types in the pack must be either @ref Segment2D or @ref MonotoneCurve2D.
     * @param renderable One or more segments/curves to insert into the arrangement.
     * @param scene Scene passed through for API symmetry; the returned object owns the geometry.
     * @param color Default edge color used for rendering.
     * @return Heap-allocated renderable wrapper for the arrangement.
     */
    template <typename ... V> requires (std::same_as<V, Segment2D> && ...) || (std::same_as<V, MonotoneCurve2D> && ...)
    std::unique_ptr<renderable::Renderable> renderable(const V& ... renderable, renderable::Scene& scene, const renderable::Color& color) {
        // Convert the segment or curve to its corresponding arrangement type if not already
        auto arrangement = []() {
            if constexpr ((std::same_as<V, Segment2D> && ...)) return CGAL::Arrangement_2<LinearTraits>{};
            else return CGAL::Arrangement_2<CurvedTraits>{};
        }();
        (CGAL::insert(arrangement, renderable), ...);
        using arrangement_t = decltype(arrangement);

        class RenderableArrangement : public renderable::Renderable {
        private:
            arrangement_t arrangement;
            renderable::Color color;
        public:
            RenderableArrangement(const arrangement_t& arrangement, const renderable::Color& color) : arrangement{arrangement}, color{color} {}
            renderable::Color defaultColor() const override {
                return this->color;
            }
            void render(renderable::Scene& scene, const renderable::Color& color) const override {
                using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, typename arrangement_t::Vertex_const_handle, typename arrangement_t::Halfedge_const_handle, typename arrangement_t::Face_const_handle>;

                graphics_options_t options;
                options.colored_face = [](const arrangement_t&, const typename arrangement_t::Face_const_handle&) -> bool {
                    return false; 
                };
                options.face_color = [color](const arrangement_t&, typename arrangement_t::Face_const_handle) -> renderable::Color {
                    return renderable::Color{0, 0, 0, 0};
                };
                options.colored_edge = [](const arrangement_t&, const typename arrangement_t::Halfedge_const_handle&) -> bool {
                    return true; 
                };
                options.edge_color = [color](const arrangement_t&, typename arrangement_t::Halfedge_const_handle) -> renderable::Color {
                    return color;
                };

                CGAL::add_to_graphics_scene(this->arrangement, scene, options);
            }
        };

        return std::make_unique<RenderableArrangement>(arrangement, color);
    }
}

#endif
