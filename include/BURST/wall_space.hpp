#ifndef BURST_WALL_SPACE_HPP
#define BURST_WALL_SPACE_HPP

#include <optional>
#include <memory>
#include <iterator>
#include <source_location>

#include <CGAL/approximated_offset_2.h>
#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/draw_arrangement_2.h>

#include <boost/container/small_vector.hpp>
#include <boost/uuid/uuid.hpp>

#include "numeric.hpp"
#include "geometry.hpp"
#include "renderable.hpp"
#include "configuration_space.hpp"
#include "robot.hpp"
#include "logging.hpp"

/**
 * @file wall_space.hpp
 * @brief Environment boundary as a (possibly holed) polygon and factory for @ref ConfigurationSpace.
 */

namespace BURST::geometry {
    
    /**
     * @brief Static environment geometry: outer walls and optional holes (obstacles).
     *
     * The outer boundary is a simple linear polygon; holes represent excluded regions. From this
     * representation and a robot radius, BURST derives the free configuration region where the
     * robot’s center can move. @ref WallSpace is also @ref renderable::Renderable (white interior,
     * black holes by default).
     */
    class WallSpace : public renderable::Renderable {
    private:
        HoledPolygon2D wall_shape;

    protected: 
        /** @brief Build from an outer polygon without holes. */
        WallSpace(const Polygon2D& shape) noexcept : Renderable{}, wall_shape{shape} {}
        /** @brief Build from a full holed polygon representation. */
        WallSpace(const HoledPolygon2D& shape) noexcept : Renderable{}, wall_shape{shape} {}

        /**
         * @brief Compute the configuration space for a robot of radius `robot_radius`.
         *
         * Performs a Minkowski sum/difference operation on the wall polygon and the robot's radius to create a configuration space.
         * The resulting region is a curvilinear polygon set suitable for @ref ConfigurationSpace. May return null when the
         * wall polygon is too small for the robot to fit in.
         *
         * @return Generated configuration space, or `nullptr` when no free region can be constructed.
         */
        std::shared_ptr<ConfigurationSpace> constructConfigurationSpace(const numeric::fscalar& robot_radius, const std::source_location location = std::source_location::current()) const {
            // TODO: Find an actually good epsilon instead of this approximation
            const double EPSILON = 0.000001;

            // Compute the inset of the outer boundary of the wall polygon
            boost::container::small_vector<CurvilinearPolygon2D, 1> outer_inset_results;
            CGAL::approximated_inset_2(this->wall_shape.outer_boundary(), robot_radius, EPSILON, std::back_inserter(outer_inset_results));

            // If there are no polygons, then the wall is too small for the robot and no configuration space could be made
            // If there are multiple polygons, then there were regions too tight for the robot to fit in, and no configuration space could be made
            // TODO: For the above case, check with Dr. Shell if that's something worth allowing in the final sim
            // In both cases, return nullptr
            if (outer_inset_results.size() != 1) {
                burst_error("Wall polygon is too small for the robot, no configuration space could be generated", location);
                return nullptr;
            }
            // Reverse the resulting polygon's rotation if it's not counterclockwise
            if (outer_inset_results.front().orientation() != CGAL::COUNTERCLOCKWISE) outer_inset_results.front().reverse_orientation();

            // Create a polygon set to store all Minkowski sum/difference results to create the configuration space
            std::unique_ptr<CurvilinearPolygonSet2D> config_polygon_set = std::make_unique<CurvilinearPolygonSet2D>();
            config_polygon_set->insert(outer_inset_results.front());

            // Compute the outset of all of the holes within the wall polygon, since the holes need to be expanded by the robot radius as well
            // No checks needed for the holes touching the wall since that's a realistic case for when an object is close to a wall
            for (const auto& hole : this->wall_shape.holes()) {
                // Holes are CW, so reverse the orientation to ensure it's a valid polygon for CGAL
                Polygon2D oriented_hole = hole;
                if (oriented_hole.orientation() != CGAL::COUNTERCLOCKWISE) oriented_hole.reverse_orientation();
 
                // Compute the difference between the resultant polygon set and outset hole
                // This is insulated against overlapping holes and border touching
                // This is possible if two holes or a hole and the wall are close enough that the space between is too small for the robot
                config_polygon_set->difference(CGAL::approximated_offset_2(oriented_hole, robot_radius, EPSILON));
            }
            
            // Create the configuration space from the resulting polygon set
            return ConfigurationSpace::create(std::move(config_polygon_set));
        }

    public:
        using Polygon = HoledPolygon2D::Polygon_2;                      /**< Linear polygon type for holes and boundaries. */
        using Hole_iterator = HoledPolygon2D::Hole_const_iterator;      /**< Iterator over hole polygons. */
        using Edge = HoledPolygon2D::Polygon_2::Segment_2;            /**< Wall edge segment type. */
        using Edge_iterator = HoledPolygon2D::Polygon_2::Edge_const_iterator; /**< Edge iterator for outer or hole rings. */
        using Vertex = HoledPolygon2D::Polygon_2::Point_2;              /**< Vertex / point type. */
        using Vertex_iterator = HoledPolygon2D::Polygon_2::Vertex_const_iterator; /**< Vertex iterator. */

        /**
         * @brief Create a simply-connected wall from an outer vertex ring.
         *
         * Delegates to @ref construct_polygon; returns `std::nullopt` for degenerate input.
         *
         * @return Wall space if construction succeeds, `std::nullopt` otherwise.
         */
        template <valid_geometric_collection<Point2D> C>
        static std::optional<WallSpace> create(C points) {
            auto wall_polygon_opt = construct_polygon(points);  
            // If nullopt, then the wall polygon was degenerate and we can't create a wall geometry
            return wall_polygon_opt.has_value() ? std::optional<WallSpace>{WallSpace{wall_polygon_opt.value()}} : std::nullopt;
        }
        /**
         * @brief Create walls with holes from an outer ring and a collection of hole polygons.
         *
         * Holes are enforced clockwise; the combined holed polygon must satisfy CGAL’s validity
         * checks (holes inside the outer boundary, pairwise non-intersection, simplicity).
         *
         * @return Wall space if construction succeeds, `std::nullopt` otherwise.
         */
        template <valid_geometric_collection<Point2D> C1, valid_geometric_collection<Polygon2D> C2>
        static std::optional<WallSpace> create(C1 points, C2 holes, const std::source_location location = std::source_location::current()) {
            auto wall_polygon_opt = construct_polygon(points);
            // Degenerate wall polygon, can't create a wall geometry
            if (!wall_polygon_opt) return std::nullopt; 

            // Create a holed polygon to hold the holes and outer boundary
            HoledPolygon2D wall_shape{wall_polygon_opt.value()};
            
            // Check that each hole is clockwise-oriented
            for (const Polygon2D& hole : holes) {
                // Degenerate hole polygon, can't create a wall geometry
                if (!hole.is_simple()) {
                    burst_error("Hole polygon is degenerate, can't create a wall geometry", location);
                    return std::nullopt;
                }
                // Holes must be oriented clockwise, so reverse the orientation if not
                Polygon2D oriented_hole = hole;
                if (hole.orientation() != CGAL::CLOCKWISE) oriented_hole.reverse_orientation();
                // Add the hole
                wall_shape.add_hole(oriented_hole);
            }

            // Check if the resulting holed polygon is valid, which means:
            // The outer boundary is a simple polygon and counterclockwise-oriented
            // Each hole is a simple polygon and clockwise-oriented
            // Holes are inside the outer boundary and do not intersect with each other
            // Return nullopt if any of these conditions are violated
            if (CGAL::is_valid_polygon_with_holes(wall_shape, LinearTraits{})) {
                return WallSpace{wall_shape};
            } else {
                burst_error("Resulting wall polygon with holes is invalid with one of: degenerate outer boundary, degenerate hole, hole not inside outer boundary, or holes intersecting each other. Can't create a wall geometry", location);
                return std::nullopt;
            }
        }
        /** @copydoc create */
        inline static std::optional<WallSpace> create(std::initializer_list<Point2D> points) {
            return create<std::initializer_list<Point2D>>(points);
        }
        /** @copydoc create(C1 points, C2 holes) */
        template <valid_geometric_collection<Point2D> C>
        inline static std::optional<WallSpace> create(C points, std::initializer_list<Polygon2D> holes) {
            return create<C, std::initializer_list<Polygon2D>>(points, holes);
        }
        /** @copydoc create(C1 points, C2 holes) */
        template <valid_geometric_collection<Polygon2D> C>
        inline static std::optional<WallSpace> create(std::initializer_list<Point2D> points, C holes) {
            return create<std::initializer_list<Point2D>, C>(points, holes);
        }
        inline static std::optional<WallSpace> create(std::initializer_list<Point2D> points, std::initializer_list<Polygon2D> holes) {
            return create<std::initializer_list<Point2D>>(points, std::initializer_list<Polygon2D>(holes));
        }

        /**
         * @brief Build and attach the @ref ConfigurationSpace for `robot`’s radius to `robot`.
         *
         * Uses @ref constructConfigurationSpace; returns `false` when the free region cannot be
         * constructed (e.g. environment too narrow). Template parameters are inferred from
         * @ref Robot.
         *
         * @return True if the configuration space was generated and attached, false otherwise.
         */
        template <typename T, typename P, typename R, typename D>
        bool generateConfigurationSpace(Robot<T, P, R, D>& robot) const {
            auto config_geometry = this->constructConfigurationSpace(robot.getRadius());
            if (!config_geometry) return false; // Degenerate configuration geometry, can't set it for the robot
            robot.setConfigurationEnvironment(std::move(config_geometry));

            return true;
        }

        /** 
         * @brief Default wall edge color (black); faces use white/black scheme in @ref render.
         * @return Default wall edge color.
         */
        renderable::Color defaultColor() const override {
            return renderable::Color{0, 0, 0}; 
        }
        
        /**
         * @brief Draw holes as black-filled regions and the outer boundary as a white face with colored edges.
         */
        void render(renderable::Scene& scene, const renderable::Color& color = renderable::Color{0, 0, 0}) const override {
            using arrangement_t = LinearPolygonSet2D::Arrangement_2;
            using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, arrangement_t::Vertex_const_handle, arrangement_t::Halfedge_const_handle, arrangement_t::Face_const_handle>;
            // Render the holes to be black
            graphics_options_t hole_options;
            hole_options.colored_face = [](const arrangement_t&, const arrangement_t::Face_const_handle&) -> bool {
                return true; 
            };
            hole_options.face_color = [](const arrangement_t&, arrangement_t::Face_const_handle) -> renderable::Color {
                return renderable::Color{0, 0, 0};
            };
            // Copy the holes since we need to reverse their orientation to render them correctly
            for (auto hole : this->wall_shape.holes()) {
                // Reverse since holes are stored clockwise
                hole.reverse_orientation(); 
                LinearPolygonSet2D hole_set{hole};
                CGAL::add_to_graphics_scene(hole_set.arrangement(), scene, hole_options);
            }

            // Render the outer boundary as a white face
            graphics_options_t boundary_options;
            boundary_options.colored_edge = [](const arrangement_t&, const arrangement_t::Halfedge_const_handle&) -> bool {
                return true; 
            };
            boundary_options.edge_color = [color](const arrangement_t&, arrangement_t::Halfedge_const_handle) -> renderable::Color {
                return color;
            };
            boundary_options.colored_face = [](const arrangement_t&, const arrangement_t::Face_const_handle&) -> bool {
                return true; 
            };
            boundary_options.face_color = [](const arrangement_t&, arrangement_t::Face_const_handle) -> renderable::Color {
                return renderable::Color{255, 255, 255};
            };
            LinearPolygonSet2D wall_set{this->wall_shape.outer_boundary()};
            CGAL::add_to_graphics_scene(wall_set.arrangement(), scene, boundary_options);
        }

        /** 
         * @brief Iterator to the first hole polygon.
         * @return Iterator to the first hole.
         */
        Hole_iterator holes_begin() const {
            return this->wall_shape.holes_begin();
        }

        /** 
         * @brief Past-the-end iterator for holes.
         * @return Past-the-end iterator for holes.
         */
        Hole_iterator holes_end() const {
            return this->wall_shape.holes_end();
        }

        /** 
         * @brief First edge of the outer boundary.
         * @return Iterator to the first outer-boundary edge.
         */
        Edge_iterator edges_begin() const {
            return this->wall_shape.outer_boundary().edges_begin();
        }
        /** 
         * @brief Past-the-end edge iterator for the outer boundary.
         * @return Past-the-end iterator for outer-boundary edges.
         */
        Edge_iterator edges_end() const {
            return this->wall_shape.outer_boundary().edges_end();
        }
        /** 
         * @brief First edge of a specific `hole` polygon.
         * @return Iterator to the first edge of `hole`.
         */
        Edge_iterator edges_begin(Polygon hole) const {
            return hole.edges_begin();
        }
        /** 
         * @brief Past-the-end edges for `hole`.
         * @return Past-the-end iterator for edges of `hole`.
         */
        Edge_iterator edges_end(Polygon hole) const {
            return hole.edges_end();
        }

        /** 
         * @brief First vertex of the outer boundary.
         * @return Iterator to the first outer-boundary vertex.
         */
        Vertex_iterator vertices_begin() const {
            return this->wall_shape.outer_boundary().vertices_begin();
        }
        /** 
         * @brief Past-the-end vertex iterator for the outer boundary.
         * @return Past-the-end iterator for outer-boundary vertices.
         */
        Vertex_iterator vertices_end() const {
            return this->wall_shape.outer_boundary().vertices_end();
        }
        /** 
         * @brief First vertex of `hole`.
         * @return Iterator to the first vertex of `hole`.
         */
        Vertex_iterator vertices_begin(Polygon hole) const {
            return hole.vertices_begin();
        }
        /** 
         * @brief Past-the-end vertices for `hole`.
         * @return Past-the-end iterator for vertices of `hole`.
         */
        Vertex_iterator vertices_end(Polygon hole) const {
            return hole.vertices_end();
        }

        friend class std::unique_ptr<WallSpace>;
    };

}

#endif 
