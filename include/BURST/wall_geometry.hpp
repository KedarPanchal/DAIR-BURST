#ifndef WALL_GEOMETRY_HPP
#define WALL_GEOMETRY_HPP

#include <initializer_list>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include "renderable.hpp"
#include "configuration_geometry.hpp"
#include "robot.hpp"

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;

namespace BURST::geometry {
    
    // This class is intended to be used internally and not as an API
    // This is because ConfigurationGeometry should never be instantiated directly
    namespace {
        class ConfigurationGeometryImpl : public ConfigurationGeometry {
        public:
            ConfigurationGeometryImpl(std::initializer_list<Point_2> edge_endpoints);

            void render() const override;
        };
    }

    /*
     * WallGeometry represents the geometry of the walls in the environment. It is defined by a polygon.
     */
    class WallGeometry : public Renderable {
    private:
        const Polygon_2 wall_shape;

    public:
        WallGeometry(std::initializer_list<Point_2> edge_endpoints);
        void generateConfigurationGeometry(Robot& robot) const;

        void render() const override;
    };

}

#endif 
