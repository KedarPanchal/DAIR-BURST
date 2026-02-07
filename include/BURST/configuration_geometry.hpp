#ifndef CONFIGURATION_GEOMETRY_HPP
#define CONFIGURATION_GEOMETRY_HPP

#include <initializer_list>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include "renderable.hpp"

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;

namespace BURST::geometry {

    /*
     * ConfigurationGeometry represents the geometry of the robot's configuration space.
     */
    class ConfigurationGeometry : public Renderable {
    protected:
        ConfigurationGeometry(std::initializer_list<Point_2> edge_endpoints);
        const Polygon_2 configuration_shape;
    
    public:
        Segment_2 getEdge(Point_2 intersection_point) const;
        Segment_2 getEdge(Segment_2 intersection_segment) const;

        virtual void render() const override = 0;

        friend class WallGeometry;
    };
    
}
#endif
