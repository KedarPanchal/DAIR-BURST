#ifndef CONFIGURATION_GEOMETRY_HPP
#define CONFIGURATION_GEOMETRY_HPP

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Graphics_scene.h>

#include "types.hpp"
#include "renderable.hpp"

namespace BURST::geometry {

    /*
     * ConfigurationGeometry represents the geometry of the robot's configuration space.
     */
    class ConfigurationGeometry : public Renderable {
    public:
        // Initialize default destructor for use in unique_ptr
        ConfigurationGeometry() = default;
        virtual ~ConfigurationGeometry() = default;
        // Delete copy constructor and assignment to only allow move semantics
        ConfigurationGeometry(const ConfigurationGeometry&) = delete;
        ConfigurationGeometry& operator= (const ConfigurationGeometry&) = delete;

        ConfigurationGeometry(ConfigurationGeometry&&) = default;
        ConfigurationGeometry& operator= (ConfigurationGeometry&&) = default;

        virtual edge_iterator edge_begin() const noexcept = 0;
        virtual edge_iterator edge_end() const noexcept = 0;
        virtual vertex_iterator vertex_begin() const noexcept = 0;
        virtual vertex_iterator vertex_end() const noexcept = 0;
        virtual void render(scene& scene) const override = 0;
    };
    
}
#endif
