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

        virtual curve_iterator curve_begin() const noexcept = 0;
        virtual curve_iterator curve_end() const noexcept = 0;
        virtual winding_order orientation() const noexcept = 0;
        virtual void render(scene& scene) const override = 0;
    };
    
}
#endif
