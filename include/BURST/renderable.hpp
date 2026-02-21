#ifndef RENDERABLE_HPP
#define RENDERABLE_HPP

#include "graphics_types.hpp"

namespace BURST {
    
    /*
     * Renderable is an interface for objects that can be rendered in a visualization.
     */
    class Renderable {
    public:
        virtual void render(graphics::Scene& scene) const = 0;
    };
}

#endif
