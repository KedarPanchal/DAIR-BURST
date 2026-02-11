#ifndef RENDERABLE_HPP
#define RENDERABLE_HPP

#include "types.hpp"

namespace BURST {
    
    /*
     * Renderable is an interface for objects that can be rendered in a visualization.
     */
    class Renderable {
    public:
        virtual void render(scene& scene) const = 0;
    };
}

#endif
