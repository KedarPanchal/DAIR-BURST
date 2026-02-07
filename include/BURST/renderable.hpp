#ifndef RENDERABLE_HPP
#define RENDERABLE_HPP

namespace BURST {
    
    /*
     * Renderable is an interface for objects that can be rendered in a visualization.
     */
    class Renderable {
    public:
        virtual void render() const = 0;
    };
}

#endif
