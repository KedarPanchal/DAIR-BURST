#ifndef BURST_RENDERABLE_HPP
#define BURST_RENDERABLE_HPP

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "graphics_types.hpp"

namespace BURST {
    
    /*
     * Renderable is an interface for objects that can be rendered in a visualization.
     */
    class Renderable {
        using UUID = boost::uuids::uuid;
    private:
        const UUID id; 
    public:
        Renderable() : id{boost::uuids::random_generator()()} {}

        UUID uuid() const noexcept { 
            return this->id; 
        }
        virtual void render(graphics::Scene& scene) = 0;
    };
}

#endif
