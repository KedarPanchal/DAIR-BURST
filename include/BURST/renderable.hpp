#ifndef BURST_RENDERABLE_HPP
#define BURST_RENDERABLE_HPP

#include <CGAL/Arrangement_2.h>
#include <CGAL/Graphics_scene_options.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include "graphics_types.hpp"

namespace BURST {
    
    // Renderable is an interface for objects that can be rendered in a visualization.
    template <typename Traits>
    class Renderable {
    protected:
        using arrangement_t = CGAL::Arrangement_2<Traits>;
        using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, typename arrangement_t::Vertex_const_handle, typename arrangement_t::Halfedge_const_handle, typename arrangement_t::Face_const_handle>;

        const size_t id;
    public:
        Renderable() : id{boost::uuids::hash_value(boost::uuids::random_generator{}())} {}

        size_t uuid() const noexcept { 
            return this->id; 
        }
        virtual void render(graphics::Scene& scene) = 0;
    };
}

#endif
