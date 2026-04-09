#ifndef BURST_RENDERABLE_HPP
#define BURST_RENDERABLE_HPP

#include <CGAL/Arrangement_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace BURST::renderable {

    using Scene = CGAL::Graphics_scene;
    using Color = CGAL::IO::Color;
    
    // Renderable is an interface for objects that can be rendered in a visualization.
    template <typename Traits, typename HalfEdgeList>
    class Renderable {
    protected:
        using arrangement_t = CGAL::Arrangement_2<Traits, HalfEdgeList>;
        using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, typename arrangement_t::Vertex_const_handle, typename arrangement_t::Halfedge_const_handle, typename arrangement_t::Face_const_handle>;

    public:
        virtual void render(Scene& scene, const Color& color) = 0;
    };

}
#endif
