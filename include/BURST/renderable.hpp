#ifndef BURST_RENDERABLE_HPP
#define BURST_RENDERABLE_HPP

#include <ranges>

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

    // Sequences the rendering of a collection of renderables
    template <template <typename> typename C, typename Traits, typename HalfEdgeList> requires std::ranges::bidirectional_range<C<Renderable<Traits, HalfEdgeList>>>
    void render_all(const C<Renderable<Traits, HalfEdgeList>>& renderables, Scene& scene) {
        for (const auto& renderable : renderables | std::views::reverse) {
            renderable.render(scene);
        }
    }
    template <typename Traits, typename HalfEdgeList>
    inline void render_all(const std::initializer_list<Renderable<Traits, HalfEdgeList>>& renderables, Scene& scene) {
        render_all<std::initializer_list<Renderable<Traits, HalfEdgeList>>>(renderables, scene);
    }

}
#endif
