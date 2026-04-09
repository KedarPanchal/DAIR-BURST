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
    class Renderable {
    public:
        virtual Color defaultColor() const = 0;
        virtual void render(Scene& scene, const Color& color) const = 0;
    };

    // Sequences the rendering of a collection of renderables
    template <typename C> 
        requires std::ranges::bidirectional_range<C> &&
        std::same_as<std::remove_cv_t<std::ranges::range_value_t<C>>, Renderable> ||
        std::same_as<C, std::initializer_list<Renderable>>
    void render_all(const C& renderables, Scene& scene) {
        for (const auto& renderable : renderables | std::views::reverse) {
            renderable.render(scene, renderable.defaultColor());
        }
    }
    inline void render_all(const std::initializer_list<Renderable>& renderables, Scene& scene) {
        render_all<std::initializer_list<Renderable>>(renderables, scene);
    }

}
#endif
