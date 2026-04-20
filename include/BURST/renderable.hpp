#ifndef BURST_RENDERABLE_HPP
#define BURST_RENDERABLE_HPP

#include <ranges>
#include <type_traits>
#include <initializer_list>

#include <CGAL/Arrangement_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/IO/Color.h>


#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

/**
 * @file renderable.hpp
 * @brief Abstraction for CGAL-backed visualization of BURST geometric objects.
 *
 * Consumers implement @ref Renderable and add geometry to a shared @ref Scene. Objects are rendered
 * in reverse order so later items in a collection can appear "above" earlier ones when overlays matter.
 */

namespace BURST::renderable {

    /** @brief CGAL graphics scene target for interactive or offline drawing. */
    using Scene = CGAL::Graphics_scene;

    /** @brief RGBA color type used when registering draw callbacks with CGAL graphics options. */
    using Color = CGAL::IO::Color;
    
    /**
     * @brief Polymorphic interface for anything that can contribute geometry to a @ref Scene.
     *
     * Typical implementations include environment geometry, configuration spaces, and the robot.
     */
    class Renderable {
    public:
        /** @brief Suggested stroke/fill color when the caller does not override a color. */
        virtual Color defaultColor() const = 0;
        /**
         * @brief Draw this object into `scene` using `color` for applicable primitives.
         * @param scene Target CGAL graphics scene.
         * @param color Override color; implementations may use it for edges, faces, or both.
         */
        virtual void render(Scene& scene, const Color& color) const = 0;
        virtual ~Renderable() = default;
    };

    /**
     * @brief Render every pointer in `renderables` into `scene`, last-to-first.
     *
     * Iterates the range in reverse order so items listed later are drawn on top of earlier ones
     * (useful when stacking semi-transparent layers). Each item is drawn with its own
     * @ref Renderable::defaultColor.
     *
     * @tparam C Bidirectional range of `const Renderable*` (or an initializer list of the same).
     * @param renderables Non-owning pointers to objects to draw; must remain valid for the call.
     * @param scene Destination scene.
     */
    template <typename C> 
        requires std::ranges::bidirectional_range<C> &&
        std::same_as<std::remove_cv_t<std::ranges::range_value_t<C>>, const Renderable*> ||
        std::same_as<C, std::initializer_list<const Renderable*>>
    void render_all(const C& renderables, Scene& scene) {
        for (const auto& renderable : renderables | std::views::reverse) {
            renderable->render(scene, renderable->defaultColor());
        }
    }
    /** @copydoc render_all */
    inline void render_all(const std::initializer_list<const Renderable*>& renderables, Scene& scene) {
        render_all<std::initializer_list<const Renderable*>>(renderables, scene);
    }

}
#endif
