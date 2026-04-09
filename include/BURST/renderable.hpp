#ifndef BURST_RENDERABLE_HPP
#define BURST_RENDERABLE_HPP

#include <CGAL/Arrangement_2.h>
#include <CGAL/Graphics_scene.h>
#include <CGAL/Graphics_scene_options.h>
#include <CGAL/draw_arrangement_2.h>
#include <CGAL/IO/Color.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace BURST {
    
    // Renderable is an interface for objects that can be rendered in a visualization.
    template <typename Traits, typename HalfEdgeList>
    class Renderable {
    private:
        using Scene = CGAL::Graphics_scene;
        using Color = CGAL::IO::Color;

        size_t id; 

    protected:
        using arrangement_t = CGAL::Arrangement_2<Traits, HalfEdgeList>;
        using graphics_options_t = CGAL::Graphics_scene_options<arrangement_t, typename arrangement_t::Vertex_const_handle, typename arrangement_t::Halfedge_const_handle, typename arrangement_t::Face_const_handle>;

        virtual arrangement_t make_arrangement() const noexcept = 0;

    public:
        Renderable() : id{boost::uuids::hash_value(boost::uuids::random_generator{}())} {}
        
        size_t uuid() const noexcept { 
            return this->id; 
        }
        void setUuid(size_t new_id) noexcept {
            this->id = new_id;
        }
        void resetUuid() noexcept {
            this->id = boost::uuids::hash_value(boost::uuids::random_generator{}());
        }

        void render(Scene& scene) {
            graphics_options_t graphics_options;
            // Set face coloring to color the renderable object
            graphics_options.colored_face = [](const arrangement_t&, arrangement_t::Face_const_handle) -> bool {
                return true;
            };
            graphics_options.face_color = [id= this->id](const arrangement_t&, arrangement_t::Face_const_handle) -> Color {
                Color object_color;
                double hue = static_cast<double>(id % 360);
                double saturation = static_cast<double>(id % 100) / 100.0;
                // Switch the upper and lower halves of the id to get more variability in the value component of the HSV color
                size_t half_size = sizeof(decltype(id)) / 2;
                double value = static_cast<double>(((id >> half_size) | (id << half_size)) % 100) / 100.0;
                return object_color.set_hsv(hue, saturation, value);
            };

            CGAL::add_to_graphics_scene(this->make_arrangement(), scene, graphics_options);
        }
    };

}
#endif
