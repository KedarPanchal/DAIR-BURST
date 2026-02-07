#ifndef MODELS_HPP
#define MODELS_HPP

#include <CGAL/Gmpq.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include "configuration_geometry.hpp"

using precise_float = CGAL::Gmpq;
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;

using ConfigurationGeometry = BURST::geometry::ConfigurationGeometry;

namespace BURST::models {
    
    /*
     * RotationModel classes define how the robot's rotation is affected by noise.
     */
    class RotationModel {
    protected:
        const precise_float max_rotation_error;
    
    public:
        RotationModel(precise_float max_error);
        virtual precise_float operator() (precise_float rotation) const = 0;
        precise_float getMaxRotationError(precise_float angle) const;
        precise_float getMinRotationError(precise_float angle) const;
    };
    
    class PRRotationModel : public RotationModel {
    public:
        PRRotationModel(precise_float max_rotation_error);
        precise_float operator() (precise_float rotation) const override;
    };
    
    class SeededPRRotationModel : public RotationModel {
    private:
        const unsigned int seed;
    
    public:
        SeededPRRotationModel(precise_float max_rotation_error, unsigned int seed);
        precise_float operator() (precise_float rotation) const override;
    };
    
    class FixedRotationModel : public RotationModel {
    private:
        const precise_float fixed_rotation_scale;
    
    public:
        FixedRotationModel(precise_float max_rotation_error, precise_float fixed_rotation_scale);
        precise_float operator() (precise_float rotation) const override;
    };
    
    /*
     * MovementModel classes define how the robot's movement is affected by noise.
     */
    class MovementModel {
    public:
        virtual Point_2 operator() (precise_float angle, ConfigurationGeometry configuration_environment) const = 0;
        virtual Segment_2 generateTrajectory(Point_2 origin, precise_float angle, ConfigurationGeometry configuration_environment) const = 0;
    };
    
    class LinearMovementModel : public MovementModel {
    public:
        Point_2 operator() (precise_float angle, ConfigurationGeometry configuration_environment) const override;
        Segment_2 generateTrajectory(Point_2 origin, precise_float angle, ConfigurationGeometry configuration_environment) const override;
    };
    
}
#endif
