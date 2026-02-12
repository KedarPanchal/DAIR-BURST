#ifndef MODELS_HPP
#define MODELS_HPP

#include "types.hpp"
#include "configuration_geometry.hpp"

using ConfigurationGeometry = BURST::geometry::ConfigurationGeometry;

namespace BURST::models {
    
    /*
     * RotationModel classes define how the robot's rotation is affected by noise.
     */
    class RotationModel {
    public:
        virtual fscalar operator() (fscalar angle) const = 0;

        virtual fscalar getMaxRotationError(fscalar angle) const = 0;
        virtual fscalar getMinRotationError(fscalar angle) const = 0;
    };
    
    class PRRotationModel : public RotationModel {
    private:
        const fscalar max_rotation_error;

    public:
        PRRotationModel(fscalar max_rotation_error);

        fscalar operator() (fscalar angle) const override;

        fscalar getMaxRotationError(fscalar angle) const override;
        fscalar getMinRotationError(fscalar angle) const override;
    };
    
    class SeededPRRotationModel : public RotationModel {
    private:
        const fscalar max_rotation_error;
        const unsigned int seed;
    
    public:
        SeededPRRotationModel(fscalar max_rotation_error, unsigned int seed);

        fscalar operator() (fscalar angle) const override;

        fscalar getMaxRotationError(fscalar angle) const override;
        fscalar getMinRotationError(fscalar angle) const override;
    };
    
    class FixedRotationModel : public RotationModel {
    private:
        const fscalar max_rotation_error;
        const fscalar fixed_rotation_scale;
    
    public:

        FixedRotationModel(fscalar max_rotation_error, fscalar fixed_rotation_scale = 1);

        fscalar operator() (fscalar angle) const override;

        fscalar getMaxRotationError(fscalar angle) const override;
        fscalar getMinRotationError(fscalar angle) const override;
    };
    
    /*
     * MovementModel classes define how the robot's movement is affected by noise.
     */
    class MovementModel {
    public:
        virtual Point_2 operator() (fscalar angle, ConfigurationGeometry configuration_environment) const = 0;
        virtual Segment_2 generateTrajectory(Point_2 origin, fscalar angle, ConfigurationGeometry configuration_environment) const = 0;
    };
    
    class LinearMovementModel : public MovementModel {
    public:
        Point_2 operator() (fscalar angle, ConfigurationGeometry configuration_environment) const override;
        Segment_2 generateTrajectory(Point_2 origin, fscalar angle, ConfigurationGeometry configuration_environment) const override;
    };
    
}
#endif
