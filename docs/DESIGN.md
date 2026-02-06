# BURST Design

BURST (Blind Unreliable Robot Simulation Tool) is a simulation framework based on the "Guaranteed Coverage with a Blind Robot" paper by Lewis, Feshbach, and O'Kane.
It's meant to simulate the behavior of a blind robot with (up to the implementation's discretion) unreliable movement and rotation.
The robot is "blind" in the sense that it has no sensors to perceive its environment save for a bumper that detects when it's hit a wall.

## Dependencies and Implementation

BURST is implemented in C++ and relies on CGAL 6.1.1 for computational geometry operations.
Qt is used for visualization purposes.

BURST is a header-only library.

## Class Diagram

```mermaid
---
title: BURST Class Diagram
---
classDiagram
    class RotationPolicy {
        +Gmpq operator() (Gmpq rotation)*
    }
    <<interface>> RotationPolicy
    
    class PRRotationPolicy 
    PRRotationPolicy --|> RotationPolicy

    class SeededPRRotationPolicy {
        -const unsigned int seed
        +SeededPRRotationPolicy(unsigned int seed)
    }
    SeededPRRotationPolicy --|> RotationPolicy

    class MovementPolicy {
        +Point_2 operator() (Gmpq angle, ConfigurationGeometry configuration_environment)*
        +Segment_2 generateTrajectory(Point_2 origin, Gmpq angle, ConfigurationGeometry configuration_environment)*
    }
    <<interface>> MovementPolicy

    class LinearMovementPolicy {
    }
    LinearMovementPolicy --|> MovementPolicy

    class Renderable {
        +void render()*
    }
    <<interface>> Renderable

    class Robot~RotationPolicy, MovementPolicy~ {
        -Gmpq radius
        -Gmpq max_rotation_error
        -Gmpq x_position
        -Gmpq y_position
        -ConfigurationGeometry* configuration_environment

        +Robot(Gmpq robot_radius, Gmpq max_rotation_error)
        -setConfigurationEnvironment(ConfigurationGeometry* configuration_environment)
        +Gmpq getRadius()
        +Gmpq getMinRotationError(Gmpq rotation)
        +Gmpq getMaxRotationError(Gmpq rotation)
        +Point_2 shootRay(Gmpq angle)
        +Polygon_2 generateStadium(Gmpq angle)
        +Polygon_2 generateCCR(Gmpq angle)
    }
    Robot --|> Renderable
    MovementPolicy --* Robot
    RotationPolicy --* Robot

    class WallGeometry {
        -Polygon_2 wall_shape
        +WallGeometry(std::initializer_list<Point_2> edge_endpoints)
        +void generateConfigurationGeometry(Robot robot)
    }
    WallGeometry --|> Renderable

    class ConfigurationGeometry {
        #Polygon_2 configuration_shape
        -ConfigurationGeometry(std::initializer_list<Point_2> edge_endpoints)
        +Segment getEdge(Point_2 point)
        +Segment getEdge(Segment_2 segment)
    }
    <<abstract>> ConfigurationGeometry
    ConfigurationGeometry --|> Renderable
    WallGeometry "1" -- "1" ConfigurationGeometry : creates
    Robot "1" -- "1" ConfigurationGeometry : uses
    Robot ..> ConfigurationGeometry

    class ConfigurationGeometryImpl
    ConfigurationGeometryImpl --|> ConfigurationGeometry
```

> <p style="color: cyan; font-weight: bold;">NOTE:</p>
> The class diagram above explicitly uses CGAL types.
>
> Certain functions may be added or removed as the implementation progresses.

## Considerations

* The above diagram omits the use of smart pointers for clarity.
* CGAL is GPLv3, so BURST will need to be GPLv3 if we go ahead and use CGAL.
* Alternatively, we could use the Shapely Python library for computational geometry.
This is BSD-3 licensed but is slower and more inaccurate than CGAL.
