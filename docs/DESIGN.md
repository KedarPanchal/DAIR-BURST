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
        #Gmpq rotation_error
        +RotationPolicy(Gmpq max_rotation_error)
        +Gmpq operator() (Gmpq rotation)*
    }
    <<interface>> RotationPolicy

    class MovementPolicy {
        +Point_2 operator(Gmpq angle, ConfigurationGeometry configuration_environment)*
    }
    <<interface>> MovementPolicy

    class Renderable {
        +void render()*
    }
    <<interface>> Renderable

    class Robot~RotationPolicy, MovementPolicy~ {
        -Gmpq radius
        +Robot(Gmpq robot_radius)
        +Gmpq getRadius()
        +Point_2 shootRay(Gmpq angle, ConfigurationGeometry configuration_environment)
    }
    Robot --> Renderable

    class WallGeometry {
        -Polygon_2 wall_shape
        +WallGeometry(std::initializer_list<Point_2> edge_endpoints)
        +ConfigurationGeometry generateConfigurationGeometry(Gmpq robot_radius)
    }
    WallGeometry --> Renderable

    class ConfigurationGeometry {
        -Polygon_2 configuration_shape
        +Segment getEdge(Point_2 point)
        +Segment getEdge(Segment_2 segment)
    }
    ConfigurationGeometry --> Renderable
    WallGeometry "1" -- "1" ConfigurationGeometry : creates
```

## Considerations

CGAL is GPLv3, so BURST will need to be GPLv3 if we go ahead and use CGAL.
