# BURST (Blind Unreliable Robot Simulation Tool)

**BURST (Blind Unreliable Robot Simulation Tool)** is a C++20 library for simulating a blind circular robot moving in 2D polygonal environments.

This project focuses on geometric reasoning for robot motion under limited sensing, using exact geometry primitives from CGAL to model wall spaces, configuration spaces, trajectories, and robot movement behavior.

### What the project includes

- A header-only C++ library under `include/BURST/`
- Core geometry and numeric abstractions (`kernel`, `geometry`, `numeric`)
- Environment and free-space representations (`wall_space`, `configuration_space`)
- Robot behavior and uncertainty models (`robot`, `models`)
- Rendering support (`renderable`) for visualizing geometry and simulation outputs
- Unit tests under `tests/` for geometry, robot behavior, and rendering-related components

### Research context

BURST was created for research in **Texas A&M University's Distributed AI and Robotics Lab (DAIR Lab)**, run by **Dr. Shell**. The codebase is intended to support experimentation and analysis of blind robot motion and coverage behavior in constrained environments.

### Documentation

API documentation is generated with Doxygen using the repository's top-level `Doxyfile`.

### License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**. See [`LICENSE`](LICENSE) for the full license text.
