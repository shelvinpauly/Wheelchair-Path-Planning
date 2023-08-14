# Wheelchair Airport Path Planning 

This project implements path planning algorithms for autonomous wheelchair navigation in an airport environment.

## Algorithms

- A* search for static environments
- RRT* sampling-based planning for dynamic environments 

## Implementation

- Simulated airport layout with pillars, seating areas, and moving people
- Compared A* and RRT* for path optimality, computation time
- Integrated optimal RRT* path into wheelchair control in Gazebo

## Key Results

- A* provides smooth, optimal paths but higher computation time
- RRT* finds feasible paths faster but are non-optimal
- RRT* works well for real-time planning with moving obstacles

## Discussion 

- Both algorithms have tradeoffs in optimality vs time
- RRT* is preferable for dynamic real-time applications
- Further improvements possible in handling dense environments

## Conclusion

- Successfully planned and executed wheelchair navigation in simulated airport
- Demonstrated the use of algorithms like RRT* for autonomous assistive robots

## References

[Final Report](Final_report.pdf)
[Presentation](presentation.pdf)

[1] Li et al., An RRT-Based Path Planning Strategy in a Dynamic Environment

