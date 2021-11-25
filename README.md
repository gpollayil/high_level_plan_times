# HIGH LEVEL PLAN TIMES

## Known Moveit! Errors that can be ignored

### 1
```
[ERROR] [1637852169.720336462]: Exception while loading planning adapter plugin 'default_planner_request_adapters/ResolveConstraintFrames': According to the loaded plugin descriptions the class default_planner_request_adapters/ResolveConstraintFrames with base class type planning_request_adapter::PlanningRequestAdapter does not exist. Declared types are  chomp/OptimizerAdapter default_planner_request_adapters/AddIterativeSplineParameterization default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/Empty default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints default_planner_request_adapters/FixWorkspaceBounds
```
https://github.com/ros-planning/moveit/issues/1655

### 2
```
[ERROR] [1637853164.323558905]: Found empty JointState message
```

https://github.com/ros-planning/moveit/issues/659