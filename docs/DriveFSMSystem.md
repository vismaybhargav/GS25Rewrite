# Drive FSM Spec

```mermaid
---
title: Drive FSM Diagram
---
stateDiagram-v2
    state "TELEOP" as T
    state "PATHFIND" as P
    state "FINAL_ALIGN" as F
    [*] --> T: DriveFSM init
    T --> P: reefAlignPressed() 
    P --> F: reefAlignPressed()
    F --> T: isPathfindingFinished()
    P --> T: !reefAlignPressed()
```

### The default state for the DriveFSM is TELOP
