# Legacy WRS/Panda3D preview (not part of the product)

This is the old WRS 1.x / Panda3D preview that was removed from the station UI,
the `/api/wrs` endpoint and the shipped package.

- It targets old WRS (`wrs.robot_sim`, `wrs.visualization.panda`), **not**
  WRS2 / WRSAgentOS.
- It is not imported, tested or packaged by the station.
- `/api/info` no longer exposes `q_deg`, so this viewer is not runnable against
  the current station; it is kept only as historical reference.

Do not use this as a starting point for a WRS2 integration. A proper integration
needs a WRS2 robot model, an independent environment process and a WRS2 viewer.
