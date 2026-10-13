
Oct 12, 2026
feat: add DLQR control and response benchmarks, optimize NBV planning

- Add exact-ZOH DLQR tracking with buoyancy compensation and legacy controller support.
- Add transient response metrics, monitoring, and plots.
- Improve local frontier detection, NBV evaluation performance, and A* search.
- Make exploration radius and route candidate limits configurable.
- Update bringup tuning, documentation, tests, and experiment results.

Oct 13, 2026

feat: improve NBV planning, routing, and waypoint control

- Improve NBV scoring by reducing information gain for seabed points and keeping the top three viewpoints per cluster.
- Update frontier detection, route recovery, A* failure handling, and spline fallbacks.
- Refresh OctoMap collision checks, inflation, route validation, and replanning behavior.
- Tune waypoint control with stronger position feedback, a 0.8 m/s cruise speed, braking, and speed logging.
- Fix BGNet imports, add prediction tracking, update assets and ignore rules, and remove obsolete files.