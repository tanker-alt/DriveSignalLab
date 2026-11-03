# Random grid appearance repair

The old merge path removed all template roads numbered 900 and higher. That removed the rounded pavement fill and junction curb/sidewalk geometry. It also copied signal elements without their top-level controllers and remapped road references but not junction references. Deleting an arm removed only its approach, leaving turn roads to the deleted approach.

`template_network.build` now calls the existing generator once for a complete four-arm grid, then removes each planned north approach and every internal movement to/from it. It keeps the standard full pavement and curb geometry, and rebuilds only the three-arm surface using the same standard geometry helper with three incoming lanes. Signal controllers and junction controller references are retained or pruned together. Surviving seams use the original full-width start-to-start connection with lane signs inverted; there are no artificial narrow connector roads or extra seam junctions.

Network appearance keeps markings, stop lines, crossings, native sidewalks and controllable native heads plus standard curved masts. Trees, grass meshes, roadside furniture and debug overlays remain disabled. Native OpenDRIVE road/sidewalk collision remains enabled; runtime decoration collision is disabled before screenshot delivery.

Verification:
- `validate_network.py MAP_DIR`: reference integrity, unique signal/controller mappings, safety-rule checks, actual CARLA waypoint traversal on all retained seams in both directions, logical junction graph connectivity.
- `network_preview.py MAP_DIR`: server map identity, signal lane associations, standard mast positioning, collision disabling, overview/street RGB captures.
- `loaded_geometry_check.json`: signal lane associations and native corner/closed-side sidewalk samples from the loaded world.

These checks do not certify every traffic movement or dynamic vehicle collision scenario. The current random plan still uses north-arm deletion; arbitrary deletion orientations and balanced sampling are separate work.
