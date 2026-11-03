# Heterogeneous networks ? 2026-11-03

Two-lane T and three-lane cross templates joined by tapered roads; optional single centered roundabout; continuous perimeter sidewalks. Per-road simple appearance supports all entry and circulating signals. Updated CARLA screenshots and Chinese README.

CARLA 0.9.15 simple appearance: 3x3 ring network 4406.53m / 12 physical junctions; 1x6 network 2746.97m / 6 junctions. Both zero collisions, respawns and teleports. Single vehicle, all-green connectivity only.

Publication branch prepared locally; no push performed.

Default simple appearance correction: ring concrete overlays are now complete-only; simple/off retain native sidewalk material across the network. Four static appearance tests pass. Earlier CARLA simple reports/screenshots predate this material-only correction; no new runtime acceptance is claimed.

CARLA retest completed for native simple sidewalk material: no ring concrete overlays; 16 native sidewalk connector samples present. Full 4406.53m route passed all 12 physical junctions, zero collisions/respawns/teleports. Updated overview/street images and simple_native_material_20261103 reports supersede the earlier pending runtime note.

Map inventory correction: existing t_1x6 and mixed_1x6 XODR hashes already match the November 3 full-network driving acceptance. Added missing full driving reports to all six homogeneous-network examples and added cross_1x6_connected_20261103. Replaced legacy package_smoke_t geometry with the accepted t_1x6 map and recorded explicit provenance. Directory commit dates do not measure last validation date.
