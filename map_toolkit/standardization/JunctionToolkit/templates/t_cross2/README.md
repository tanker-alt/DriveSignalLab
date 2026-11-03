# TCross2Clean runtime appearance

Reference: RoadRunner `TCross2Clean.udatasmith` and its adjacent exported XODR.
`profile.json` records the road dimensions and source hash; `appearance.json`
records the visual reference, runtime substitutions and vegetation policy.

The toolkit uses CARLA's cooked asphalt, white/yellow paint meshes, curved lamp
masts and foliage to reproduce the reference's style after OpenDRIVE loading.
It does not load Datasmith meshes directly. Commercial binary assets are not
bundled in this toolkit.

- Two incoming and two outgoing lanes per active arm, 3.5 m each.
- Six individually controllable native signal heads per T junction.
- 2 m sidewalks plus 0.2 m curbs; the absent arm has a continuous closed edge.
- Native sidewalk collisions retained; concrete visual caps are at 0.16 m.
- Runtime paint top is 0.006 m above the flat road; decorative collisions off.
- Trees occupy the union of real approach corridors, including network seams;
  exclude sidewalk buffers and junction visibility areas.

Generate with `toolkit.py generate --config CONFIG --out OUTPUT`, then load with
`toolkit.py view OUTPUT`. The view command applies the runtime appearance and
captures `overview.png` and `street.png`. Validate the loaded result with
`check_appearance.py OUTPUT` using the CARLA Python environment.

`t_single2c` and `t_network_template_v1` are the refreshed demonstration maps.
The latter is a connected 1 by 6 chain. Same-facing T junctions repeated across
multiple rows do not make a connected two-dimensional network; such a layout
requires alternating arm orientations and a separate connectivity design.
