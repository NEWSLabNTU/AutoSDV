// The seven phases and what each needs from the others.
// Roadmap: docs/roadmap/16-autoware-drift/README.md
// Render: typst compile --format svg stages.typ
#import "style.typ": *
#show: page-setup

#align(center, text(size: 13pt, weight: "bold")[autoware-drift on the AutoSDV cars: phases])
#v(4pt)

#let ph(pos, n, t, exit, fill, stroke, id) = bx(pos,
  [#text(weight: "bold")[#n. #t] \ #text(size: 8pt, fill: dim)[#exit]], fill, stroke, id, w: 50mm, new: true)

#diagram(
  spacing: (12mm, 10mm),
  edge-stroke: 0.8pt + dim,
  mark-scale: 80%,

  tag((0, -0.8), [software, in simulation], s-race, <t-sw>),
  ph((0, 0), 1, [Submodule, gate, benchmark], [builds in AutoSDV · full-size runs bit for bit · L1 and NMPC timed on an Orin under load], c-race, s-race, <p1>),
  ph((1, 0), 3, [Scale and course], [scale profile, `v_max` · COSS course file passes `just race check` · offline plan on the map], c-race, s-race, <p3>),
  ph((2, 0), 4, [ROS nodes in simulation], [laps past the route's goal on the RC sim, then on car A's twin, under the cap], c-race, s-race, <p4>),

  tag((0, 1.2), [on the cars], s-sdv, <t-car>),
  ph((0, 2), 2, [Car facts], [A, B, C measured: mass, CoM, coast-down, skidpad, motor, servo, differential, braking], c-sdv, s-sdv, <p2>),
  ph((2, 2), 5, [Cameras and estimator], [three ZEDs calibrated · rig's vy judged on the twin's truth and on real laps], c-rig, s-rig, <p5>),
  ph((1, 3.4), 6, [Real car, grip], [accel plug-in, overspeed guard · grip laps from ≈ 1.0 m/s, A then B, C], c-sdv, s-sdv, <p6>),
  ph((2, 3.4), 7, [Real car, drift], [planned drifts on COSS once vy holds and the differential is modelled], c-sdv, s-sdv, <p7>),

  edge(<p1>, <p3>, "-|>"),
  edge(<p3>, <p4>, "-|>"),
  edge(<p2>, <p4>, "--|>", topic[specs (estimates until then)], label-side: right, label-pos: 0.4),
  edge(<p4>, <p5>, "-|>", topic[twin truth], label-side: left),
  edge(<p2>, (0, 3.4), <p6>, "-|>", topic[measured before the cap rises], label-side: right, label-pos: 0.3),
  edge(<p5>, <p6>, "-|>", topic[state], label-side: right),
  edge(<p6>, <p7>, "-|>"),
  edge(<p5>, <p7>, "-|>", topic[vy], label-side: left),
  edge(<p2>, (0, 4.4), (2, 4.4), <p7>, "-|>", topic[differential type], label-side: right, label-pos: 0.6),
)

#v(6pt)
#text(size: 8.5pt, fill: dim)[Box colour: where most of the phase's work lands (green autoware-drift, purple AutoSDV, orange rig_localization). Solid arrow: needs the result. Dashed: uses it once it exists. Phases 1 and 2 start together.]
