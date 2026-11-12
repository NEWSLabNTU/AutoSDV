// The field, offline: from a GLIM scan of COSS to the race planner's course file.
// Spec: docs/superpowers/specs/2026-11-12-autoware-drift-integration-design.md ("Field model")
// Render: typst compile --format svg field.typ
#import "style.typ": *
#show: page-setup

#align(center, text(size: 13pt, weight: "bold")[The field, offline: COSS scan → course file])
#v(4pt)

#diagram(
  spacing: (10mm, 8mm),
  edge-stroke: 0.8pt + dim,
  mark-scale: 80%,

  // ── capture and map ────────────────────────────────────────────────────
  bx((0, 0), title[`just bag record`][drive the site by hand], c-sdv, s-sdv, <rec>, w: 34mm),
  bx((1.4, 0), title[`just bag map`][GLIM], c-sdv, s-sdv, <glim>, w: 30mm),
  file((2.7, 0), [`raw_map.pcd`], <raw>, w: 28mm),
  bx((4.0, 0), title[register][into COSS's map frame], c-sdv, s-sdv, <reg>, w: 32mm, new: true),
  file((4.0, 1.2), [`pointcloud_map.pcd` \ #text(size: 8pt, fill: dim)[also NDT's map]], <pcd>, w: 34mm),

  bx((2.4, 1.2), title[`pcd_to_pgm`][0.05 m band above ground], c-sdv, s-sdv, <pgm>, w: 34mm),
  file((2.4, 2.4), [occupancy grid \ #text(size: 8pt, fill: dim)[clearance by distance transform]], <grid>, w: 38mm),
  bx((5.6, 1.2), title[ground fit][per-cell minimum \ (`ground_estimate`)], c-sdv, s-sdv, <ground>, w: 30mm),

  bx((0, 1.2), title[Vector Map Builder][the loop's lanelets, by hand], c-hw, s-hw, <vmb>, w: 34mm),
  file((0, 2.4), [`lanelet2_map.osm` \ #text(size: 8pt, fill: dim)[\+ loop ids, `surface` tags]], <osm>, w: 34mm),
  bx((5.6, 2.4), title[skidpad runs][per surface], c-sdv, s-sdv, <skid>, w: 30mm, new: true),
  file((5.6, 3.5), [surface classes \ #text(size: 8pt, fill: dim)[`mu`, sliding, peak slips]], <surf>, w: 34mm, new: true),

  // ── course ─────────────────────────────────────────────────────────────
  bx((2.4, 3.6), title[`just race course`][calls `Track.from_lanelet2`], c-sdv, s-sdv, <course>, w: 40mm, new: true),
  bx((2.4, 4.8), title[`Track.from_lanelet2`][centreline at 0.05 m, smoothed ≈ 0.2 m \ widths = min(lanelet bound, grid clearance) − margin \ z and grade from the ground fit · `mu` per class], c-race, s-race, <track>, w: 70mm, new: true),
  file((2.4, 6.0), [COSS course file \ #text(size: 8pt, fill: dim)[s, x, y, z, ψ, κ, w#sub[left], w#sub[right], μ, closed]], <cfile>, w: 56mm, new: true),
  bx((0.4, 7.1), title[`just race check`][closed · widths ≥ car + margins \ κ within the lock · inside free space], c-sdv, s-sdv, <check>, w: 46mm, new: true),
  bx((4.4, 7.1), title[race_planner][L0 plan (optional) · on the car: \ L1 re-plans from the course file], c-race, s-race, <rp>, w: 46mm),

  // ── edges ──────────────────────────────────────────────────────────────
  edge(<rec>, <glim>, "-|>", topic[bag]),
  edge(<glim>, <raw>, "-|>"),
  edge(<raw>, <reg>, "-|>"),
  edge(<reg>, <pcd>, "-|>"),
  edge(<pcd>, <pgm>, "-|>"),
  edge(<pcd>, <ground>, "-|>"),
  edge(<pgm>, <grid>, "-|>"),
  edge(<pcd>, (4.0, 0.6), (0, 0.6), <vmb>, "-|>", topic[drawn over the cloud], label-pos: 0.6, label-side: right),
  edge(<vmb>, <osm>, "-|>"),
  edge(<skid>, <surf>, "-|>"),
  edge(<osm>, (0, 3.6), <course>, "-|>"),
  edge(<grid>, <course>, "-|>"),
  edge(<ground>, (6.6, 1.2), (6.6, 4.8), <track>, "-|>", topic[ground z], label-pos: 0.5, label-side: left),
  edge(<surf>, (5.6, 3.6), <course>, "-|>"),
  edge(<course>, <track>, "-|>"),
  edge(<track>, <cfile>, "-|>"),
  edge(<cfile>, <check>, "-|>"),
  edge(<cfile>, <rp>, "-|>"),
)

#v(6pt)
#legend((sw(c-data, s-data, square: true), [file]))
#v(2pt)
#text(size: 8.5pt, fill: dim)[One frame throughout: the PCD, the lanelet2 map and the grid share COSS's map frame (`autosdv_map.yaml`: `frame_consistent_with_lanelet2: true`); a re-scan is registered into it, so the lanelet map stays valid.]
