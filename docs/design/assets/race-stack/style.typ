// Shared look for the race-stack diagrams (architecture, field, repos, stages).
// fletcher 0.5.1: the newest that the installed typst (0.12) can load.
#import "@preview/fletcher:0.5.1" as fletcher: diagram, node, edge

#let ink = rgb("#1f2933")
#let dim = rgb("#52606d")

// colour = the repository that owns the box
#let c-aw = rgb("#e8eef7");    #let s-aw = rgb("#7b93b8")    // Autoware 1.5.0 (stock)
#let c-race = rgb("#e6f4ea");  #let s-race = rgb("#5f9e74")  // autoware-drift
#let c-rig = rgb("#fdf1dc");   #let s-rig = rgb("#c89b4a")   // rig_localization
#let c-sdv = rgb("#f1e8f7");   #let s-sdv = rgb("#9a7bb0")   // AutoSDV
#let c-hw = rgb("#eceff1");    #let s-hw = rgb("#8a959e")    // hardware, external tools
#let c-data = white;           #let s-data = rgb("#7b8794")  // files

#let page-setup(body) = {
  set page(width: auto, height: auto, margin: 14pt, fill: white)
  set text(font: "Noto Sans", size: 10pt, fill: ink)
  body
}

#let title(t, sub) = [#text(weight: "bold", t) \ #text(size: 8pt, fill: dim, sub)]
#let mono(t) = text(font: "DejaVu Sans Mono", size: 8.5pt, t)
#let topic(t) = text(size: 7pt, fill: dim, font: "DejaVu Sans Mono", t)

// new: dashed border (built by this work); solid: exists today
#let bx(pos, body, fill, stroke, name, w: 36mm, new: false) = node(
  pos, align(center, body), name: name, fill: fill,
  stroke: if new { (paint: stroke, thickness: 1pt, dash: "dashed") } else { 0.9pt + stroke },
  corner-radius: 4pt, inset: 6pt, width: w)
// a file: square corners, white, thin
#let file(pos, body, name, w: 34mm, new: false) = node(
  pos, align(center, body), name: name, fill: c-data,
  stroke: if new { (paint: s-data, thickness: 0.8pt, dash: "dashed") } else { 0.7pt + s-data },
  corner-radius: 0pt, inset: 5pt, width: w)
#let group(nodes, fill, stroke, name: none) = node(
  enclose: nodes, fill: fill.transparentize(60%),
  stroke: (paint: stroke, thickness: 0.6pt, dash: "dotted"),
  corner-radius: 8pt, inset: 9pt, snap: false, name: name)
#let tag(pos, t, stroke, name) = node(pos, text(size: 8.5pt, weight: "bold", fill: stroke.darken(30%), t),
  name: name, stroke: none, fill: none, inset: 2pt)

#let sw(fill, stroke, new: false, square: false) = box(
  width: 9pt, height: 9pt, radius: if square { 0pt } else { 2pt }, fill: fill,
  stroke: if new { (paint: stroke, thickness: 1pt, dash: "dashed") } else { 0.9pt + stroke })

// legend; `extra` adds (swatch, label) pairs
#let legend(..extra) = {
  set text(size: 8.5pt, fill: dim)
  let items = (
    (sw(c-aw, s-aw), [Autoware 1.5.0 (stock)]),
    (sw(c-race, s-race), [autoware-drift]),
    (sw(c-rig, s-rig), [rig_localization]),
    (sw(c-sdv, s-sdv), [AutoSDV]),
    (sw(c-hw, s-hw), [hardware, external tool]),
  ) + extra.pos() + ((sw(white, dim, new: true), [dashed: new in this work]),)
  grid(columns: items.len() * 2, column-gutter: 4pt, align: horizon,
    ..items.map(((s, l)) => (s, [#l #h(8pt)])).flatten())
}
