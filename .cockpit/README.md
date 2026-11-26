# .cockpit

This folder describes the project for the person who owns it and for the agents who work on
it. Cockpit (a viewer) reads it, checks it against the code, draws the diagrams, and lights
them up while an agent edits the code they show. You keep it true as you work.

| File | What it holds |
|---|---|
| `project.json` | `summary` and `features` |
| `diagrams/<id>.json` | one diagram per file; the file name is its id and the name people see: 2-4 words, kebab-case (`main-loop`, `physics-step`) |
| `layout.json` | where people placed the diagram boxes. Written by cockpit: don't edit it |

When to update: in the same turn as a change that adds, removes or moves modules, changes what a
feature does or its status, or makes a diagram wrong. Not for changes inside a function that
don't change what's written here.

A check runs when you finish a turn. If this folder doesn't match the code (unknown links,
modules in no feature, broken diagrams), you get the list and should fix it before stopping.

## Before you write

1. Read what the project says about itself first: its README, any docs folder, and any
   files that explain its context or architecture (CONTEXT.md, ARCHITECTURE.md, design notes,
   a CLAUDE.md or AGENTS.md). Use their words for things, and check them against the code:
   where the docs and the code disagree, the code wins, and say so in a feature's `note`.
2. List what cockpit knows about the code: the module list command in your brief prints every
   module and item exactly as links must write them, grouped by crate or build project, with
   test code marked. Give it a filter to see one part. Link only to names from that list.
3. Then read the code behind what you are about to describe. Describe what the code does
   today, not what a comment or a plan says it should do.

## Code links

Features and diagram nodes link to code by module or item, written exactly as the module list
prints them: `crate/module`, `crate/module::Item`
or `crate/module::Type::method`. The crate root module is just the crate, e.g. `game::main`.
In C/C++ a module is one file: `<CMake project>/<path inside it, without src/ or include/>`,
e.g. `engine/World.hpp`, `engine/World.cpp::World::step`, `net/Socket.hpp::Socket`.
In Zig a module is one file: `<package>/<path inside src/, without .zig>`, and a file is a struct, so
its own functions hang off the module: `ghostty/terminal/Terminal::print`, `ghostty/Surface::init`,
a nested type `ghostty/renderer/generic::Renderer::drawFrame`.

## Summary

`summary`: what this is and what matters most, in 2-3 sentences.

## Features

What the software does for its user, as it stands today. People see the count by status, and
the list, in cockpit. Each feature: `{"id", "name", "status", "description", "modules", "note"}`.
- `status`: `working`, `partial` (some of it works), `broken` (it used to work and doesn't
  now), or `planned` (no code yet, `modules` empty). Be honest: this is what people read to
  know where the project stands.
- `modules`: the modules that make it. A module can serve several features (the UI usually
  does). Every module serves at least one feature, except test code (marked `[test]` in the
  module list): cockpit shows tests on their own. A folder module covers the modules under it:
  listing `ghostty/font` covers `ghostty/font/Atlas`, so in a big project list folders and
  name single files only where a folder would claim too much.
- `note`: one line on what's missing or wrong. Required for `partial` and `broken`.
- Ids are short kebab-case and stable: edit features, don't rename them.

## Diagrams

Diagrams are how you explain this code to the person who owns the project. They know software
well but didn't write this code, and they understand a picture faster than prose. Cockpit
draws each diagram, explains each box on hover, links it to its code, and lights it up while
an agent edits that code.

People read in layers: a short line first, details only when they click. Every diagram and every
box has a `tldr`, read first, and longer text (`answer`, `explain`) shown only on request. Write
the `tldr` as the one thing to remember, in plain words: about 10 words for a diagram, about 8
for a box. Keep the longer texts short too: two sentences is plenty.

What earns a diagram:
- It answers one question the owner would ask, such as "what happens in one physics step?"
  The `question` is its title, `tldr` the answer in about 10 words, and `answer` the claim the
  picture makes, in one or two sentences.
- It shows a mechanism the reader would otherwise have to piece together from several files:
  the order things happen in, what data moves where, which boundary is crossed. If two
  sentences say it as well, write the sentences somewhere else and don't draw it. Say in `why`
  what the picture shows that prose doesn't.
- Start with the code where most behavior lives and most changes land. A diagram of the core
  is worth more than one about a single use case.
- As many diagrams as this project needs, no fixed number: a small tool may need one, a large
  system several. Each one answers a question none of the others answers; if a new one mostly
  repeats an existing one, improve that one instead.

The first diagram is always the main loop, `kind: "loop"`. Every program has one: a game's
frame loop, a server's request loop, a CLI's single pass. It shows where the program starts,
what repeats and how often (`rate`, e.g. "every screen frame, ~60 Hz"), what runs each round in
the order the code runs it, what is kept from one round to the next, and where it ends. It is
the map of when things happen. The other diagrams zoom into its boxes to show what happens inside.

Be explicit about time. When something repeats (a loop, a step, a frame), draw the state kept
between rounds as a `carries` node, with edges to what reads it this round and from what writes
it for the next, the latter marked `next`. A reader should be able to point at where a round
starts, where it ends and what goes around.

How to draw:
- `kind` is one of:
  - `loop`: the main loop or another repeating process, top to bottom in `nodes` order: a
    `start` node, the steps of one round in the order the code runs them, the `carries` state,
    an `end` node. One `next` edge goes from the last step back to the first. Needs `rate`.
  - `dataflow`: parts placed by what depends on what. Parts that don't depend on each other sit
    side by side, and parts that need several results sit below all of them. Use it for how
    values move through a computation.
  - `pipeline`: a strict sequence, top to bottom in `nodes` order, for steps where each really
    does need the one before.
  - `zones`: boxes inside boxes: what owns what, and what crosses each boundary.
- Draw what the code depends on, not the order it is written in. Code written top to bottom
  often has parts that don't use each other at all. For each part, check which values it reads
  and which earlier part wrote them, and draw only those edges.
- Nodes: `{"id", "label", "tldr", "explain", "anchors", ...}` with the optional flags below.
  - `label`: a few words. Use the code's names when the reader will meet them in the code.
  - `tldr`: what it does, in about 8 words, shown first.
  - `explain`: what it is and what it does, in plain words, with any project term spelled out.
    A name like `Frame` means nothing to someone who hasn't read the code, so say what it holds.
  - `anchors`: the code it stands for (see "Code links" above).
  - `zone: true` makes a node a box that holds others, and `"in": "<zone id>"` puts a node inside it.
  - `outside: true` is for things that aren't code: the gamepad, the screen, a file on disk.
  - `focus: true` marks the one node the diagram is about, if there is one, and its `explain`
    says why it matters. Leave it out when no single box is the point.
  - `start: true` / `end: true`: where it begins and where it stops (program start, window closed).
  - `carries: true`: state that is kept from one round to the next (the car's state, a buffer,
    the recording so far).
  - `detail: "<diagram id>"`: this box is explained by another diagram (its id is its file name
    without `.json`), which opens beside this one when clicked. Use it to go from the main loop down to the diagram of one of its steps.
  - `total: true` is for a shared total that several parts add into (an accumulator such as a
    force sum, a buffer, an event queue). Draw an edge from each part that adds to it.
- Edges: `{"from", "to", "label", "explain"}`. The `label` is what moves along the edge (a type,
  a value, "calls", "writes"). An edge without a label says nothing. `"next": true` marks a
  value that is used in the next round (the next step, frame or request) rather than this one,
  such as the new state that everything reads next time, or a measurement that a controller
  reacts to one step later. Feedback like this is often why the code behaves the way it does.
  Only the edge that closes a round is `next`. Writing into the carried state and reading it
  later in the same round are plain edges. If several parts read the previous round's state,
  draw that state once as a `carries` node ("the car at t") with plain edges to each reader, and
  a single `next` edge into it from where the new state is written. More than 2 `next` edges in
  one diagram fails the check.
- An edge is a real dependency: draw `A -> B` only when B uses something A produces, and label
  it with that thing. In a pipeline, the stages' order already comes from the `nodes` order, so
  don't add edges just to chain stages. When a later stage uses the results of several earlier
  ones, draw each of those edges: that is often the most useful thing the picture shows.
- Anchor each node to the most specific code that does it: a function or method rather than a
  whole module, and the helper it calls rather than the long function around it, when there is one.
- Leave out what the question doesn't need. Fewer than 12 nodes is a good sign.

Keep them true: when you change code that a diagram shows, check the diagram in the same turn.
Cockpit flags a diagram whose code changed while the diagram didn't.
