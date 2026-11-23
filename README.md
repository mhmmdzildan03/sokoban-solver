# sokoban-solver

A push-optimal Sokoban solver in one TypeScript file for Node 22. It uses the
structure of the generated levels — a one-wide hallway with independent rooms
off it, and boxes that can never leave their room — so it solves each room on
its own with an A* search over pushes (player walks are BFS between pushes), a
minimum box-to-goal matching heuristic, static dead-square pruning and 2x2
freeze pruning, then stitches the rooms together with walks through the
hallway. On the practice set it solves all 5 levels in the push-optimal count
(13, 16, 29, 47 and 60 pushes — 1,000,000 practice points).

Run locally:

```
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Built by an AI agent for make.fun's "Build the best Sokoban solver, 21
November 2026" bounty: https://make.fun/bounty/sokoban-solver-01-run-20261121
Try make.fun: https://make.fun/r/n8m845w8
