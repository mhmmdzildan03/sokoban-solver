// Sokoban solver: room-decomposed push-optimal A*.
// The generator builds each level as a one-wide hallway with independent rooms off it;
// a box pushed into the hallway can never return, so every room is solved on its own,
// in pushes, with an A* over pushes (player walks are BFS between pushes), a minimum
// box-to-goal matching heuristic, static dead-square pruning and 2x2 freeze pruning.
// Rooms are then stitched together with plain walks through the hallway.
// If decomposition does not apply, the same push A* runs on the whole level.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DIRS: [number, number, string][] = [[0, -1, 'u'], [0, 1, 'd'], [-1, 0, 'l'], [1, 0, 'r']];

// ---------- generic push A* ----------
// walkable: cells the player may stand on. boxCells: cells a box may occupy (subset).
// startPlayer: a walkable cell. Returns {moves, endPlayer, pushes} or null.
function solvePush(w: number, wall: boolean[], walkable: Set<number>, boxCells: Set<number>,
  startBoxes: number[], goals: number[], startPlayer: number, cap: number):
  { moves: string, endPlayer: number, pushes: number } | null {
  const goalSet = new Set(goals);
  // static push distance box->each goal (BFS over pulls, other boxes ignored) for heuristic + dead squares
  const live = new Set<number>(goals);
  {
    const q = [...goals];
    while (q.length) {
      const b = q.pop()!;
      for (const [dx, dy] of DIRS) {
        const prev = b - (dy * w + dx);
        const stand = b - 2 * (dy * w + dx);
        if (boxCells.has(prev) && walkable.has(stand) && !live.has(prev)) { live.add(prev); q.push(prev); }
      }
    }
  }
  const distTo: number[][] = goals.map((g) => {
    const d = new Array(wall.length).fill(1e9);
    d[g] = 0; const q = [g];
    while (q.length) {
      const b = q.shift()!;
      for (const [dx, dy] of DIRS) {
        const prev = b - (dy * w + dx); const stand = b - 2 * (dy * w + dx);
        if (boxCells.has(prev) && walkable.has(stand) && d[prev] === 1e9) { d[prev] = d[b] + 1; q.push(prev); }
      }
    }
    return d;
  });
  const perms: number[][] = [];
  {
    const idx = goals.map((_, i) => i);
    const gen = (a: number[], n: number): void => {
      if (n <= 1) { perms.push([...a]); return; }
      for (let i = 0; i < n; i++) { gen(a, n - 1); const j = n % 2 === 0 ? i : 0; [a[j], a[n - 1]] = [a[n - 1]!, a[j]!]; }
    };
    if (idx.length <= 7) gen(idx, idx.length); else perms.push(idx);
  }
  const heur = (bs: number[]): number => {
    let bestH = 1e9;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < bs.length; i++) { const dd = distTo[p[i]!]![bs[i]!]!; if (dd >= 1e9) { s = 1e9; break; } s += dd; }
      if (s < bestH) bestH = s;
    }
    return bestH;
  };
  // player reachability BFS; returns map cell -> path string
  const reach = (from: number, boxSet: Set<number>): Map<number, string> => {
    const seen = new Map<number, string>([[from, '']]);
    const q = [from];
    while (q.length) {
      const c = q.shift()!; const path = seen.get(c)!;
      for (const [dx, dy, ch] of DIRS) {
        const n = c + dy * w + dx;
        if (!walkable.has(n) || boxSet.has(n) || seen.has(n)) continue;
        seen.set(n, path + ch); q.push(n);
      }
    }
    return seen;
  };
  type Node = { boxes: number[], player: number, g: number, parent: number, seg: string };
  const nodes: Node[] = [];
  const heap: [number, number, number][] = []; // f, -g, nodeIdx
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const hpush = (e: [number, number, number]): void => {
    let i = heap.length; heap.push(e);
    while (i > 0) { const p = (i - 1) >> 1; if (!less(heap[i]!, heap[p]!)) break; [heap[i], heap[p]] = [heap[p]!, heap[i]!]; i = p; }
  };
  const hpop = (): [number, number, number] => {
    const top = heap[0]!; const last = heap.pop()!;
    if (heap.length) {
      heap[0] = last;
      for (let i = 0; ;) { let c = 2 * i + 1; if (c >= heap.length) break; if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++; if (!less(heap[c]!, heap[i]!)) break; [heap[i], heap[c]] = [heap[c]!, heap[i]!]; i = c; }
    }
    return top;
  };
  const start = [...startBoxes].sort((a, b) => a - b);
  if (start.some((b) => !live.has(b))) return null;
  const h0 = heur(start); if (h0 >= 1e9) return null;
  nodes.push({ boxes: start, player: startPlayer, g: 0, parent: -1, seg: '' });
  const best = new Map<string, number>();
  {
    const r = reach(startPlayer, new Set(start));
    let rep = startPlayer; for (const k of r.keys()) if (k < rep) rep = k;
    best.set(start.join(',') + '|' + rep, 0);
  }
  hpush([h0, 0, 0]);
  let expanded = 0;
  while (heap.length && expanded < cap) {
    const [, , ni] = hpop();
    const node = nodes[ni]!;
    if (node.boxes.every((b) => goalSet.has(b))) {
      let moves = '';
      for (let t = ni; t > 0; t = nodes[t]!.parent) moves = nodes[t]!.seg + moves;
      return { moves, endPlayer: node.player, pushes: node.g };
    }
    expanded++;
    const boxSet = new Set(node.boxes);
    const r = reach(node.player, boxSet);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi]!;
      for (const [dx, dy, ch] of DIRS) {
        const d = dy * w + dx;
        const stand = b - d; const target = b + d;
        if (!boxCells.has(target) || boxSet.has(target)) continue;
        if (!live.has(target)) continue;
        const walkPath = r.get(stand); if (walkPath === undefined) continue;
        const nb = [...node.boxes]; nb[bi] = target; nb.sort((a, b2) => a - b2);
        // 2x2 freeze: any 2x2 of walls/boxes with a box off-goal is dead
        let frozen = false;
        const nbSet = new Set(nb);
        outer: for (const c of nb) {
          for (const [ox, oy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]] as [number, number][]) {
            const cells = [c + ox + oy * w, c + ox + 1 + oy * w, c + ox + (oy + 1) * w, c + ox + 1 + (oy + 1) * w];
            if (cells.every((cc) => wall[cc] || nbSet.has(cc)) && cells.some((cc) => nbSet.has(cc) && !goalSet.has(cc))) { frozen = true; break outer; }
          }
        }
        if (frozen) continue;
        const h = heur(nb); if (h >= 1e9) continue;
        const g = node.g + 1;
        const nr = reach(b, nbSet);
        let rep = b; for (const k of nr.keys()) if (k < rep) rep = k;
        const key = nb.join(',') + '|' + rep;
        const prev = best.get(key); if (prev !== undefined && prev <= g) continue;
        best.set(key, g);
        nodes.push({ boxes: nb, player: b, g, parent: ni, seg: walkPath + ch });
        hpush([g + h, -g, nodes.length - 1]);
      }
    }
  }
  return null;
}

// walk path (no pushes) on the whole board with boxes as obstacles; null if unreachable
function walkPath(w: number, wall: boolean[], from: number, to: number, boxes: Set<number>): string | null {
  if (from === to) return '';
  const seen = new Map<number, string>([[from, '']]);
  const q = [from];
  while (q.length) {
    const c = q.shift()!; const path = seen.get(c)!;
    for (const [dx, dy, ch] of DIRS) {
      const n = c + dy * w + dx;
      if (wall[n] || boxes.has(n) || seen.has(n)) continue;
      const np = path + ch;
      if (n === to) return np;
      seen.set(n, np); q.push(n);
    }
  }
  return null;
}

function solveLevel(text: string): string {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Array(w * h).fill(true);
  const goals: number[] = []; let boxes: number[] = []; let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < w; x++) {
      const ch = row[x] ?? '#'; const i = y * w + x;
      wall[i] = ch === '#';
      if ('.*+'.includes(ch)) goals.push(i);
      if ('$*'.includes(ch)) boxes.push(i);
      if ('@+'.includes(ch)) player = i;
    }
  });
  const allFloor = new Set<number>(); for (let i = 0; i < w * h; i++) if (!wall[i]) allFloor.add(i);
  const py = Math.floor(player / w);
  const hallway = new Set<number>();
  for (let x = 0; x < w; x++) { const i = py * w + x; if (!wall[i]) hallway.add(i); }
  // components of floor minus hallway
  const compOf = new Map<number, number>(); const comps: Set<number>[] = [];
  for (const c of allFloor) {
    if (hallway.has(c) || compOf.has(c)) continue;
    const comp = new Set<number>(); const q = [c]; compOf.set(c, comps.length);
    while (q.length) { const cur = q.pop()!; comp.add(cur); for (const [dx, dy] of DIRS) { const n = cur + dy * w + dx; if (allFloor.has(n) && !hallway.has(n) && !compOf.has(n)) { compOf.set(n, comps.length); q.push(n); } } }
    comps.push(comp);
  }
  const rooms = comps.filter((c) => boxes.some((b) => c.has(b)));
  const decompOk = rooms.length > 0 && rooms.every((c) => {
    const rb = boxes.filter((b) => c.has(b)); const rg = goals.filter((g) => c.has(g));
    return rb.length === rg.length && rb.length > 0;
  }) && boxes.every((b) => rooms.some((c) => c.has(b)));
  if (decompOk) {
    let moves = ''; let cur = player; const globalBoxes = new Set(boxes);
    // deterministic room order: by smallest cell
    const ordered = [...rooms].sort((a, b) => Math.min(...a) - Math.min(...b));
    let failed = false;
    for (const room of ordered) {
      const rb = boxes.filter((b) => room.has(b)); const rg = goals.filter((g) => room.has(g));
      const doors = [...hallway].filter((hc) => DIRS.some(([dx, dy]) => room.has(hc + dy * w + dx)));
      if (!doors.length) { failed = true; break; }
      const walkable = new Set<number>([...room, ...doors]);
      const startDoor = doors.reduce((a, b) => (a < b ? a : b));
      const toDoor = walkPath(w, wall, cur, startDoor, globalBoxes);
      if (toDoor === null) { failed = true; break; }
      const res = solvePush(w, wall, walkable, room, rb, rg, startDoor, 400000);
      if (!res) { failed = true; break; }
      moves += toDoor + res.moves;
      for (const b of rb) globalBoxes.delete(b);
      for (const g of rg) globalBoxes.add(g);
      cur = res.endPlayer;
    }
    if (!failed) return moves;
  }
  // fallback: whole level as one push problem
  const res = solvePush(w, wall, allFloor, allFloor, boxes, goals, player, 1500000);
  return res ? res.moves : '';
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
