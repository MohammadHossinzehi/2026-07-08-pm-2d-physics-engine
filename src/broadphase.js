// A uniform spatial hash grid used as the broad phase: instead of testing
// every pair of bodies (O(n^2)), bodies are inserted into grid cells by their
// AABB and only bodies that share a cell are proposed as candidate pairs.
export class SpatialHashGrid {
  constructor(cellSize = 4) {
    this.cellSize = cellSize;
    this.cells = new Map();
  }

  _key(cx, cy) {
    return `${cx},${cy}`;
  }

  clear() {
    this.cells.clear();
  }

  insert(body) {
    const box = body.aabb();
    const minCX = Math.floor(box.minX / this.cellSize);
    const maxCX = Math.floor(box.maxX / this.cellSize);
    const minCY = Math.floor(box.minY / this.cellSize);
    const maxCY = Math.floor(box.maxY / this.cellSize);

    for (let cx = minCX; cx <= maxCX; cx++) {
      for (let cy = minCY; cy <= maxCY; cy++) {
        const key = this._key(cx, cy);
        let bucket = this.cells.get(key);
        if (!bucket) {
          bucket = [];
          this.cells.set(key, bucket);
        }
        bucket.push(body);
      }
    }
  }

  // Returns a de-duplicated list of [bodyA, bodyB] candidate pairs (bodyA.id < bodyB.id).
  possiblePairs() {
    const seen = new Set();
    const pairs = [];

    for (const bucket of this.cells.values()) {
      for (let i = 0; i < bucket.length; i++) {
        for (let j = i + 1; j < bucket.length; j++) {
          const x = bucket[i];
          const y = bucket[j];
          if (x === y) continue;
          const a = x.id < y.id ? x : y;
          const b = x.id < y.id ? y : x;
          const key = `${a.id}_${b.id}`;
          if (seen.has(key)) continue;
          seen.add(key);
          pairs.push([a, b]);
        }
      }
    }

    return pairs;
  }
}
