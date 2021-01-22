/* global SPE */

SPE._OverlapPool = []
SPE.Overlap = class {
  constructor() {
    this.shapes = []
    this.point = SPE.Vec3.reuse()
    this.overlap = SPE.Vec3.reuse()
  }
  static reuse() {
    return SPE._OverlapPool.pop() || new SPE.Overlap()
  }
  recycle() {
    // while (this.shapes.length) this.shapes.pop().body.collision = null
    SPE._OverlapPool.push(this)
  }
  flip() {
    this.shapes.push(this.shapes.shift())
    this.overlap.multiplyScalar(-1)
    return this
  }
}
