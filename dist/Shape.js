/* global SPE */

SPE.Shape = class {
  constructor(body) {
    this.body = body
    this.radius = SPE.Vec3.reuse().set(0.5)
    this.position = SPE.Vec3.reuse().set(0)
    this.quaternion = SPE.Quaternion.reuse().set(0)
  }
  get worldPosition() {
    if (this._worldPosition) return this._worldPosition
    return (this._worldPosition = SPE.Vec3.reuse()
      .copy(this.position)
      .applyQuaternion(this.body.quaternion)
      .add(this.body.position))
  }
  set worldPosition(x) {
    if (this._worldPosition) this._worldPosition.recycle()
    this._worldPosition = null
  }
  get worldQuaternion() {
    if (this._worldQuaternion) return this._worldQuaternion
    return (this._worldQuaternion = SPE.Quaternion.reuse()
      .copy(this.quaternion)
      .multiply(this.body.quaternion))
  }
  set worldQuaternion(x) {
    if (this._worldQuaternion) this._worldQuaternion.recycle()
    this._worldQuaternion = null
  }
  get outerRadius() {
    return Math.sqrt(this.position.lengthSq()) + Math.max(this.radius.x, this.radius.y, this.radius.z)
  }
  get volume() {
    return 1
  }
  overlaps(shape) { }
}

SPE.Sphere = class extends SPE.Shape {
  get outerRadius() {
    return Math.sqrt(this.position.lengthSq()) + this.radius.x
  }
  get volume() {
    return (4 / 3) * Math.PI * this.radius.x * this.radius.x * this.radius.x
  }
  overlaps(shape) {
    let overlap = SPE.Overlap.reuse()
    overlap.shapes[0] = this
    overlap.shapes[1] = shape
    if (shape instanceof SPE.Sphere) {
      overlap.point.copy(shape.worldPosition).sub(this.worldPosition)
      let dist = Math.sqrt(overlap.point.lengthSq())
      let overlapLen = this.radius.x + shape.radius.x - dist
      if (overlapLen < 0) {
        overlap.recycle()
        return false
      } else {
        overlap.overlap.copy(overlap.point).multiplyScalar(-overlapLen / dist)
        overlap.point.multiplyScalar((this.radius.x - overlapLen / 2) / dist).add(this.worldPosition)
        return overlap
      }
    } else {
      overlap.recycle()
      if ((overlap = shape.overlaps(this))) overlap.flip()
      return overlap
    }
  }
}
SPE.Box = class extends SPE.Shape {
  get outerRadius() {
    return Math.sqrt(this.position.lengthSq()) + Math.sqrt(this.radius.lengthSq())
  }
  get volume() {
    return this.radius.x * this.radius.y * this.radius.z * 8
  }
  overlaps(shape) {
    let overlap = SPE.Overlap.reuse()
    overlap.shapes[0] = this
    overlap.shapes[1] = shape
    if (shape instanceof SPE.Box) {
      // Box vs Box
      let proj = Vec3.reuse()
      let max = 0
      let thisPoints = [
        Vec3.reuse().set(1, 1, 1),
        Vec3.reuse().set(1, 1, -1),
        Vec3.reuse().set(1, -1, 1),
        Vec3.reuse().set(1, -1, -1),
        Vec3.reuse().set(-1, 1, 1),
        Vec3.reuse().set(-1, 1, -1),
        Vec3.reuse().set(-1, -1, 1),
        Vec3.reuse().set(-1, -1, -1)
      ]
      for (let p of thisPoints) {
        p.multiply(this.radius).applyQuaternion(this.worldQuaternion).add(this.worldPosition)
        max = Math.max(max, p.lengthSq())
        // projs.push(Vec3.reuse())
      }
      let shapePoints = [
        Vec3.reuse().set(1, 1, 1),
        Vec3.reuse().set(1, 1, -1),
        Vec3.reuse().set(1, -1, 1),
        Vec3.reuse().set(1, -1, -1),
        Vec3.reuse().set(-1, 1, 1),
        Vec3.reuse().set(-1, 1, -1),
        Vec3.reuse().set(-1, -1, 1),
        Vec3.reuse().set(-1, -1, -1)
      ]
      for (let p of shapePoints) {
        p.multiply(shape.radius).applyQuaternion(shape.worldQuaternion).add(shape.worldPosition)
        max = Math.max(max, p.lengthSq())
        // projs.push(Vec3.reuse())
      }
      let axes = [
        Vec3.reuse().set(1, 0, 0),
        Vec3.reuse().set(1, 0, 0),
        Vec3.reuse().set(0, 1, 0),
        Vec3.reuse().set(0, 1, 0),
        Vec3.reuse().set(0, 0, 1),
        Vec3.reuse().set(0, 0, 1)
      ]
      overlap.overlap.x = max * 2
      for (let i = 0; i < axes.length; i++) {
        let axis = axes[i]
        let axisshape = i % 2 ? this : shape
        axis.applyQuaternion(axisshape.worldQuaternion).multiplyScalar(max)
        let thisMin = Infinity, thisMax = 0
        for (let p of thisPoints) {
          proj.copy(p).projectOnVector(axis).add(axis)
          thisMin = Math.min(thisMin, proj.lengthSq())
          thisMax = Math.max(thisMax, proj.lengthSq())
        }
        let shapeMin = Infinity, shapeMax = 0
        for (let p of shapePoints) {
          proj.copy(p).projectOnVector(axis).add(axis)
          shapeMin = Math.min(shapeMin, proj.lengthSq())
          shapeMax = Math.max(shapeMax, proj.lengthSq())
        }
        if (thisMin > shapeMax || thisMax < shapeMin) {
          overlap.recycle()
          overlap = false
          break
        }
        thisMin = Math.sqrt(thisMin)
        thisMax = Math.sqrt(thisMax)
        shapeMin = Math.sqrt(shapeMin)
        shapeMax = Math.sqrt(shapeMax)
        overlap.point.projectOnPlane(axis)
        let overlapLen = Math.min(thisMax, shapeMax) - Math.max(thisMin, shapeMin)
        let midOverlap = (Math.min(thisMax, shapeMax) + Math.max(thisMin, shapeMin)) / 2
        proj.multiplyScalar(midOverlap / Math.sqrt(proj.lengthSq()))
        overlap.point.add(proj).sub(axis)
        if (overlapLen < Math.sqrt(overlap.overlap.lengthSq())) {
          overlap.overlap.copy(proj).multiplyScalar(overlapLen / midOverlap)
        }
      }
      proj.recycle()
      while (thisPoints.length) thisPoints.pop().recycle()
      while (shapePoints.length) shapePoints.pop().recycle()
      while (axes.length) axes.pop().recycle()
      return overlap
    } else if (shape instanceof SPE.Sphere) {
      // TODO! Box vs Sphere
      return overlap
    } else {
      overlap.recycle()
      if ((overlap = shape.overlaps(this))) overlap.flip()
      return overlap
    }
  }
}
