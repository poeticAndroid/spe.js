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
  overlaps(shape) {}
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
      let overlap = this.radius.x + shape.radius.x - dist
      if (overlap < 0) {
        overlap.recycle()
        return false
      } else {
        overlap.overlap.copy(overlap.point).multiplyScalar(-overlap / dist)
        overlap.point.multiplyScalar((this.radius.x - overlap / 2) / dist).add(this.worldPosition)
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
      // TODO! Box vs Box
    } else if (shape instanceof SPE.Sphere) {
      // TODO! Box vs Sphere
    } else {
      overlap.recycle()
      if ((overlap = shape.overlaps(this))) overlap.flip()
      return overlap
    }
  }
}
