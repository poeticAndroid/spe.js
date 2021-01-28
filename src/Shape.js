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
    return (this._worldPosition = SPE.Vec3.reuse().copy(this.position).applyQuaternion(this.body.quaternion).add(this.body.position))
  }
  set worldPosition(x) {
    if (this._worldPosition) this._worldPosition.recycle()
    this._worldPosition = null
  }
  get worldQuaternion() {
    if (this._worldQuaternion) return this._worldQuaternion
    return (this._worldQuaternion = SPE.Quaternion.reuse().copy(this.quaternion).multiply(this.body.quaternion))
  }
  set worldQuaternion(x) {
    if (this._worldQuaternion) this._worldQuaternion.recycle()
    this._worldQuaternion = null
  }
  get outerRadius() {
    return this.position.length() + Math.max(this.radius.x, this.radius.y, this.radius.z)
  }
  get volume() {
    return 1
  }
  sat(axes, thisPoints, shapePoints, overlap) {
    let proj = SPE.Vec3.reuse()
    let midbb = SPE.Vec3.reuse()
    // overlap.point.set(0)
    // for (let p of thisPoints) {
    //   overlap.point.add(p)
    // }
    // for (let p of shapePoints) {
    //   overlap.point.add(p)
    // }
    // overlap.point.multiplyScalar(1 / (thisPoints.length + shapePoints.length))

    if (overlap) {
      let thisMin = Infinity, thisMax = -Infinity
      for (let p of thisPoints) {
        thisMin = Math.min(thisMin, p.x)
        thisMax = Math.max(thisMax, p.x)
      }
      let shapeMin = Infinity, shapeMax = -Infinity
      for (let p of shapePoints) {
        shapeMin = Math.min(shapeMin, p.x)
        shapeMax = Math.max(shapeMax, p.x)
      }
      midbb.x = (Math.min(thisMax, shapeMax) + Math.max(thisMin, shapeMin)) / 2

      thisMin = Infinity, thisMax = -Infinity
      for (let p of thisPoints) {
        thisMin = Math.min(thisMin, p.y)
        thisMax = Math.max(thisMax, p.y)
      }
      shapeMin = Infinity, shapeMax = -Infinity
      for (let p of shapePoints) {
        shapeMin = Math.min(shapeMin, p.y)
        shapeMax = Math.max(shapeMax, p.y)
      }
      midbb.y = (Math.min(thisMax, shapeMax) + Math.max(thisMin, shapeMin)) / 2

      thisMin = Infinity, thisMax = -Infinity
      for (let p of thisPoints) {
        thisMin = Math.min(thisMin, p.z)
        thisMax = Math.max(thisMax, p.z)
      }
      shapeMin = Infinity, shapeMax = -Infinity
      for (let p of shapePoints) {
        shapeMin = Math.min(shapeMin, p.z)
        shapeMax = Math.max(shapeMax, p.z)
      }
      midbb.z = (Math.min(thisMax, shapeMax) + Math.max(thisMin, shapeMin)) / 2
    }

    for (let i = 0; i < axes.length; i++) {
      let axis = axes[i]
      let thisMin = Infinity, thisMax = -Infinity
      let thisMinP, thisMaxP
      for (let p of thisPoints) {
        let val = axis.dot(p)
        if (thisMin > val) {
          thisMin = val
          thisMinP = p
        }
        if (thisMax < val) {
          thisMax = val
          thisMaxP = p
        }
      }
      let shapeMin = Infinity, shapeMax = -Infinity
      let shapeMinP, shapeMaxP
      for (let p of shapePoints) {
        let val = axis.dot(p)
        if (shapeMin > val) {
          shapeMin = val
          shapeMinP = p
        }
        if (shapeMax < val) {
          shapeMax = val
          shapeMaxP = p
        }
      }
      if (thisMin > shapeMax || thisMax < shapeMin) {
        overlap.recycle()
        overlap = false
        break
      }
      // overlap.point.projectOnPlane(axis)
      let overlapLen //= Math.min(thisMax, shapeMax) - Math.max(thisMin, shapeMin)
      if (thisMax - shapeMin < shapeMax - thisMin) {
        overlapLen = shapeMin - thisMax
      } else {
        overlapLen = shapeMax - thisMin
      }
      // let midOverlap = (Math.min(thisMax, shapeMax) + Math.max(thisMin, shapeMin)) / 2
      // proj.multiplyScalar(midOverlap / proj.length())
      // overlap.point.add(proj)//.sub(axis)
      if (Math.abs(overlapLen) < overlap.overlap.length()) {
        overlap.overlap.copy(axis).multiplyScalar(overlapLen)
        let dist = proj.copy(midbb).sub(thisMinP).length()
        let closest = dist
        overlap.point.copy(thisMinP)
        dist = proj.copy(midbb).sub(thisMaxP).length()
        if (dist < closest) {
          overlap.point.copy(thisMaxP)
          closest = dist
        }
        dist = proj.copy(midbb).sub(shapeMinP).length()
        if (dist < closest) {
          overlap.point.copy(shapeMinP)
          closest = dist
        }
        dist = proj.copy(midbb).sub(shapeMaxP).length()
        if (dist < closest) {
          overlap.point.copy(shapeMaxP)
          closest = dist
        }
      }
    }

    proj.recycle()
    midbb.recycle()
    return overlap
  }
  overlaps(shape) { }
}

SPE.Sphere = class extends SPE.Shape {
  get outerRadius() {
    return this.position.length() + this.radius.x
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
      let dist = overlap.point.length()
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
    return this.position.length() + this.radius.length()
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
      let max = 0
      let thisPoints = [
        SPE.Vec3.reuse().set(1, 1, 1),
        SPE.Vec3.reuse().set(1, 1, -1),
        SPE.Vec3.reuse().set(1, -1, 1),
        SPE.Vec3.reuse().set(1, -1, -1),
        SPE.Vec3.reuse().set(-1, 1, 1),
        SPE.Vec3.reuse().set(-1, 1, -1),
        SPE.Vec3.reuse().set(-1, -1, 1),
        SPE.Vec3.reuse().set(-1, -1, -1)
      ]
      for (let p of thisPoints) {
        p.multiply(this.radius).applyQuaternion(this.worldQuaternion).add(this.worldPosition)
        max = Math.max(max, p.lengthSq())
      }
      let shapePoints = [
        SPE.Vec3.reuse().set(1, 1, 1),
        SPE.Vec3.reuse().set(1, 1, -1),
        SPE.Vec3.reuse().set(1, -1, 1),
        SPE.Vec3.reuse().set(1, -1, -1),
        SPE.Vec3.reuse().set(-1, 1, 1),
        SPE.Vec3.reuse().set(-1, 1, -1),
        SPE.Vec3.reuse().set(-1, -1, 1),
        SPE.Vec3.reuse().set(-1, -1, -1)
      ]
      for (let p of shapePoints) {
        p.multiply(shape.radius).applyQuaternion(shape.worldQuaternion).add(shape.worldPosition)
        max = Math.max(max, p.lengthSq())
      }
      let axes = [
        SPE.Vec3.reuse().set(1, 0, 0),
        SPE.Vec3.reuse().set(1, 0, 0),
        SPE.Vec3.reuse().set(0, 1, 0),
        SPE.Vec3.reuse().set(0, 1, 0),
        SPE.Vec3.reuse().set(0, 0, 1),
        SPE.Vec3.reuse().set(0, 0, 1)
      ]
      overlap.overlap.x = max * 2
      for (let i = 0; i < axes.length; i++) {
        let axis = axes[i]
        let axisshape = i % 2 ? this : shape
        axis.applyQuaternion(axisshape.worldQuaternion)//.multiplyScalar(max)
      }
      overlap = this.sat(axes, thisPoints, shapePoints, overlap)
      while (thisPoints.length) thisPoints.pop().recycle()
      while (shapePoints.length) shapePoints.pop().recycle()
      while (axes.length) axes.pop().recycle()
      return overlap
    } else if (shape instanceof SPE.Sphere) {
      // Box vs Sphere
      let max = 0
      let thisPoints = [
        SPE.Vec3.reuse().set(1, 1, 1),
        SPE.Vec3.reuse().set(1, 1, -1),
        SPE.Vec3.reuse().set(1, -1, 1),
        SPE.Vec3.reuse().set(1, -1, -1),
        SPE.Vec3.reuse().set(-1, 1, 1),
        SPE.Vec3.reuse().set(-1, 1, -1),
        SPE.Vec3.reuse().set(-1, -1, 1),
        SPE.Vec3.reuse().set(-1, -1, -1)
      ]
      for (let p of thisPoints) {
        p.multiply(this.radius).applyQuaternion(this.worldQuaternion).add(this.worldPosition)
        max = Math.max(max, p.lengthSq())
      }
      let shapePoints = [
        SPE.Vec3.reuse().set(1, 0, 0),
        SPE.Vec3.reuse().set(-1, 0, 0),
        SPE.Vec3.reuse().set(0, 1, 0),
        SPE.Vec3.reuse().set(0, -1, 0),
        SPE.Vec3.reuse().set(0, 0, 1),
        SPE.Vec3.reuse().set(0, 0, -1)
      ]
      for (let p of shapePoints) {
        p.multiplyScalar(shape.radius.x).applyQuaternion(this.worldQuaternion).add(shape.worldPosition)
        max = Math.max(max, p.lengthSq())
      }
      let axes = [
        SPE.Vec3.reuse().set(1, 0, 0),
        SPE.Vec3.reuse().set(0, 1, 0),
        SPE.Vec3.reuse().set(0, 0, 1)
      ]
      overlap.overlap.x = max * 2
      for (let i = 0; i < axes.length; i++) {
        let axis = axes[i]
        axis.applyQuaternion(this.worldQuaternion)//.multiplyScalar(max)
      }
      overlap = this.sat(axes, thisPoints, shapePoints, overlap)
      while (thisPoints.length) thisPoints.pop().recycle()
      while (shapePoints.length) shapePoints.pop().recycle()
      while (axes.length) axes.pop().recycle()
      return overlap
    } else {
      overlap.recycle()
      if ((overlap = shape.overlaps(this))) overlap.flip()
      return overlap
    }
  }
}
