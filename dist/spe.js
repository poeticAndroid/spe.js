/* global SPE */

const SPE = {}

setInterval(() => {
  console.table({
    _Vec3Pool: SPE._Vec3Pool.length,
    _QuaternionPool: SPE._QuaternionPool.length,
    _OverlapPool: SPE._OverlapPool.length
    // _CollisionPool: SPE._CollisionPool.length,
  })
  SPE._Vec3Pool.pop()
  SPE._QuaternionPool.pop()
  SPE._OverlapPool.pop()
  // SPE._CollisionPool.pop()
}, 16 * 1024)

/* global SPE */

SPE.Body = class {
  constructor(world) {
    this.world = world
    this.type = "dynamic"
    this.shapes = []
    this.radius = 0
    this.density = 1
    this.mass = 1
    this.collisionMask = 1
    this.impact = {
      count: 0,
      nudge: SPE.Vec3.reuse().set(0),
      point: SPE.Vec3.reuse().set(0),
      force: SPE.Vec3.reuse().set(0)
    }

    this.position = SPE.Vec3.reuse().set(0)
    this.velocity = SPE.Vec3.reuse().set(0)
    this.force = SPE.Vec3.reuse().set(0)

    this.quaternion = SPE.Quaternion.reuse().set(0)
    this.angularVelocity = SPE.Quaternion.reuse().set(0)
    this.angularForce = SPE.Quaternion.reuse().set(0)
  }
  get radius() {
    if (this._radius) return this._radius
    for (let shape of this.shapes) {
      this._radius = Math.max(this._radius, shape.outerRadius)
    }
    return this._radius
  }
  set radius(x) {
    this._radius = 0
  }
  get mass() {
    if (this._mass) return this._mass
    for (let shape of this.shapes) {
      this._mass += shape.volume
    }
    this._mass *= this.density
    return this._mass
  }
  set mass(x) {
    this._mass = 0
  }

  step() {
    if (this.type === "static") this.sleep()
    if (this.type === "dynamic") {
      let force = SPE.Vec3.reuse()
      this.velocity.multiplyScalar(1 - this.world.airFriction)
      force.copy(this.world.gravity).multiplyScalar(this.mass)
      this.velocity.add(force)
      this.velocity.add(this.force)
      this.position.add(this.velocity)
      this.angularVelocity.multiply(this.angularForce)
      this.quaternion.multiply(this.angularVelocity)
      force.recycle()
    }
    if (this.type === "kinematic") {
      this._lastPosition = this._lastPosition || SPE.Vec3.reuse().copy(this.position)
      this._lastQuaternion = this._lastQuaternion || SPE.Quaternion.reuse().copy(this.quaternion)
      this.velocity.copy(this.position).sub(this._lastPosition)
      this.angularVelocity.copy(this.quaternion).multiply(this._lastQuaternion.conjugate())
      this._lastPosition.copy(this.position)
      this._lastQuaternion.copy(this.quaternion)
    }

    for (let shape of this.shapes) {
      shape.worldPosition = null
      shape.worldQuaternion = null
    }
  }

  createShape(config) {
    let shape
    switch (config.type) {
      case "sphere":
        shape = new SPE.Sphere(this)
        break
      case "box":
        shape = new SPE.Box(this)
        break
    }
    for (let prop in config) {
      if (typeof shape[prop] == "object" && shape[prop].copy) {
        shape[prop].copy(config[prop])
      } else {
        shape[prop] = config[prop]
      }
    }
    this.shapes.push(shape)
    this.radius = null
    return shape
  }
  removeShape(shape) {
    let i = this.shapes.indexOf(shape)
    if (i == this.shapes.length - 1) this.shapes.pop()
    else if (i >= 0) this.shapes[i] = this.shapes.pop()
    this.radius = null
  }

  collideWith(body) {
    if (this.type + body.type === "staticstatic") return false
    let delta = SPE.Vec3.reuse()
      .copy(body.position)
      .sub(this.position)
    let distSq = delta.lengthSq()
    delta.recycle()
    if (distSq > (this.radius + body.radius) * (this.radius + body.radius)) return false

    let massRatio = body.mass / (this.mass + body.mass)
    if (this.type === "static") massRatio = 0
    if (body.type === "static") massRatio = 1
    let count = 0
    let point = SPE.Vec3.reuse().set(0)
    let nudge = SPE.Vec3.reuse()

    for (let shapeA of this.shapes) {
      for (let shapeB of body.shapes) {
        let overlap = shapeA.overlaps(shapeB)
        if (overlap) {
          count++
          point.add(overlap.point)
          this.impact.count++
          this.impact.point.add(overlap.point)
          this.impact.nudge.add(nudge.copy(overlap.overlap).multiplyScalar(massRatio))
          overlap.flip()
          massRatio = 1 - massRatio
          body.impact.count++
          body.impact.point.add(overlap.point)
          body.impact.nudge.add(nudge.copy(overlap.overlap).multiplyScalar(massRatio))
          overlap.recycle()
          massRatio = 1 - massRatio
        }
      }
    }

    if (count) {
      point.multiplyScalar(1 / count)
      let relForce = SPE.Vec3.reuse()
        .copy(this.velocity)
        .multiplyScalar(-1)
      let startPos = SPE.Vec3.reuse()
        .copy(point)
        .sub(this.position)
      let endPos = SPE.Vec3.reuse()
        .copy(startPos)
        .applyQuaternion(this.angularVelocity)
      relForce.add(endPos.sub(startPos).multiplyScalar(-1))

      relForce.add(body.velocity)
      startPos.copy(point).sub(body.position)
      endPos.copy(startPos).applyQuaternion(body.angularVelocity)
      relForce.add(endPos.sub(startPos))

      this.impact.force.add(nudge.copy(relForce).multiplyScalar(massRatio))
      body.impact.force.add(
        nudge
          .copy(relForce)
          .multiplyScalar(-1)
          .multiplyScalar(1 - massRatio)
      )

      relForce.recycle()
      startPos.recycle()
      endPos.recycle()
    }

    nudge.recycle()
    point.recycle()
    return count
  }

  applyImpact() {
    if (this.type !== "static") {
      this.position.add(this.impact.nudge)
      this.applyImpulse(this.impact.point.multiplyScalar(1 / this.impact.count), this.impact.force)
    }

    this.impact.count = 0
    this.impact.nudge.set(0)
    this.impact.point.set(0)
    this.impact.force.set(0)
  }

  applyImpulse(point, force) {
    if (this.type === "static") return this.sleep()
    if (force.lengthSq() > this.radius / 1024) this.wakeUp()
    let fromPos = SPE.Vec3.reuse()
      .copy(point)
      .sub(this.position)
    let toPos = SPE.Vec3.reuse()
      .copy(fromPos)
      .add(force)
    if (fromPos.lengthSq() === 0 || toPos.lengthSq() === 0) {
      this.velocity.add(force)
    } else {
      let fromNorm = SPE.Vec3.reuse()
        .copy(fromPos)
        .multiplyScalar(1 / Math.sqrt(fromPos.lengthSq()))
      let toNorm = SPE.Vec3.reuse()
        .copy(toPos)
        .multiplyScalar(1 / Math.sqrt(toPos.lengthSq()))
      let angularForce = SPE.Quaternion.reuse().setFromUnitVectors(fromNorm, toNorm)

      this.angularVelocity.multiply(angularForce)
      fromPos.applyQuaternion(angularForce)
      this.velocity.add(toPos.sub(fromPos))

      fromNorm.recycle()
      toNorm.recycle()
      angularForce.recycle()
    }
    fromPos.recycle()
    toPos.recycle()
  }

  sleep() {
    if (!this.sleeping) this.world.putBodyToSleep(this)
  }
  wakeUp() {
    if (this.sleeping) this.world.wakeBodyUp(this)
  }
  remove() {
    this.world.removeBodyById(this.id)
    this.world = null
  }
}
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
    SPE._OverlapPool.push(this)
  }
  flip() {
    this.shapes.push(this.shapes.shift())
    this.overlap.multiplyScalar(-1)
    return this
  }
}
/* global SPE */

SPE._QuaternionPool = []
SPE.Quaternion = class {
  constructor() {
    this.x = 0
    this.y = 0
    this.z = 0
    this.w = 1
  }
  static reuse() {
    return SPE._QuaternionPool.pop() || new SPE.Quaternion()
  }
  recycle() {
    SPE._QuaternionPool.push(this)
  }
  set(x = 0, y = x, z = x, w = 1) {
    this.x = x
    this.y = y
    this.z = z
    this.w = w
    return this
  }
  copy(quat) {
    this.x = quat.x
    this.y = quat.y
    this.z = quat.z
    this.w = quat.w
    return this
  }

  setFromUnitVectors(vFrom, vTo) {
    // assumes direction vectors vFrom and vTo are normalized
    const EPS = 0.000001

    let r = vFrom.dot(vTo) + 1

    if (r < EPS) {
      r = 0

      if (Math.abs(vFrom.x) > Math.abs(vFrom.z)) {
        this.x = -vFrom.y
        this.y = vFrom.x
        this.z = 0
        this.w = r
      } else {
        this.x = 0
        this.y = -vFrom.z
        this.z = vFrom.y
        this.w = r
      }
    } else {
      // crossVectors( vFrom, vTo ); // inlined to avoid cyclic dependency on Vector3

      this.x = vFrom.y * vTo.z - vFrom.z * vTo.y
      this.y = vFrom.z * vTo.x - vFrom.x * vTo.z
      this.z = vFrom.x * vTo.y - vFrom.y * vTo.x
      this.w = r
    }

    return this.normalize()
  }

  conjugate() {
    this.x *= -1
    this.y *= -1
    this.z *= -1
    return this
  }

  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w)
  }

  normalize() {
    let l = this.length()

    if (l === 0) {
      this.x = 0
      this.y = 0
      this.z = 0
      this.w = 1
    } else {
      l = 1 / l
      this.x = this.x * l
      this.y = this.y * l
      this.z = this.z * l
      this.w = this.w * l
    }
    return this
  }

  multiply(q) {
    return this.multiplyQuaternions(this, q)
  }

  premultiply(q) {
    return this.multiplyQuaternions(q, this)
  }

  multiplyQuaternions(a, b) {
    // from http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/code/index.htm

    const qax = a.x,
      qay = a.y,
      qaz = a.z,
      qaw = a.w
    const qbx = b.x,
      qby = b.y,
      qbz = b.z,
      qbw = b.w

    this.x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby
    this.y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz
    this.z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx
    this.w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz

    return this
  }
}
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
  sat(axes, thisPoints, shapePoints, overlap) {
    let proj = SPE.Vec3.reuse()
    for (let i = 0; i < axes.length; i++) {
      let axis = axes[i]
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
        overlap.overlap.copy(proj).multiplyScalar(-overlapLen / midOverlap)
      }
    }
    proj.recycle()
    return overlap
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
        axis.applyQuaternion(axisshape.worldQuaternion).multiplyScalar(max)
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
        axis.applyQuaternion(this.worldQuaternion).multiplyScalar(max)
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
/* global SPE */

SPE._Vec3Pool = []
SPE.Vec3 = class {
  constructor() {
    this.x = 0
    this.y = 0
    this.z = 0
  }
  static reuse() {
    return SPE._Vec3Pool.pop() || new SPE.Vec3()
  }
  recycle() {
    SPE._Vec3Pool.push(this)
  }
  set(x = 0, y = x, z = x) {
    this.x = x
    this.y = y
    this.z = z
    return this
  }
  copy(vec) {
    this.x = vec.x || 0
    this.y = vec.y || 0
    this.z = vec.z || 0
    return this
  }

  lengthSq() {
    return this.x * this.x + this.y * this.y + this.z * this.z
  }

  dot(vec) {
    return this.x * vec.x + this.y * vec.y + this.z * vec.z
  }

  add(vec) {
    this.x += vec.x
    this.y += vec.y
    this.z += vec.z
    return this
  }
  sub(vec) {
    this.x -= vec.x
    this.y -= vec.y
    this.z -= vec.z
    return this
  }
  multiply(vec) {
    this.x *= vec.x
    this.y *= vec.y
    this.z *= vec.z
    return this
  }
  multiplyScalar(n) {
    this.x *= n
    this.y *= n
    this.z *= n
    return this
  }

  applyQuaternion(q) {
    const x = this.x,
      y = this.y,
      z = this.z
    const qx = q.x,
      qy = q.y,
      qz = q.z,
      qw = q.w

    // calculate quat * vector
    const ix = qw * x + qy * z - qz * y
    const iy = qw * y + qz * x - qx * z
    const iz = qw * z + qx * y - qy * x
    const iw = -qx * x - qy * y - qz * z

    // calculate result * inverse quat
    this.x = ix * qw + iw * -qx + iy * -qz - iz * -qy
    this.y = iy * qw + iw * -qy + iz * -qx - ix * -qz
    this.z = iz * qw + iw * -qz + ix * -qy - iy * -qx

    return this
  }


  projectOnVector(vec) {
    const denominator = vec.lengthSq()
    if (denominator === 0) return this.set(0, 0, 0)
    const scalar = vec.dot(this) / denominator
    return this.copy(vec).multiplyScalar(scalar)
  }

  projectOnPlane(planeNormal) {
    _vector.copy(this).projectOnVector(planeNormal)
    return this.sub(_vector)
  }
}
let _vector = SPE.Vec3.reuse()
/* global SPE */

SPE.World = class {
  constructor() {
    this.bodies = []
    this.awakeBodies = []
    this.impactedBodies = []
    this.gravity = SPE.Vec3.reuse().set(0)
    this.airFriction = 0
  }
  step() {
    for (let body of this.awakeBodies) {
      body.step()
    }
    for (let bodyA of this.bodies) {
      if (!bodyA) continue
      for (let bodyB of this.awakeBodies) {
        if (!bodyB) continue
        if (bodyA.id >= bodyB.id) continue
        if (!(bodyA.collisionMask & bodyB.collisionMask)) continue
        bodyA.collideWith(bodyB)
      }
      if (bodyA.impact.count) this.impactedBodies.push(bodyA)
    }
    while (this.impactedBodies.length) this.impactedBodies.pop().applyImpact()
  }
  createBody(config) {
    if (config.id == undefined) config.id = this.bodies.indexOf(null)
    if (config.id < 0) config.id = this.bodies.length
    let body = new SPE.Body(this)
    for (let prop in config) {
      if (typeof body[prop] == "object" && body[prop].copy) {
        body[prop].copy(config[prop])
      } else {
        body[prop] = config[prop]
      }
    }
    this.bodies[body.id] = body
    this.awakeBodies.push(body)
    return body
  }
  removeBodyById(id) {
    let body = this.bodies[id]
    if (body) {
      this.putToSleep(body)
      this.bodies[id] = null
    }
  }
  putBodyToSleep(body) {
    setTimeout(() => {
      body.velocity.set(0)
      body.angularVelocity.set(0)
      let i = this.awakeBodies.indexOf(body)
      if (i == this.awakeBodies.length - 1) this.awakeBodies.pop()
      else if (i >= 0) this.awakeBodies[i] = this.awakeBodies.pop()
      body.sleeping = true
    })
    body.sleeping = true
  }
  wakeBodyUp(body) {
    setTimeout(() => {
      let i = this.awakeBodies.indexOf(body)
      if (i < 0) this.awakeBodies.push(body)
      body.sleeping = false
    })
    body.sleeping = false
  }
  clear() {
    this.bodies = []
    this.awakeBodies = []
    this.impactedBodies = []
  }
}
