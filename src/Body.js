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
    let delta = SPE.Vec3.reuse().copy(body.position).sub(this.position)
    let dist = delta.length()
    delta.recycle()
    if (dist > this.radius + body.radius) return false

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
      let relForce = SPE.Vec3.reuse().copy(this.velocity).multiplyScalar(-1)
      let startPos = SPE.Vec3.reuse().copy(point).sub(this.position)
      let endPos = SPE.Vec3.reuse().copy(startPos).applyQuaternion(this.angularVelocity)
      relForce.add(endPos.sub(startPos).multiplyScalar(-1))

      relForce.add(body.velocity)
      startPos.copy(point).sub(body.position)
      endPos.copy(startPos).applyQuaternion(body.angularVelocity)
      relForce.add(endPos.sub(startPos))

      this.impact.force.add(nudge.copy(relForce).multiplyScalar(massRatio))
      body.impact.force.add(
        nudge.copy(relForce).multiplyScalar(-1).multiplyScalar(1 - massRatio)
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
    if (force.length() > this.radius / 1024) this.wakeUp()
    let fromPos = SPE.Vec3.reuse().copy(point).sub(this.position)
    let toPos = SPE.Vec3.reuse().copy(fromPos).add(force)
    if (fromPos.length() === 0 || toPos.length() === 0) {
      this.velocity.add(force)
    } else {
      let fromNorm = SPE.Vec3.reuse().copy(fromPos).multiplyScalar(1 / fromPos.length())
      let toNorm = SPE.Vec3.reuse().copy(toPos).multiplyScalar(1 / toPos.length())
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
