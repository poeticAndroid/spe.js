/* global SPE */

SPE.World = class {
  constructor() {
    this.bodies = []
    this.awakeBodies = []
    this.impactedBodies = []
    this.gravity = SPE.Vec3.reuse().set(0)
    this.airFriction = 0
    this.collisions = []
  }
  step() {
    for (let body of this.awakeBodies) {
      body.step()
    }
    for (let bodyA of this.bodies) {
      if (!bodyA) continue
      for (let bodyB of this.awakeBodies) {
        if (!bodyB) continue
        if (bodyA.id <= bodyB.id) continue
        if (!(bodyA.collisionMask & bodyB.collisionMask)) continue
        bodyA.collideWith(bodyB)
      }
      if (bodyA.impact.count) this.impactedBodies.push(bodyA)
    }
    while (this.impactedBodies.length) this.impactedBodies.pop().applyImpact()
    while (this.collisions.length) this.collisions.pop().recycle()
  }
  addBody(config) {
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
  removeBody(id) {
    let body = this.bodies[id]
    if (body) {
      this.putToSleep(body)
      this.bodies[id] = null
    }
  }
  putToSleep(body) {
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
  wakeUp(body) {
    setTimeout(() => {
      let i = this.awakeBodies.indexOf(body)
      if (i < 0) this.awakeBodies.push(body)
      body.sleeping = false
    })
    body.sleeping = false
  }
}
