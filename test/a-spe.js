/* global AFRAME, THREE, SPE */

; (function () {

  AFRAME.registerSystem("physics", {
    schema: {
      framerate: { type: "number", default: 60 },
      gravity: { type: "vec3", default: { x: 0, y: -0.01, z: 0 } },
      airFriction: { type: "number", default: 0.01 },
      debug: { type: "boolean", default: false }
    },

    init: function () {
      this.el.physicsWorld = new SPE.World()
      this._nextTick = 0
    },

    update: function () {
      this.frameInterval = 1000 / this.data.framerate
      this.el.physicsWorld.gravity.copy(this.data.gravity)
      this.el.physicsWorld.airFriction = this.data.airFriction
      this.el.physicsWorld.debug = this.data.debug
    },

    remove: function () {
      this.el.physicsWorld.clear()
      this.el.physicsWorld = null
    },

    tick: function (time, timeDelta) {
      if (time > this._nextTick) {
        this.el.physicsWorld.step()
        this._nextTick += this.frameInterval
        if (time > this._nextTick) this._nextTick = time
      }
    }
  })

  AFRAME.registerComponent("body", {
    dependencies: ["position", "rotation", "scale"],

    schema: {
      type: { type: "string", default: "dynamic" },
      density: { type: "number", default: 1 },
      collisionMask: { type: "number", default: 1 },
      debug: { type: "boolean", default: false },
    },

    init: function () {
      if (!this.el.sceneEl.physicsWorld) return setTimeout(() => this.init(), 256)
      if (!this._initwait) {
        this._initwait = true
        return setTimeout(() => this.init(), 256)
      }
      let wpos = this.el.object3D.getWorldPosition(THREE.Vector3.reuse())
      let wquat = this.el.object3D.getWorldQuaternion(THREE.Quaternion.reuse())
      this.el.body = this.el.sceneEl.physicsWorld.createBody({
        position: wpos,
        quaternion: wquat
      })
      this.update()

      let shapes = this.el.querySelectorAll("a-box, a-sphere, a-cylinder")
      for (let shape of shapes) {
        if (shape.tagName.toLowerCase() === "a-box") {
          this.el.body.createShape({
            type: "box",
            position: shape.object3D.position,
            quaternion: shape.object3D.quaternion,
            radius: {
              x: parseFloat(shape.getAttribute("width") || 1) / 2,
              y: parseFloat(shape.getAttribute("height") || 1) / 2,
              z: parseFloat(shape.getAttribute("depth") || 1) / 2,
            }
          })
        }
        if (shape.tagName.toLowerCase() === "a-sphere") {
          this.el.body.createShape({
            type: "sphere",
            position: shape.object3D.position,
            quaternion: shape.object3D.quaternion,
            radius: {
              x: parseFloat(shape.getAttribute("radius") || 1),
              y: parseFloat(shape.getAttribute("radius") || 1),
              z: parseFloat(shape.getAttribute("radius") || 1),
            }
          })
        }
      }

      wpos.recycle()
      wquat.recycle()
    },

    update: function () {
      if (!this.el.body) return setTimeout(() => this.update(), 256)
      this.el.body.type = this.data.type
      this.el.body.density = this.data.density
      this.el.body.collisionMask = this.data.collisionMask
      this.el.body.debug = this.data.debug
    },

    play: function () {
      if (!this.el.body) return
      this.el.body.awake()
    },
    pause: function () {
      if (!this.el.body) return setTimeout(() => this.pause(), 256)
      this.el.body.sleep()
    },

    remove: function () {
      if (this.el.body) this.el.body.remove()
      this.el.body = null
    },

    tick: function () {
      if (!this.el.body) return

      if (this.el.body.type === "kinematic") {
        console.log("iz kinematix!")
        let pos = THREE.Vector3.reuse()
        let quat = THREE.Quaternion.reuse()

        this.el.object3D.getWorldPosition(pos)
        this.el.body.position.copy(pos)
        this.el.object3D.getWorldQuaternion(quat)
        this.el.body.quaternion.copy(quat)
        this.el.body.awake()

        pos.recycle()
        quat.recycle()
      } else {
        let quat = THREE.Quaternion.reuse()

        this.el.object3D.position.copy(this.el.body.position)
        this.el.object3D.parent.worldToLocal(this.el.object3D.position)

        this.el.object3D.getWorldQuaternion(quat)
        this.el.object3D.quaternion.multiply(quat.conjugate().normalize())
        quat.copy(this.el.body.quaternion)
        this.el.object3D.quaternion.multiply(quat.normalize())

        quat.recycle()
      }
    }
  })

  AFRAME.registerComponent("joint", {
    dependencies: ["body", "shape"],

    schema: {
      type: { type: "string", default: "prisme" },
      with: { type: "selector", default: "[body]" },
      min: { type: "number", default: 0 },
      max: { type: "number", default: 0 },
      pos1: { type: "vec3", default: { x: 0, y: 0, z: 0 } },
      pos2: { type: "vec3", default: { x: 0, y: 0, z: 0 } },
      axe1: { type: "vec3", default: { x: 1, y: 0, z: 0 } },
      axe2: { type: "vec3", default: { x: 1, y: 0, z: 0 } },
      collision: { type: "boolean", default: false },
      limit: { type: "array" },
      motor: { type: "array" },
      spring: { type: "array" },
    },

    update: function () {
      if (!this.el.body) return setTimeout(() => this.update(), 256)
      if (!this.data.with.body) return setTimeout(() => this.update(), 256)
      if (!this.joint) {
        let jc = new OIMO.JointConfig()
        jc.body1 = this.el.body
        jc.body2 = this.data.with.body
        let deg2rad = Math.PI / 180
        switch (this.data.type) {
          case "distance":
            this.joint = new OIMO.DistanceJoint(jc, this.data.min, this.data.max)
            break
          case "hinge":
            this.joint = new OIMO.HingeJoint(jc, this.data.min * deg2rad, this.data.max * deg2rad)
            break
          case "prisme":
            this.joint = new OIMO.PrismaticJoint(jc, this.data.min * deg2rad, this.data.max * deg2rad)
            break
          case "slide":
            this.joint = new OIMO.SliderJoint(jc, this.data.min, this.data.max)
            break
          case "ball":
            this.joint = new OIMO.BallAndSocketJoint(jc)
            break
          case "wheel":
            this.joint = new OIMO.WheelJoint(jc)
            break
        }
        this.el.sceneEl.physicsWorld.addJoint(this.joint)
      }
      this.joint.localAnchorPoint1.copy(this.data.pos1)
      this.joint.localAnchorPoint2.copy(this.data.pos2)
      if (this.joint.localAxis1) {
        this.joint.localAxis1.copy(this.data.axe1)
        this.joint.localAxis2.copy(this.data.axe2)
      }
      this.joint.allowCollision = this.data.collision

      let lm = this.joint.rotationalLimitMotor1 || this.joint.limitMotor
      // if (this.data.limit.length == 2)
      lm.setLimit(parseFloat(this.data.limit[0]) || 0, parseFloat(this.data.limit[1]) || 0)
      // if (this.data.motor.length == 2)
      lm.setMotor(parseFloat(this.data.motor[0]) || 0, parseFloat(this.data.motor[1]) || 0)
      // if (this.data.spring.length == 2)
      lm.setSpring(parseFloat(this.data.spring[0]) || 0, parseFloat(this.data.spring[1]) || 0)
    },

    remove: function () {
      if (this.joint) {
        this.joint.body1.awake()
        this.joint.body2.awake()
        this.joint.remove()
      }
      this.joint = null
    },

  })

}.call(this))
