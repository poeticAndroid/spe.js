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
