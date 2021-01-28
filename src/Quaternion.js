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

  slerp(qb, t) {
    if (t === 0) return this;
    if (t === 1) return this.copy(qb);

    const x = this.x, y = this.y, z = this.z, w = this.w;

    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/

    let cosHalfTheta = w * qb.w + x * qb.x + y * qb.y + z * qb.z;

    if (cosHalfTheta < 0) {

      this.w = - qb.w;
      this.x = - qb.x;
      this.y = - qb.y;
      this.z = - qb.z;

      cosHalfTheta = - cosHalfTheta;

    } else {

      this.copy(qb);

    }

    if (cosHalfTheta >= 1.0) {

      this.w = w;
      this.x = x;
      this.y = y;
      this.z = z;

      return this;

    }

    const sqrSinHalfTheta = 1.0 - cosHalfTheta * cosHalfTheta;

    if (sqrSinHalfTheta <= Number.EPSILON) {

      const s = 1 - t;
      this.w = s * w + t * this.w;
      this.x = s * x + t * this.x;
      this.y = s * y + t * this.y;
      this.z = s * z + t * this.z;

      this.normalize();

      return this;

    }

    const sinHalfTheta = Math.sqrt(sqrSinHalfTheta);
    const halfTheta = Math.atan2(sinHalfTheta, cosHalfTheta);
    const ratioA = Math.sin((1 - t) * halfTheta) / sinHalfTheta,
      ratioB = Math.sin(t * halfTheta) / sinHalfTheta;

    this.w = (w * ratioA + this.w * ratioB);
    this.x = (x * ratioA + this.x * ratioB);
    this.y = (y * ratioA + this.y * ratioB);
    this.z = (z * ratioA + this.z * ratioB);

    return this;

  }
}
