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

