export {
  shapeFromDesc,
  createBodyFromDesc,
  jointFromDesc,
  type ShapeDesc,
  type ColliderDesc,
  type BodyDesc,
  type JointDesc,
  type Vec3Tuple,
  type QuatTuple,
} from './descriptors';
export {
  SLOT_STRIDE,
  type MessagePortLike,
  type WorldDesc,
  type ToWorker,
  type FromWorker,
  type EventRecord,
  type BodyOp,
} from './protocol';
export {
  PhysicsWorkerHost,
  runPhysicsWorker,
  type PhysicsWorkerOptions,
  type CustomHandler,
} from './host';
export {
  WorkerWorld,
  BodyProxy,
  type WorkerWorldEvents,
  type WorkerCollisionEvent,
  type WorkerSensorEvent,
  type WorkerRaycastHit,
} from './WorkerWorld';
