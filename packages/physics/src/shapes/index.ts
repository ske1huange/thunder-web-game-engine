export { ShapeType, Shape, MassProperties, type ShapeRayHit } from './Shape';
export {
  ConvexPolyhedron,
  computeConvexHull,
  type PolyFace,
  type PolyEdge,
} from './ConvexPolyhedron';
export { SphereShape } from './SphereShape';
export { CapsuleShape } from './CapsuleShape';
export { BoxShape } from './BoxShape';
export { CylinderShape } from './CylinderShape';
export { ConvexHullShape } from './ConvexHullShape';
export { PlaneShape, PLANE_EXTENT } from './PlaneShape';
export { MeshShape } from './MeshShape';
export { TriMeshShape, type TriMeshOptions } from './TriMeshShape';
export { HeightfieldShape, type HeightfieldOptions } from './HeightfieldShape';
export { MeshBVH } from './MeshBVH';
export {
  EDGE_AB,
  EDGE_BC,
  EDGE_CA,
  ALL_EDGES_ACTIVE,
  isEdgeActive,
  rayTriangle,
  triangleAabb,
} from './meshUtils';
