import { chain } from './chain';
import { character } from './character';
import { dominoes } from './dominoes';
import { lights } from './lights';
import { machines } from './machines';
import { pyramid } from './pyramid';
import { ragdoll } from './ragdoll';
import { raycast } from './raycast';
import { sensor } from './sensor';
import { shapes } from './shapes';
import { terrain } from './terrain';
import { vehicle } from './vehicle';
import { worker } from './worker';
import { carParts } from './carParts';
import type { Demo } from './types';
import { wall } from './wall';

export const demos: Demo[] = [
  pyramid,
  dominoes,
  chain,
  ragdoll,
  machines,
  raycast,
  sensor,
  shapes,
  wall,
  lights,
  terrain,
  character,
  vehicle,
  worker,
  carParts,
];
export type { Demo } from './types';
