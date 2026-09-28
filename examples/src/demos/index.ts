import { chain } from './chain';
import { dominoes } from './dominoes';
import { lights } from './lights';
import { machines } from './machines';
import { pyramid } from './pyramid';
import { ragdoll } from './ragdoll';
import { raycast } from './raycast';
import { sensor } from './sensor';
import { shapes } from './shapes';
import { terrain } from './terrain';
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
];
export type { Demo } from './types';
