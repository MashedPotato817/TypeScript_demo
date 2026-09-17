import * as THREE from 'three';
import { forwardKinematics, initialQ, solvePoseIK } from './robot';

export interface Trajectory {
  samples: number[][];
  actualPoints: THREE.Vector3[];
  maxPositionError: number;
  maxOrientationError: number;
}

export const shapeLabels = { circle: '圆形', square: '正方形', rectangle: '矩形', heart: '心形' } as const;
export type ShapeName = keyof typeof shapeLabels;

function targetPoint(shape: ShapeName, start: THREE.Vector3, u: number): THREE.Vector3 {
  const t = Math.PI * 2 * u;
  if (shape === 'circle') return start.clone().add(new THREE.Vector3(1.45 * (Math.cos(t) - 1), 0, 1.45 * Math.sin(t)));
  if (shape === 'heart') {
    const scale = 0.115;
    const x = 16 * Math.sin(t) ** 3 * scale;
    const z = (13 * Math.cos(t) - 5 * Math.cos(2 * t) - 2 * Math.cos(3 * t) - Math.cos(4 * t) - 5) * scale;
    return start.clone().add(new THREE.Vector3(x, 0, z));
  }
  const width = shape === 'square' ? 2.6 : 3.6;
  const height = shape === 'square' ? 2.6 : 1.9;
  const perimeter = 2 * (width + height);
  let distance = u * perimeter;
  if (distance <= width) return start.clone().add(new THREE.Vector3(-distance, 0, 0));
  distance -= width;
  if (distance <= height) return start.clone().add(new THREE.Vector3(-width, 0, -distance));
  distance -= height;
  if (distance <= width) return start.clone().add(new THREE.Vector3(-width + distance, 0, -height));
  distance -= width;
  return start.clone().add(new THREE.Vector3(0, 0, -height + distance));
}

/** Creates a closed Cartesian shape and keeps the tool orientation fixed. */
export function buildShapeTrajectory(shape: ShapeName, sampleCount = 144): Trajectory {
  const startResult = forwardKinematics(initialQ);
  const start = startResult.points.at(-1)!;
  const lockedOrientation = new THREE.Quaternion().setFromRotationMatrix(startResult.transform);
  const samples: number[][] = [];
  const actualPoints: THREE.Vector3[] = [];
  let seed = [...initialQ];
  let maxPositionError = 0;
  let maxOrientationError = 0;
  for (let index = 0; index < sampleCount; index += 1) {
    const solved = solvePoseIK(targetPoint(shape, start, index / (sampleCount - 1)), lockedOrientation, seed);
    seed = solved.q;
    samples.push(seed);
    actualPoints.push(forwardKinematics(seed).points.at(-1)!);
    maxPositionError = Math.max(maxPositionError, solved.error);
    maxOrientationError = Math.max(maxOrientationError, solved.orientationError);
  }
  return { samples, actualPoints, maxPositionError, maxOrientationError };
}
