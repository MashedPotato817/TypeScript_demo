import * as THREE from 'three';

export type JointKind = 'revolute' | 'prismatic';

export interface JointDefinition {
  name: string;
  kind: JointKind;
  d: number;
  a: number;
  alpha: number;
  min: number;
  max: number;
}

export interface KinematicsResult {
  transform: THREE.Matrix4;
  transforms: THREE.Matrix4[];
  points: THREE.Vector3[];
}

const deg = (value: number) => THREE.MathUtils.degToRad(value);

/** 标准 DH: Rot(z, θ) · Trans(z, d) · Trans(x, a) · Rot(x, α). */
export const joints: JointDefinition[] = [
  { name: 'J1', kind: 'revolute', d: 5, a: 0, alpha: -Math.PI / 2, min: deg(-170), max: deg(170) },
  { name: 'J2', kind: 'revolute', d: 2, a: 0, alpha: Math.PI / 2, min: deg(-135), max: deg(135) },
  { name: 'J3', kind: 'prismatic', d: 0, a: 0, alpha: 0, min: 3, max: 10 },
  { name: 'J4', kind: 'revolute', d: 0, a: 3, alpha: -Math.PI / 2, min: deg(-180), max: deg(180) },
  { name: 'J5', kind: 'revolute', d: 0, a: 2, alpha: Math.PI / 2, min: deg(-90), max: deg(90) },
  { name: 'J6', kind: 'revolute', d: 4, a: 0, alpha: 0, min: deg(-180), max: deg(180) },
  { name: 'J7', kind: 'revolute', d: 0, a: 3, alpha: -Math.PI / 2, min: deg(-135), max: deg(135) },
  { name: 'J8', kind: 'revolute', d: 3, a: 0, alpha: Math.PI / 2, min: deg(-90), max: deg(90) },
];

export const initialQ = [0, Math.PI / 6, 5, -Math.PI / 4, Math.PI / 6, 0, Math.PI / 6, 0];

export interface CoursePreset {
  id: string;
  name: string;
  tag: string;
  description: string;
  q: number[];
  theoreticalPos: [number, number, number];
  theoreticalMatrix: number[][];
}

export const COURSE_PRESETS: CoursePreset[] = [
  {
    id: 'init',
    name: '初始位姿 (init)',
    tag: '基础位形',
    description: '实验一、二、三的基准构型，第3关节滑动位移 5.0，其余转角为 [0°, 30°, 5, -45°, 30°, 0°, 30°, 0°]',
    q: [0, Math.PI / 6, 5, -Math.PI / 4, Math.PI / 6, 0, Math.PI / 6, 0],
    theoreticalPos: [10.672, -0.535, 6.823],
    theoreticalMatrix: [
      [0.549, 0.390, 0.739, 10.672],
      [-0.177, 0.919, -0.354, -0.535],
      [-0.817, 0.063, 0.573, 6.823],
      [0, 0, 0, 1]
    ]
  },
  {
    id: 'aid0',
    name: '实验二 目标 1 (aid0)',
    tag: '目标 1',
    description: '关节变量 [0°, 45°, 5, 0°, -90°, 0°, 30°, 0°]，用于验证齐次变换矩阵 T0',
    q: [0, Math.PI / 4, 5, 0, -Math.PI / 2, 0, Math.PI / 6, 0],
    theoreticalPos: [5.019, 6.098, 11.433],
    theoreticalMatrix: [
      [0.6124, -0.3536, -0.7071, 5.019],
      [0.5, 0.866, 0.0, 6.098],
      [0.6124, -0.3536, 0.7071, 11.433],
      [0, 0, 0, 1]
    ]
  },
  {
    id: 'aid1',
    name: '实验二 目标 2 (aid1)',
    tag: '目标 2',
    description: '关节变量 [-90°, 45°, 6, 0°, 90°, 0°, -30°, 0°]，移动关节伸长至 6.0',
    q: [-Math.PI / 2, Math.PI / 4, 6, 0, Math.PI / 2, 0, -Math.PI / 6, 0],
    theoreticalPos: [3.098, -4.880, -0.019],
    theoreticalMatrix: [
      [-0.5, 0.866, 0.0, 3.098],
      [0.6124, 0.3536, -0.7071, -4.880],
      [-0.6124, -0.3536, -0.7071, -0.019],
      [0, 0, 0, 1]
    ]
  },
  {
    id: 'aid2',
    name: '实验二 目标 3 (aid2)',
    tag: '目标 3',
    description: '关节变量 [-90°, 45°, 7, 0°, 90°, 0°, 45°, 0°]，移动关节伸长至 7.0',
    q: [-Math.PI / 2, Math.PI / 4, 7, 0, Math.PI / 2, 0, Math.PI / 4, 0],
    theoreticalPos: [6.243, -8.485, 3.586],
    theoreticalMatrix: [
      [0.7071, 0.7071, 0.0, 6.243],
      [0.5, -0.5, -0.7071, -8.485],
      [-0.5, 0.5, -0.7071, 3.586],
      [0, 0, 0, 1]
    ]
  },
  {
    id: 'aid3',
    name: '实验二 目标 4 (aid3)',
    tag: '目标 4',
    description: '关节变量 [90°, 45°, 8, 0°, -90°, 0°, -45°, 0°]，移动关节伸长至 8.0',
    q: [Math.PI / 2, Math.PI / 4, 8, 0, -Math.PI / 2, 0, -Math.PI / 4, 0],
    theoreticalPos: [-2.000, 9.364, 15.778],
    theoreticalMatrix: [
      [0.7071, -0.7071, 0.0, -2.000],
      [0.5, 0.5, -0.7071, 9.364],
      [0.5, 0.5, 0.7071, 15.778],
      [0, 0, 0, 1]
    ]
  },
  {
    id: 'aid4',
    name: '实验二 目标 5 (aid4)',
    tag: '目标 5',
    description: '关节变量 [-90°, -45°, 5, 0°, -90°, 0°, 60°, 0°]，移动副 5.0，关节5为负角度',
    q: [-Math.PI / 2, -Math.PI / 4, 5, 0, -Math.PI / 2, 0, Math.PI / 3, 0],
    theoreticalPos: [6.098, 4.880, 8.466],
    theoreticalMatrix: [
      [0.866, 0.5, 0.0, 6.098],
      [0.3536, -0.6124, 0.7071, 4.880],
      [0.3536, -0.6124, -0.7071, 8.466],
      [0, 0, 0, 1]
    ]
  }
];

export function evaluatePresetDiscrepancy(q: number[], theoreticalMatrix: number[][]) {
  const result = forwardKinematics(q);
  const elements = result.transform.elements;
  let maxDiff = 0;
  for (let row = 0; row < 4; row += 1) {
    for (let col = 0; col < 4; col += 1) {
      const actual = elements[col * 4 + row];
      const theo = theoreticalMatrix[row][col];
      const diff = Math.abs(actual - theo);
      if (diff > maxDiff) maxDiff = diff;
    }
  }
  const endPoint = result.points.at(-1)!;
  const theoPos = new THREE.Vector3(theoreticalMatrix[0][3], theoreticalMatrix[1][3], theoreticalMatrix[2][3]);
  const posError = endPoint.distanceTo(theoPos);
  return { maxDiff, posError, actualPos: endPoint };
}

export function clampConfiguration(q: number[]): number[] {
  return q.map((value, index) => THREE.MathUtils.clamp(value, joints[index].min, joints[index].max));
}

export function forwardKinematics(q: number[]): KinematicsResult {
  let current = new THREE.Matrix4().identity();
  const transforms: THREE.Matrix4[] = [current.clone()];
  const points: THREE.Vector3[] = [new THREE.Vector3()];
  joints.forEach((joint, index) => {
    const theta = joint.kind === 'revolute' ? q[index] : 0;
    const d = joint.kind === 'prismatic' ? q[index] : joint.d;
    const dh = new THREE.Matrix4()
      .makeRotationZ(theta)
      .multiply(new THREE.Matrix4().makeTranslation(0, 0, d))
      .multiply(new THREE.Matrix4().makeTranslation(joint.a, 0, 0))
      .multiply(new THREE.Matrix4().makeRotationX(joint.alpha));
    current = current.clone().multiply(dh);
    transforms.push(current.clone());
    points.push(new THREE.Vector3().setFromMatrixPosition(current));
  });
  return { transform: current, transforms, points };
}

export interface IkResult {
  q: number[];
  error: number;
  iterations: number;
  converged: boolean;
}

export interface PoseIkResult extends IkResult {
  orientationError: number;
}

/** Damped least-squares position IK; the current configuration is the seed. */
export function solvePositionIK(target: THREE.Vector3, seed: number[], maxIterations = 180): IkResult {
  let q = clampConfiguration(seed);
  const epsilon = 1e-4;
  const damping = 0.18;
  let error = Infinity;

  for (let iteration = 0; iteration < maxIterations; iteration += 1) {
    const end = forwardKinematics(q).points.at(-1)!;
    const delta = target.clone().sub(end);
    error = delta.length();
    if (error < 0.015) return { q, error, iterations: iteration + 1, converged: true };

    const jacobian = Array.from({ length: 3 }, () => Array(8).fill(0));
    for (let column = 0; column < 8; column += 1) {
      const sample = [...q];
      sample[column] = Math.min(sample[column] + epsilon, joints[column].max);
      const derivative = forwardKinematics(sample).points.at(-1)!.sub(end).multiplyScalar(1 / epsilon);
      jacobian[0][column] = derivative.x;
      jacobian[1][column] = derivative.y;
      jacobian[2][column] = derivative.z;
    }
    const jjT = new THREE.Matrix3();
    const e = jjT.elements;
    for (let row = 0; row < 3; row += 1) {
      for (let col = 0; col < 3; col += 1) {
        e[col * 3 + row] = jacobian[row].reduce((sum, value, k) => sum + value * jacobian[col][k], 0) + (row === col ? damping ** 2 : 0);
      }
    }
    const scaledError = delta.applyMatrix3(jjT.invert());
    const step = Array.from({ length: 8 }, (_, col) => jacobian[0][col] * scaledError.x + jacobian[1][col] * scaledError.y + jacobian[2][col] * scaledError.z);
    q = clampConfiguration(q.map((value, index) => value + THREE.MathUtils.clamp(step[index], -0.16, 0.16)));
  }
  error = forwardKinematics(q).points.at(-1)!.distanceTo(target);
  return { q, error, iterations: maxIterations, converged: error < 0.04 };
}

function rotationVector(from: THREE.Quaternion, to: THREE.Quaternion): THREE.Vector3 {
  const delta = to.clone().multiply(from.clone().invert()).normalize();
  if (delta.w < 0) delta.set(-delta.x, -delta.y, -delta.z, -delta.w);
  const sine = Math.sqrt(delta.x ** 2 + delta.y ** 2 + delta.z ** 2);
  if (sine < 1e-8) return new THREE.Vector3();
  return new THREE.Vector3(delta.x, delta.y, delta.z).multiplyScalar((2 * Math.atan2(sine, delta.w)) / sine);
}

function solveLinearSystem(matrix: number[][], vector: number[]): number[] | null {
  const augmented = matrix.map((row, index) => [...row, vector[index]]);
  for (let pivot = 0; pivot < augmented.length; pivot += 1) {
    let best = pivot;
    for (let row = pivot + 1; row < augmented.length; row += 1) if (Math.abs(augmented[row][pivot]) > Math.abs(augmented[best][pivot])) best = row;
    if (Math.abs(augmented[best][pivot]) < 1e-10) return null;
    [augmented[pivot], augmented[best]] = [augmented[best], augmented[pivot]];
    const divisor = augmented[pivot][pivot];
    for (let col = pivot; col <= augmented.length; col += 1) augmented[pivot][col] /= divisor;
    for (let row = 0; row < augmented.length; row += 1) {
      if (row === pivot) continue;
      const factor = augmented[row][pivot];
      for (let col = pivot; col <= augmented.length; col += 1) augmented[row][col] -= factor * augmented[pivot][col];
    }
  }
  return augmented.map(row => row.at(-1)!);
}

/** Six-dimensional damped least-squares IK: position and tool orientation are locked together. */
export function solvePoseIK(target: THREE.Vector3, targetOrientation: THREE.Quaternion, seed: number[], maxIterations = 110): PoseIkResult {
  let q = clampConfiguration(seed);
  const epsilon = 1e-4;
  const orientationWeight = 2.2;
  const damping = 0.22;
  let positionError = Infinity;
  let orientationError = Infinity;

  for (let iteration = 0; iteration < maxIterations; iteration += 1) {
    const current = forwardKinematics(q);
    const position = current.points.at(-1)!;
    const orientation = new THREE.Quaternion().setFromRotationMatrix(current.transform);
    const positionDelta = target.clone().sub(position);
    const rotationDelta = rotationVector(orientation, targetOrientation);
    positionError = positionDelta.length();
    orientationError = rotationDelta.length();
    if (positionError < 0.012 && orientationError < 0.018) return { q, error: positionError, orientationError, iterations: iteration + 1, converged: true };

    const jacobian = Array.from({ length: 6 }, () => Array(8).fill(0));
    for (let column = 0; column < 8; column += 1) {
      const sample = [...q];
      const direction = q[column] + epsilon <= joints[column].max ? 1 : -1;
      sample[column] = THREE.MathUtils.clamp(q[column] + direction * epsilon, joints[column].min, joints[column].max);
      const change = sample[column] - q[column];
      if (Math.abs(change) < 1e-10) continue;
      const perturbed = forwardKinematics(sample);
      const translated = perturbed.points.at(-1)!.sub(position).multiplyScalar(1 / change);
      const rotated = rotationVector(orientation, new THREE.Quaternion().setFromRotationMatrix(perturbed.transform)).multiplyScalar(1 / change);
      jacobian[0][column] = translated.x; jacobian[1][column] = translated.y; jacobian[2][column] = translated.z;
      jacobian[3][column] = rotated.x * orientationWeight; jacobian[4][column] = rotated.y * orientationWeight; jacobian[5][column] = rotated.z * orientationWeight;
    }
    const residual = [positionDelta.x, positionDelta.y, positionDelta.z, rotationDelta.x * orientationWeight, rotationDelta.y * orientationWeight, rotationDelta.z * orientationWeight];
    const normal = Array.from({ length: 8 }, (_, row) => Array.from({ length: 8 }, (_, col) => jacobian.reduce((sum, values) => sum + values[row] * values[col], row === col ? damping ** 2 : 0)));
    const right = Array.from({ length: 8 }, (_, column) => jacobian.reduce((sum, values, row) => sum + values[column] * residual[row], 0));
    const step = solveLinearSystem(normal, right);
    if (!step) break;
    q = clampConfiguration(q.map((value, index) => value + THREE.MathUtils.clamp(step[index], -0.12, 0.12)));
  }
  const result = forwardKinematics(q);
  positionError = result.points.at(-1)!.distanceTo(target);
  orientationError = rotationVector(new THREE.Quaternion().setFromRotationMatrix(result.transform), targetOrientation).length();
  return { q, error: positionError, orientationError, iterations: maxIterations, converged: positionError < 0.05 && orientationError < 0.07 };
}

export function formatJoint(value: number, index: number): string {
  return joints[index].kind === 'prismatic' ? value.toFixed(2) : `${THREE.MathUtils.radToDeg(value).toFixed(1)}°`;
}
