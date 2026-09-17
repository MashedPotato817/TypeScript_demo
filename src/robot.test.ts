import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { COURSE_PRESETS, evaluatePresetDiscrepancy, forwardKinematics, initialQ, joints, solvePositionIK } from './robot';
import { buildShapeTrajectory } from './trajectory';

describe('8-DOF kinematics', () => {
  it('keeps all generated shape trajectories inside joint limits', () => {
    const trajectory = buildShapeTrajectory('circle', 40);
    expect(trajectory.samples).toHaveLength(40);
    trajectory.samples.forEach(q => q.forEach((value, index) => expect(value).toBeGreaterThanOrEqual(joints[index].min)));
    trajectory.samples.forEach(q => q.forEach((value, index) => expect(value).toBeLessThanOrEqual(joints[index].max)));
  });

  it('draws a closed circle while preserving the tool orientation', () => {
    const trajectory = buildShapeTrajectory('circle', 48);
    expect(trajectory.maxPositionError).toBeLessThan(0.07);
    expect(trajectory.maxOrientationError).toBeLessThan(0.09);
    expect(trajectory.actualPoints[0].distanceTo(trajectory.actualPoints.at(-1)!)).toBeLessThan(0.04);
  });

  it('recovers a forward-kinematics position with seeded IK', () => {
    const source = [0.25, 0.55, 5.7, -0.3, 0.25, 0.1, 0.2, -0.1];
    const target = forwardKinematics(source).points.at(-1)!;
    const result = solvePositionIK(target, initialQ);
    expect(result.error).toBeLessThan(0.04);
  });

  it('matches textbook reference matrices for all 5 course presets (aid0 - aid4)', () => {
    COURSE_PRESETS.forEach(preset => {
      const { maxDiff, posError } = evaluatePresetDiscrepancy(preset.q, preset.theoreticalMatrix);
      // Theoretical values in PDF have 3 to 4 decimal digits
      expect(posError).toBeLessThan(0.005);
      expect(maxDiff).toBeLessThan(0.005);
    });
  });

  it('solves Lab 3 waypoints with solvePositionIK', () => {
    const waypoints = [
      new THREE.Vector3(5.019, 6.098, 11.433), // aid0
      new THREE.Vector3(0.5, 0.0, 0.0),
      new THREE.Vector3(0.5, 0.0, -0.4),
      new THREE.Vector3(-0.1, 0.5, -0.4),
      new THREE.Vector3(-0.1, 0.5, 0.1),
    ];
    let seed = [...COURSE_PRESETS[1].q];
    for (const pt of waypoints) {
      const res = solvePositionIK(pt, seed, 250);
      expect(res.converged).toBe(true);
      seed = res.q;
    }
  });

  it('generates continuous course trajectories within joint limits', () => {
    const patrol = buildShapeTrajectory('lab2_patrol');
    expect(patrol.samples.length).toBeGreaterThan(100);
    patrol.samples.forEach(q => q.forEach((val, idx) => {
      expect(val).toBeGreaterThanOrEqual(joints[idx].min - 1e-4);
      expect(val).toBeLessThanOrEqual(joints[idx].max + 1e-4);
    }));

    const lab3 = buildShapeTrajectory('lab3_loop');
    expect(lab3.samples.length).toBeGreaterThan(100);
    lab3.samples.forEach(q => q.forEach((val, idx) => {
      expect(val).toBeGreaterThanOrEqual(joints[idx].min - 1e-4);
      expect(val).toBeLessThanOrEqual(joints[idx].max + 1e-4);
    }));
    // Lab 3 is a closed loop trajectory: start point and end point should match
    expect(lab3.actualPoints[0].distanceTo(lab3.actualPoints.at(-1)!)).toBeLessThan(0.01);
  });
});
