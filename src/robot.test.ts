import { describe, expect, it } from 'vitest';
import { forwardKinematics, initialQ, joints, solvePositionIK } from './robot';
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
});
