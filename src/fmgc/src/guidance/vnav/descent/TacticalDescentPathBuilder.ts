import { DescentStrategy } from '@fmgc/guidance/vnav/descent/DescentStrategy';
import { StepResults } from '@fmgc/guidance/vnav/Predictions';
import { BaseGeometryProfile } from '@fmgc/guidance/vnav/profile/BaseGeometryProfile';
import { MaxSpeedConstraint, VerticalCheckpointReason } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { TemporaryCheckpointSequence } from '@fmgc/guidance/vnav/profile/TemporaryCheckpointSequence';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';

export class TacticalDescentPathBuilder {
    constructor(private observer: VerticalProfileComputationParametersObserver) { }

    /**
     * Builds a path from the last checkpoint to the finalAltitude
     * @param profile
     * @param descentStrategy
     * @param windProfile
     * @param finalAltitude
     */
    buildTacticalDescentPath(profile: BaseGeometryProfile, descentStrategy: DescentStrategy, windProfile: HeadwindProfile, finalAltitude: Feet) {
        const constraintsToUse = profile.descentSpeedConstraints
            .slice()
            .reverse()
            .sort((a, b) => a.distanceFromStart - b.distanceFromStart);

        const sequence = new TemporaryCheckpointSequence(profile.lastCheckpoint);
        const { managedDescentSpeedMach } = this.observer.get();

        for (const constraint of constraintsToUse) {
            this.handleSpeedConstraint(sequence, constraint, descentStrategy, windProfile);

            if (sequence.lastCheckpoint.altitude <= finalAltitude) {
                break;
            }
        }

        if (sequence.lastCheckpoint.altitude > finalAltitude) {
            const descentSegment = descentStrategy.predictToAltitude(
                sequence.lastCheckpoint.altitude,
                finalAltitude,
                sequence.lastCheckpoint.speed,
                managedDescentSpeedMach,
                sequence.lastCheckpoint.remainingFuelOnBoard,
                windProfile.getHeadwindComponent(sequence.lastCheckpoint.distanceFromStart, sequence.lastCheckpoint.altitude),
            );

            sequence.addCheckpointFromStep(descentSegment, VerticalCheckpointReason.CrossingFcuAltitudeDescent);
        }

        profile.checkpoints.push(...sequence.get());
    }

    private handleSpeedConstraint(sequence: TemporaryCheckpointSequence, constraint: MaxSpeedConstraint, descentStrategy: DescentStrategy, windProfile: HeadwindProfile) {
        if (sequence.lastCheckpoint.speed <= constraint.maxSpeed) {
            return;
        }

        const { managedDescentSpeedMach } = this.observer.get();

        const distanceToConstraint = constraint.distanceFromStart - sequence.lastCheckpoint.distanceFromStart;
        const maxDecelerationDistance = Math.min(distanceToConstraint, 20);

        let descentSegment: StepResults = null;
        let decelerationSegment: StepResults = null;

        const tryDecelDistance = (decelerationSegmentDistance: NauticalMiles): NauticalMiles => {
            descentSegment = descentStrategy.predictToDistance(
                sequence.lastCheckpoint.altitude,
                distanceToConstraint - decelerationSegmentDistance,
                sequence.lastCheckpoint.speed,
                managedDescentSpeedMach,
                sequence.lastCheckpoint.remainingFuelOnBoard,
                windProfile.getHeadwindComponent(sequence.lastCheckpoint.distanceFromStart, sequence.lastCheckpoint.altitude),
            );

            decelerationSegment = descentStrategy.predictToSpeed(
                descentSegment.finalAltitude,
                constraint.maxSpeed,
                descentSegment.speed,
                managedDescentSpeedMach,
                sequence.lastCheckpoint.remainingFuelOnBoard - descentSegment.fuelBurned,
                windProfile.getHeadwindComponent(sequence.lastCheckpoint.distanceFromStart + descentSegment.distanceTraveled, descentSegment.finalAltitude),
            );

            const totalDistanceTravelled = sequence.lastCheckpoint.distanceFromStart + descentSegment.distanceTraveled + decelerationSegment.distanceTraveled;
            const distanceOvershoot = totalDistanceTravelled - distanceToConstraint;

            return distanceOvershoot;
        };

        let a = 0;
        let b = maxDecelerationDistance;

        let fa = tryDecelDistance(0);
        if (fa > 0) {
            let i = 0;
            while (i++ < 10) {
                const c = (a + b) / 2;
                const fc = tryDecelDistance(c);

                if (Math.abs(fc) < 0.05 || (b - a) < 0.05) {
                    if (VnavConfig.DEBUG_PROFILE) {
                        console.log(`[FMS/VNAV] Final error ${fc} after ${i} iterations.`);
                    }

                    break;
                }

                if (fa * fc > 0) {
                    a = c;
                } else {
                    b = c;
                }

                fa = tryDecelDistance(a);
            }
        }

        sequence.addCheckpointFromStep(descentSegment, VerticalCheckpointReason.AtmosphericConditions);
        sequence.addCheckpointFromStep(decelerationSegment, VerticalCheckpointReason.SpeedConstraint);
    }
}
