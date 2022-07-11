import { Interpolator } from '@fmgc/flightmanagement/vnav/common/Interpolator';
import { AircraftStateWithPhase, McduPseudoWaypointRequest, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { Geometry } from '@fmgc/guidance/Geometry';
import { AltitudeConstraint, AltitudeConstraintType, SpeedConstraint } from '@fmgc/guidance/lnav/legs';
import { VMLeg } from '@fmgc/guidance/lnav/legs/VM';
import { XFLeg } from '@fmgc/guidance/lnav/legs/XF';
import { McduPseudoWaypointType, SpeedConstraintPrediction } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { AccelFactorMode } from '@fmgc/guidance/vnav/common';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { FmgcFlightPhase } from '@shared/flightphase';
import { FlightPlanManager } from '@shared/flightplan';

enum VerticalFlightPlanState {
    BeforeComputation,
    InComputation,
    AfterComputation,
}

export class VerticalFlightPlan {
    private state: VerticalFlightPlanState = VerticalFlightPlanState.BeforeComputation

    private waypointPredictions: Map<number, VerticalWaypointPrediction> = new Map();

    mcduPseudoWaypointRequests: McduPseudoWaypointRequest[] = []

    constructor(private flightPlanManager: FlightPlanManager, private observer: VerticalProfileComputationParametersObserver, private atmosphericConditions: AtmosphericConditions) {

    }

    get displayState(): VerticalFlightPlanState {
        return this.state;
    }

    update(builder: ProfileBuilder, geometry: Geometry) {
        this.state = VerticalFlightPlanState.InComputation;

        this.computePredictionsAtWaypoints(builder.allCheckpointsWithPhase, geometry);
        this.updateMcduPseudoWaypoints(builder);

        this.state = VerticalFlightPlanState.AfterComputation;
    }

    getWaypointPrediction(index: number): VerticalWaypointPrediction | null {
        if (this.state !== VerticalFlightPlanState.AfterComputation) {
            return null;
        }

        return this.waypointPredictions.get(index);
    }

    private computePredictionsAtWaypoints(checkpoints: AircraftStateWithPhase[], geometry: Geometry) {
        this.waypointPredictions.clear();

        let totalDistance = 0;
        const { zeroFuelWeight } = this.observer.get();

        for (let i = 0; i < this.flightPlanManager.getWaypointsCount(); i++) {
            const leg = geometry.legs.get(i);
            if (!leg || leg.isNull) {
                continue;
            }

            const inboundTransition = geometry.transitions.get(i - 1);
            const outboundTransition = geometry.transitions.get(i);

            const [inboundLength, legDistance, outboundLength] = Geometry.completeLegPathLengths(
                leg, (inboundTransition?.isNull || !inboundTransition?.isComputed) ? null : inboundTransition, outboundTransition,
            );

            const correctedInboundLength = Number.isNaN(inboundLength) ? 0 : inboundLength;

            const totalLegLength = legDistance + correctedInboundLength + outboundLength;

            totalDistance += totalLegLength;

            const { time, altitude, speeds, weight } = Interpolator.interpolateEverythingFromStart(checkpoints, totalDistance);

            this.waypointPredictions.set(i, {
                waypointIndex: i,
                distanceFromStart: totalDistance,
                time,
                altitude,
                speed: speeds.speedTargetType === AccelFactorMode.CONSTANT_MACH ? speeds.mach : speeds.calibratedAirspeed,
                estimatedFuelOnBoard: weight - zeroFuelWeight,
                altitudeConstraint: leg.metadata.altitudeConstraint,
                isAltitudeConstraintMet: this.isAltitudeConstraintMet(altitude, leg.metadata.altitudeConstraint),
                speedConstraint: leg.metadata.speedConstraint,
                isSpeedConstraintMet: this.isSpeedConstraintMet(speeds.calibratedAirspeed, leg.metadata.speedConstraint),
                altError: this.computeAltError(altitude, leg.metadata.altitudeConstraint),
                distanceToTopOfDescent: null, // TODO
            });

            let distanceInDiscontinuity = 0;
            const nextLeg = geometry.legs.get(i + 1);
            const previousLeg = geometry.legs.get(i - 1);

            if (leg instanceof XFLeg && leg.fix.endsInDiscontinuity && nextLeg instanceof XFLeg) {
                distanceInDiscontinuity = Avionics.Utils.computeGreatCircleDistance(leg.fix.infos.coordinates, nextLeg.fix.infos.coordinates);
            } else if (leg instanceof VMLeg && previousLeg instanceof XFLeg && nextLeg instanceof XFLeg) {
                distanceInDiscontinuity = Avionics.Utils.computeGreatCircleDistance(previousLeg.fix.infos.coordinates, nextLeg.fix.infos.coordinates);
            }

            totalDistance += distanceInDiscontinuity;
        }
    }

    private updateMcduPseudoWaypoints(builder: ProfileBuilder) {
        this.mcduPseudoWaypointRequests = builder.mcduPseudoWaypointRequests;
        this.addSpeedLimitPseudoWaypoints(builder);
    }

    private addSpeedLimitPseudoWaypoints(builder: ProfileBuilder) {
        const { climbSpeedLimit, descentSpeedLimit, cruiseAltitude, presentPosition, flightPhase } = this.observer.get();

        if (
            Number.isFinite(climbSpeedLimit.speed) && Number.isFinite(climbSpeedLimit.underAltitude) && cruiseAltitude >= climbSpeedLimit.underAltitude
            && flightPhase <= FmgcFlightPhase.Climb && presentPosition.alt <= climbSpeedLimit.underAltitude
        ) {
            const climbCheckpoints = builder.checkpointsOfPhase(FmgcFlightPhase.Climb).map((state) => ({ ...state, phase: FmgcFlightPhase.Climb }));
            const distanceToSpeedLimitCrossing = Interpolator.interpolateDistanceAtAltitude(climbCheckpoints, climbSpeedLimit.underAltitude);
            const stateAtSpeedLimitAlt = Interpolator.interpolateEverythingFromStart(climbCheckpoints, distanceToSpeedLimitCrossing);

            this.mcduPseudoWaypointRequests.push({
                type: McduPseudoWaypointType.SpeedLimit,
                state: stateAtSpeedLimitAlt,
                speedConstraint: {
                    speed: climbSpeedLimit.speed,
                    isMet: this.isSpeedConstraintMet(stateAtSpeedLimitAlt.speeds.calibratedAirspeed, climbSpeedLimit),
                },
            });
        }

        if (
            Number.isFinite(descentSpeedLimit.speed) && Number.isFinite(descentSpeedLimit.underAltitude) && cruiseAltitude >= descentSpeedLimit.underAltitude
            && flightPhase <= FmgcFlightPhase.Approach && presentPosition.alt >= descentSpeedLimit.underAltitude
        ) {
            const descentCheckpoints = builder.checkpointsOfPhase(FmgcFlightPhase.Descent).map((state) => ({ ...state, phase: FmgcFlightPhase.Descent }));
            const approachCheckpoints = builder.checkpointsOfPhase(FmgcFlightPhase.Approach).map((state) => ({ ...state, phase: FmgcFlightPhase.Approach }));
            const relevantCheckpoints = [...descentCheckpoints, ...approachCheckpoints];

            const distanceToSpeedLimitCrossing = Interpolator.interpolateDistanceAtAltitude(relevantCheckpoints, descentSpeedLimit.underAltitude, true);
            const stateAtSpeedLimitAlt = Interpolator.interpolateEverythingFromStart(relevantCheckpoints, distanceToSpeedLimitCrossing);

            this.mcduPseudoWaypointRequests.push({
                type: McduPseudoWaypointType.SpeedLimit,
                state: stateAtSpeedLimitAlt,
                speedConstraint: {
                    speed: descentSpeedLimit.speed,
                    isMet: this.isSpeedConstraintMet(stateAtSpeedLimitAlt.speeds.calibratedAirspeed, descentSpeedLimit),
                },
            });
        }
    }

    private isAltitudeConstraintMet(altitude: Feet, constraint?: AltitudeConstraint): boolean {
        if (!constraint) {
            return true;
        }

        switch (constraint.type) {
        case AltitudeConstraintType.at:
            return Math.abs(altitude - constraint.altitude1) < 250;
        case AltitudeConstraintType.atOrAbove:
            return (altitude - constraint.altitude1) > -250;
        case AltitudeConstraintType.atOrBelow:
            return (altitude - constraint.altitude1) < 250;
        case AltitudeConstraintType.range:
            return (altitude - constraint.altitude2) > -250 && (altitude - constraint.altitude1) < 250;
        default:
            console.error('Invalid altitude constraint type');
            return null;
        }
    }

    private isSpeedConstraintMet(speed: Knots, constraint?: { speed: Knots }): boolean {
        if (!constraint) {
            return true;
        }

        return speed - constraint.speed < 5;
    }

    private computeAltError(predictedAltitude: Feet, constraint?: AltitudeConstraint): number {
        if (!constraint) {
            return 0;
        }

        switch (constraint.type) {
        case AltitudeConstraintType.at:
            return predictedAltitude - constraint.altitude1;
        case AltitudeConstraintType.atOrAbove:
            return Math.min(predictedAltitude - constraint.altitude1, 0);
        case AltitudeConstraintType.atOrBelow:
            return Math.max(predictedAltitude - constraint.altitude1, 0);
        case AltitudeConstraintType.range:
            if (predictedAltitude >= constraint.altitude1) {
                return predictedAltitude - constraint.altitude1;
            } if (predictedAltitude <= constraint.altitude2) {
                return predictedAltitude - constraint.altitude1;
            }

            return 0;
        default:
            console.error('Invalid altitude constraint type');
            return 0;
        }
    }
}

export interface VerticalWaypointPrediction {
    waypointIndex: number,
    distanceFromStart: NauticalMiles,
    time: Seconds,
    altitude: Feet,
    speed: Knots | Mach,
    estimatedFuelOnBoard: Pounds,
    altitudeConstraint: AltitudeConstraint,
    speedConstraint: SpeedConstraint,
    isAltitudeConstraintMet: boolean,
    isSpeedConstraintMet: boolean,
    altError: number,
    distanceToTopOfDescent: NauticalMiles | null,
}

export interface VerticalPseudoWaypointPrediction {
    distanceFromStart: NauticalMiles,
    altitude: Feet,
    speed: Knots,
    time: Seconds,
    speedConstraint?: SpeedConstraintPrediction,
}
