import { AircraftStateWithPhase, McduPseudoWaypointRequest, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { Geometry } from '@fmgc/guidance/Geometry';
import { AltitudeConstraint, AltitudeConstraintType, SpeedConstraint, SpeedConstraintType } from '@fmgc/guidance/lnav/legs';
import { VMLeg } from '@fmgc/guidance/lnav/legs/VM';
import { XFLeg } from '@fmgc/guidance/lnav/legs/XF';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { Common } from '@fmgc/guidance/vnav/common';
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

    private climbCrossoverAltitude: Feet = 25000;

    private cruiseCrossoverAltitude: Feet = 25000;

    private descentCrossoverAltitude: Feet = 25000;

    mcduPseudoWaypointRequests: McduPseudoWaypointRequest[] = []

    constructor(private flightPlanManager: FlightPlanManager, private observer: VerticalProfileComputationParametersObserver, private atmosphericConditions: AtmosphericConditions) {

    }

    get displayState(): VerticalFlightPlanState {
        return this.state;
    }

    update(builder: ProfileBuilder, geometry: Geometry) {
        this.state = VerticalFlightPlanState.InComputation;

        this.updateCrossoverAltitudes();
        this.computePredictionsAtWaypoints(builder.allCheckpointsWithPhase, geometry);
        this.mcduPseudoWaypointRequests = builder.mcduPseudoWaypointRequests;

        this.state = VerticalFlightPlanState.AfterComputation;
    }

    private updateCrossoverAltitudes() {
        const { managedClimbSpeed, managedClimbSpeedMach, managedCruiseSpeed, managedCruiseSpeedMach, managedDescentSpeed, managedDescentSpeedMach } = this.observer.get();

        this.climbCrossoverAltitude = this.atmosphericConditions.crossoverAltitude(managedClimbSpeed, managedClimbSpeedMach);
        this.cruiseCrossoverAltitude = this.atmosphericConditions.crossoverAltitude(managedCruiseSpeed, managedCruiseSpeedMach);
        this.descentCrossoverAltitude = this.atmosphericConditions.crossoverAltitude(managedDescentSpeed, managedDescentSpeedMach);
    }

    getWaypointPrediction(index: number): VerticalWaypointPrediction | null {
        if (this.state !== VerticalFlightPlanState.AfterComputation) {
            return null;
        }

        return this.waypointPredictions.get(index);
    }

    private getCrossoverAltitudeByPhase(phase: FmgcFlightPhase): Mach {
        switch (phase) {
        case FmgcFlightPhase.Preflight:
        case FmgcFlightPhase.Takeoff:
        case FmgcFlightPhase.Climb:
        case FmgcFlightPhase.GoAround:
            return this.climbCrossoverAltitude;
        case FmgcFlightPhase.Cruise:
            return this.cruiseCrossoverAltitude;
        case FmgcFlightPhase.Descent:
        case FmgcFlightPhase.Approach:
        case FmgcFlightPhase.Done:
            return this.descentCrossoverAltitude;
        default:
            return 0.78; // This is here so eslint is happy
        }
    }

    private computePredictionsAtWaypoints(checkpoints: AircraftStateWithPhase[], geometry: Geometry) {
        this.waypointPredictions.clear();

        let totalDistance = 0;

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

            const { time, altitude, speed, mach, phase } = this.interpolateEverythingFromStart(checkpoints, totalDistance);

            this.waypointPredictions.set(i, {
                waypointIndex: i,
                distanceFromStart: totalDistance,
                time,
                altitude,
                speed: altitude >= this.getCrossoverAltitudeByPhase(phase) ? mach : speed,
                altitudeConstraint: leg.metadata.altitudeConstraint,
                isAltitudeConstraintMet: this.isAltitudeConstraintMet(altitude, leg.metadata.altitudeConstraint),
                speedConstraint: leg.metadata.speedConstraint,
                isSpeedConstraintMet: this.isSpeedConstraintMet(speed, leg.metadata.speedConstraint),
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

    private isSpeedConstraintMet(speed: Knots, constraint?: SpeedConstraint): boolean {
        if (!constraint) {
            return true;
        }

        switch (constraint.type) {
        case SpeedConstraintType.at:
            return Math.abs(speed - constraint.speed) < 5;
        case SpeedConstraintType.atOrBelow:
            return speed - constraint.speed < 5;
        case SpeedConstraintType.atOrAbove:
            return speed - constraint.speed > -5;
        default:
            console.error('Invalid speed constraint type');
            return null;
        }
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

    private interpolateEverythingFromStart(checkpoints: AircraftStateWithPhase[], distanceFromStart: NauticalMiles): AircraftStateWithPhase {
        if (distanceFromStart <= checkpoints[0].distanceFromStart) {
            return {
                distanceFromStart,
                time: checkpoints[0].time,
                altitude: checkpoints[0].altitude,
                weight: checkpoints[0].weight,
                speed: checkpoints[0].speed,
                mach: checkpoints[0].mach,
                trueAirspeed: checkpoints[0].trueAirspeed,
                config: checkpoints[0].config,
                phase: checkpoints[0].phase,
            };
        }

        for (let i = 0; i < checkpoints.length - 1; i++) {
            if (distanceFromStart > checkpoints[i].distanceFromStart && distanceFromStart <= checkpoints[i + 1].distanceFromStart) {
                return {
                    distanceFromStart,
                    time: Common.interpolate(
                        distanceFromStart,
                        checkpoints[i].distanceFromStart,
                        checkpoints[i + 1].distanceFromStart,
                        checkpoints[i].time,
                        checkpoints[i + 1].time,
                    ),
                    altitude: Common.interpolate(
                        distanceFromStart,
                        checkpoints[i].distanceFromStart,
                        checkpoints[i + 1].distanceFromStart,
                        checkpoints[i].altitude,
                        checkpoints[i + 1].altitude,
                    ),
                    weight: Common.interpolate(
                        distanceFromStart,
                        checkpoints[i].distanceFromStart,
                        checkpoints[i + 1].distanceFromStart,
                        checkpoints[i].weight,
                        checkpoints[i + 1].weight,
                    ),
                    speed: Common.interpolate(
                        distanceFromStart,
                        checkpoints[i].distanceFromStart,
                        checkpoints[i + 1].distanceFromStart,
                        checkpoints[i].speed,
                        checkpoints[i + 1].speed,
                    ),
                    trueAirspeed: Common.interpolate(
                        distanceFromStart,
                        checkpoints[i].distanceFromStart,
                        checkpoints[i + 1].distanceFromStart,
                        checkpoints[i].trueAirspeed,
                        checkpoints[i + 1].trueAirspeed,
                    ),
                    mach: Common.interpolate(
                        distanceFromStart,
                        checkpoints[i].distanceFromStart,
                        checkpoints[i + 1].distanceFromStart,
                        checkpoints[i].mach,
                        checkpoints[i + 1].mach,
                    ),
                    config: checkpoints[i].config,
                    phase: checkpoints[i].phase,
                };
            }
        }

        return {
            distanceFromStart,
            time: checkpoints[checkpoints.length - 1].time,
            altitude: checkpoints[checkpoints.length - 1].altitude,
            weight: checkpoints[checkpoints.length - 1].weight,
            speed: checkpoints[checkpoints.length - 1].speed,
            trueAirspeed: checkpoints[checkpoints.length - 1].trueAirspeed,
            mach: checkpoints[checkpoints.length - 1].mach,
            config: checkpoints[checkpoints.length - 1].config,
            phase: checkpoints[checkpoints.length - 1].phase,
        };
    }
}

export interface VerticalWaypointPrediction {
    waypointIndex: number,
    distanceFromStart: NauticalMiles,
    time: Seconds,
    altitude: Feet,
    speed: Knots | Mach,
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
}
