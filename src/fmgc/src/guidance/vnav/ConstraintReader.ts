import { GeographicCruiseStep, DescentAltitudeConstraint, MaxAltitudeConstraint, MaxSpeedConstraint } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { Geometry } from '@fmgc/guidance/Geometry';
import { AltitudeConstraintType, getAltitudeConstraintFromWaypoint, getSpeedConstraintFromWaypoint } from '@fmgc/guidance/lnav/legs';
import { FlightPlans, WaypointConstraintType } from '@fmgc/flightplanning/FlightPlanManager';
import { VMLeg } from '@fmgc/guidance/lnav/legs/VM';
import { PathCaptureTransition } from '@fmgc/guidance/lnav/transitions/PathCaptureTransition';
import { FixedRadiusTransition } from '@fmgc/guidance/lnav/transitions/FixedRadiusTransition';
import { GuidanceController } from '@fmgc/guidance/GuidanceController';

export class ConstraintReader {
    public climbAlitudeConstraints: MaxAltitudeConstraint[] = [];

    public descentAltitudeConstraints: DescentAltitudeConstraint[] = [];

    public climbSpeedConstraints: MaxSpeedConstraint[] = [];

    public descentSpeedConstraints: MaxSpeedConstraint[] = [];

    public cruiseSteps: GeographicCruiseStep[] = [];

    public totalFlightPlanDistance = 0;

    public distanceToEnd: NauticalMiles = -1;

    public get distanceToPresentPosition(): NauticalMiles {
        return this.totalFlightPlanDistance - this.distanceToEnd;
    }

    constructor(private guidanceController: GuidanceController) {
        this.reset();
    }

    updateGeometry(geometry: Geometry, ppos: LatLongAlt) {
        this.reset();
        this.updateDistancesToEnd(geometry);

        const fpm = this.guidanceController.flightPlanManager;
        for (let i = 0; i < fpm.getWaypointsCount(FlightPlans.Active); i++) {
            const waypoint = fpm.getWaypoint(i, FlightPlans.Active);

            if (waypoint.additionalData.cruiseStep) {
                const { waypointIndex, toAltitude, distanceBeforeTermination } = waypoint.additionalData.cruiseStep;

                this.cruiseSteps.push({
                    distanceFromStart: this.totalFlightPlanDistance - waypoint.additionalData.distanceToEnd - distanceBeforeTermination,
                    toAltitude,
                    waypointIndex,
                    isIgnored: false,
                });
            }

            const altConstraint = getAltitudeConstraintFromWaypoint(waypoint);
            const speedConstraint = getSpeedConstraintFromWaypoint(waypoint);

            if (waypoint.additionalData.constraintType === WaypointConstraintType.CLB) {
                if (altConstraint && altConstraint.type !== AltitudeConstraintType.atOrAbove) {
                    this.climbAlitudeConstraints.push({
                        distanceFromStart: this.totalFlightPlanDistance - waypoint.additionalData.distanceToEnd,
                        maxAltitude: altConstraint.altitude1,
                    });
                }

                if (speedConstraint && waypoint.speedConstraint > 100) {
                    this.climbSpeedConstraints.push({
                        distanceFromStart: this.totalFlightPlanDistance - waypoint.additionalData.distanceToEnd,
                        maxSpeed: speedConstraint.speed,
                    });
                }
            } else if (waypoint.additionalData.constraintType === WaypointConstraintType.DES) {
                if (altConstraint) {
                    this.descentAltitudeConstraints.push({
                        distanceFromStart: this.totalFlightPlanDistance - waypoint.additionalData.distanceToEnd,
                        constraint: altConstraint,
                    });
                }

                if (speedConstraint && waypoint.speedConstraint > 100) {
                    this.descentSpeedConstraints.push({
                        distanceFromStart: this.totalFlightPlanDistance - waypoint.additionalData.distanceToEnd,
                        maxSpeed: speedConstraint.speed,
                    });
                }
            }
        }

        this.updateDistanceToEnd(ppos);
    }

    public updateDistanceToEnd(ppos: LatLongAlt) {
        const geometry = this.guidanceController.activeGeometry;
        const activeLegIndex = this.guidanceController.activeLegIndex;
        const activeTransIndex = this.guidanceController.activeTransIndex;
        const fpm = this.guidanceController.flightPlanManager;

        const leg = geometry.legs.get(activeLegIndex);
        if (!leg || leg.isNull) {
            return;
        }

        const nextWaypoint = fpm.getWaypoint(activeLegIndex, FlightPlans.Active);

        const inboundTransition = geometry.transitions.get(activeLegIndex - 1);
        const outboundTransition = geometry.transitions.get(activeLegIndex);

        const [_, legDistance, outboundLength] = Geometry.completeLegPathLengths(
            leg, (inboundTransition?.isNull || !inboundTransition?.isComputed) ? null : inboundTransition, outboundTransition,
        );

        if (activeTransIndex < 0) {
            const distanceToGo = leg instanceof VMLeg
                ? Avionics.Utils.computeGreatCircleDistance(ppos, nextWaypoint.infos.coordinates)
                : leg.getDistanceToGo(ppos);

            this.distanceToEnd = distanceToGo + outboundLength + (nextWaypoint.additionalData.distanceToEnd ?? 0);
        } else if (activeTransIndex === activeLegIndex) {
            // On an outbound transition
            // We subtract `outboundLength` because getDistanceToGo will include the entire distance while we only want the part that's on this leg.
            // For a FixedRadiusTransition, there's also a part on the next leg.
            this.distanceToEnd = outboundTransition.getDistanceToGo(ppos) - outboundLength + (nextWaypoint.additionalData.distanceToEnd ?? 0);
        } else if (activeTransIndex === activeLegIndex - 1) {
            // On an inbound transition
            const trueTrack = SimVar.GetSimVarValue('GPS GROUND TRUE TRACK', 'degree');

            let transitionDistanceToGo = inboundTransition.getDistanceToGo(ppos);

            if (inboundTransition instanceof PathCaptureTransition) {
                transitionDistanceToGo = inboundTransition.getActualDistanceToGo(ppos, trueTrack);
            } else if (inboundTransition instanceof FixedRadiusTransition && inboundTransition.isReverted) {
                transitionDistanceToGo = inboundTransition.revertTo.getActualDistanceToGo(ppos, trueTrack);
            }

            this.distanceToEnd = transitionDistanceToGo + legDistance + outboundLength + (nextWaypoint.additionalData.distanceToEnd ?? 0);
        } else {
            console.error(`[FMS/VNAV] Unexpected transition index (legIndex: ${activeLegIndex}, transIndex: ${activeTransIndex})`);
        }

        SimVar.SetSimVarValue('L:A32NX_FM_VNAV_DEBUG_DISTANCE_TO_END', 'number', this.distanceToEnd);
        SimVar.SetSimVarValue('L:A32NX_FM_VNAV_DEBUG_DISTANCE_FROM_START', 'number', this.distanceToPresentPosition);
    }

    reset() {
        this.climbAlitudeConstraints = [];
        this.descentAltitudeConstraints = [];
        this.climbSpeedConstraints = [];
        this.descentSpeedConstraints = [];
        this.cruiseSteps = [];

        this.totalFlightPlanDistance = 0;
        this.distanceToEnd = 0;
    }

    private updateDistancesToEnd(geometry: Geometry) {
        const { legs, transitions } = geometry;
        const fpm = this.guidanceController.flightPlanManager;

        this.totalFlightPlanDistance = 0;

        for (let i = fpm.getWaypointsCount(FlightPlans.Active) - 1; i > fpm.getActiveWaypointIndex() - 1 && i >= 0; i--) {
            const leg = legs.get(i);
            const waypoint = fpm.getWaypoint(i, FlightPlans.Active);
            const nextWaypoint = fpm.getWaypoint(i + 1, FlightPlans.Active);

            if (!leg || leg.isNull) {
                return;
            }

            if (waypoint.endsInDiscontinuity) {
                const startingPointOfDisco = waypoint.discontinuityCanBeCleared
                    ? waypoint
                    : fpm.getWaypoint(i - 1, FlightPlans.Active); // MANUAL

                this.totalFlightPlanDistance += Avionics.Utils.computeGreatCircleDistance(startingPointOfDisco.infos.coordinates, nextWaypoint.infos.coordinates);
            }

            waypoint.additionalData.distanceToEnd = this.totalFlightPlanDistance;

            const inboundTransition = transitions.get(i - 1);
            const outboundTransition = transitions.get(i);

            const [inboundLength, legDistance, outboundLength] = Geometry.completeLegPathLengths(
                leg, (inboundTransition?.isNull || !inboundTransition?.isComputed) ? null : inboundTransition, outboundTransition,
            );

            const correctedInboundLength = Number.isNaN(inboundLength) ? 0 : inboundLength;
            const totalLegLength = legDistance + correctedInboundLength + outboundLength;

            this.totalFlightPlanDistance += totalLegLength;
        }
    }
}