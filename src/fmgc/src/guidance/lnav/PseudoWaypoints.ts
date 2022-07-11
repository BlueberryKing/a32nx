// Copyright (c) 2021-2022 FlyByWire Simulations
// Copyright (c) 2021-2022 Synaptic Simulations
//
// SPDX-License-Identifier: GPL-3.0

import { GuidanceComponent } from '@fmgc/guidance/GuidanceComponent';
import { PseudoWaypoint, PseudoWaypointSequencingAction } from '@fmgc/guidance/PseudoWaypoint';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { Geometry } from '@fmgc/guidance/Geometry';
import { Coordinates } from '@fmgc/flightplanning/data/geo';
import { GuidanceController } from '@fmgc/guidance/GuidanceController';
import { LateralMode } from '@shared/autopilot';
import { FixedRadiusTransition } from '@fmgc/guidance/lnav/transitions/FixedRadiusTransition';
import { XFLeg } from '@fmgc/guidance/lnav/legs/XF';
import { VMLeg } from '@fmgc/guidance/lnav/legs/VM';
import { AircraftState } from '@fmgc/flightmanagement/vnav/segments';
import { VerticalFlightPlan, VerticalPseudoWaypointPrediction } from '@fmgc/flightmanagement/vnav/VerticalFlightPlan';
import { AccelFactorMode } from '@fmgc/guidance/vnav/common';

type McduPseudoWaypointTemplate = {
    readonly type: McduPseudoWaypointType,
    readonly mcduIdent: string,
    readonly mcduHeader?: string,
    readonly sequenceType?: PseudoWaypointSequencingAction,
}

interface McduPseudoWaypoint extends McduPseudoWaypointTemplate {
    alongLegIndex: number,
    distanceFromLastFix: NauticalMiles;
    prediction: VerticalPseudoWaypointPrediction,
    speedConstraint?: Knots
}

export enum McduPseudoWaypointType {
    SpeedLimit,
    TopOfClimb,
    TopOfDescent,
    StepClimb,
    StepDescent,
    Decel,
    Flap1,
    Flap2,
}

const pwpByType: Map<McduPseudoWaypointType, McduPseudoWaypointTemplate> = new Map([
    [McduPseudoWaypointType.SpeedLimit, { type: McduPseudoWaypointType.SpeedLimit, mcduIdent: '(LIM)', mcduHeader: '(SPD)' }],
    [McduPseudoWaypointType.TopOfClimb, { type: McduPseudoWaypointType.TopOfClimb, mcduIdent: '(T/C)' }],
    [McduPseudoWaypointType.TopOfDescent, { type: McduPseudoWaypointType.TopOfDescent, mcduIdent: '(T/D)' }],
    [McduPseudoWaypointType.StepClimb, { type: McduPseudoWaypointType.StepClimb, mcduIdent: '(S/C)' }],
    [McduPseudoWaypointType.StepDescent, { type: McduPseudoWaypointType.StepDescent, mcduIdent: '(S/D)' }],
    [McduPseudoWaypointType.Decel, { type: McduPseudoWaypointType.Decel, mcduIdent: '(DECEL)', sequenceType: PseudoWaypointSequencingAction.EngageApproachPhase }],
    [McduPseudoWaypointType.Flap1, { type: McduPseudoWaypointType.Flap1, mcduIdent: '(FLAP1)' }],
    [McduPseudoWaypointType.Flap2, { type: McduPseudoWaypointType.Flap2, mcduIdent: '(FLAP2)' }],
]);

export class PseudoWaypoints implements GuidanceComponent {
    ndPseudoWaypoints: PseudoWaypoint[] = [];

    /**
     * Pseudowaypoints that are displayed on the flight plan page in the MCDU
     */
    mcduPseudoWaypoints: McduPseudoWaypoint[] = [];

    constructor(private guidanceController: GuidanceController) { }

    acceptVerticalProfile(verticalFlightPlan: VerticalFlightPlan) {
        if (DEBUG) {
            console.log('[FMS/PWP] Computed new pseudo waypoints because of new vertical profile.');
        }

        this.mcduPseudoWaypoints.length = 0;

        for (const { type, state, speedConstraint } of verticalFlightPlan.mcduPseudoWaypointRequests) {
            this.registerMcduPseudoWaypoint(type, state, speedConstraint);
        }
    }

    acceptMultipleLegGeometry(_geometry: Geometry) {
        if (DEBUG) {
            console.log('[FMS/PWP] Computed new pseudo waypoints because of new lateral geometry.');
        }
    }

    init() {
        console.log('[FMGC/Guidance] PseudoWaypoints initialized!');
    }

    update(_: number) {
        // Pass our pseudo waypoints to the GuidanceController
        this.guidanceController.currentPseudoWaypoints.length = 0;

        let idx = 0;
        for (const pseudoWaypoint of this.ndPseudoWaypoints) {
            const onPreviousLeg = pseudoWaypoint.alongLegIndex === this.guidanceController.activeLegIndex - 1;
            const onActiveLeg = pseudoWaypoint.alongLegIndex === this.guidanceController.activeLegIndex;
            const afterActiveLeg = pseudoWaypoint.alongLegIndex > this.guidanceController.activeLegIndex;
            const inSelectedHdg = !this.guidanceController.vnavDriver.isInManagedNav();

            // TODO we also consider the previous leg as active because we sequence Type I transitions at the same point
            // for both guidance and legs list. IRL, the display sequences after the guidance, which means the pseudo-waypoints
            // on the first half of the transition are considered on the active leg, whereas without this hack they are
            // on the previous leg by the time we try to re-add them to the list.

            // We only want to add the pseudo waypoint if it's after the active leg or it isn't yet passed
            if (
                inSelectedHdg
                || afterActiveLeg
                || (onPreviousLeg && this.guidanceController.displayActiveLegCompleteLegPathDtg > pseudoWaypoint.distanceFromLegTermination)
                || (onActiveLeg && this.guidanceController.activeLegCompleteLegPathDtg > pseudoWaypoint.distanceFromLegTermination)
            ) {
                this.guidanceController.currentPseudoWaypoints[++idx] = pseudoWaypoint;
            }
        }
    }

    /**
     * Notifies the FMS that a pseudo waypoint must be sequenced.
     *
     * This is to be sued by {@link GuidanceController} only.
     *
     * @param pseudoWaypoint the {@link PseudoWaypoint} to sequence.
     */
    sequencePseudoWaypoint(pseudoWaypoint: PseudoWaypoint): void {
        if (true) {
            console.log(`[FMS/PseudoWaypoints] Pseudo-waypoint '${pseudoWaypoint.ident}' sequenced.`);
        }

        switch (pseudoWaypoint.sequencingType) {
        case PseudoWaypointSequencingAction.TdReached:
            // TODO EFIS message;
            break;
        case PseudoWaypointSequencingAction.EngageApproachPhase:
            const apLateralMode = SimVar.GetSimVarValue('L:A32NX_FMA_LATERAL_MODE', 'Number');
            const agl = Simplane.getAltitudeAboveGround();

            if (agl < 9500 && (apLateralMode === LateralMode.NAV || apLateralMode === LateralMode.LOC_CPT || apLateralMode === LateralMode.LOC_TRACK)) {
                // Request APPROACH phase engagement for 5 seconds
                SimVar.SetSimVarValue('L:A32NX_FM_ENABLE_APPROACH_PHASE', 'Bool', true).then(() => [
                    setTimeout(() => {
                        SimVar.SetSimVarValue('L:A32NX_FM_ENABLE_APPROACH_PHASE', 'Bool', false);
                    }, 5_000),
                ]);
            }
            break;
        case PseudoWaypointSequencingAction.StepReached:
            break;
        default:
        }
    }

    private pointOnPath(
        path: Geometry,
        wptCount: number,
        distanceFromStart: NauticalMiles,
        debugString?: string,
    ): [lla: Coordinates, distanceFromLastFix: number, legIndex: number] | undefined {
        if (!distanceFromStart || distanceFromStart < 0) {
            if (VnavConfig.DEBUG_PROFILE) {
                console.warn('[FMS/PWP](pointFromEndOfPath) distanceFromEnd was negative or undefined');
            }

            return undefined;
        }

        let accumulator = 0;

        if (DEBUG) {
            console.log(`[FMS/PWP] Starting placement of PWP '${debugString}': dist: ${distanceFromStart.toFixed(2)}nm`);
        }

        for (let i = 1; i < wptCount; i++) {
            const leg = path.legs.get(i);

            if (!leg || leg.isNull) {
                continue;
            }

            let distanceInDiscontinuity = 0;
            const nextLeg = path.legs.get(i + 1);
            const previousLeg = path.legs.get(i - 1);

            if (leg instanceof XFLeg && leg.fix.endsInDiscontinuity && nextLeg instanceof XFLeg) {
                distanceInDiscontinuity = Avionics.Utils.computeGreatCircleDistance(leg.fix.infos.coordinates, nextLeg.fix.infos.coordinates);
            } else if (leg instanceof VMLeg && previousLeg instanceof XFLeg && nextLeg instanceof XFLeg) {
                distanceInDiscontinuity = Avionics.Utils.computeGreatCircleDistance(previousLeg.fix.infos.coordinates, nextLeg.fix.infos.coordinates);
            }

            accumulator += distanceInDiscontinuity;

            const inboundTrans = path.transitions.get(i - 1);
            const outboundTrans = path.transitions.get(i);

            const [inboundTransLength, legPartLength, outboundTransLength] = Geometry.completeLegPathLengths(
                leg,
                inboundTrans,
                (outboundTrans instanceof FixedRadiusTransition) ? outboundTrans : null,
            );

            const totalLegPathLength = inboundTransLength + legPartLength + outboundTransLength;
            accumulator += totalLegPathLength;

            if (DEBUG) {
                const inb = inboundTransLength.toFixed(2);
                const legd = legPartLength.toFixed(2);
                const outb = outboundTransLength.toFixed(2);
                const acc = accumulator.toFixed(2);

                console.log(`[FMS/PWP] Trying to place PWP '${debugString}' ${distanceFromStart.toFixed(2)} along leg #${i}; inb: ${inb}, leg: ${legd}, outb: ${outb}, acc: ${acc}`);
            }

            if (accumulator > distanceFromStart) {
                if (accumulator - distanceInDiscontinuity < distanceFromStart) {
                    // Points lies on discontinuity (on the direct line between the two fixes)
                    // In this case, we don't want to place the
                    return undefined;
                }

                const distanceFromLastFix = distanceFromStart - (accumulator - distanceInDiscontinuity - totalLegPathLength);

                let lla;
                if (distanceFromLastFix > inboundTransLength + legPartLength) {
                    // Point is in outbound transition segment
                    const distanceBeforeTerminator = outboundTransLength + legPartLength + inboundTransLength - distanceFromLastFix;

                    if (DEBUG) {
                        console.log(`[FMS/PWP] Placed PWP '${debugString}' on leg #${i} outbound segment (${distanceBeforeTerminator.toFixed(2)}nm before end)`);
                    }

                    lla = outboundTrans.getPseudoWaypointLocation(distanceBeforeTerminator);
                } else if (distanceFromLastFix >= inboundTransLength && distanceFromLastFix < inboundTransLength + legPartLength) {
                    // Point is in leg segment
                    const distanceBeforeTerminator = distanceFromLastFix - inboundTransLength - legPartLength;

                    if (DEBUG) {
                        console.log(`[FMS/PWP] Placed PWP '${debugString}' on leg #${i} leg segment (${distanceBeforeTerminator.toFixed(2)}nm before end)`);
                    }

                    lla = leg.getPseudoWaypointLocation(distanceBeforeTerminator);
                } else {
                    // Point is in inbound transition segment
                    const distanceBeforeTerminator = inboundTransLength - distanceFromLastFix;

                    if (DEBUG) {
                        console.log(`[FMS/PWP] Placed PWP '${debugString}' on leg #${i} inbound segment (${distanceBeforeTerminator.toFixed(2)}nm before end)`);
                    }

                    lla = inboundTrans.getPseudoWaypointLocation(distanceBeforeTerminator);
                }

                if (lla) {
                    return [lla, distanceFromLastFix, i];
                }

                console.error(`[FMS/PseudoWaypoints] Tried to place PWP ${debugString} on ${leg.repr}, but failed`);
                return undefined;
            }
        }

        if (DEBUG) {
            console.error(`[FMS/PseudoWaypoints] ${distanceFromStart.toFixed(2)}nm is larger than the total lateral path.`);
        }

        return undefined;
    }

    registerMcduPseudoWaypoint(type: McduPseudoWaypointType, state: AircraftState, speedConstraint?: SpeedConstraintPrediction) {
        const geometry = this.guidanceController.activeGeometry;
        const wptCount = this.guidanceController.flightPlanManager.getWaypointsCount();

        // Find position in flight plan
        const position = this.pointOnPath(geometry, wptCount, state.distanceFromStart);
        if (!position) {
            console.warn(`[FMS/VNAV] Could not place PWP of type ${type}: ${JSON.stringify(state)}`);
            return;
        }

        const [_, distanceFromLastFix, alongLegIndex] = position;

        const pwp = { ...pwpByType.get(type), alongLegIndex, distanceFromLastFix, prediction: this.formatPseudoWaypointPrediction(state, speedConstraint) };

        this.mcduPseudoWaypoints.push(pwp);
    }

    private formatPseudoWaypointPrediction(state: AircraftState, speedConstraint?: SpeedConstraintPrediction): VerticalPseudoWaypointPrediction {
        return {
            distanceFromStart: state.distanceFromStart,
            altitude: state.altitude,
            speed: state.speeds.speedTargetType === AccelFactorMode.CONSTANT_MACH ? state.speeds.mach : state.speeds.calibratedAirspeed,
            time: state.time,
            speedConstraint,
        };
    }
}

export type SpeedConstraintPrediction = {
    speed: Knots, isMet: boolean
}
