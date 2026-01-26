// @ts-strict-ignore
// Copyright (c) 2021-2023 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

import {
  DescentAltitudeConstraint,
  GeographicCruiseStep,
  MaxAltitudeConstraint,
  MaxSpeedConstraint,
} from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { GuidanceController } from '@fmgc/guidance/GuidanceController';
import {
  MathUtils,
  ApproachType,
  ApproachWaypointDescriptor,
  WaypointConstraintType,
  ConstraintUtils,
} from '@flybywiresim/fbw-sdk';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { FlightPlanService } from '@fmgc/flightplanning/FlightPlanService';
import { isLeg } from '../../flightplanning/legs/FlightPlanLeg';
import { SegmentClass } from '../../flightplanning/segments/SegmentClass';

/**
 * This entire class essentially represents an interface to the flightplan.
 */
export class ConstraintReader {
  public climbAlitudeConstraints: MaxAltitudeConstraint[] = [];

  public descentAltitudeConstraints: DescentAltitudeConstraint[] = [];

  public climbSpeedConstraints: MaxSpeedConstraint[] = [];

  public descentSpeedConstraints: MaxSpeedConstraint[] = [];

  public cruiseSteps: GeographicCruiseStep[] = [];

  public totalFlightPlanDistance = 0;

  public get distanceToEnd(): NauticalMiles {
    if (VnavConfig.ALLOW_DEBUG_PARAMETER_INJECTION) {
      return SimVar.GetSimVarValue('L:A32NX_FM_VNAV_DEBUG_DISTANCE_TO_END', 'nautical miles');
    }

    return this.guidanceController.getAlongTrackDistanceToDestination();
  }

  // If you change this property here, make sure you also reset it properly in `reset`
  public finalDescentAngle = -3;

  // If you change this property here, make sure you also reset it properly in `reset`
  public fafDistanceToEnd =
    1000 / Math.tan(-this.finalDescentAngle * MathUtils.DEGREES_TO_RADIANS) / MathUtils.FEET_TO_NAUTICAL_MILES;

  public get distanceToPresentPosition(): NauticalMiles {
    return this.totalFlightPlanDistance - this.distanceToEnd;
  }

  public finalAltitude: Feet = 50;

  public iafDistanceToEnd = 0;

  public fdpDistanceToEnd = this.fafDistanceToEnd;

  constructor(
    private flightPlanService: FlightPlanService,
    private guidanceController: GuidanceController,
  ) {
    this.reset();
  }

  updateFlightPlan() {
    this.reset();
    this.updateDistancesToEnd();
    this.updateFinalAltitude();

    let maxSpeed = Infinity;

    const plan = this.flightPlanService.active;
    let platformAltitude: number | undefined = undefined;

    for (let i = 0; i < plan.firstMissedApproachLegIndex; i++) {
      const leg = plan.elementAt(i);

      if (leg.isDiscontinuity === true) {
        continue;
      }

      const legDistanceToEnd = leg.calculated?.cumulativeDistanceToEndWithTransitions ?? this.totalFlightPlanDistance;
      const legDistanceFromStart = leg.calculated?.cumulativeDistanceWithTransitions ?? 0;

      if (leg.cruiseStep && !leg.cruiseStep.isIgnored) {
        if (i >= plan.activeLegIndex) {
          const { waypointIndex, toAltitude, distanceBeforeTermination } = leg.cruiseStep;

          this.cruiseSteps.push({
            distanceFromStart: legDistanceFromStart - distanceBeforeTermination,
            toAltitude,
            waypointIndex,
            isIgnored: false,
          });
        }
      }

      const altConstraint = leg.altitudeConstraint;
      const speedConstraint = leg.speedConstraint;

      if (leg.constraintType === WaypointConstraintType.CLB) {
        if (altConstraint) {
          switch (altConstraint.altitudeDescriptor) {
            case '@': // at alt 1
            case '-': // at or below alt 1
            case 'B': // between alt 1 and alt 2
              this.climbAlitudeConstraints.push({
                distanceFromStart: legDistanceFromStart,
                maxAltitude: altConstraint.altitude1,
              });
              break;
            default:
            // not constraining
          }
        }

        if (speedConstraint && speedConstraint.speed > 100) {
          this.climbSpeedConstraints.push({
            distanceFromStart: legDistanceFromStart,
            maxSpeed: speedConstraint.speed,
          });
        }
      } else if (leg.constraintType === WaypointConstraintType.DES) {
        if (altConstraint) {
          switch (altConstraint.altitudeDescriptor) {
            case '@': // at alt 1
            case '+': // at or above alt 1
            case 'I': // alt1 is at for FACF, Alt2 is glidelope intercept
            case 'J': // alt1 is at or above for FACF, Alt2 is glideslope intercept
            case 'V': // alt1 is procedure alt for step-down, Alt2 is at alt for vertical path angle
            case 'X': // alt 1 is at, Alt 2 is on the vertical angle
            case '-': // at or below alt 1
            case 'B': // between alt 1 and alt 2
              this.descentAltitudeConstraints.push({
                distanceFromStart: legDistanceFromStart,
                constraint: altConstraint,
                leg,
              });
              break;
            default:
            // not constraining
          }
        }

        if (speedConstraint && speedConstraint.speed > 100) {
          maxSpeed = Math.min(maxSpeed, speedConstraint.speed);

          this.descentSpeedConstraints.push({
            distanceFromStart: legDistanceFromStart,
            maxSpeed,
          });
        }
      }

      if (i === plan.destinationLegIndex && leg.definition.verticalAngle) {
        this.finalDescentAngle = leg.definition.verticalAngle;

        if (platformAltitude !== undefined) {
          this.fdpDistanceToEnd =
            (this.finalAltitude - platformAltitude) /
            MathUtils.FEET_TO_NAUTICAL_MILES /
            Math.tan(this.finalDescentAngle * MathUtils.DEGREES_TO_RADIANS);
        }
      }

      const nextElement = plan.maybeElementAt(i + 1);
      if (platformAltitude === undefined && isLeg(nextElement) && nextElement.definition.verticalAngle !== undefined) {
        platformAltitude = ConstraintUtils.minimumAltitude(leg.altitudeConstraint);
      }

      if (leg.definition.approachWaypointDescriptor === ApproachWaypointDescriptor.FinalApproachFix) {
        this.fafDistanceToEnd = legDistanceToEnd;
      }
    }

    this.updateIafDistance();
  }

  private updateIafDistance(): void {
    const plan = this.flightPlanService.active;

    const iafLegIndex = plan.allLegs.findIndex(
      (el) => isLeg(el) && el.definition.approachWaypointDescriptor === ApproachWaypointDescriptor.InitialApproachFix,
    );
    const fafLegIndex = plan.allLegs.findIndex(
      (el) => isLeg(el) && el.definition.approachWaypointDescriptor === ApproachWaypointDescriptor.FinalApproachFix,
    );
    const lastEnrouteLegIndex = plan.allLegs.reduce(
      (acc, el, i) => (isLeg(el) && el.segment.class === SegmentClass.Enroute ? Math.max(acc, i) : acc),
      -1,
    );

    // We identify the IAF directly
    if (iafLegIndex >= 0) {
      const iaf = plan.legElementAt(iafLegIndex);
      this.iafDistanceToEnd = iaf.calculated?.cumulativeDistanceToEndWithTransitions ?? 0;

      return;
    }

    // Fallback if we perform a direct to past the IAF, in which case the IAF leg would be removed from the flight plan
    if (lastEnrouteLegIndex >= 0) {
      const firstApproachLeg = plan.legElementAt(lastEnrouteLegIndex);
      this.iafDistanceToEnd = firstApproachLeg.calculated?.cumulativeDistanceToEndWithTransitions ?? 0;

      return;
    }

    if (fafLegIndex >= 0) {
      const firstApproachLeg = plan.legElementAt(fafLegIndex);
      this.iafDistanceToEnd = firstApproachLeg.calculated?.cumulativeDistanceToEndWithTransitions ?? 0;

      return;
    }

    this.iafDistanceToEnd = 0;
  }

  private updateFinalAltitude(): void {
    const plan = this.flightPlanService.active;

    const approach = plan.approach;

    // Check if we have a procedure loaded from which we can extract the final altitude
    if (approach && approach.type !== ApproachType.Unknown) {
      for (const leg of approach.legs) {
        if (
          leg.approachWaypointDescriptor === ApproachWaypointDescriptor.MissedApproachPoint &&
          Number.isFinite(leg.altitude1)
        ) {
          this.finalAltitude = leg.altitude1;

          return;
        }
      }
    }

    // Check if we only have a runway loaded. In this case, take the threshold elevation.
    const runway = plan.destinationRunway;
    if (runway && Number.isFinite(runway.thresholdLocation.alt)) {
      this.finalAltitude = runway.thresholdLocation.alt + 50;

      return;
    }

    // Check if we only have a destination airport loaded. In this case, take the airport elevation.
    const destinationAirport = plan.destinationAirport;
    // TODO: I think selecting an approach and then deleting it will cause destinationAirport.infos.coordinates.alt to be zero.
    if (destinationAirport && Number.isFinite(destinationAirport.location.alt)) {
      this.finalAltitude = destinationAirport.location.alt + 50;

      return;
    }

    // Last resort. Not sure how we'd get here.
    // If I do change this, I should probably change it in the reset() function.
    this.finalAltitude = 50;
  }

  reset() {
    this.climbAlitudeConstraints = [];
    this.descentAltitudeConstraints = [];
    this.climbSpeedConstraints = [];
    this.descentSpeedConstraints = [];
    this.cruiseSteps = [];

    this.totalFlightPlanDistance = 0;
    this.finalDescentAngle = -3;
    this.fafDistanceToEnd =
      1000 / Math.tan(-this.finalDescentAngle * MathUtils.DEGREES_TO_RADIANS) / MathUtils.FEET_TO_NAUTICAL_MILES;
    this.iafDistanceToEnd = this.fafDistanceToEnd;
    this.fdpDistanceToEnd = this.fafDistanceToEnd;
    this.finalAltitude = 50;
  }

  private updateDistancesToEnd() {
    this.totalFlightPlanDistance = 0;

    const plan = this.flightPlanService.active;

    for (let i = Math.max(0, plan.activeLegIndex - 1); i < plan.firstMissedApproachLegIndex; i++) {
      const leg = plan.maybeElementAt(i);
      if (!leg || leg.isDiscontinuity === true || !leg.calculated) {
        continue;
      }

      this.totalFlightPlanDistance = leg.calculated.cumulativeDistanceToEndWithTransitions;
      return;
    }
  }

  ignoreCruiseStep(waypointIndex: number) {
    const leg = this.flightPlanService.active.maybeElementAt(waypointIndex);

    if (leg?.isDiscontinuity === false && leg?.cruiseStep) {
      leg.cruiseStep.isIgnored = true;
    }
  }
}
