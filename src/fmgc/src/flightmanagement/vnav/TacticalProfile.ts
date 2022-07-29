import { ClimbProfileRequest } from '@fmgc/flightmanagement/vnav/ClimbProfileRequest';
import {
    ClimbThrustSetting,
    constantPitchPropagator,
    constantThrustPropagator,
    accelerationPropagator,
    speedChangePropagator,
    FlightPathAnglePitchTarget,
    PropagatorOptions,
    VerticalSpeedPitchTarget,
    IntegrationPropagator,
} from '@fmgc/flightmanagement/vnav/integrators';
import { SegmentContext } from '@fmgc/flightmanagement/vnav/segments';
import { ClimbSegment } from '@fmgc/flightmanagement/vnav/segments/climb/ClimbSegment';
import { TakeoffSegment } from '@fmgc/flightmanagement/vnav/segments/climb/TakeoffSegment';
import { CruiseAndDescentSegment } from '@fmgc/flightmanagement/vnav/segments/CruiseAndDescentSegment';
import { ProfileSegment } from '@fmgc/flightmanagement/vnav/segments/ProfileSegment';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { VerticalProfileComputationParameters } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { WindProfileType } from '@fmgc/guidance/vnav/wind/WindProfile';
import { ArmedLateralMode, ArmedVerticalMode, isArmed, LateralMode, VerticalMode } from '@shared/autopilot';
import { FmgcFlightPhase } from '@shared/flightphase';

export class TacticalProfile extends ProfileSegment {
    constructor(private context: SegmentContext, constraints: ConstraintReader, stepCoordinator: StepCoordinator) {
        super();

        const parameters = this.context.observer.get();

        const [climbPropagator, accelerationPropagator] = this.getPropagators(context, parameters);
        const climbProfileRequest = new ClimbProfileRequest()
            .withClimbPropagator(climbPropagator)
            .withAccelerationPropagator(accelerationPropagator)
            .withMaxSpeed(this.getMaxSpeed(parameters))
            .withMaxMach(this.getMaxMach(parameters));

        if (this.shouldObeySpeedConstraints(parameters)) {
            climbProfileRequest.withSpeedConstraints(constraints.climbSpeedConstraints);
        }

        if (this.shouldObeyAltitudeConstraints(parameters)) {
            climbProfileRequest.withAltitudeConstraints(constraints.climbAlitudeConstraints);
        }

        if (this.shouldObeySpeedLimit(parameters)) {
            climbProfileRequest.withSpeedLimit(parameters.climbSpeedLimit);
        }

        switch (parameters.flightPhase) {
        case FmgcFlightPhase.GoAround:
        case FmgcFlightPhase.Takeoff:
            this.children.push(new TakeoffSegment(context));
        case FmgcFlightPhase.Climb:
            this.children.push(new ClimbSegment(context, climbProfileRequest));
            break;
        case FmgcFlightPhase.Cruise:
        case FmgcFlightPhase.Descent:
        case FmgcFlightPhase.Approach:
            this.children.push(new CruiseAndDescentSegment(context, constraints, stepCoordinator));
            break;
        default:
            break;
        }
    }

    private getMaxSpeed(parameters: VerticalProfileComputationParameters): Knots {
        if (parameters.fcuExpediteModeActive) {
            return parameters.cleanSpeed;
        } if (this.isSpeedAutoControlActive(parameters.fcuSpeed)) {
            return parameters.managedClimbSpeed;
        }
        // If we have a Mach number selected, we set the max speed to zero,
        return parameters.fcuSpeed < 1 ? 0 : parameters.fcuSpeed;
    }

    private getMaxMach(parameters: VerticalProfileComputationParameters): Mach {
        if (parameters.fcuSpeed >= 0.1 && parameters.fcuSpeed < 1) {
            return parameters.fcuSpeed;
        }

        // TODO: Use manual crossover altitude law if there is a selected speed selected.
        return parameters.managedClimbSpeedMach;
    }

    private getPropagators(context: SegmentContext, parameters: VerticalProfileComputationParameters): [IntegrationPropagator, IntegrationPropagator] {
        const climbThrust = new ClimbThrustSetting(context.atmosphericConditions);
        const propagatorOptions: PropagatorOptions = { stepSize: 5, windProfileType: WindProfileType.Climb };

        if (parameters.fcuExpediteModeActive) {
            return [constantThrustPropagator(climbThrust, context, propagatorOptions), accelerationPropagator(climbThrust, context, propagatorOptions)];
        }

        switch (parameters.fcuVerticalMode) {
        case VerticalMode.VS:
            const vsPitchTarget = new VerticalSpeedPitchTarget(parameters.fcuVerticalSpeed);
            return [constantPitchPropagator(vsPitchTarget, context, propagatorOptions), speedChangePropagator(context, climbThrust, vsPitchTarget, true, propagatorOptions)];
        case VerticalMode.FPA:
            const fpaPitchTarget = new FlightPathAnglePitchTarget(parameters.fcuFlightPathAngle);
            return [constantPitchPropagator(fpaPitchTarget, context, propagatorOptions), speedChangePropagator(context, climbThrust, fpaPitchTarget, true, propagatorOptions)];
        default: // For anything but the modes above, we assume we'e in a managed mode and use propagators for CLB mode
            return [constantThrustPropagator(climbThrust, context, propagatorOptions), accelerationPropagator(climbThrust, context, propagatorOptions)];
        }
    }

    private shouldObeySpeedLimit(parameters: VerticalProfileComputationParameters): boolean {
        return this.isSpeedAutoControlActive(parameters.fcuSpeed)
            && !parameters.fcuExpediteModeActive;
    }

    private shouldObeySpeedConstraints(parameters: VerticalProfileComputationParameters): boolean {
        const { fcuSpeed, fcuLateralMode, fcuArmedLateralMode, fcuExpediteModeActive } = parameters;

        return this.isLatAutoControlActive(fcuLateralMode, fcuArmedLateralMode)
            && this.isSpeedAutoControlActive(fcuSpeed)
            && !fcuExpediteModeActive;
    }

    private shouldObeyAltitudeConstraints(parameters: VerticalProfileComputationParameters): boolean {
        const { fcuArmedLateralMode, fcuArmedVerticalMode, fcuVerticalMode } = parameters;

        const verticalModesToApplyAltitudeConstraintsFor = [
            VerticalMode.CLB,
            VerticalMode.ALT,
            VerticalMode.ALT_CPT,
            VerticalMode.ALT_CST_CPT,
            VerticalMode.ALT_CST,
        ];

        return isArmed(fcuArmedVerticalMode, ArmedVerticalMode.CLB)
            || isArmed(fcuArmedLateralMode, ArmedLateralMode.NAV)
            || verticalModesToApplyAltitudeConstraintsFor.includes(fcuVerticalMode);
    }

    private isSpeedAutoControlActive(fcuSpeed: Knots | Mach | -1) {
        return fcuSpeed <= 0;
    }

    private isLatAutoControlActive(fcuLateralMode: LateralMode, fcuArmedLateralMode: ArmedLateralMode): boolean {
        return fcuLateralMode === LateralMode.NAV || isArmed(fcuArmedLateralMode, ArmedLateralMode.NAV);
    }
}
