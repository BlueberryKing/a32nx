import { AccelFactorMode, FlapConf } from '@fmgc/guidance/vnav/common';
import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';
import { AircraftState, BuilderVisitor, McduProfile, SegmentContext, ProfileBuilder, NdPseudoWaypointRequest } from '@fmgc/flightmanagement/vnav/segments';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { NavHeadingProfile } from '@fmgc/guidance/vnav/wind/AircraftHeadingProfile';
import { WindProfileFactory } from '@fmgc/guidance/vnav/wind/WindProfileFactory';
import { StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { Geometry } from '@fmgc/guidance/Geometry';
import { VerticalFlightPlan, VerticalWaypointPrediction } from '@fmgc/flightmanagement/vnav/VerticalFlightPlan';
import { FlightPlanManager } from '@shared/flightplan';
import { FmgcFlightPhase } from '@shared/flightphase';
import { CpuTimer, measurePerformance } from '@fmgc/flightmanagement/vnav/common/profiling';
import { HeadwindRepository } from '@fmgc/guidance/vnav/wind/HeadwindRepository';
import { TacticalProfile } from '@fmgc/flightmanagement/vnav/TacticalProfile';
import { NdPseudoWaypointType } from '@fmgc/guidance/lnav/PseudoWaypoints';
import { ArmedLateralMode, ArmedVerticalMode, isArmed, LateralMode, VerticalMode } from '@shared/autopilot';
import { Interpolator } from '@fmgc/flightmanagement/vnav/common/Interpolator';

// Tasks: Compute a vertical profile for different use cases:
//  - A tactical profile used to display pseudowaypoints such as level off arrows on the ND
//  - A long term profile indicating predictions on the flight plan page
//  - Additional predictions, such as a profile predicting a climb/descent in Expedite mode. This is shown on the PERF page.
export class VerticalProfileManager {
    private verticalFlightPlan: VerticalFlightPlan;

    // TODO: Remove
    ndPseudoWaypointRequests: NdPseudoWaypointRequest[] = [];

    constructor(
        private observer: VerticalProfileComputationParametersObserver,
        private atmosphericConditions: AtmosphericConditions,
        private constraintReader: ConstraintReader,
        private headingProfile: NavHeadingProfile,
        private windProfileFactory: WindProfileFactory,
        private stepCoordinator: StepCoordinator,
        flightPlanManager: FlightPlanManager,
    ) {
        this.verticalFlightPlan = new VerticalFlightPlan(flightPlanManager, this.observer, atmosphericConditions);
    }

    update(geometry: Geometry) {
        if (this.observer.canComputeProfile()) {
            const mcduProfile = this.computeFlightPlanProfile();
            this.verticalFlightPlan.update(mcduProfile, geometry);

            const tacticalProfile = this.computeTacticalProfile();
            this.updateNdPseudoWaypointRequests(tacticalProfile, mcduProfile);
        }
    }

    private computeFlightPlanProfile(): ProfileBuilder {
        const climbWinds = new HeadwindProfile(this.windProfileFactory.getClimbWinds(), this.headingProfile);

        const windRepository = new HeadwindRepository(
            climbWinds,
            new HeadwindProfile(this.windProfileFactory.getCruiseWinds(), this.headingProfile),
            new HeadwindProfile(this.windProfileFactory.getDescentWinds(), this.headingProfile),
        );

        const context = new SegmentContext(
            this.atmosphericConditions,
            this.observer,
            windRepository,
        );

        const builder = new ProfileBuilder(this.getInitialStateForProfile(climbWinds), FmgcFlightPhase.Takeoff);
        const visitor = new BuilderVisitor(builder);

        CpuTimer.reset();

        const profile = measurePerformance(() => new McduProfile(context, this.constraintReader, this.stepCoordinator), (time) => CpuTimer.initializationTime = time);
        measurePerformance(() => profile.accept(visitor), (time) => CpuTimer.visitorTime = time);

        return builder;
    }

    private computeTacticalProfile(): ProfileBuilder {
        const climbWinds = new HeadwindProfile(this.windProfileFactory.getClimbWinds(), this.headingProfile);

        const windRepository = new HeadwindRepository(
            climbWinds,
            new HeadwindProfile(this.windProfileFactory.getCruiseWinds(), this.headingProfile),
            new HeadwindProfile(this.windProfileFactory.getDescentWinds(), this.headingProfile),
        );

        const context = new SegmentContext(
            this.atmosphericConditions,
            this.observer,
            windRepository,
        );

        const builder = new ProfileBuilder(this.getInitialStateForProfile(climbWinds), FmgcFlightPhase.Takeoff);
        const visitor = new BuilderVisitor(builder);

        CpuTimer.reset();

        const profile = new TacticalProfile(context, this.constraintReader, this.stepCoordinator);
        profile.accept(visitor);

        return builder;
    }

    private getInitialStateForProfile(climbWinds: HeadwindProfile): AircraftState {
        const { v2Speed, originAirfieldElevation, flightPhase, presentPosition, fuelOnBoard, zeroFuelWeight, takeoffFlapsSetting } = this.observer.get();

        const cas = SimVar.GetSimVarValue('AIRSPEED INDICATED', 'knots');

        // I am not sure if this logic is correct
        if (flightPhase < FmgcFlightPhase.Takeoff || cas < 100) {
            const takeoffTas = this.atmosphericConditions.computeTasFromCas(originAirfieldElevation, v2Speed + 10);
            const headwind = climbWinds.getHeadwindComponent(0, originAirfieldElevation);

            return {
                altitude: originAirfieldElevation,
                distanceFromStart: 0,
                time: 0,
                speeds: {
                    calibratedAirspeed: v2Speed + 10,
                    mach: this.atmosphericConditions.computeMachFromCas(originAirfieldElevation, v2Speed + 10),
                    trueAirspeed: takeoffTas,
                    groundSpeed: takeoffTas - headwind.value,
                    speedTarget: v2Speed + 10,
                    speedTargetType: AccelFactorMode.CONSTANT_CAS,
                },
                weight: fuelOnBoard + zeroFuelWeight,
                config: {
                    flapConfig: takeoffFlapsSetting,
                    speedbrakesExtended: false,
                    gearExtended: true,
                },
            };
        }

        const speedTarget = Math.round(Simplane.getAutoPilotAirspeedSelected()
            ? SimVar.GetSimVarValue('L:A32NX_AUTOPILOT_SPEED_SELECTED', 'knots')
            : SimVar.GetSimVarValue('L:A32NX_SPEEDS_MANAGED_ATHR', 'knots'));

        return {
            altitude: presentPosition.alt,
            distanceFromStart: this.constraintReader.distanceToPresentPosition,
            time: 0,
            speeds: {
                // TODO: Take vars from FMGC instead of simvars
                calibratedAirspeed: cas,
                mach: SimVar.GetSimVarValue('AIRSPEED MACH', 'number'),
                trueAirspeed: SimVar.GetSimVarValue('AIRSPEED TRUE', 'knots'),
                groundSpeed: SimVar.GetSimVarValue('GPS GROUND SPEED', 'knots'),
                speedTarget,
                speedTargetType: speedTarget < 1 ? AccelFactorMode.CONSTANT_MACH : AccelFactorMode.CONSTANT_CAS,
            },
            weight: fuelOnBoard + zeroFuelWeight,
            config: {
                flapConfig: FlapConf.CLEAN, // TODO: Take current flaps setting for this
                speedbrakesExtended: false,
                gearExtended: false,
            },
        };
    }

    private updateNdPseudoWaypointRequests(tacticalProfile: ProfileBuilder, mcduProfile: ProfileBuilder) {
        const {
            fcuAltitude,
            fcuArmedLateralMode,
            fcuArmedVerticalMode,
            fcuExpediteModeActive,
            fcuFlightPathAngle,
            fcuLateralMode,
            fcuSpeed,
            fcuVerticalMode,
            fcuVerticalSpeed,
            flightPhase,
            presentPosition,
        } = this.observer.get();
        const existingPseudoWaypoints = new Set<NdPseudoWaypointType>();

        this.ndPseudoWaypointRequests = [];

        for (const pwp of tacticalProfile.ndPseudoWaypointRequests) {
            if (pwp.type === NdPseudoWaypointType.Level1Climb) {
                const isBelowAircraftAlt = presentPosition.alt > pwp.state.altitude - 100;
                const isAboveFcuAltitude = Math.round(pwp.state.altitude) > Math.round(fcuAltitude);
                const isAltitudeBeingCaptured = VerticalMode.ALT_CPT && Math.round(pwp.state.altitude) === Math.round(fcuAltitude)
                 || fcuVerticalMode === VerticalMode.ALT_CST_CPT && Math.round(pwp.state.altitude) === SimVar.GetSimVarValue('L:A32NX_FG_ALTITUDE_CONSTRAINT', 'ft');

                if (isBelowAircraftAlt || isAboveFcuAltitude || isAltitudeBeingCaptured) {
                    continue;
                } else if (Math.round(pwp.state.altitude) === Math.round(fcuAltitude)) {
                    pwp.type = NdPseudoWaypointType.Level2Climb;
                }
            } else if (pwp.type === NdPseudoWaypointType.StartOfClimb1) {
                if (Math.round(pwp.state.altitude) < Math.round(fcuAltitude) && isArmed(fcuArmedVerticalMode, ArmedVerticalMode.CLB)
                    || existingPseudoWaypoints.has(NdPseudoWaypointType.Level1Climb)) {
                    pwp.type = NdPseudoWaypointType.StartOfClimb2;
                }
            } else if (pwp.type === NdPseudoWaypointType.SpeedChange1 && (!isSpeedAutoControlActive(fcuSpeed) || fcuExpediteModeActive)) {
                continue;
            }

            if (existingPseudoWaypoints.has(pwp.type)) {
                continue;
            }

            existingPseudoWaypoints.add(pwp.type);
            this.ndPseudoWaypointRequests.push(pwp);
        }

        // Use waypoints of MCDU profile (i.e managed modes) for the upcoming flight phases
        const checkpointsOfCurrentPhase = mcduProfile.checkpointsOfPhase(Math.max(flightPhase, FmgcFlightPhase.Climb));
        const minDistance = checkpointsOfCurrentPhase.length > 0 ? checkpointsOfCurrentPhase[checkpointsOfCurrentPhase.length - 1].distanceFromStart : 0;
        for (const pwp of mcduProfile.ndPseudoWaypointRequests) {
            if (existingPseudoWaypoints.has(pwp.type) || pwp.state.distanceFromStart <= minDistance) {
                continue;
            }

            existingPseudoWaypoints.add(pwp.type);
            this.ndPseudoWaypointRequests.push(pwp);
        }

        const tacticalProfileCheckpoints = tacticalProfile.allCheckpointsWithPhase;

        // Dynamically compute level offs
        if (isInClimbMode(fcuVerticalMode, fcuVerticalSpeed, fcuFlightPathAngle) || isClimbArmed(fcuArmedVerticalMode)) {
            const levelOffDistance = Interpolator.interpolateDistanceAtAltitude(tacticalProfileCheckpoints, fcuAltitude);
            const state = Interpolator.interpolateEverythingFromStart(tacticalProfileCheckpoints, levelOffDistance);

            this.ndPseudoWaypointRequests.push({ type: isInManagedNav(fcuLateralMode, fcuArmedLateralMode) ? NdPseudoWaypointType.Level2Climb : NdPseudoWaypointType.Level3Climb, state });
        } else if (isInDescentMode(fcuVerticalMode, fcuVerticalSpeed, fcuFlightPathAngle) || isDescentArmed(fcuArmedVerticalMode)) {
            const levelOffDistance = Interpolator.interpolateDistanceAtAltitude(tacticalProfileCheckpoints, fcuAltitude);
            const state = Interpolator.interpolateEverythingFromStart(tacticalProfileCheckpoints, levelOffDistance);

            this.ndPseudoWaypointRequests.push({ type: isInManagedNav(fcuLateralMode, fcuArmedLateralMode) ? NdPseudoWaypointType.Level2Descent : NdPseudoWaypointType.Level3Descent, state });
        }
    }

    getWaypointPrediction(waypointIndex: number): VerticalWaypointPrediction | null {
        return this.verticalFlightPlan.getWaypointPrediction(waypointIndex);
    }

    get verticalFlightPlanForMcdu(): VerticalFlightPlan {
        return this.verticalFlightPlan;
    }
}

const verticalModesDescent = new Set([VerticalMode.OP_DES, VerticalMode.DES]);
const verticalModesClimb = new Set([VerticalMode.OP_CLB, VerticalMode.CLB, VerticalMode.SRS, VerticalMode.SRS_GA]);

function isInClimbMode(verticalMode: VerticalMode, verticalSpeed: FeetPerMinute, flightPathAngle: Degrees): boolean {
    return verticalModesClimb.has(verticalMode)
        || verticalMode === VerticalMode.VS && verticalSpeed > 0
        || verticalMode === VerticalMode.FPA && flightPathAngle > 0;
}

function isClimbArmed(armedVerticalMode: ArmedVerticalMode): boolean {
    return isArmed(armedVerticalMode, ArmedVerticalMode.CLB);
}

function isDescentArmed(armedVerticalMode: ArmedVerticalMode): boolean {
    return isArmed(armedVerticalMode, ArmedVerticalMode.DES);
}

function isInDescentMode(verticalMode: VerticalMode, verticalSpeed: FeetPerMinute, flightPathAngle: Degrees): boolean {
    return verticalModesDescent.has(verticalMode)
        || verticalMode === VerticalMode.VS && verticalSpeed < 0
        || verticalMode === VerticalMode.FPA && flightPathAngle < 0;
}

function isInManagedNav(fcuLateralMode: LateralMode, fcuArmedLateralMode: ArmedLateralMode): boolean {
    return fcuLateralMode === LateralMode.NAV || isArmed(fcuArmedLateralMode, ArmedLateralMode.NAV);
}

function isSpeedAutoControlActive(fcuSpeed: Knots | Mach | -1) {
    return fcuSpeed <= 0;
}
