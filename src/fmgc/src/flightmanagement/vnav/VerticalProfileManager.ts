import { AccelFactorMode, FlapConf } from '@fmgc/guidance/vnav/common';
import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';
import { AircraftState, BuilderVisitor, McduProfile, SegmentContext, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
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

// Tasks: Compute a vertical profile for different use cases:
//  - A tactical profile used to display pseudowaypoints such as level off arrows on the ND
//  - A long term profile indicating predictions on the flight plan page
//  - Additional predictions, such as a profile predicting a climb/descent in Expedite mode. This is shown on the PERF page.
export class VerticalProfileManager {
    private verticalFlightPlan: VerticalFlightPlan;

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
            const profile = this.computeFlightPlanProfile();
            this.verticalFlightPlan.update(profile, geometry);
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

        const builder = new ProfileBuilder(this.getInitialStateForMcduProfile(climbWinds), FmgcFlightPhase.Takeoff);
        const visitor = new BuilderVisitor(builder);

        CpuTimer.reset();

        const profile = measurePerformance(() => new McduProfile(context, this.constraintReader, this.stepCoordinator), (time) => CpuTimer.initializationTime = time);
        measurePerformance(() => profile.accept(visitor), (time) => CpuTimer.visitorTime = time);

        return builder;
    }

    private getInitialStateForMcduProfile(climbWinds: HeadwindProfile): AircraftState {
        const { v2Speed, originAirfieldElevation, flightPhase, presentPosition, fuelOnBoard, zeroFuelWeight, takeoffFlapsSetting } = this.observer.get();

        const takeoffTas = this.atmosphericConditions.computeTasFromCas(originAirfieldElevation, v2Speed + 10);
        const headwind = climbWinds.getHeadwindComponent(0, originAirfieldElevation);

        if (flightPhase <= FmgcFlightPhase.Takeoff) {
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

        const speedTarget = Simplane.getAutoPilotAirspeedSelected()
            ? SimVar.GetSimVarValue('A32NX_AUTOPILOT_SPEED_SELECTED', 'knots')
            : SimVar.GetSimVarValue('A32NX_SPEEDS_MANAGED_ATHR', 'knots');

        return {
            altitude: presentPosition.alt,
            distanceFromStart: this.constraintReader.distanceToPresentPosition,
            time: 0,
            speeds: {
                // TODO: Take vars from FMGC instead of simvars
                calibratedAirspeed: SimVar.GetSimVarValue('AIRSPEED INDICATED', 'knots'),
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

    getWaypointPrediction(waypointIndex: number): VerticalWaypointPrediction | null {
        return this.verticalFlightPlan.getWaypointPrediction(waypointIndex);
    }

    get verticalFlightPlanForMcdu(): VerticalFlightPlan {
        return this.verticalFlightPlan;
    }
}
