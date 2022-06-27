import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { FlapConf } from '@fmgc/guidance/vnav/common';
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
        const context = new SegmentContext(
            this.atmosphericConditions,
            this.observer,
            new HeadwindProfile(this.windProfileFactory.getClimbWinds(), this.headingProfile),
        );

        const { v2Speed, originAirfieldElevation } = this.observer.get();

        const initialState: AircraftState = {
            altitude: originAirfieldElevation,
            distanceFromStart: 0,
            time: 0,
            weight: this.observer.get().fuelOnBoard + this.observer.get().zeroFuelWeight,
            speed: v2Speed + 10,
            mach: this.atmosphericConditions.computeMachFromCas(originAirfieldElevation, v2Speed + 10),
            trueAirspeed: this.atmosphericConditions.computeTasFromCas(originAirfieldElevation, v2Speed + 10),
            config: {
                flapConfig: FlapConf.CONF_1,
                speedbrakesExtended: false,
                gearExtended: false,
            },
        };

        const builder = new ProfileBuilder(initialState, FmgcFlightPhase.Takeoff);
        const visitor = new BuilderVisitor(builder);

        CpuTimer.reset();

        const profile = measurePerformance(() => new McduProfile(context, this.constraintReader, this.stepCoordinator), (time) => CpuTimer.initializationTime = time);
        measurePerformance(() => profile.accept(visitor), (time) => CpuTimer.visitorTime = time);

        if (VnavConfig.DEBUG_PROFILE) {
            console.log(visitor);
        }

        return builder;
    }

    getWaypointPrediction(waypointIndex: number): VerticalWaypointPrediction | null {
        return this.verticalFlightPlan.getWaypointPrediction(waypointIndex);
    }

    get verticalFlightPlanForMcdu(): VerticalFlightPlan {
        return this.verticalFlightPlan;
    }
}
