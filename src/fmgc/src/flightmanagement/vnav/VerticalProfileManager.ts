import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { FlapConf } from '@fmgc/guidance/vnav/common';
import { HeadwindProfile } from '@fmgc/guidance/vnav/wind/HeadwindProfile';
import { AircraftState, BuilderVisitor, McduProfile, NodeContext, PrinterVisitor, ProfileBuilder } from '@fmgc/flightmanagement/vnav/segments';
import { ManagedDescentSegment } from '@fmgc/flightmanagement/vnav/segments/ManagedDescentSegment';
import { ApproachSegment } from '@fmgc/flightmanagement/vnav/segments/ApproachSegment';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { ConstraintReader } from '@fmgc/guidance/vnav/ConstraintReader';
import { NavHeadingProfile } from '@fmgc/guidance/vnav/wind/AircraftHeadingProfile';
import { WindProfileFactory } from '@fmgc/guidance/vnav/wind/WindProfileFactory';

// Tasks: Compute a vertical profile for different use cases:
//  - A tactical profile used to display pseudowaypoints such as level off arrows on the ND
//  - A long term profile indicating predictions on the flight plan page
//  - Additional predictions, such as a profile predicting a climb/descent in Expedite mode. This is shown on the PERF page.
export class VerticalProfileManager {
    constructor(
        private observer: VerticalProfileComputationParametersObserver,
        private atmosphericConditions: AtmosphericConditions,
        private constraintReader: ConstraintReader,
        private headingProfile: NavHeadingProfile,
        private windProfileFactory: WindProfileFactory,
    ) {
    }

    private computeFlightPlanProfile() {
        if (!this.observer.canComputeProfile()) {
            return;
        }

        const context = new NodeContext(
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

        const builder = new ProfileBuilder(initialState);

        const visitor = new BuilderVisitor(builder);
        const printer = new PrinterVisitor();

        const profile = new McduProfile(context, this.constraintReader);

        profile.accept(visitor);

        if (VnavConfig.DEBUG_PROFILE) {
            profile.accept(printer);
            console.log(visitor);
        }
    }

    private computeDescentProfileV2() {
        if (!this.observer.canComputeProfile()) {
            return;
        }

        const context = new NodeContext(
            this.atmosphericConditions,
            this.observer,
            new HeadwindProfile(this.windProfileFactory.getDescentWinds(), this.headingProfile),
        );

        const { v2Speed, destinationAirfieldElevation, approachSpeed } = this.observer.get();

        const initialState: AircraftState = {
            altitude: destinationAirfieldElevation + 50,
            distanceFromStart: this.constraintReader.totalFlightPlanDistance,
            time: 0,
            weight: this.observer.get().zeroFuelWeight + 2500,
            speed: approachSpeed,
            mach: this.atmosphericConditions.computeMachFromCas(destinationAirfieldElevation + 50, approachSpeed),
            trueAirspeed: this.atmosphericConditions.computeTasFromCas(destinationAirfieldElevation + 50, approachSpeed),
            config: {
                flapConfig: FlapConf.CONF_FULL,
                speedbrakesExtended: false,
                gearExtended: true,
            },
        };

        const builder = new ProfileBuilder(initialState);

        const visitor = new BuilderVisitor(builder);
        const printer = new PrinterVisitor();

        const approachProfile = new ApproachSegment(context, this.constraintReader);
        const descentProfile = new ManagedDescentSegment(context, this.constraintReader);

        approachProfile.accept(visitor);
        descentProfile.accept(visitor);

        if (VnavConfig.DEBUG_PROFILE) {
            // profile.accept(printer);
            console.log(visitor);

            SimVar.SetSimVarValue('L:A32NX_FM_VNAV_DEBUG_POINT', 'number', builder.lastState.distanceFromStart);
        }
    }
}
