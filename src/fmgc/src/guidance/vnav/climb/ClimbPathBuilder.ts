import { Geometry } from '@fmgc/guidance/Geometry';
import { Predictions, StepResults } from '../Predictions';
import { ClimbProfileBuilderResult } from './ClimbProfileBuilderResult';
import { Common, FlapConf } from '../common';
import { FlightPlanManager } from '@fmgc/wtsdk';
import { Fmgc } from '@fmgc/guidance/GuidanceController';
import { EngineModel } from '../EngineModel';

export class ClimbPathBuilder {
    private static TONS_TO_POUNDS = 2240;

    private isaDev: number;
    private accelerationAltitude: number;

    private tropoPause: number

    constructor(private fmgc: Fmgc) {
        this.isaDev = 8
        this.accelerationAltitude = SimVar.GetSimVarValue('L:AIRLINER_ACC_ALT', 'number')

        this.tropoPause = fmgc.getTropoPause() ?? 36090;
    }

    computeClimbPath(geometry: Geometry): ClimbProfileBuilderResult {
        let totalDistance = 0;
        for (const [i, leg] of geometry.legs.entries()) {
            console.log(`Distance: ${totalDistance} + ${leg.distance} = ${totalDistance + leg.distance}`)
            totalDistance += leg.distance
        }

        const takeoffRollDistance = this.computeTakeOffRollDistance();
        const { distanceTraveled: distanceTraveledSrs } = this.computeTakeoffStepPrediction()

        const climbSpeed = 250;
        const cruiseAltitude = SimVar.GetGameVarValue("AIRCRAFT CRUISE ALTITUDE", "feet");
        console.log(`cruiseAltitude: ${JSON.stringify(cruiseAltitude)}`);

        const midwayAltitudeClimb = (cruiseAltitude + this.accelerationAltitude) / 2;
        console.log(`midwayAltitudeClimb: ${JSON.stringify(midwayAltitudeClimb)}`);

        const machClimb = this.computeMachFromCas(midwayAltitudeClimb, this.isaDev, climbSpeed);
        console.log(`machClimb: ${JSON.stringify(machClimb)}`);

        const estimatedTat = this.totalAirTemperatureFromMach(midwayAltitudeClimb, machClimb)
        console.log(`estimatedTat: ${JSON.stringify(estimatedTat)}`);

        const commandedN1Climb = this.getClimbThrustN1Limit(estimatedTat, midwayAltitudeClimb);
        console.log(`commandedN1Climb: ${JSON.stringify(commandedN1Climb)}`);

        const { distanceTraveled: distanceTraveledClb } = Predictions.altitudeStep(this.accelerationAltitude, cruiseAltitude - this.accelerationAltitude, climbSpeed, machClimb, commandedN1Climb, this.fmgc.getZeroFuelWeight() * ClimbPathBuilder.TONS_TO_POUNDS, this.fmgc.getFOB() * ClimbPathBuilder.TONS_TO_POUNDS, 0, this.isaDev, this.tropoPause);

        const distanceToTopOfClimb = takeoffRollDistance + distanceTraveledSrs + distanceTraveledClb;

        return {
            distanceToRotation: takeoffRollDistance,
            distanceToAccelerationAltitude: takeoffRollDistance + distanceTraveledSrs,
            distanceToTopOfClimb,
            distanceToTopOfClimbFromEnd: totalDistance - distanceToTopOfClimb
        }
    }

    private isaTemperatureAtAltitude(altitude: number): number {
        return 15 - 0.0019812 * Math.min(altitude, 36_089)
    }

    private staticAirTemperatureAtAltitude(altitude: number): number {
        return this.isaTemperatureAtAltitude(altitude) + this.isaDev;
    }

    private totalAirTemperatureFromMach(altitude: number, mach: number) {
        // From https://en.wikipedia.org/wiki/Total_air_temperature, using gamma = 1.4
        return this.staticAirTemperatureAtAltitude(altitude) * (1 + 0.2 * Math.pow(mach, 2))
    }

    private computeMachFromCas(altitude: number, isaDev: number, speed: number): number {
        const thetaSrs = Common.getTheta(altitude, isaDev);
        const deltaSrs = Common.getDelta(thetaSrs);

        return Common.CAStoMach(speed, deltaSrs);
    }

    private computeTakeoffStepPrediction(): StepResults {
        const airfieldElevation = SimVar.GetSimVarValue('L:A32NX_DEPARTURE_ELEVATION', 'feet') ?? 0;

        const midwayAltitudeSrs = (this.accelerationAltitude + airfieldElevation) / 2;

        const commandedN1Toga = SimVar.GetSimVarValue('L:A32NX_AUTOTHRUST_THRUST_LIMIT', 'Percent') ?? 0;

        const machSrs = this.computeMachFromCas(midwayAltitudeSrs, this.isaDev, this.fmgc.getV2Speed() + 10);

        const zeroFuelWeight = this.fmgc.getZeroFuelWeight() * ClimbPathBuilder.TONS_TO_POUNDS;
        const fuelWeight = this.fmgc.getFOB() * ClimbPathBuilder.TONS_TO_POUNDS;

        return Predictions.altitudeStep(airfieldElevation, this.accelerationAltitude - airfieldElevation, this.fmgc.getV2Speed() + 10, machSrs, commandedN1Toga, zeroFuelWeight, fuelWeight, 0, this.isaDev, this.tropoPause, false, FlapConf.CONF_1);
    }

    /**
     *
     * @param tat Total air temperature in celsius
     * @param pressureAltitude Pressure altitude in feet
     * @returns N1 limit
     */
    private getClimbThrustN1Limit(tat: number, pressureAltitude: number) {
        return EngineModel.tableInterpolation(EngineModel.maxClimbThrustTable, tat, pressureAltitude);
    }

    private computeTakeOffRollDistance(): number {
        // TODO
        return 1;
    }
}
