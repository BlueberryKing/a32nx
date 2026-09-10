import { ConsumerValue, SimVarValueType, UnitType } from '@microsoft/msfs-sdk';
import { Fms } from '../Fms';
import { FmsModule } from './FmsModule';
import { EngineModel } from '../guidance/vnav/EngineModel';
import { FlightModel } from '../guidance/vnav/FlightModel';
import { AccelFactorMode, Common, FlapConf } from '../guidance/vnav/common';
import { NavigationEvents } from '../navigation/Navigation';
import { RegisteredSimVar } from '@flybywiresim/fbw-sdk';
import { A320AircraftConfig } from '../flightplanning/A320AircraftConfig';

export class SoftGaMonitor extends FmsModule {
  private readonly sub = this.bus.getSubscriber<NavigationEvents>();

  private fms?: Fms;

  private readonly config = A320AircraftConfig;
  private readonly trueAirspeed = ConsumerValue.create(this.sub.on('fms_nav_true_airspeed'), null);
  private readonly pressureAltitude = ConsumerValue.create(this.sub.on('fms_nav_pressure_altitude'), null);

  private readonly oat = RegisteredSimVar.create('AMBIENT TEMPERATURE', SimVarValueType.Celsius);
  private readonly fob = RegisteredSimVar.create('L:A32NX_TOTAL_FUEL_QUANTITY', SimVarValueType.Number);

  private readonly softGaN1Limit = RegisteredSimVar.create(
    'L:A32NX_AUTOTHRUST_THRUST_LIMIT_SOFT_GA',
    SimVarValueType.Number,
  );

  /** @inheritdoc */
  public init(fms: Fms): void {
    this.fms = fms;
  }

  /** @inheritdoc */
  public onUpdate() {
    const target = this.computeSoftGaN1Target();
    if (target !== undefined) {
      this.softGaN1Limit.set(target);
    }
  }

  private computeSoftGaN1Target() {
    const vsTarget = 2000;

    const tas = this.trueAirspeed.get();
    const alt = this.pressureAltitude.get();
    const zfw = this.fms?.flightPlanService.active?.performanceData.zeroFuelWeight.get() ?? null;

    if (tas === null || tas === 0 || alt === null || zfw === null) {
      return;
    }

    const tropo = this.fms?.flightPlanService.active?.performanceData.tropopause.get() ?? 36089;
    const oat = this.oat.get();
    const weight = UnitType.TONNE.convertTo(zfw + this.fob.get() / 1000, UnitType.POUND);
    const flapsConfig = FlapConf.CONF_3;

    const aboveTropo = alt > tropo;

    const isaDev = oat - Common.getIsaTemp(alt, aboveTropo);

    const theta = Common.getTheta(alt, isaDev, aboveTropo);
    const mach = Common.TAStoMach(tas, theta);

    const delta = Common.getDelta(alt, aboveTropo);
    const delta2 = Common.getDelta2(delta, mach);

    const drag = FlightModel.getDrag(this.config.flightModelParameters, weight, mach, delta, false, false, flapsConfig);
    const accelFactor = Common.getAccelerationFactor(mach, alt, isaDev, aboveTropo, AccelFactorMode.CONSTANT_CAS);

    // sin(gamma) = (T - D) / W / accel_factor
    // V * sin(gamma) = V * (T - D) / W / accel_factor == 2000
    // T = D + 2000 / V * accel_factor * W
    const thrust = drag + accelFactor * weight * (UnitType.FPM.convertTo(vsTarget, UnitType.KNOT) / tas);

    const correctedThrust = thrust / delta2 / this.config.engineModelParameters.numberOfEngines;

    return EngineModel.reverseTableInterpolation(
      this.config.engineModelParameters.table1506,
      mach,
      correctedThrust / this.config.engineModelParameters.maxThrust,
    );
  }
}
