use core::panic;

use crate::UpdateContext;

use uom::si::{
    f64::*,
    pressure::{pascal, psi},
    ratio::{percent, ratio},
    thermodynamic_temperature::degree_celsius,
    volume::cubic_meter,
    volume_rate::cubic_meter_per_second,
};

use systems::{
    hydraulic::Fluid,
    overhead::{AutoOffFaultPushButton, OnOffFaultPushButton},
    pneumatic::{
        ApuCompressionChamberController, CompressionChamber, ConstantConsumerController,
        ControllablePneumaticValve, ControlledPneumaticValveSignal, CrossBleedValveSelectorKnob,
        CrossBleedValveSelectorMode, DefaultConsumer, DefaultPipe, DefaultValve,
        EngineCompressionChamberController, EngineState, PneumaticContainer, TargetPressureSignal,
    },
    shared::{
        ControllerSignal, EngineCorrectedN1, EngineCorrectedN2, EngineFirePushButtons,
        PneumaticValve,
    },
    simulation::{
        Read, SimulationElement, SimulationElementVisitor, SimulatorReader, SimulatorWriter, Write,
    },
};

use pid::Pid;

struct IntermediatePressureValveSignal {
    target_open_amount: Ratio,
}

impl IntermediatePressureValveSignal {
    fn new(target_open_amount: Ratio) -> Self {
        Self { target_open_amount }
    }

    fn new_open() -> Self {
        Self::new(Ratio::new::<percent>(100.))
    }

    fn new_closed() -> Self {
        Self::new(Ratio::new::<percent>(0.))
    }
}

impl ControlledPneumaticValveSignal for IntermediatePressureValveSignal {
    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}

struct HighPressureValveSignal {
    target_open_amount: Ratio,
}

impl HighPressureValveSignal {
    fn new(target_open_amount: Ratio) -> Self {
        Self { target_open_amount }
    }

    fn new_closed() -> Self {
        Self::new(Ratio::new::<percent>(0.))
    }

    fn new_open() -> Self {
        Self::new(Ratio::new::<percent>(100.))
    }
}
impl ControlledPneumaticValveSignal for HighPressureValveSignal {
    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}

struct PressureRegulatingValveSignal {
    target_open_amount: Ratio,
}

impl PressureRegulatingValveSignal {
    fn new(target_open_amount: Ratio) -> Self {
        Self { target_open_amount }
    }

    fn new_closed() -> Self {
        Self::new(Ratio::new::<percent>(0.))
    }

    fn new_open() -> Self {
        Self::new(Ratio::new::<percent>(100.))
    }
}

impl ControlledPneumaticValveSignal for PressureRegulatingValveSignal {
    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}

struct EngineStarterValveSignal {
    target_open_amount: Ratio,
}

impl EngineStarterValveSignal {
    fn new(target_open_amount: Ratio) -> Self {
        Self { target_open_amount }
    }

    fn new_closed() -> Self {
        Self::new(Ratio::new::<percent>(0.))
    }

    fn new_open() -> Self {
        Self::new(Ratio::new::<percent>(100.))
    }
}

impl ControlledPneumaticValveSignal for EngineStarterValveSignal {
    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}

struct CrossBleedValveSignal {
    target_open_amount: Ratio,
}

impl CrossBleedValveSignal {
    fn new(target_open_amount: Ratio) -> Self {
        Self { target_open_amount }
    }

    fn new_closed() -> Self {
        Self::new(Ratio::new::<percent>(0.))
    }

    fn new_open() -> Self {
        Self::new(Ratio::new::<percent>(100.))
    }
}

impl ControlledPneumaticValveSignal for CrossBleedValveSignal {
    fn target_open_amount(&self) -> Ratio {
        self.target_open_amount
    }
}

pub struct A320Pneumatic {
    bmcs: [BleedMonitoringComputer; 2],
    engine_systems: [EngineBleedAirSystem; 2],

    cross_bleed_valve_controller: CrossBleedValveController,
    cross_bleed_valve: DefaultValve,

    // TODO: Not really sure this should be in the pneumatic system
    fadec: FullAuthorityDigitalEngineControl,
    engine_starter_valve_controllers: [EngineStarterValveController; 2],

    apu: CompressionChamber,
    apu_bleed_air_valve: DefaultValve,
    apu_bleed_air_controller: ApuCompressionChamberController,
}
impl A320Pneumatic {
    pub fn new() -> Self {
        Self {
            bmcs: [
                BleedMonitoringComputer::new(1, 2),
                BleedMonitoringComputer::new(2, 1),
            ],
            engine_systems: [EngineBleedAirSystem::new(1), EngineBleedAirSystem::new(2)],
            cross_bleed_valve_controller: CrossBleedValveController::new(),
            cross_bleed_valve: DefaultValve::new_closed(),
            fadec: FullAuthorityDigitalEngineControl::new(),
            engine_starter_valve_controllers: [
                EngineStarterValveController::new(1),
                EngineStarterValveController::new(2),
            ],
            apu: CompressionChamber::new(Volume::new::<cubic_meter>(1.)),
            apu_bleed_air_valve: DefaultValve::new_closed(),
            apu_bleed_air_controller: ApuCompressionChamberController::new(),
        }
    }

    pub fn update<T: EngineCorrectedN1 + EngineCorrectedN2>(
        &mut self,
        context: &UpdateContext,
        engines: [&T; 2],
        overhead_panel: &A320PneumaticOverheadPanel,
        engine_fire_push_buttons: &impl EngineFirePushButtons,
    ) {
        // Update cross bleed
        self.cross_bleed_valve_controller.update(
            self.apu_bleed_air_valve.is_open(),
            overhead_panel.cross_bleed_mode(),
        );
        self.cross_bleed_valve
            .update_open_amount(&self.cross_bleed_valve_controller);

        self.apu.update(&self.apu_bleed_air_controller);

        for bmc in self.bmcs.iter_mut() {
            bmc.update(
                &self.engine_systems,
                self.apu_bleed_air_valve.is_open(),
                overhead_panel,
                engine_fire_push_buttons,
                self.cross_bleed_valve.is_open(),
            );
        }

        for esv_controller in self.engine_starter_valve_controllers.iter_mut() {
            esv_controller.update(&self.fadec);
        }

        for engine_system in self.engine_systems.iter_mut() {
            let index = engine_system.number - 1;

            engine_system.update(
                context,
                &self.bmcs[index].main_channel,
                &self.bmcs[index].main_channel,
                &self.bmcs[index].main_channel,
                &self.engine_starter_valve_controllers[index],
                engines[index],
            );
        }

        let (left, right) = self.engine_systems.split_at_mut(1);
        self.apu_bleed_air_valve
            .update_move_fluid(context, &mut self.apu, &mut left[0]);

        self.cross_bleed_valve
            .update_move_fluid(context, &mut left[0], &mut right[0]);
    }

    // TODO: Returning a mutable reference here is not great. I was running into an issue with the update order:
    // - The APU turbine must know about the bleed valve being open as soon as possible to update EGT properly
    // - To open the bleed valve, we need a signal from the ecb
    // - To get a signal from the ECB to open the bleed valve, we have to update the APU.
    // For now, we just pass over control of the bleed valve to the APU, so it can be updated after the ECB update but before the turbine update.
    pub fn apu_bleed_air_valve(&mut self) -> &mut impl ControllablePneumaticValve {
        &mut self.apu_bleed_air_valve
    }
}
impl SimulationElement for A320Pneumatic {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
    where
        Self: Sized,
    {
        self.apu_bleed_air_controller.accept(visitor);

        for engine_system in self.engine_systems.iter_mut() {
            engine_system.accept(visitor);
        }

        self.fadec.accept(visitor);

        visitor.visit(self);
    }

    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write("PNEU_XBLEED_VALVE_OPEN", self.cross_bleed_valve.is_open());
        writer.write(
            "APU_BLEED_AIR_VALVE_OPEN",
            self.apu_bleed_air_valve.is_open(),
        );
    }
}

struct EngineStarterValveController {
    number: usize,
    engine_state: EngineState,
}
impl ControllerSignal<EngineStarterValveSignal> for EngineStarterValveController {
    fn signal(&self) -> Option<EngineStarterValveSignal> {
        match self.engine_state {
            EngineState::Starting => Some(EngineStarterValveSignal::new_open()),
            _ => Some(EngineStarterValveSignal::new_closed()),
        }
    }
}
impl EngineStarterValveController {
    fn new(number: usize) -> Self {
        Self {
            number,
            engine_state: EngineState::Off,
        }
    }

    fn update(&mut self, fadec: &FullAuthorityDigitalEngineControl) {
        self.engine_state = fadec.engine_state(self.number);
    }
}

struct CrossBleedValveController {
    is_apu_bleed_valve_open: bool,
    cross_bleed_valve_selector: CrossBleedValveSelectorMode,
}
impl CrossBleedValveController {
    fn new() -> Self {
        Self {
            is_apu_bleed_valve_open: false,
            cross_bleed_valve_selector: CrossBleedValveSelectorMode::Auto,
        }
    }

    fn update(
        &mut self,
        apu_bleed_air_valve_is_open: bool,
        cross_bleed_valve_selector: CrossBleedValveSelectorMode,
    ) {
        self.is_apu_bleed_valve_open = apu_bleed_air_valve_is_open;
        self.cross_bleed_valve_selector = cross_bleed_valve_selector;
    }
}
impl ControllerSignal<CrossBleedValveSignal> for CrossBleedValveController {
    fn signal(&self) -> Option<CrossBleedValveSignal> {
        match self.cross_bleed_valve_selector {
            CrossBleedValveSelectorMode::Shut => Some(CrossBleedValveSignal::new_closed()),
            CrossBleedValveSelectorMode::Open => Some(CrossBleedValveSignal::new_open()),
            CrossBleedValveSelectorMode::Auto => {
                if self.is_apu_bleed_valve_open {
                    Some(CrossBleedValveSignal::new_open())
                } else {
                    Some(CrossBleedValveSignal::new_closed())
                }
            }
        }
    }
}

trait EngineBleedDataProvider {
    fn ip_pressure(&self) -> Pressure;
    fn hp_pressure(&self) -> Pressure;
    fn transfer_pressure(&self) -> Pressure;
    fn regulated_pressure(&self) -> Pressure;
    fn ip_temperature(&self) -> ThermodynamicTemperature;
    fn hp_temperature(&self) -> ThermodynamicTemperature;
    fn transfer_temperature(&self) -> ThermodynamicTemperature;
    fn regulated_temperature(&self) -> ThermodynamicTemperature;
    fn prv_open_amount(&self) -> Ratio;
    fn hpv_open_amount(&self) -> Ratio;
    fn esv_is_open(&self) -> bool;
}

struct BleedMonitoringComputer {
    main_channel_engine_number: usize,
    backup_channel_engine_number: usize,
    main_channel: BleedMonitoringComputerChannel,
    backup_channel: BleedMonitoringComputerChannel,
}
impl BleedMonitoringComputer {
    fn new(main_channel_engine_number: usize, backup_channel_engine_number: usize) -> Self {
        Self {
            // This is the BMC number, not directly related to any engine number.
            main_channel_engine_number,
            backup_channel_engine_number,
            main_channel: BleedMonitoringComputerChannel::new(main_channel_engine_number),
            backup_channel: BleedMonitoringComputerChannel::new(backup_channel_engine_number),
        }
    }

    fn update(
        &mut self,
        sensors: &[EngineBleedAirSystem; 2],
        apu_bleed_valve_is_open: bool,
        overhead_panel: &A320PneumaticOverheadPanel,
        engine_fire_push_buttons: &impl EngineFirePushButtons,
        cross_bleed_valve_is_open: bool,
    ) {
        self.main_channel.update(
            &sensors[self.main_channel_engine_number - 1],
            overhead_panel.engine_bleed_pb_is_auto(self.main_channel_engine_number),
            engine_fire_push_buttons.is_released(self.main_channel_engine_number),
            overhead_panel.apu_bleed_is_on(),
            apu_bleed_valve_is_open,
            overhead_panel.cross_bleed_mode(),
            cross_bleed_valve_is_open,
        );

        self.backup_channel.update(
            &sensors[self.backup_channel_engine_number - 1],
            overhead_panel.engine_bleed_pb_is_auto(self.backup_channel_engine_number),
            engine_fire_push_buttons.is_released(self.backup_channel_engine_number),
            overhead_panel.apu_bleed_is_on(),
            apu_bleed_valve_is_open,
            overhead_panel.cross_bleed_mode(),
            cross_bleed_valve_is_open,
        );
    }
}

struct BleedMonitoringComputerChannel {
    engine_number: usize,
    ip_compressor_pressure: Pressure,
    hp_compressor_pressure: Pressure,
    // Pressure between IP/HP valves and the PRV
    transfer_pressure: Pressure,
    // Pressure after PRV
    regulated_pressure: Pressure,
    prv_open_amount: Ratio,
    hpv_open_position: Ratio,
    esv_is_open: bool,
    is_engine_bleed_pushbutton_auto: bool,
    is_engine_fire_pushbutton_released: bool,
    is_apu_bleed_valve_open: bool,
    is_apu_bleed_on: bool,
    hpv_pid: Pid<f64>,
    hpv_output: f64,
    prv_pid: Pid<f64>,
    prv_output: f64,
    cross_bleed_valve_selector: CrossBleedValveSelectorMode,
    cross_bleed_valve_is_open: bool,
}
impl BleedMonitoringComputerChannel {
    fn new(engine_number: usize) -> Self {
        Self {
            engine_number,
            ip_compressor_pressure: Pressure::new::<psi>(0.),
            hp_compressor_pressure: Pressure::new::<psi>(0.),
            transfer_pressure: Pressure::new::<psi>(0.),
            regulated_pressure: Pressure::new::<psi>(0.),
            hpv_open_position: Ratio::new::<percent>(0.),
            prv_open_amount: Ratio::new::<percent>(0.),
            esv_is_open: false,
            is_engine_bleed_pushbutton_auto: true,
            is_engine_fire_pushbutton_released: false,
            is_apu_bleed_valve_open: false,
            is_apu_bleed_on: false,
            hpv_pid: Pid::new(0.05, 0., 0., 1., 1., 1., 1., 65.),
            hpv_output: 0.,
            prv_pid: Pid::new(0., 0.01, 0., 1., 1., 1., 1., 46.),
            prv_output: 0.,
            cross_bleed_valve_selector: CrossBleedValveSelectorMode::Auto,
            cross_bleed_valve_is_open: false,
        }
    }

    fn update(
        &mut self,
        sensors: &impl EngineBleedDataProvider,
        is_engine_bleed_pushbutton_auto: bool,
        is_engine_fire_pushbutton_released: bool,
        is_apu_bleed_on: bool,
        apu_bleed_valve_is_open: bool,
        cross_bleed_valve_selector: CrossBleedValveSelectorMode,
        cross_bleed_valve_is_open: bool,
    ) {
        self.ip_compressor_pressure = sensors.ip_pressure();
        self.hp_compressor_pressure = sensors.hp_pressure();
        self.transfer_pressure = sensors.transfer_pressure();
        self.regulated_pressure = sensors.regulated_pressure();

        self.hpv_output = self
            .hpv_pid
            .next_control_output(self.transfer_pressure.get::<psi>())
            .output;
        self.prv_output = self
            .prv_pid
            .next_control_output(self.regulated_pressure.get::<psi>())
            .output;

        self.prv_open_amount = sensors.prv_open_amount();
        self.hpv_open_position = sensors.hpv_open_amount();
        self.esv_is_open = sensors.esv_is_open();

        self.is_engine_bleed_pushbutton_auto = is_engine_bleed_pushbutton_auto;
        self.is_engine_fire_pushbutton_released = is_engine_fire_pushbutton_released;

        self.is_apu_bleed_valve_open = apu_bleed_valve_is_open;
        self.is_apu_bleed_on = is_apu_bleed_on;

        self.cross_bleed_valve_selector = cross_bleed_valve_selector;
        self.cross_bleed_valve_is_open = cross_bleed_valve_is_open;
    }
}
impl ControllerSignal<IntermediatePressureValveSignal> for BleedMonitoringComputerChannel {
    fn signal(&self) -> Option<IntermediatePressureValveSignal> {
        if self.transfer_pressure - self.ip_compressor_pressure > Pressure::new::<pascal>(100.) {
            Some(IntermediatePressureValveSignal::new_closed())
        } else {
            Some(IntermediatePressureValveSignal::new_open())
        }
    }
}
impl ControllerSignal<HighPressureValveSignal> for BleedMonitoringComputerChannel {
    fn signal(&self) -> Option<HighPressureValveSignal> {
        if self.transfer_pressure < Pressure::new::<psi>(18.) {
            return Some(HighPressureValveSignal::new_closed());
        }

        Some(HighPressureValveSignal::new(Ratio::new::<ratio>(
            self.hpv_output.max(0.).min(1.),
        )))
    }
}
impl ControllerSignal<PressureRegulatingValveSignal> for BleedMonitoringComputerChannel {
    fn signal(&self) -> Option<PressureRegulatingValveSignal> {
        if self.transfer_pressure < Pressure::new::<psi>(15.) {
            return Some(PressureRegulatingValveSignal::new_closed());
        }

        if !self.is_engine_bleed_pushbutton_auto || self.is_engine_fire_pushbutton_released {
            return Some(PressureRegulatingValveSignal::new_closed());
        }

        if self.is_apu_bleed_on
            && self.is_apu_bleed_valve_open
            && (self.engine_number == 1 || self.cross_bleed_valve_is_open)
        {
            return Some(PressureRegulatingValveSignal::new_closed());
        }

        if self.esv_is_open {
            return Some(PressureRegulatingValveSignal::new_closed());
        }

        Some(PressureRegulatingValveSignal::new(Ratio::new::<ratio>(
            self.prv_output.max(0.).min(1.),
        )))
    }
}

struct EngineBleedAirSystem {
    number: usize,
    ip_compression_chamber_controller: EngineCompressionChamberController,
    hp_compression_chamber_controller: EngineCompressionChamberController,
    ip_compression_chamber: CompressionChamber,
    hp_compression_chamber: CompressionChamber,
    ip_valve: DefaultValve,
    hp_valve: DefaultValve,
    pr_valve: DefaultValve,
    transfer_pressure_pipe: DefaultPipe,
    regulated_pressure_pipe: DefaultPipe,
    engine_starter_consumer: DefaultConsumer,
    engine_starter_consumer_controller: ConstantConsumerController,
    es_valve: DefaultValve,
}
impl EngineBleedAirSystem {
    fn new(number: usize) -> Self {
        Self {
            number,
            ip_compression_chamber_controller: EngineCompressionChamberController::new(3., 0., 2.),
            hp_compression_chamber_controller: EngineCompressionChamberController::new(
                1.5, 2.5, 4.,
            ),
            ip_compression_chamber: CompressionChamber::new(Volume::new::<cubic_meter>(1.)),
            hp_compression_chamber: CompressionChamber::new(Volume::new::<cubic_meter>(1.)),
            ip_valve: DefaultValve::new(Ratio::new::<percent>(100.)),
            hp_valve: DefaultValve::new(Ratio::new::<percent>(0.)),
            pr_valve: DefaultValve::new(Ratio::new::<percent>(0.)),
            transfer_pressure_pipe: DefaultPipe::new(
                Volume::new::<cubic_meter>(1.), // TODO: Figure out volume to use
                Fluid::new(Pressure::new::<pascal>(142000.)), // https://en.wikipedia.org/wiki/Bulk_modulus#Selected_values
                Pressure::new::<psi>(14.7),
                ThermodynamicTemperature::new::<degree_celsius>(15.),
            ),
            regulated_pressure_pipe: DefaultPipe::new(
                Volume::new::<cubic_meter>(1.), // TODO: Figure out volume to use
                Fluid::new(Pressure::new::<pascal>(142000.)), // https://en.wikipedia.org/wiki/Bulk_modulus#Selected_values
                Pressure::new::<psi>(14.7),
                ThermodynamicTemperature::new::<degree_celsius>(15.),
            ),
            engine_starter_consumer: DefaultConsumer::new(Volume::new::<cubic_meter>(1.)),
            engine_starter_consumer_controller: ConstantConsumerController::new(VolumeRate::new::<
                cubic_meter_per_second,
            >(0.1)),
            es_valve: DefaultValve::new(Ratio::new::<ratio>(0.)),
        }
    }

    fn update<T: EngineCorrectedN1 + EngineCorrectedN2>(
        &mut self,
        context: &UpdateContext,
        ipv_controller: &impl ControllerSignal<IntermediatePressureValveSignal>,
        hpv_controller: &impl ControllerSignal<HighPressureValveSignal>,
        prv_controller: &impl ControllerSignal<PressureRegulatingValveSignal>,
        esv_controller: &impl ControllerSignal<EngineStarterValveSignal>,
        engine: &T,
    ) {
        // Update engines
        self.ip_compression_chamber_controller
            .update(context, engine);
        self.hp_compression_chamber_controller
            .update(context, engine);

        self.ip_compression_chamber
            .update(&self.ip_compression_chamber_controller);
        self.hp_compression_chamber
            .update(&self.hp_compression_chamber_controller);

        // Update controllers
        self.engine_starter_consumer_controller.update(context);

        // Update consumers
        self.engine_starter_consumer
            .update(&self.engine_starter_consumer_controller);

        // Update valves (open amount)
        self.ip_valve.update_open_amount(ipv_controller);
        self.hp_valve.update_open_amount(hpv_controller);
        self.pr_valve.update_open_amount(prv_controller);
        self.es_valve.update_open_amount(esv_controller);

        // Update valves (fluid movement)
        self.ip_valve.update_move_fluid(
            context,
            &mut self.ip_compression_chamber,
            &mut self.transfer_pressure_pipe,
        );
        self.hp_valve.update_move_fluid(
            context,
            &mut self.hp_compression_chamber,
            &mut self.transfer_pressure_pipe,
        );
        self.pr_valve.update_move_fluid(
            context,
            &mut self.transfer_pressure_pipe,
            &mut self.regulated_pressure_pipe,
        );
        self.es_valve.update_move_fluid(
            context,
            &mut self.regulated_pressure_pipe,
            &mut self.engine_starter_consumer,
        );
    }
}
impl EngineBleedDataProvider for EngineBleedAirSystem {
    fn ip_pressure(&self) -> Pressure {
        self.ip_compression_chamber.pressure()
    }

    fn hp_pressure(&self) -> Pressure {
        self.hp_compression_chamber.pressure()
    }

    fn transfer_pressure(&self) -> Pressure {
        self.transfer_pressure_pipe.pressure()
    }

    fn regulated_pressure(&self) -> Pressure {
        self.regulated_pressure_pipe.pressure()
    }

    fn ip_temperature(&self) -> ThermodynamicTemperature {
        self.ip_compression_chamber.temperature()
    }

    fn hp_temperature(&self) -> ThermodynamicTemperature {
        self.hp_compression_chamber.temperature()
    }

    fn transfer_temperature(&self) -> ThermodynamicTemperature {
        self.transfer_pressure_pipe.temperature()
    }

    fn regulated_temperature(&self) -> ThermodynamicTemperature {
        self.regulated_pressure_pipe.temperature()
    }

    fn prv_open_amount(&self) -> Ratio {
        self.pr_valve.open_amount()
    }

    fn hpv_open_amount(&self) -> Ratio {
        self.hp_valve.open_amount()
    }

    fn esv_is_open(&self) -> bool {
        self.es_valve.is_open()
    }
}
impl SimulationElement for EngineBleedAirSystem {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
    where
        Self: Sized,
    {
        self.ip_compression_chamber_controller.accept(visitor);
        self.hp_compression_chamber_controller.accept(visitor);

        visitor.visit(self);
    }

    // TODO: Possibly move these to their respective struct (e.g IP_PRESSURE to IPCompressionChamber or its controller)
    fn write(&self, writer: &mut SimulatorWriter) {
        writer.write(
            &format!("PNEU_ENG_{}_IP_PRESSURE", self.number),
            self.ip_pressure(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_HP_PRESSURE", self.number),
            self.hp_pressure(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_TRANSFER_PRESSURE", self.number),
            self.transfer_pressure(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_PRECOOLER_INLET_PRESSURE", self.number),
            self.regulated_pressure(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_IP_TEMPERATURE", self.number),
            self.ip_temperature(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_HP_TEMPERATURE", self.number),
            self.hp_temperature(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_TRANSFER_TEMPERATURE", self.number),
            self.transfer_temperature(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_PRECOOLER_INLET_TEMPERATURE", self.number),
            self.regulated_temperature(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_IP_VALVE_OPEN", self.number),
            self.ip_valve.is_open(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_HP_VALVE_OPEN", self.number),
            self.hp_valve.is_open(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_PR_VALVE_OPEN", self.number),
            self.pr_valve.is_open(),
        );

        writer.write(
            &format!("PNEU_ENG_{}_STARTER_VALVE_OPEN", self.number),
            self.es_valve.is_open(),
        );
    }
}
impl PneumaticContainer for EngineBleedAirSystem {
    // This implementation is to connect the two engine systems via the cross bleed valve

    fn pressure(&self) -> Pressure {
        self.regulated_pressure_pipe.pressure()
    }

    fn volume(&self) -> Volume {
        self.regulated_pressure_pipe.volume()
    }

    fn temperature(&self) -> ThermodynamicTemperature {
        self.regulated_pressure_pipe.temperature()
    }

    fn change_volume(&mut self, volume: Volume) {
        self.regulated_pressure_pipe.change_volume(volume)
    }
}

pub struct A320PneumaticOverheadPanel {
    apu_bleed: OnOffFaultPushButton,
    cross_bleed: CrossBleedValveSelectorKnob,
    engine_1_bleed: AutoOffFaultPushButton,
    engine_2_bleed: AutoOffFaultPushButton,
}
impl A320PneumaticOverheadPanel {
    pub fn new() -> Self {
        A320PneumaticOverheadPanel {
            apu_bleed: OnOffFaultPushButton::new_on("PNEU_APU_BLEED"),
            cross_bleed: CrossBleedValveSelectorKnob::new_auto(),
            engine_1_bleed: AutoOffFaultPushButton::new_auto("PNEU_ENG_1_BLEED"),
            engine_2_bleed: AutoOffFaultPushButton::new_auto("PNEU_ENG_2_BLEED"),
        }
    }

    pub fn apu_bleed_is_on(&self) -> bool {
        self.apu_bleed.is_on()
    }

    pub fn cross_bleed_mode(&self) -> CrossBleedValveSelectorMode {
        self.cross_bleed.mode()
    }

    pub fn engine_bleed_pb_is_auto(&self, engine_number: usize) -> bool {
        match engine_number {
            1 => self.engine_1_bleed.is_auto(),
            2 => self.engine_2_bleed.is_auto(),
            _ => panic!("Invalid engine number"),
        }
    }

    pub fn engine_bleed_pb_has_fault(&self, engine_number: usize) -> bool {
        match engine_number {
            1 => self.engine_1_bleed.has_fault(),
            2 => self.engine_2_bleed.has_fault(),
            _ => panic!("Invalid engine number"),
        }
    }
}
impl SimulationElement for A320PneumaticOverheadPanel {
    fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T) {
        self.apu_bleed.accept(visitor);
        self.cross_bleed.accept(visitor);
        self.engine_1_bleed.accept(visitor);
        self.engine_2_bleed.accept(visitor);

        visitor.visit(self);
    }
}

struct FullAuthorityDigitalEngineControl {
    engine_1_state: EngineState,
    engine_2_state: EngineState,
}
impl FullAuthorityDigitalEngineControl {
    fn new() -> Self {
        Self {
            engine_1_state: EngineState::Off,
            engine_2_state: EngineState::Off,
        }
    }

    fn engine_state(&self, number: usize) -> EngineState {
        match number {
            1 => self.engine_1_state,
            2 => self.engine_2_state,
            _ => panic!("Invalid engine number"),
        }
    }
}
impl SimulationElement for FullAuthorityDigitalEngineControl {
    fn read(&mut self, reader: &mut SimulatorReader) {
        self.engine_1_state = reader.read("ENGINE_STATE:1");
        self.engine_2_state = reader.read("ENGINE_STATE:2");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use systems::{
        engine::leap_engine::LeapEngine,
        pneumatic::{EngineState, PneumaticContainer},
        shared::{ApuBleedAirValveSignal, MachNumber, ISA},
        simulation::{
            test::{SimulationTestBed, TestBed},
            Aircraft, SimulationElement, Write,
        },
    };

    use std::{fs::File, time::Duration};

    use uom::si::{length::foot, pressure::pascal, thermodynamic_temperature::degree_celsius};

    struct TestApu {
        bleed_air_valve_signal: ApuBleedAirValveSignal,
    }
    impl TestApu {
        fn new() -> Self {
            Self {
                bleed_air_valve_signal: ApuBleedAirValveSignal::Close,
            }
        }

        fn update(&self, bleed_valve: &mut impl ControllablePneumaticValve) {
            bleed_valve.update_open_amount(self);
        }

        fn set_bleed_air_valve_signal(&mut self, signal: ApuBleedAirValveSignal) {
            self.bleed_air_valve_signal = signal;
        }
    }
    impl ControllerSignal<ApuBleedAirValveSignal> for TestApu {
        fn signal(&self) -> Option<ApuBleedAirValveSignal> {
            Some(self.bleed_air_valve_signal)
        }
    }

    struct TestEngineFirePushButtons {
        is_released: [bool; 2],
    }
    impl TestEngineFirePushButtons {
        fn new() -> Self {
            Self {
                is_released: [false, false],
            }
        }

        fn release(&mut self, engine_number: usize) {
            self.is_released[engine_number - 1] = true;
        }
    }
    impl EngineFirePushButtons for TestEngineFirePushButtons {
        fn is_released(&self, engine_number: usize) -> bool {
            self.is_released[engine_number - 1]
        }
    }

    struct PneumaticTestAircraft {
        pneumatic: A320Pneumatic,
        apu: TestApu,
        engine_1: LeapEngine,
        engine_2: LeapEngine,
        overhead_panel: A320PneumaticOverheadPanel,
        fire_pushbuttons: TestEngineFirePushButtons,
    }
    impl PneumaticTestAircraft {
        fn new() -> Self {
            Self {
                pneumatic: A320Pneumatic::new(),
                apu: TestApu::new(),
                engine_1: LeapEngine::new(1),
                engine_2: LeapEngine::new(2),
                overhead_panel: A320PneumaticOverheadPanel::new(),
                fire_pushbuttons: TestEngineFirePushButtons::new(),
            }
        }
    }
    impl Aircraft for PneumaticTestAircraft {
        fn update_after_power_distribution(&mut self, context: &UpdateContext) {
            self.apu.update(self.pneumatic.apu_bleed_air_valve());
            self.pneumatic.update(
                context,
                [&self.engine_1, &self.engine_2],
                &self.overhead_panel,
                &self.fire_pushbuttons,
            );
        }
    }
    impl SimulationElement for PneumaticTestAircraft {
        fn accept<T: SimulationElementVisitor>(&mut self, visitor: &mut T)
        where
            Self: Sized,
        {
            self.pneumatic.accept(visitor);
            self.engine_1.accept(visitor);
            self.engine_2.accept(visitor);
            self.overhead_panel.accept(visitor);

            visitor.visit(self);
        }
    }
    struct PneumaticTestBed {
        test_bed: SimulationTestBed<PneumaticTestAircraft>,
    }
    impl TestBed for PneumaticTestBed {
        type Aircraft = PneumaticTestAircraft;

        fn test_bed(&self) -> &SimulationTestBed<PneumaticTestAircraft> {
            &self.test_bed
        }

        fn test_bed_mut(&mut self) -> &mut SimulationTestBed<PneumaticTestAircraft> {
            &mut self.test_bed
        }
    }
    impl PneumaticTestBed {
        fn new() -> Self {
            Self {
                test_bed: SimulationTestBed::<PneumaticTestAircraft>::new(|_| {
                    PneumaticTestAircraft::new()
                }),
            }
        }

        fn and_run(mut self) -> Self {
            self.run();

            self
        }

        fn and_stabilize(mut self) -> Self {
            for _ in 1..1000 {
                self.run_with_delta(Duration::from_millis(16));
            }

            self
        }

        fn mach_number(mut self, mach: MachNumber) -> Self {
            self.write("AIRSPEED MACH", mach);

            self
        }

        fn in_isa_atmosphere(mut self, altitude: Length) -> Self {
            self.set_ambient_pressure(ISA::pressure_at_altitude(altitude));
            self.set_ambient_temperature(ISA::temperature_at_altitude(altitude));

            self
        }

        fn idle_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", true);
            self.write("TURB ENG CORRECTED N2:1", Ratio::new::<ratio>(0.55));
            self.write("TURB ENG CORRECTED N1:1", Ratio::new::<ratio>(0.2));
            self.write("ENGINE_STATE:1", EngineState::On);

            self
        }

        fn idle_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", true);
            self.write("TURB ENG CORRECTED N2:2", Ratio::new::<ratio>(0.55));
            self.write("TURB ENG CORRECTED N1:2", Ratio::new::<ratio>(0.2));
            self.write("ENGINE_STATE:2", EngineState::On);

            self
        }

        fn stop_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", false);
            self.write("TURB ENG CORRECTED N2:1", Ratio::new::<ratio>(0.));
            self.write("TURB ENG CORRECTED N1:1", Ratio::new::<ratio>(0.));

            self
        }

        fn stop_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", false);
            self.write("TURB ENG CORRECTED N2:2", Ratio::new::<ratio>(0.));
            self.write("TURB ENG CORRECTED N1:2", Ratio::new::<ratio>(0.));

            self
        }

        fn start_eng1(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:1", true);
            self.write("ENGINE_STATE:1", EngineState::Starting);

            self
        }

        fn start_eng2(mut self) -> Self {
            self.write("GENERAL ENG STARTER ACTIVE:2", true);
            self.write("ENGINE_STATE:2", EngineState::Starting);

            self
        }

        fn cross_bleed_valve_selector_knob(mut self, mode: CrossBleedValveSelectorMode) -> Self {
            self.write("KNOB_OVHD_AIRCOND_XBLEED_Position", mode);

            self
        }

        fn for_both_engine_systems<T: Fn(&EngineBleedAirSystem) -> ()>(&self, func: T) {
            self.query(|a| a.pneumatic.engine_systems.iter().for_each(|sys| func(sys)));
        }

        fn ip_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].ip_pressure())
        }

        fn hp_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].hp_pressure())
        }

        fn transfer_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].transfer_pressure())
        }

        fn regulated_pressure(&self, number: usize) -> Pressure {
            self.query(|a| a.pneumatic.engine_systems[number - 1].regulated_pressure())
        }

        fn ip_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].ip_temperature())
        }

        fn hp_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].hp_temperature())
        }

        fn transfer_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].transfer_temperature())
        }

        fn regulated_temperature(&self, number: usize) -> ThermodynamicTemperature {
            self.query(|a| a.pneumatic.engine_systems[number - 1].regulated_temperature())
        }

        fn ip_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].ip_valve.is_open())
        }

        fn hp_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].hp_valve.is_open())
        }

        fn pr_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].pr_valve.is_open())
        }

        fn es_valve_is_open(&self, number: usize) -> bool {
            self.query(|a| a.pneumatic.engine_systems[number - 1].esv_is_open())
        }

        fn apu_bleed_valve_is_open(&self) -> bool {
            self.query(|a| a.pneumatic.apu_bleed_air_valve.is_open())
        }

        fn set_engine_bleed_push_button_off(mut self, number: usize) -> Self {
            self.write(&format!("OVHD_PNEU_ENG_{}_BLEED_PB_IS_AUTO", number), false);

            self
        }

        fn set_engine_bleed_push_button_has_fault(
            mut self,
            number: usize,
            has_fault: bool,
        ) -> Self {
            self.write(
                &format!("OVHD_PNEU_ENG_{}_BLEED_PB_HAS_FAULT", number),
                has_fault,
            );

            self
        }

        fn set_apu_bleed_valve_signal(mut self, signal: ApuBleedAirValveSignal) -> Self {
            self.command(|a| a.apu.set_bleed_air_valve_signal(signal));

            self
        }

        fn set_bleed_air_pb(mut self, is_on: bool) -> Self {
            self.write("OVHD_APU_BLEED_PB_IS_ON", is_on);

            self
        }

        fn set_bleed_air_running(mut self) -> Self {
            self.write("APU_BLEED_AIR_PRESSURE", Pressure::new::<psi>(35.));
            self.set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Open)
                .set_bleed_air_pb(true)
        }

        fn release_fire_pushbutton(mut self, number: usize) -> Self {
            self.command(|a| a.fire_pushbuttons.release(number));

            self
        }

        fn set_engine_state(mut self, number: usize, engine_state: EngineState) -> Self {
            self.write(&format!("ENGINE_STATE:{}", number), engine_state);

            self
        }

        fn engine_state(&self, number: usize) -> EngineState {
            self.query(|a| a.pneumatic.fadec.engine_state(number))
        }

        fn is_fire_pushbutton_released(&self, number: usize) -> bool {
            self.query(|a| a.fire_pushbuttons.is_released(number))
        }

        fn cross_bleed_valve_is_open(&self) -> bool {
            self.query(|a| a.pneumatic.cross_bleed_valve.is_open())
        }

        fn cross_bleed_valve_selector(&self) -> CrossBleedValveSelectorMode {
            self.query(|a| a.overhead_panel.cross_bleed_mode())
        }

        fn engine_bleed_push_button_is_auto(&self, number: usize) -> bool {
            self.query(|a| a.overhead_panel.engine_bleed_pb_is_auto(number))
        }

        fn engine_bleed_push_button_has_fault(&self, number: usize) -> bool {
            self.query(|a| a.overhead_panel.engine_bleed_pb_has_fault(number))
        }
    }

    fn test_bed() -> PneumaticTestBed {
        PneumaticTestBed::new()
    }

    fn test_bed_with() -> PneumaticTestBed {
        test_bed()
    }

    fn pressure_tolerance() -> Pressure {
        Pressure::new::<pascal>(100.)
    }

    // Just a way for me to plot some graphs
    #[test]
    fn test() {
        let alt = Length::new::<foot>(0.);

        let mut test_bed = test_bed_with()
            .mach_number(MachNumber(0.))
            .in_isa_atmosphere(alt)
            .idle_eng1()
            .stop_eng2();
        // .set_bleed_air_running();

        let mut ts = Vec::new();
        let mut hps = Vec::new();
        let mut ips = Vec::new();
        let mut c2s = Vec::new();
        let mut c1s = Vec::new();
        let mut ipts = Vec::new();
        let mut hpts = Vec::new();
        let mut c2ts = Vec::new();
        let mut c1ts = Vec::new();
        let mut hpv_open = Vec::new();
        let mut prv_open = Vec::new();
        let mut ipv_open = Vec::new();
        let mut esv_open = Vec::new();
        let mut abv_open = Vec::new();

        for i in 1..500 {
            ts.push(i as f64 * 16.);

            // if i == 100 {
            //     test_bed = test_bed.start_eng1();
            // }

            hps.push(test_bed.hp_pressure(1).get::<psi>());
            ips.push(test_bed.ip_pressure(1).get::<psi>());
            c2s.push(test_bed.transfer_pressure(1).get::<psi>());
            c1s.push(test_bed.regulated_pressure(1).get::<psi>());

            ipts.push(test_bed.ip_temperature(1).get::<degree_celsius>());
            hpts.push(test_bed.hp_temperature(1).get::<degree_celsius>());
            c2ts.push(test_bed.transfer_temperature(1).get::<degree_celsius>());
            c1ts.push(test_bed.regulated_temperature(1).get::<degree_celsius>());

            hpv_open.push(test_bed.query(|aircraft| {
                aircraft.pneumatic.engine_systems[0]
                    .hp_valve
                    .open_amount()
                    .get::<ratio>()
                    * 10.
            }));

            prv_open.push(test_bed.query(|aircraft| {
                aircraft.pneumatic.engine_systems[0]
                    .pr_valve
                    .open_amount()
                    .get::<ratio>()
                    * 10.
            }));

            ipv_open.push(test_bed.query(|aircraft| {
                aircraft.pneumatic.engine_systems[0]
                    .ip_valve
                    .open_amount()
                    .get::<ratio>()
                    * 10.
            }));

            esv_open.push(test_bed.query(|aircraft| {
                aircraft.pneumatic.engine_systems[0]
                    .es_valve
                    .open_amount()
                    .get::<ratio>()
                    * 10.
            }));

            abv_open.push(if test_bed.apu_bleed_valve_is_open() {
                10.
            } else {
                0.
            });

            test_bed.run_with_delta(Duration::from_millis(16));
        }

        // If anyone is wondering, I am using python to plot pressure curves. This will be removed once the model is complete.
        let data = vec![
            ts, hps, ips, c2s, c1s, hpts, ipts, c2ts, c1ts, hpv_open, prv_open, ipv_open, esv_open,
            abv_open,
        ];
        let mut file = File::create("DO NOT COMMIT.txt").expect("Could not create file");

        use std::io::Write;

        writeln!(file, "{:?}", data).expect("Could not write file");
    }

    #[test]
    fn cold_dark_valves_closed() {
        let altitude = Length::new::<foot>(0.);
        let ambient_pressure = ISA::pressure_at_altitude(altitude);

        let test_bed = test_bed_with()
            .stop_eng1()
            .stop_eng2()
            .in_isa_atmosphere(altitude)
            .mach_number(MachNumber(0.))
            .and_run();

        test_bed.for_both_engine_systems(|sys| {
            assert!((sys.ip_pressure() - ambient_pressure).abs() < pressure_tolerance());
            assert!((sys.hp_pressure() - ambient_pressure).abs() < pressure_tolerance());
            assert!((sys.transfer_pressure() - ambient_pressure).abs() < pressure_tolerance());
            assert!((sys.regulated_pressure() - ambient_pressure).abs() < pressure_tolerance());

            assert!(sys.ip_valve.is_open());
            assert!(!sys.hp_valve.is_open());
            assert!(!sys.pr_valve.is_open());
        });

        assert!(!test_bed.cross_bleed_valve_is_open())
    }

    #[test]
    fn single_engine_idle() {
        let altitude = Length::new::<foot>(0.);
        let mut test_bed = test_bed_with()
            .idle_eng1()
            .stop_eng2()
            .in_isa_atmosphere(altitude)
            .mach_number(MachNumber(0.))
            .and_run();

        let ambient_pressure = ISA::pressure_at_altitude(altitude);

        // Three updates for now until propagation logic is fixed
        test_bed.run_with_delta(Duration::from_secs(5));
        test_bed.run_with_delta(Duration::from_secs(5));

        assert!(test_bed.ip_pressure(1) - ambient_pressure > pressure_tolerance());
        assert!((test_bed.ip_pressure(2) - ambient_pressure).abs() < pressure_tolerance());

        assert!(test_bed.hp_pressure(1) - ambient_pressure > pressure_tolerance());
        assert!((test_bed.hp_pressure(2) - ambient_pressure).abs() < pressure_tolerance());

        assert!(test_bed.transfer_pressure(1) - ambient_pressure > pressure_tolerance());
        assert!((test_bed.transfer_pressure(2) - ambient_pressure).abs() < pressure_tolerance());

        assert!(test_bed.regulated_pressure(1) - ambient_pressure > pressure_tolerance());
        assert!((test_bed.regulated_pressure(2) - ambient_pressure).abs() < pressure_tolerance());

        assert!(!test_bed.ip_valve_is_open(1));
        assert!(test_bed.ip_valve_is_open(2));

        assert!(test_bed.hp_valve_is_open(1));
        assert!(!test_bed.hp_valve_is_open(2));

        assert!(test_bed.pr_valve_is_open(1));
        assert!(!test_bed.pr_valve_is_open(2));

        test_bed.for_both_engine_systems(|sys| assert!(!sys.es_valve.is_open()));
        assert!(!test_bed.cross_bleed_valve_is_open());
    }

    #[test]
    fn starter_valve_opens_on_engine_start() {
        let mut test_bed = test_bed_with().stop_eng1().stop_eng2().and_run();

        assert!(!test_bed.es_valve_is_open(1));
        assert!(!test_bed.es_valve_is_open(2));

        test_bed = test_bed.start_eng1().start_eng2().and_run();

        assert!(test_bed.es_valve_is_open(1));
        assert!(test_bed.es_valve_is_open(2));
    }

    #[test]
    fn cross_bleed_valve_opens_when_apu_bleed_valve_opens() {
        let mut test_bed = test_bed_with()
            .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Auto)
            .and_run();

        assert!(!test_bed.cross_bleed_valve_is_open());

        test_bed = test_bed
            .set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Open)
            .and_run();

        assert!(test_bed.cross_bleed_valve_is_open());
    }

    #[test]
    fn cross_bleed_valve_closes_when_apu_bleed_valve_closes() {
        let mut test_bed = test_bed_with()
            .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Auto)
            .set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Open)
            .and_run();

        assert!(test_bed.cross_bleed_valve_is_open());

        test_bed = test_bed
            .set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Close)
            .and_run();

        assert!(!test_bed.cross_bleed_valve_is_open());
    }

    #[test]
    fn cross_bleed_valve_manual_overrides_everything() {
        let mut test_bed = test_bed_with()
            .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Shut)
            .set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Open)
            .and_run();

        assert!(!test_bed.cross_bleed_valve_is_open());

        test_bed = test_bed
            .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Open)
            .set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Close)
            .and_run();

        assert!(test_bed.cross_bleed_valve_is_open());
    }

    #[test]
    fn simvars_initialized_properly() {
        let test_bed = test_bed()
            .in_isa_atmosphere(Length::new::<foot>(0.))
            .and_run();

        assert!(test_bed.contains_key("PNEU_ENG_1_IP_PRESSURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_IP_PRESSURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_HP_PRESSURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_HP_PRESSURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_TRANSFER_PRESSURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_TRANSFER_PRESSURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_PRECOOLER_INLET_PRESSURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_PRECOOLER_INLET_PRESSURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_IP_TEMPERATURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_IP_TEMPERATURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_HP_TEMPERATURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_HP_TEMPERATURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_TRANSFER_TEMPERATURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_TRANSFER_TEMPERATURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_PRECOOLER_INLET_TEMPERATURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_PRECOOLER_INLET_TEMPERATURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_IP_PRESSURE"));
        assert!(test_bed.contains_key("PNEU_ENG_2_IP_PRESSURE"));

        assert!(test_bed.contains_key("PNEU_ENG_1_IP_VALVE_OPEN"));
        assert!(test_bed.contains_key("PNEU_ENG_2_IP_VALVE_OPEN"));

        assert!(test_bed.contains_key("PNEU_ENG_1_HP_VALVE_OPEN"));
        assert!(test_bed.contains_key("PNEU_ENG_2_HP_VALVE_OPEN"));

        assert!(test_bed.contains_key("PNEU_ENG_1_PR_VALVE_OPEN"));
        assert!(test_bed.contains_key("PNEU_ENG_2_PR_VALVE_OPEN"));

        assert!(test_bed.contains_key("PNEU_ENG_1_STARTER_VALVE_OPEN"));
        assert!(test_bed.contains_key("PNEU_ENG_2_STARTER_VALVE_OPEN"));

        assert!(test_bed.contains_key("OVHD_PNEU_ENG_1_BLEED_PB_HAS_FAULT"));
        assert!(test_bed.contains_key("OVHD_PNEU_ENG_2_BLEED_PB_HAS_FAULT"));

        assert!(test_bed.contains_key("OVHD_PNEU_ENG_1_BLEED_PB_IS_AUTO"));
        assert!(test_bed.contains_key("OVHD_PNEU_ENG_2_BLEED_PB_IS_AUTO"));

        assert!(test_bed.contains_key("PNEU_XBLEED_VALVE_OPEN"));
    }

    #[test]
    fn prv_closes_with_ovhd_engine_bleed_off() {
        let mut test_bed = test_bed().idle_eng1().idle_eng2().and_run();

        // TODO: I shouldn't have to run this twice, but it just takes two updates for the pressure to propagate through the system.
        test_bed.run_with_delta(Duration::from_secs(5));

        assert!(test_bed.pr_valve_is_open(1));
        assert!(test_bed.pr_valve_is_open(2));

        test_bed = test_bed.set_engine_bleed_push_button_off(1).and_run();

        assert!(!test_bed.pr_valve_is_open(1));
        assert!(test_bed.pr_valve_is_open(2));

        test_bed = test_bed.set_engine_bleed_push_button_off(2).and_run();

        assert!(!test_bed.pr_valve_is_open(1));
        assert!(!test_bed.pr_valve_is_open(2));
    }

    #[test]
    fn prv_closes_with_ovhd_engine_fire_pushbutton_released() {
        let mut test_bed = test_bed().idle_eng1().idle_eng2().and_run();

        // TODO: I shouldn't have to run this twice, but it just takes two updates for the pressure to propagate through the system.
        test_bed.run_with_delta(Duration::from_secs(5));

        assert!(test_bed.pr_valve_is_open(1));
        assert!(test_bed.pr_valve_is_open(2));

        test_bed = test_bed.release_fire_pushbutton(1).and_run();

        assert!(!test_bed.pr_valve_is_open(1));
        assert!(test_bed.pr_valve_is_open(2));

        test_bed = test_bed.release_fire_pushbutton(2).and_run();

        assert!(!test_bed.pr_valve_is_open(1));
        assert!(!test_bed.pr_valve_is_open(2));
    }

    #[test]
    fn prv_1_closes_with_apu_bleed_on() {
        let mut test_bed = test_bed_with().idle_eng1().idle_eng2().and_run();

        test_bed.run_with_delta(Duration::from_secs(5));

        assert!(test_bed.pr_valve_is_open(1));

        test_bed = test_bed
            .set_bleed_air_pb(true)
            .set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Open)
            .and_run();

        assert!(!test_bed.pr_valve_is_open(1));
    }

    #[test]
    fn prv_2_closes_with_apu_bleed_on_and_cross_bleed_open() {
        let mut test_bed = test_bed_with().idle_eng1().idle_eng2().and_run();

        test_bed.run_with_delta(Duration::from_secs(5));

        assert!(test_bed.pr_valve_is_open(2));

        test_bed = test_bed
            .set_bleed_air_pb(true)
            .set_apu_bleed_valve_signal(ApuBleedAirValveSignal::Open)
            .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Open)
            .and_run();

        assert!(!test_bed.pr_valve_is_open(2));
    }

    #[test]
    fn fadec_represents_engine_state() {
        let mut test_bed = test_bed_with()
            .set_engine_state(1, EngineState::Off)
            .set_engine_state(2, EngineState::Off);

        assert_eq!(test_bed.engine_state(1), EngineState::Off);
        assert_eq!(test_bed.engine_state(2), EngineState::Off);

        test_bed = test_bed
            .set_engine_state(1, EngineState::Starting)
            .and_run();

        assert_eq!(test_bed.engine_state(1), EngineState::Starting);
        assert_eq!(test_bed.engine_state(2), EngineState::Off);

        test_bed = test_bed
            .set_engine_state(2, EngineState::Starting)
            .and_run();

        assert_eq!(test_bed.engine_state(1), EngineState::Starting);
        assert_eq!(test_bed.engine_state(2), EngineState::Starting);

        test_bed = test_bed.set_engine_state(1, EngineState::On).and_run();

        assert_eq!(test_bed.engine_state(1), EngineState::On);
        assert_eq!(test_bed.engine_state(2), EngineState::Starting);

        test_bed = test_bed.set_engine_state(2, EngineState::On).and_run();

        assert_eq!(test_bed.engine_state(1), EngineState::On);
        assert_eq!(test_bed.engine_state(2), EngineState::On);

        test_bed = test_bed
            .set_engine_state(1, EngineState::Shutting)
            .and_run();

        assert_eq!(test_bed.engine_state(1), EngineState::Shutting);
        assert_eq!(test_bed.engine_state(2), EngineState::On);

        test_bed = test_bed
            .set_engine_state(2, EngineState::Shutting)
            .and_run();

        assert_eq!(test_bed.engine_state(1), EngineState::Shutting);
        assert_eq!(test_bed.engine_state(2), EngineState::Shutting);
    }

    #[test]
    fn apu_bleed_provides_35_psi_with_open_cross_bleed_valve() {
        let test_bed = test_bed_with()
            .stop_eng1()
            .stop_eng2()
            .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Auto)
            .set_bleed_air_running()
            .and_stabilize();

        assert!(test_bed.cross_bleed_valve_is_open());

        assert!(!test_bed.pr_valve_is_open(1));
        assert!(!test_bed.pr_valve_is_open(2));

        assert!(
            (test_bed.regulated_pressure(1) - Pressure::new::<psi>(35.)).abs()
                < pressure_tolerance()
        );
        assert!(
            (test_bed.regulated_pressure(2) - Pressure::new::<psi>(35.)).abs()
                < pressure_tolerance()
        )
    }

    #[test]
    fn apu_bleed_provides_35_psi_to_left_system_with_closed_cross_bleed_valve() {
        let test_bed = test_bed_with()
            .stop_eng1()
            .stop_eng2()
            .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Shut)
            .set_bleed_air_running()
            .and_stabilize();

        assert!(!test_bed.pr_valve_is_open(1));

        assert!(
            (test_bed.regulated_pressure(1) - Pressure::new::<psi>(35.)).abs()
                < pressure_tolerance()
        );
        assert!(
            (test_bed.regulated_pressure(2) - Pressure::new::<psi>(14.7)).abs()
                < pressure_tolerance()
        )
    }

    mod ovhd {
        use super::*;

        #[test]
        fn ovhd_engine_bleed_push_buttons() {
            let mut test_bed = test_bed().and_run();

            assert!(test_bed.engine_bleed_push_button_is_auto(1));
            assert!(test_bed.engine_bleed_push_button_is_auto(2));

            test_bed = test_bed
                .set_engine_bleed_push_button_off(1)
                .set_engine_bleed_push_button_off(2)
                .and_run();

            assert!(!test_bed.engine_bleed_push_button_is_auto(1));
            assert!(!test_bed.engine_bleed_push_button_is_auto(2));
        }

        #[test]
        fn ovhd_engine_bleed_push_buttons_fault_lights() {
            let mut test_bed = test_bed().and_run();

            assert!(!test_bed.engine_bleed_push_button_has_fault(1));
            assert!(!test_bed.engine_bleed_push_button_has_fault(2));

            test_bed = test_bed
                .set_engine_bleed_push_button_has_fault(1, true)
                .set_engine_bleed_push_button_has_fault(2, true)
                .and_run();

            assert!(test_bed.engine_bleed_push_button_has_fault(1));
            assert!(test_bed.engine_bleed_push_button_has_fault(2));
        }

        #[test]
        fn ovhd_cross_bleed_valve_mode_selector() {
            let mut test_bed = test_bed().and_run();

            assert_eq!(
                test_bed.cross_bleed_valve_selector(),
                CrossBleedValveSelectorMode::Auto
            );

            test_bed = test_bed
                .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Open)
                .and_run();

            assert_eq!(
                test_bed.cross_bleed_valve_selector(),
                CrossBleedValveSelectorMode::Open
            );

            test_bed = test_bed
                .cross_bleed_valve_selector_knob(CrossBleedValveSelectorMode::Shut)
                .and_run();

            assert_eq!(
                test_bed.cross_bleed_valve_selector(),
                CrossBleedValveSelectorMode::Shut
            );
        }
    }
}
