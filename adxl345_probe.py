from . import probe, adxl345

# ADXL345 register addresses
REG_INT_MAP = 0x2F
REG_INT_ENABLE = 0x2E
REG_INT_SOURCE = 0x30
REG_THRESH_ACT = 0x24
REG_ACT_INACT_CTL = 0x27

ADXL345_REST_TIME = 0.1

Commands = {'SET_ACCEL_PROBE: 1'}

# ADXL "endstop" wrapper
class ADXL345Probe:
    def __init__(self, config, probe_offsets, param_helper):
        self.printer = config.get_printer()
        adxl345_name = config.get("chip", "adxl345")
        self.adxl345 = self.printer.lookup_object(adxl345_name)

        # get variables from .cfg
        # zzz change act_thresh_z to thresh_act to match adxl datasheet. 8 bits unsigned. The scale factor is 62.5 mg/LSB. A value of 0 may result in undesirable behavior if the activity interrupt is enabled
        # zzz should validate that it's an int?
        self.act_thresh_z = config.getfloat("act_thresh_z", minval=1, maxval=255)
        
        # Create an "endstop" object to handle the sensor pin
        # change probe_pin to sensor_pin to match bltouch
        ppins = self.printer.lookup_object('pins')
        self.mcu_endstop = ppins.setup_pin('endstop', config.get('probe_pin'))




        # from .cfg get and validate the adxl345 interrupt pin (int1 or int2)
        int_pin = config.get("int_pin").strip()
        if int_pin != "int1" and int_pin != "int2":
            raise config.error("int_pin must be specified with either int1 or int2")
        
        # used later to init adxl registers
        self.int_map = 0b01000000 if int_pin == "int2" else 0b00000000

        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop

        # Probing via homing to endstop
        self.homing_helper = probe.DescendToEndstopHelper(config, self, probe_offsets, param_helper, always_check_movement=True)
        # zzz always_check_movement=False in original

        # multi probes state
        self.multi = 'OFF'

        # # Register commands
        # self.gcode = self.printer.lookup_object('gcode')
        # self.gcode.register_mux_command("SET_ACCEL_PROBE", self.cmd_SET_ACCEL_PROBE, desc=self.cmd_SET_ACCEL_PROBE_help, )
        # self.gcode.register_command("HOTEND_FAN_ON", self.cmd_HOTEND_FAN_ON, desc=self.cmd_HOTEND_FAN_ON_help)
        # self.gcode.register_command("HOTEND_FAN_OFF", self.cmd_HOTEND_FAN_OFF, desc=self.cmd_HOTEND_FAN_OFF_help)
        
        # Register events
        self.printer.register_event_handler("klippy:connect", self._init_adxl)

    # initializes the adxl345 with register values. Sets it in activity mode.
    def _init_adxl(self, axis=None):
        chip = self.adxl345
        chip.set_reg(adxl345.REG_POWER_CTL, 0b00000000)       # Set to standby mode. It is recommended to configure the device in standby mode.
        chip.set_reg(adxl345.REG_DATA_FORMAT, 0b00001011)     # SELF_TEST off, SPI 4 wire mode, Interrupts active high, FULL_RES in full resolution mode, Justify 1, Range 11 (+/-16g)
        
        # zz bring the if int1 int2 and register values down to here?
        chip.set_reg(REG_INT_MAP, self.int_map)

        chip.set_reg(REG_ACT_INACT_CTL, 0b11110000) # Activity AC-coupled operation (cancels out gravity). Enable all 3 axes for Activity mode.

        act_thresh = self.act_thresh_z
        chip.set_reg(REG_THRESH_ACT, int(act_thresh))   # Threshold value for detecting activity. Scale factor is 62.5 mg/LSB. Value of 0 may result in undesirable behavior if the act int is enabled.

        # the adxl should be prepared to measure now, only the interrupt needs to be enabled with below?
        # chip.set_reg(REG_INT_ENABLE, 0b00010000, minclock=clock)  # Enable interrupt activity mode.
    

    def run_probe(self, gcmd):
        self._probe_prepare()
        try:
            self.homing_helper.descend_until_trigger(gcmd)
        except self.printer.command_error as e:
            self._probe_finish()
            raise
        self._probe_finish()
    

    def _probe_prepare(self):
        chip = self.adxl345

        chip.set_reg(adxl345.REG_POWER_CTL, 0x08)

        
        #self.activate_gcode.run_gcode_from_command()
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()
        print_time = toolhead.get_last_move_time()
        clock = self.adxl345.mcu.print_time_to_clock(print_time)
        chip.set_reg(REG_INT_ENABLE, 0x00) # If we could only get rid of the minclock=clock, sometimes it goes wicked-fast! But sometimes ends up "triggered prior to movement".
        if not self._try_clear_int():
            raise self.printer.command_error(
                "ADXL345 triggered before move, it may be set too sensitive."
            )
        chip.set_reg(REG_INT_ENABLE, self.int_reg_value, minclock=clock) # Enables either TAP or ACT



    def start_probe_session(self, gcmd):
        self.homing_helper.clear_trigger_positions()
        return self



    def _probe_finish(self): # We want this function to run and finish as quickly as possible, so the probe can be pulled up away from the bed.

        chip = self.adxl345
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.dwell(ADXL345_REST_TIME)
        chip.set_reg(adxl345.REG_POWER_CTL, 0b00000000)       # Set to standby mode.







# Main external probe interface
class PrinterADXL345:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        self.param_helper = probe.ProbeParameterHelper(config)
        self.mcu_probe = ADXL345Probe(config, self.probe_offsets, self.param_helper)
        self.probe_session = probe.SampleAveragingHelper(config, self.param_helper, self.mcu_probe.start_probe_session)
        query_endstop = self.mcu_probe.query_endstop
        self.cmd_helper = probe.ProbeCommandHelper(config, self, query_endstop)
        probe.HomingViaProbeHelper(config, self.probe_offsets.get_offsets()[2], query_endstop)
    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)
    def get_offsets(self, gcmd=None):
        return self.probe_offsets.get_offsets(gcmd)
    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)
    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

def load_config(config):
    adxl = PrinterADXL345(config)
    config.get_printer().add_object('probe', adxl)
    return adxl
