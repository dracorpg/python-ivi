"""

Python Interchangeable Virtual Instrument Library

Copyright (c) 2012-2014 Alex Forencich

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

"""

import time
import struct

from ivi import ivi
from ivi import scope
from ivi import scpi
from ivi import extra

AcquisitionTypeMapping = {
        'normal': 'normal',
        'peak_detect': 'peak',
        'average': 'average'}
AcquisitionAverageCount = (2, 4, 8, 16, 32, 64, 128, 256)
VerticalCouplingMapping = {
        'ac': 'AC1M',
        'dc': 'DC1M',
        'gnd': 'GND'}
VerticalCouplingReverseMapping = {
        'AC1M': 'ac',
        'DC1M': 'dc',
        'DC50': 'dc',
        'GND': 'gnd'}
TriggerTypeMapping = {
        'edge': 'edge',
        'ac_line': 'edge'}
TriggerCouplingMapping = {
        'ac': 'ac',
        'dc': 'dc',
        'hf_reject': 'hf',
        'lf_reject': 'lf'}
SampleModeMapping = {'real_time': 'off',
        'equivalent_time': 'on'}
SlopeMapping = {
        'positive': 'pos',
        'negative': 'neg'}
MeasurementFunctionMapping = {
        'rise_time': 'tr10-90',
        'fall_time': 'tf90-10',
        'frequency': 'freq',
        'period': 'period',
        'voltage_rms': 'vrms',
        'voltage_peak_to_peak': 'p-p',
        'voltage_max': 'max',
        'voltage_min': 'min',
        'voltage_high': 'top',
        'voltage_low': 'base',
        'voltage_average': 'vmean',
        'width_negative': '-width',
        'width_positive': '+width',
        'duty_cycle_positive': 'duty',
        'amplitude': 't-b',
        'voltage_cycle_rms': 'cvrms',
        'voltage_cycle_average': 'cvmean',
        'overshoot': '+oshot',
        'preshoot': '-oshot'}
ScreenshotImageFormatMapping = {
        'bmp': 'BMP',
        'png': 'PNG',
        'tiff': 'TIFF'}
TimebaseModeMapping = {
        'main': 'yt',
        'xy': 'xy',
        'xytrg' : 'xytrg',
        'roll': 'yt'}
ESRBitErrorMapping = {
        4: "Query error",
        8: "Device-specific error",
        16: "Execution error",
        32: "Command error" }

class lecroyWaveJet300(ivi.Driver, scope.Base,
                scope.ContinuousAcquisition, scope.AverageAcquisition,
                scope.WaveformMeasurement, scope.SampleMode,
                scope.AutoSetup, scope.ProbeAutoSense,
                scpi.common.Memory,
                extra.common.SystemSetup, extra.common.Screenshot):
    "Lecroy WaveJet 300/300A series IVI oscilloscope driver"
    
    def __init__(self, *args, **kwargs):
        self.__dict__.setdefault('_instrument_id', '')
        self._channel_count = 4
        self._channel_probe_skew = list()
        self._channel_scale = list()
        self._channel_bw_limit = list()

        super(lecroyWaveJet300, self).__init__(*args, **kwargs)
        
        self._memory_size = 6
        
        self._channel_count = 4
        self._bandwidth = 500e6
        
        self._horizontal_divisions = 10
        self._vertical_divisions = 8
        
        self._timebase_mode = 'main'
        self._timebase_position = 0.0
        self._timebase_range = 1e-3
        self._timebase_scale = 100e-6
        self._display_vectors = True
        
        self._measurement_slots = { 'A': None,
                                    'B': None,
                                    'C': None,
                                    'D': None }
        
        self._identity_description = "Lecroy WaveJet 300/300A series IVI oscilloscope driver"
        self._identity_identifier = ""
        self._identity_revision = ""
        self._identity_vendor = ""
        self._identity_instrument_manufacturer = "Teledyne LeCroy"
        self._identity_instrument_model = ""
        self._identity_instrument_firmware_revision = ""
        self._identity_specification_major_version = 4
        self._identity_specification_minor_version = 1
        self._identity_supported_instrument_models = ['WJ312','WJ314','WJ322','WJ324','WJ332',
                'WJ334','WJ352','WJ354','WJ312A','WJ314A','WJ322A','WJ324A','WJ332A','WJ334A',
                'WJ352A','WJ354A']
        
        ivi.add_property(self, 'channels[].bw_limit',
                        self._get_channel_bw_limit,
                        self._set_channel_bw_limit,
                        None,
                        ivi.Doc("""
                        Commands an internal low-pass filter.  There are three limit options:
                        
                        * False or 'full': no bandwidth limit
                        * True or '20m': 20MHz low-pass filter
                        * '100m': 100MHz low-pass filter
                        """))
        ivi.add_property(self, 'channels[].scale',
                        self._get_channel_scale,
                        self._set_channel_scale,
                        None,
                        ivi.Doc("""
                        Specifies the vertical scale, or units per division, of the channel.  Units
                        are volts.
                        """))
        ivi.add_property(self, 'timebase.mode',
                        self._get_timebase_mode,
                        self._set_timebase_mode,
                        None,
                        ivi.Doc("""
                        Sets the current time base. There are three time base modes:
                        
                        * 'main': normal timebase
                        * 'xy': channels are plotted against each other, no timebase
                        * 'roll': data moves continuously from left to right
                        """))
        ivi.add_property(self, 'timebase.position',
                        self._get_timebase_position,
                        self._set_timebase_position,
                        None,
                        ivi.Doc("""
                        Sets the time interval between the trigger event and the center of the
                        screen. The maximum position value depends on the time/division settings.
                        """))
        ivi.add_property(self, 'timebase.range',
                        self._get_timebase_range,
                        self._set_timebase_range,
                        None,
                        ivi.Doc("""
                        Sets the full-scale horizontal time in seconds for the main window. The
                        range is 10 times the current time-per-division setting.
                        """))
        ivi.add_property(self, 'timebase.scale',
                        self._get_timebase_scale,
                        self._set_timebase_scale,
                        None,
                        ivi.Doc("""
                        Sets the horizontal scale or units per division for the main window.
                        """))
        ivi.add_property(self, 'display.vectors',
                        self._get_display_vectors,
                        self._set_display_vectors,
                        None,
                        ivi.Doc("""
                        When enabled, draws a line between consecutive waveform data points.
                        """))
        
        self._init_channels()
    
    def _initialize(self, resource = None, id_query = False, reset = False, **keywargs):
        "Opens an I/O session to the instrument."

        super(lecroyWaveJet300, self)._initialize(resource, id_query, reset, **keywargs)
        
        self._set_termination_character('\r')
        
        # interface clear
        if not self._driver_operation_simulate:
            self._clear()
        
        # check ID
        if id_query and not self._driver_operation_simulate:
            id = self.identity.instrument_model
            id_check = self._instrument_id
            id_short = id[:len(id_check)]
            if id_short != id_check:
                raise Exception("Instrument ID mismatch, expecting %s, got %s", id_check, id_short)
        
        # reset
        if reset:
            self.utility.reset()
    
    def _load_id_string(self):
        if self._driver_operation_simulate:
            self._identity_instrument_manufacturer = "Not available while simulating"
            self._identity_instrument_model = "Not available while simulating"
            self._identity_instrument_firmware_revision = "Not available while simulating"
        else:
            lst = self._ask("*IDN?").split(",")
            self._identity_instrument_manufacturer = lst[0]
            self._identity_instrument_model = lst[1]
            self._identity_instrument_firmware_revision = lst[3]
            self._set_cache_valid(True, 'identity_instrument_manufacturer')
            self._set_cache_valid(True, 'identity_instrument_model')
            self._set_cache_valid(True, 'identity_instrument_firmware_revision')
    
    def _get_identity_instrument_manufacturer(self):
        if self._get_cache_valid():
            return self._identity_instrument_manufacturer
        self._load_id_string()
        return self._identity_instrument_manufacturer
    
    def _get_identity_instrument_model(self):
        if self._get_cache_valid():
            return self._identity_instrument_model
        self._load_id_string()
        return self._identity_instrument_model
    
    def _get_identity_instrument_firmware_revision(self):
        if self._get_cache_valid():
            return self._identity_instrument_firmware_revision
        self._load_id_string()
        return self._identity_instrument_firmware_revision
    
    def _utility_disable(self):
        pass
    
    def _utility_error_query(self):
        error_code = 0
        error_message = ""
        if not self._driver_operation_simulate:
            error_code = int(self._ask("*ESR?"))
            for mask, message in ESRBitErrorMapping.items():
                if error_code & mask:
                    if error_message:
                        error_message += " + "
                    error_message += message 
        if not error_message:
            error_message = "No error"
        return (error_code, error_message)
    
    def _utility_lock_object(self):
        pass
    
    def _utility_reset(self):
        if not self._driver_operation_simulate:
            self._write("*RST")
            self.driver_operation.invalidate_all_attributes()
    
    def _utility_reset_with_defaults(self):
        self._utility_reset()
    
    def _utility_self_test(self):
        code = 0
        message = "Self test passed"
        if not self._driver_operation_simulate:
            self._write("*TST?")
            # wait for test to complete
            time.sleep(40)
            code = int(self._read())
            if code != 0:
                message = "Self test failed"
        return (code, message)
    
    def _utility_unlock_object(self):
        pass
    
    def _init_channels(self):
        try:
            super(lecroyWaveJet300, self)._init_channels()
        except AttributeError:
            pass
        
        self._channel_name = list()
        self._channel_command_name = list()
        self._channel_scale = list()
        self._channel_bw_limit = list()
        for i in range(self._channel_count):
            self._channel_name.append("CH%d" % (i+1))
            self._channel_command_name.append("C%d" % (i+1))
            self._channel_scale.append(1.0)
            self._channel_bw_limit.append(False)

        self.channels._set_list(self._channel_name)
    
    def _system_fetch_setup(self):
        if self._driver_operation_simulate:
            return b''
        self._write("DTSTUP?")
        return self._read_ieee_block()
    
    def _system_load_setup(self, data):
        if self._driver_operation_simulate:
            return
        self._write("DTSTUP")
        self._write_ieee_block(data)
        self.driver_operation.invalidate_all_attributes()

    def _display_fetch_screenshot(self, format='png'):
        if self._driver_operation_simulate:
            return b''
        if format not in ScreenshotImageFormatMapping:
            raise ivi.ValueNotSupportedException()
        self._write("TSCRN? %s" % ScreenshotImageFormatMapping[format])
        return self._read_ieee_block()

    def _get_timebase_mode(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            if self._ask("ROLL?").lower() == "on":
                self._timebase_mode = 'roll'
            else:
                value = self._ask("XYDS?").lower()
                self._timebase_mode = [k for k,v in TimebaseModeMapping.items() if v==value and k != 'roll'][0]
            self._set_cache_valid()
        return self._timebase_mode
    
    def _set_timebase_mode(self, value):
        if value not in TimebaseModeMapping:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("XYDS %s" % TimebaseModeMapping[value])
            if value == 'roll':
                self._write("ROLL ON")
        self._timebase_mode = value
        self._set_cache_valid()

    def _get_timebase_position(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            self._timebase_position = float(self._ask("TRDL?"))
            self._set_cache_valid()
        return self._timebase_position
    
    def _set_timebase_position(self, value):
        value = float(value)
        if not self._driver_operation_simulate:
            self._write("TRDL %.5e" % value)
        self._timebase_position = value
        self._set_cache_valid()
        
    def _get_timebase_range(self):
        self._get_timebase_scale()
        return self._timebase_range

    def _set_timebase_range(self, value):
        self._set_timebase_scale(value / self._horizontal_divisions)
        
    def _get_timebase_scale(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            self._timebase_scale = float(self._ask("TDIV?"))
            self._timebase_range = round(self._timebase_scale * self._horizontal_divisions, 6)
            self._set_cache_valid()
            self._set_cache_valid(True, 'timebase_range')
        return self._timebase_scale
    
    def _set_timebase_scale(self, value):
        value = float(value)
        if not self._driver_operation_simulate:
            self._write("TDIV %.5e" % value)
        # Coercion algorithm not implemented in driver, so not caching the value
        self._set_cache_valid(False)
        self._set_cache_valid(False, 'timebase_range')

    def _get_display_vectors(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            self._display_vectors = (self._ask("VECT?").lower() == 'on')
            self._set_cache_valid()
        return self._display_vectors
    
    def _set_display_vectors(self, value):
        if not self._driver_operation_simulate:
            self._write("VECT %s" % self._bool_to_onoff(value))
        self._display_vectors = value
        self._set_cache_valid()
    
#TODO
#    def _get_acquisition_start_time(self):
#        if not self._driver_operation_simulate and not self._get_cache_valid():
#            self._acquisition_start_time = float(self._ask(":waveform:xorigin?"))
#            self._set_cache_valid()
#        return self._acquisition_start_time
    
#TODO
#    def _set_acquisition_start_time(self, value):
#        value = float(value)
#        value = value + self._get_acquisition_time_per_record() * 5 / 10
#        if not self._driver_operation_simulate:
#            self._write(":timebase:position %.5e" % value)
#        self._acquisition_start_time = value
#        self._set_cache_valid()
    
    def _get_acquisition_type(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = self._ask("ACQ?").lower()
            self._acquisition_type = [k for k,v in AcquisitionTypeMapping.items() if v==value][0]
            self._set_cache_valid()
        return self._acquisition_type
    
    def _set_acquisition_type(self, value):
        if value not in AcquisitionTypeMapping:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("ACQ %s" % AcquisitionTypeMapping[value])
        self._acquisition_type = value
        self._set_cache_valid()
    
    def _get_acquisition_number_of_points_minimum(self):
        return self._acquisition_number_of_points_minimum
    
    def _set_acquisition_number_of_points_minimum(self, value):
        value = int(value)
        self._acquisition_number_of_points_minimum = value
    
    def _get_acquisition_record_length(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = int(self._suffixed_string_to_float(self._ask("MLEN?")))
            self._acquisition_record_length = value
            self._set_cache_valid()
        return self._acquisition_record_length

    def _get_acquisition_time_per_record(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            self._acquisition_time_per_record = self._get_timebase_range()
            self._set_cache_valid()
        return self._acquisition_time_per_record
    
#TODO
#    def _set_acquisition_time_per_record(self, value):
#        value = float(value)
#        if not self._driver_operation_simulate:
#            self._write(":timebase:range %.5e" % value)
#        self._acquisition_time_per_record = value
#        self._set_cache_valid()
#        self._set_cache_valid(False, 'acquisition_start_time')

    def _get_channel_enabled(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            self._channel_enabled[index] = (self._ask("%s:TRA?" % self._channel_command_name[index]).lower() == ('on'))
            self._set_cache_valid(index=index)
        return self._channel_enabled[index]
    
    def _set_channel_enabled(self, index, value):
        value = bool(value)
        index = ivi.get_index(self._channel_command_name, index)
        if not self._driver_operation_simulate:
            self._write("%s:TRA %s" % (self._channel_command_name[index], self._bool_to_onoff(value)))
        self._channel_enabled[index] = value
        self._set_cache_valid(index=index)
    
    def _get_channel_input_impedance(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            if self._ask("%s:CPL?" % self._channel_command_name[index]).count("50") > 0:
                self._channel_input_impedance[index] = 50
            else:
                self._channel_input_impedance[index] = 1000000
            self._set_cache_valid(index=index)
        return self._channel_input_impedance[index]
    
    def _set_channel_input_impedance(self, index, value):
        value = float(value)
        index = ivi.get_index(self._channel_name, index)
        if not (value == 1000000 or (value == 50 and self._channel_coupling[index] == 'dc')):
            raise Exception('Invalid impedance selection')
        if not self._driver_operation_simulate:
            if self._channel_coupling[index] == 'dc':
                if value == 1000000:
                    self._write("%s:CPL DC1M" % self._channel_command_name[index])
                elif value == 50:
                    self._write("%s:CPL DC50" % self._channel_command_name[index])
        self._channel_input_impedance[index] = value
        self._set_cache_valid(index=index)
    
    def _get_channel_input_frequency_max(self, index):
        index = ivi.get_index(self._channel_name, index)
        return self._channel_input_frequency_max[index]
    
    def _set_channel_input_frequency_max(self, index, value):
        value = float(value)
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate:
            self._set_channel_bw_limit(index, value < 20e6)
        self._channel_input_frequency_max[index] = value
        self._set_cache_valid(index=index)
    
    def _get_channel_probe_attenuation(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            self._channel_probe_attenuation[index] = int(self._ask("%s:PROBE?" % self._channel_command_name[index]).split(',')[1])
            self._set_cache_valid(index=index)
        return self._channel_probe_attenuation[index]
    
    def _set_channel_probe_attenuation(self, index, value):
        index = ivi.get_index(self._channel_name, index)
        value = int(value)
        if not self._driver_operation_simulate:
            self._write("%s:PROBE manual,%d" % (self._channel_command_name[index], value))
        self._channel_probe_attenuation[index] = value
        self._set_cache_valid(False, "channel_probe_attenuation_auto", index)
        self._set_cache_valid(index=index)
    
    def _get_channel_probe_attenuation_auto(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            self._channel_probe_attenuation_auto[index] = (self._ask("%s:PROBE?" % self._channel_command_name[index]).split(',')[0].lower() == 'auto')
            self._set_cache_valid(index=index)
        return self._channel_probe_attenuation_auto[index]
    
    def _set_channel_probe_attenuation_auto(self, index, value):
        index = ivi.get_index(self._channel_name, index)
        if bool(value):
            value = 'auto'
        else:
            value = 'manual'
        if not self._driver_operation_simulate:
            self._write("%s:PROBE %s,1" % (self._channel_command_name[index], value))
        self._channel_probe_attenuation_auto[index] = value
        self._set_cache_valid(False, "channel_probe_attenuation", index)
        self._set_cache_valid(index=index)
    
    def _get_channel_bw_limit(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            value = self._ask("%s:BWL?" % self._channel_command_name[index]).lower()
            if value == 'full':
                value = False
            self._channel_bw_limit[index] = value
            self._set_cache_valid(index=index)
        return self._channel_bw_limit[index]
    
    def _set_channel_bw_limit(self, index, value):
        index = ivi.get_index(self._channel_name, index)
        if type(value) == bool:
            if value:
                value = '20m'
            else:
                value = 'full'
        if value.lower() not in ['full', '20m', '100m']:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("%s:BWL %s" % (self._channel_command_name[index], value))
        self._channel_bw_limit[index] = value
        self._set_cache_valid(index=index)
    
    def _get_channel_coupling(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            self._channel_coupling[index] = VerticalCouplingReverseMapping[self._ask("%s:CPL?" % self._channel_command_name[index])]
            self._set_cache_valid(index=index)
        return self._channel_coupling[index]
    
    def _set_channel_coupling(self, index, value):
        index = ivi.get_index(self._channel_name, index)
        if value not in VerticalCouplingMapping:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("%s:CPL %s" % (self._channel_command_name[index], VerticalCouplingMapping[value]))
        self._channel_coupling[index] = value
        self._set_cache_valid(False, "channel_input_impedance", index)
        self._set_cache_valid(index=index)
    
    def _get_channel_offset(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            self._channel_offset[index] = float(self._ask("%s:OFST?" % self._channel_command_name[index]))
            self._set_cache_valid(index=index)
        return self._channel_offset[index]
    
    def _set_channel_offset(self, index, value):
        index = ivi.get_index(self._channel_name, index)
        value = float(value)
        if not self._driver_operation_simulate:
            self._write("%s:OFST %.5e" % (self._channel_command_name[index], value))
        self._channel_offset[index] = value
        self._set_cache_valid(index=index)
    
    def _get_channel_range(self, index):
        index = ivi.get_index(self._channel_name, index)
        self._get_channel_scale(index)
        return self._channel_range[index]
    
    def _set_channel_range(self, index, value):
        self._set_channel_scale(index, value / self._vertical_divisions)
    
    def _get_channel_scale(self, index):
        index = ivi.get_index(self._channel_name, index)
        if not self._driver_operation_simulate and not self._get_cache_valid(index=index):
            self._channel_scale[index] = float(self._ask("%s:VDIV?" % self._channel_command_name[index]))
            self._channel_range[index] = self._channel_scale[index] * self._vertical_divisions
            self._set_cache_valid(index=index)
            self._set_cache_valid(True, "channel_range", index)
        return self._channel_scale[index]
    
    def _set_channel_scale(self, index, value):
        index = ivi.get_index(self._channel_name, index)
        value = float(value)
        if not self._driver_operation_simulate:
            self._write("%s:VDIV %.5e" % (self._channel_command_name[index], value))
        # Coercion algorithm not implemented in driver, so not caching the value
        self._set_cache_valid(False, index=index)
        self._set_cache_valid(False, "channel_range", index)
    
    def _get_measurement_status(self):
        return self._measurement_status
    
    def _get_trigger_coupling(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = self._ask("TCPL?").lower()
            self._trigger_coupling = [k for k,v in TriggerCouplingMapping.items() if v==value][0]
        return self._trigger_coupling
    
    def _set_trigger_coupling(self, value):
        if value not in TriggerCouplingMapping:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("TCPL %s" % TriggerCouplingMapping[value])
        self._trigger_coupling = value
        self._set_cache_valid()
    
    def _get_trigger_holdoff(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            self._trigger_holdoff = float(self._ask("THTM?"))
            self._set_cache_valid()
        return self._trigger_holdoff
    
    def _set_trigger_holdoff(self, value):
        value = float(value)
        if not self._driver_operation_simulate:
            self._write("THTM %.5e" % value)
        self._trigger_holdoff = value
        self._set_cache_valid()
    
    def _get_trigger_level(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            self._trigger_level = float(self._ask("TLVL?"))
            self._set_cache_valid()
        return self._trigger_level
    
    def _set_trigger_level(self, value):
        value = float(value)
        if not self._driver_operation_simulate:
            self._write("TLVL %.5e" % value)
        self._trigger_level = value
        self._set_cache_valid()
    
    def _get_trigger_edge_slope(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = self._ask("TSLP?").lower()
            self._trigger_edge_slope = [k for k,v in SlopeMapping.items() if v==value][0]
            self._set_cache_valid()
        return self._trigger_edge_slope
    
    def _set_trigger_edge_slope(self, value):
        if value not in SlopeMapping:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("TSLP %s" % SlopeMapping[value])
        self._trigger_edge_slope = value
        self._set_cache_valid()
    
    def _get_trigger_source(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = self._ask("TSRC?")
            if value.startswith('EXT'):
                value = 'external'
            else:
                value = value.lower()
            self._trigger_source = value
            self._set_cache_valid()
        return self._trigger_source
    
    def _set_trigger_source(self, value):
        value = str(value)
        if value.lower() == 'external':
            value = 'EXT10'
        elif not value in self._channel_name:
            raise ivi.UnknownPhysicalNameException()
        if not self._driver_operation_simulate:
            self._write("TSRC %s" % value)
        self._trigger_source = value
        self._set_cache_valid()
    
    def _get_trigger_type(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = self._ask("TTYP?").lower()
            if value == 'edge':
                src = self._ask("TSRC?").lower()
                if src == 'line':
                    value = 'ac_line'
            else:
                value = [k for k,v in TriggerTypeMapping.items() if v==value][0]
            self._trigger_type = value
            self._set_cache_valid()
        return self._trigger_type
    
    def _set_trigger_type(self, value):
        if value not in TriggerTypeMapping:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("TTYP %s" % TriggerTypeMapping[value])
            if value == 'ac_line':
                self._write("TSRC LINE")
        self._trigger_type = value
        self._set_cache_valid()
    
    def _measurement_abort(self):
        pass

#TODO get this to work properly!
    def _measurement_fetch_waveform(self, index):
        index = ivi.get_index(self._channel_name, index)
        
        if self._driver_operation_simulate:
            return list()

        self._write("DTFORM WORD")
        self._write("DTBORD H/L")
        self._write("DTSTART 0")
        self._write("DTPOINTS 500K")
        self._write("WAVESRC %s" % self._channel_name[index])
        
        # Read preamble
        pre = self._ask("DTINF?").split(',')
        points = int(pre[2+4*self._channel_count+5].split('=')[1].lstrip())
        xincrement = self._suffixed_string_to_float(pre[2+4*self._channel_count+2].split('=')[1].lstrip().rstrip('s'))
        tdiv = self._suffixed_string_to_float(pre[2+4*self._channel_count+2].split('=')[1].lstrip().rstrip('s'))
        trigdelay = self._suffixed_string_to_float(pre[2+4*self._channel_count+3].split('=')[1].lstrip().rstrip('s'))
        xorigin = - (tdiv * self._horizontal_divisions / 2 + trigdelay)
        yincrement = self._suffixed_string_to_float(pre[2+4*index+2].split('=')[1].lstrip().rstrip('V'))
        yorigin = self._suffixed_string_to_float(pre[2+4*index+3].split('=')[1].lstrip().rstrip('V'))
        # Probe ratio is not taken into account in the info from DTINF, so do it ourselves (dirty, the ratio might have changed since acquisition...)
        yincrement /= self._get_channel_probe_attenuation(index)
        yorigin /= self._get_channel_probe_attenuation(index)

        self._write("DTWAVE?")

        # Read waveform data
        raw_data = self._read_ieee_block()
        
        # Split out points and convert to time and voltage pairs
        data = list()
        for i in range(points):
            x = (i * xincrement) + xorigin
            # Convert big-endian unsigned word to Python integer
            yval = struct.unpack(">H", raw_data[i*2:i*2+2])[0]
            y = (yval * yincrement) + yorigin            
            data.append((x, y))
        
        return data
    
    def _measurement_read_waveform(self, index, maximum_time):
        self._measurement_initiate(wait=True)
        return self._measurement_fetch_waveform(index)
    
    def _measurement_initiate(self, wait=False):
        if not self._driver_operation_simulate:
            if wait:
                self._write("WSGL?")
            else:
                self._write("WSGL")
            self._set_cache_valid(False, 'trigger_continuous')

    def _measurement_fetch_waveform_measurement(self, index, measurement_function, ref_channel = None):
        index = ivi.get_index(self._channel_name, index)
        if measurement_function not in MeasurementFunctionMapping:
            raise ivi.ValueNotSupportedException()
        measurement_function = MeasurementFunctionMapping[measurement_function]
        if not self._driver_operation_simulate:
            # Look for a measurement slot to use (preferrably one available or already measuring what we want so as not to require overriding an existing measurement)
            selected_slot = None
            must_configure = True
            for slot, value in self._measurement_slots.items():
                # If a slot is already configured for our exact measurement, use it and stop looking
                if value == (self._channel_name[index], measurement_function):
                    selected_slot = slot
                    must_configure = False
                    break
                # If a slot is available (not configured), we might use it, but keep looking for better
                elif value is None:
                    selected_slot = slot
                # If nothing better is found, we'll just have to use this one anyway...
                elif selected_slot is None:
                    selected_slot = slot
            # Configure the measurement slot if it wasn't already just as we need
            if must_configure:
                self._write("MDSP ON")
                self._write("MINMAX OFF")
                self._measurement_slots[selected_slot] = (self._channel_name[index], measurement_function)
                self._write("DIRM %s" % selected_slot)
                self._write("MSEL %s, %s" % self._measurement_slots[selected_slot])
                # Wait half a second so the measurement has at least a small chance to be computed
                time.sleep(0.5)
            value = float(self._ask("MSR%s?" % selected_slot).split(',')[0])
            if value == 9.91e37:
                return float('NaN')
            return value
        return 0

    def _measurement_read_waveform_measurement(self, index, measurement_function, ref_channel = None):
        self._measurement_initiate(wait=True)
        return self._measurement_fetch_waveform_measurement(index, measurement_function, ref_channel)

    def _get_trigger_continuous(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = self._ask("TRMD?").lower()
            self._trigger_continuous = (value == 'auto' or value == 'endless')
            self._set_cache_valid()
        return self._trigger_continuous
    
    def _set_trigger_continuous(self, value):
        value = bool(value)
        if not self._driver_operation_simulate:
            if value:
                self._write("RUN")
            else:
                self._write("STOP")
        self._trigger_continuous = value
        self._set_cache_valid()   
    
    def _get_acquisition_number_of_averages(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            self._acquisition_number_of_averages = int(self._ask("AVGCNT?"))
            self._set_cache_valid()
        return self._acquisition_number_of_averages
    
    def _set_acquisition_number_of_averages(self, value):
        value = int(value)
        if value not in AcquisitionAverageCount:
            raise ivi.OutOfRangeException()
        if not self._driver_operation_simulate:
            self._write("AVGCNT %d" % value)
        self._acquisition_number_of_averages = value
        self._set_cache_valid()
    
    def _get_acquisition_sample_mode(self):
        if not self._driver_operation_simulate and not self._get_cache_valid():
            value = self._ask("EQU?").lower()
            self._acquisition_sample_mode = [k for k,v in SampleModeMapping.items() if v==value][0]
            self._set_cache_valid()
        return self._acquisition_sample_mode
    
    def _set_acquisition_sample_mode(self, value):
        if value not in SampleModeMapping:
            raise ivi.ValueNotSupportedException()
        if not self._driver_operation_simulate:
            self._write("EQU %s" % SampleModeMapping[value])
        self._acquisition_sample_mode = value
        self._set_cache_valid()
    
    def _measurement_auto_setup(self):
        if not self._driver_operation_simulate:
            self._write("ASET")

    def _bool_to_onoff(self, value):
        return "on" if bool(value) else "off"

    def _suffixed_string_to_float(self, string):
        if string.endswith('n'):
            return float(string.rstrip('n'))*1e-9
        elif string.endswith('u'):
            return float(string.rstrip('u'))*1e-6
        elif string.endswith('m'):
            return float(string.rstrip('m'))*1e-3
        elif string.lower().endswith('k'):
            return float(string.rstrip('kK'))*1e3
        elif string.endswith('M'):
            return float(string.rstrip('M'))*1e6
        else:
            return float(string)
