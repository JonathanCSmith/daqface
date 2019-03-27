# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 13:51:48 2015

@author: Andrew Erskine
"""

# region [Import]
import queue

from PyDAQmx import *
from PyDAQmx.DAQmxCallBack import *
from ctypes import *
import daqface.Utils as Util
import numpy
import matplotlib.pyplot as plt
import time
import threading

# region [DigitalTasks]

# JON
from Threading.synchronized import synchronized_with_attr


class DigitalInput(Task):
    def __init__(self, device, channels, samprate, secs, clock=''):
        Task.__init__(self)
        self.CreateDIChan(device, "", DAQmx_Val_ChanPerLine)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.digitalData = numpy.ones((channels, self.totalLength), dtype=numpy.uint32)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        print('Starting digital input')
        self.StartTask()
        self.ReadDigitalU32(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.digitalData,
                            self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


class TriggeredDigitalInput(Task):
    def __init__(self, device, channels, samprate, secs, trigger_source, clock=''):
        Task.__init__(self)
        self.CreateDIChan(device, "", DAQmx_Val_ChanPerLine)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.digitalData = numpy.zeros((channels, self.totalLength), dtype=numpy.uint32)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.CfgDigEdgeStartTrig(trigger_source, DAQmx_Val_Rising)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.StartTask()
        self.ReadDigitalU32(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.digitalData,
                            self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        print(status.value)
        self.StopTask()
        self.ClearTask()
        return 0


class DigitalOut(Task):
    def __init__(self, device, samprate, secs, write, clock=''):
        Task.__init__(self)
        self.CreateDOChan(device, "", DAQmx_Val_ChanPerLine)

        self.sampsPerChanWritten = int32()
        self.totalLength = samprate * secs
        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))

        self.AutoRegisterDoneEvent(0)

        self.write = Util.binaryToDigitalMap(write)

    def DoTask(self):
        print ('Starting digital output')
        self.WriteDigitalU32(self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        self.StartTask()

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


class ThreadSafeDigitalOut:
    def __init__(self, device, samprate, secs, write, clock=''):
        self.do_handle = TaskHandle(0)

        DAQmxCreateTask("", byref(self.do_handle))

        DAQmxCreateDOChan(self.do_handle, device, '', DAQmx_Val_ChanPerLine)

        self.sampsPerChanWritten = int32()
        self.write = Util.binary_to_digital_map(write)

        self.totalLength = numpy.uint64(samprate * secs)

        DAQmxCfgSampClkTiming(self.do_handle, clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxWriteDigitalU32(self.do_handle, self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.do_handle)
        DAQmxWaitUntilTaskDone(self.do_handle, DAQmx_Val_WaitInfinitely)

        self.ClearTasks()

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.do_handle)

        DAQmxClearTask(self.do_handle)


# region [AnalogTasks]


class AnalogInput(Task):
    def __init__(self, device, channels, samprate, secs, clock=''):
        Task.__init__(self)
        self.CreateAIVoltageChan(device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, None)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.analogRead = numpy.zeros((channels, self.totalLength), dtype=numpy.float64)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.StartTask()
        self.ReadAnalogF64(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogRead,
                           self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        self.StopTask()
        self.ClearTask()
        return 0


class ThreadSafeAnalogInput:
    def __init__(self, ai_device, channels, samp_rate, secs, clock=''):
        self.ai_handle = TaskHandle(0)

        DAQmxCreateTask("", byref(self.ai_handle))

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)

        self.ai_read = int32()
        self.ai_channels = channels
        self.totalLength = numpy.uint64(samp_rate * secs)
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxStartTask(self.ai_handle)
        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           numpy.uint32(self.ai_channels*self.totalLength), byref(self.ai_read), None)
        self.ClearTasks()
        return self.analogData

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.ai_handle)
        DAQmxClearTask(self.ai_handle)


class TriggeredAnalogInput(Task):
    def __init__(self, device, channels, samprate, secs, trigger_source, clock=''):
        Task.__init__(self)
        self.CreateAIVoltageChan(device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, None)

        self.read = int32()
        self.channels = channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.analogRead = numpy.zeros((channels, self.totalLength), dtype=numpy.float64)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.WaitUntilTaskDone(-1)
        self.CfgDigEdgeStartTrig(trigger_source, DAQmx_Val_Rising)
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.StartTask()
        self.ReadAnalogF64(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogRead,
                           self.totalLength * self.channels, byref(self.read), None)

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


class AnalogOutput(Task):
    def __init__(self, device, samprate, secs, write, clock=''):
        Task.__init__(self)
        self.CreateAOVoltageChan(device, "", -10.0, 10.0, DAQmx_Val_Volts, None)

        self.sampsPerChanWritten = int32()
        self.write = write
        self.totalLength = numpy.uint32(samprate * secs)

        self.CfgSampClkTiming(clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, numpy.uint64(self.totalLength))
        self.AutoRegisterDoneEvent(0)

    def DoTask(self):
        self.WriteAnalogF64(self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel,
                            self.write, byref(self.sampsPerChanWritten), None)
        self.StartTask()

    def DoneCallback(self, status):
        print(status)
        self.StopTask()
        self.ClearTask()
        return 0


# region [MultiTasks]


class DoAiMultiTask:
    def __init__(self, ai_device, ai_channels, do_device, samp_rate, secs, write, sync_clock):
        self.ai_handle = TaskHandle(0)
        self.do_handle = TaskHandle(1)

        DAQmxCreateTask('', byref(self.ai_handle))
        DAQmxCreateTask('', byref(self.do_handle))

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, '', DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
        DAQmxCreateDOChan(self.do_handle, do_device, '', DAQmx_Val_ChanForAllLines)

        self.ai_read = int32()
        self.ai_channels = ai_channels
        self.sampsPerChanWritten = int32()

        self.write = Util.binary_to_digital_map(write)
        self.sampsPerChan = self.write.shape[1]
        self.write = numpy.sum(self.write, axis=0)

        self.totalLength = numpy.uint64(samp_rate * secs)
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        DAQmxCfgSampClkTiming(self.do_handle, sync_clock, samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxWriteDigitalU32(self.do_handle, self.sampsPerChan, 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.do_handle)
        DAQmxStartTask(self.ai_handle)

        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           numpy.uint32(self.ai_channels*self.totalLength), byref(self.ai_read), None)

        self.ClearTasks()
        return self.analogData

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.do_handle)
        DAQmxStopTask(self.ai_handle)

        DAQmxClearTask(self.do_handle)
        DAQmxClearTask(self.ai_handle)


class AoAiMultiTask:
    def __init__(self, ai_device, ai_channels, ao_device, samprate, secs, write, sync_clock):
        self.ai_handle = TaskHandle(0)
        self.ao_handle = TaskHandle(1)

        DAQmxCreateTask("", byref(self.ai_handle))
        DAQmxCreateTask("", byref(self.ao_handle))

        self.sampsPerChanWritten = int32()
        self.write = write
        self.totalLength = numpy.uint32(samprate * secs)

        self.ai_read = int32()
        self.ai_channels = ai_channels
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts,
                                 None)
        DAQmxCreateAOVoltageChan(self.ao_handle, ao_device, "", -10.0, 10.0, DAQmx_Val_Volts, None)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        DAQmxCfgSampClkTiming(self.ao_handle, sync_clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxWriteAnalogF64(self.ao_handle, self.write.shape[1], 0, -1, DAQmx_Val_GroupByChannel,
                            self.write, byref(self.sampsPerChanWritten), None)

        DAQmxStartTask(self.ao_handle)
        DAQmxStartTask(self.ai_handle)

        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           numpy.uint32(self.ai_channels*self.totalLength), byref(self.ai_read), None)

        self.ClearTasks()
        return self.analogData

    def ClearTasks(self):
        time.sleep(0.05)
        DAQmxStopTask(self.ao_handle)
        DAQmxStopTask(self.ai_handle)

        DAQmxClearTask(self.ao_handle)
        DAQmxClearTask(self.ai_handle)



class MultiTask:
    def __init__(self, ai_device, ai_channels, di_device, di_channels, do_device, samprate, secs, write, sync_clock):
        self.ai_handle = TaskHandle(0)
        self.di_handle = TaskHandle(1)
        self.do_handle = TaskHandle(2)

        DAQmxCreateTask("", byref(self.ai_handle))
        DAQmxCreateTask("", byref(self.di_handle))
        DAQmxCreateTask("", byref(self.do_handle))

        # NOTE - Cfg_Default values may differ for different DAQ hardware
        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts,
                                 None)
        DAQmxCreateDIChan(self.di_handle, di_device, "", DAQmx_Val_ChanPerLine)

        self.ai_read = int32()
        self.di_read = int32()
        self.ai_channels = ai_channels
        self.di_channels = di_channels
        self.totalLength = numpy.uint32(samprate * secs)
        self.analogData = numpy.zeros((self.ai_channels, self.totalLength), dtype=numpy.float64)
        self.digitalData = numpy.ones((self.di_channels, self.totalLength), dtype=numpy.uint32)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))
        DAQmxCfgSampClkTiming(self.di_handle, sync_clock, samprate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              numpy.uint64(self.totalLength))

    def DoTask(self):
        DAQmxStartTask(self.di_handle)
        DAQmxStartTask(self.ai_handle)

        DAQmxReadAnalogF64(self.ai_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analogData,
                           self.totalLength * self.ai_channels, byref(self.ai_read), None)
        DAQmxReadDigitalU32(self.di_handle, self.totalLength, -1, DAQmx_Val_GroupByChannel, self.digitalData,
                            self.totalLength * self.di_channels, byref(self.di_read), None)

# Sliding Window for Lick Analysis - Cristina Marin, April 2018

class DoAiCallbackTask:
    def __init__(self, ai_device, ai_channels, do_device, samp_rate, secs, write, sync_clock, samps_per_callback,
                 response_length_secs, response_start_secs, lick_fraction, lick_channel):
        self.ai_handle = TaskHandle(0)
        self.do_handle = TaskHandle(1)
        self.ai_channels = ai_channels
        self.total_length = numpy.uint64(samp_rate * secs)
        self.samps_per_callback = samps_per_callback
        self.callback_counter = 0
        self.response_length = response_length_secs * samp_rate
        self.response_start = response_start_secs * samp_rate
        self.lick_fraction = lick_fraction
        self.response_window = []
        self.last_pos = 0
        self.trial_length = samp_rate * secs
        self.lick_channel = lick_channel

        # set up data buffers (analog)
        self.analog_data = numpy.zeros((self.ai_channels, self.total_length), dtype=numpy.float64)
        self.ai_read = int32()
        self.samps_per_chan_written = int32()

        # set up data buffers (digital)
        self.write = Util.binary_to_digital_map(write)
        self.samps_per_chan = self.write.shape[1]
        self.write = numpy.sum(self.write, axis=0)

        # data pointer to make callback happy
        self.data_pointer = create_callbackdata_id(self.analog_data)

        # set up tasks
        DAQmxCreateTask('', byref(self.ai_handle))
        DAQmxCreateTask('', byref(self.do_handle))

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, '', DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
        DAQmxCreateDOChan(self.do_handle, do_device, '', DAQmx_Val_ChanForAllLines)

        DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, self.total_length)
        DAQmxCfgSampClkTiming(self.do_handle, sync_clock, samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
                              self.total_length)

        self.start = time.time()

        # JON
        self.lock = threading.RLock()

    @synchronized_with_attr("lock")
    def DoCallback(self, handle, every_n_samples_event_type, n_samples, data_pointer):
        self.callback_counter += 1
        print(self.callback_counter)
        self.data_of_interest = self.analog_data[self.lick_channel][0:(self.samps_per_callback * self.callback_counter)]

        current_pos = self.samps_per_callback * self.callback_counter
        self.response_window = self.data_of_interest[(current_pos - self.response_length):current_pos]

        if current_pos >= (self.response_start + self.response_length):
            response = self.AnalyseLicks(self.response_window, 2, self.lick_fraction)
            print(response)
            print(time.time() - self.start)
            if response:
                print('animal licked')
                self.last_pos = current_pos
                DAQmxTaskControl(self.ai_handle, DAQmx_Val_Task_Abort)
                DAQmxTaskControl(self.do_handle, DAQmx_Val_Task_Abort)
                #DAQmxTaskControl(self.ai_handle, DAQmx_Val_Task_Stop)
                #DAQmxTaskControl(self.do_handle, DAQmx_Val_Task_Stop)
                self.ClearTasks()

            elif current_pos == self.trial_length:
                print('time is up')
                self.last_pos = current_pos
                DAQmxTaskControl(self.ai_handle, DAQmx_Val_Task_Abort)
                DAQmxTaskControl(self.do_handle, DAQmx_Val_Task_Abort)
                #DAQmxTaskControl(self.ai_handle, DAQmx_Val_Task_Stop)
                #DAQmxTaskControl(self.do_handle, DAQmx_Val_Task_Stop)
                self.ClearTasks()


        # if current_pos > self.response_start:
        #     print('stopping')
        #     self.last_pos = current_pos
        #     DAQmxTaskControl(self.ai_handle, DAQmx_Val_Task_Abort)
        #     DAQmxTaskControl(self.do_handle, DAQmx_Val_Task_Abort)
        #     # self.ClearTasks()
        #     return 0

        return 0

    def DoTask(self):
        # Define and register callback function
        EveryNCallback = DAQmxEveryNSamplesEventCallbackPtr(self.DoCallback)
        DAQmxRegisterEveryNSamplesEvent(self.ai_handle, DAQmx_Val_Acquired_Into_Buffer, self.samps_per_callback, 0, EveryNCallback, self.data_pointer)

        # Start tasks
        DAQmxWriteDigitalU32(self.do_handle, self.samps_per_chan, 0, -1, DAQmx_Val_GroupByChannel, self.write,
                             byref(self.samps_per_chan_written), None)

        DAQmxStartTask(self.do_handle)
        DAQmxStartTask(self.ai_handle)

        try:
            DAQmxReadAnalogF64(self.ai_handle, self.total_length, -1, DAQmx_Val_GroupByChannel, self.analog_data,
                               numpy.uint32(self.ai_channels * self.total_length), byref(self.ai_read), None)
            print(self.start - time.time())
            print("Reading")
        except:
            print("Reading causes an error")

        # self.ClearTasks()
        return self.analog_data

    def AnalyseLicks(self, lick_data, threshold, percent_accepted):
        # first binarise the data
        lick_response = numpy.zeros(self.response_length)
        lick_response[lick_data > threshold] = 1
        # then determine percentage responded
        percent_responded = numpy.sum(lick_response) / len(lick_response)
        # return whether this is accepted as a response or not
        return percent_responded >= percent_accepted

    def ClearTasks(self):
        print('clearing tasks')
        time.sleep(0.05)
        DAQmxStopTask(self.ai_handle)
        DAQmxStopTask(self.do_handle)
        DAQmxClearTask(self.ai_handle)
        DAQmxClearTask(self.do_handle)


class Workable:
    def do_work(self):
        pass


class FinishWork(Workable):
    def __init__(self, daq, callback):
        self.daq = daq
        self.callback = callback

    def do_work(self):
        self.daq.ClearTask()
        self.callback.has_finished = True


class WorkToDo(Workable):
    def __init__(self, daq, callback, data, from_index, to_index):
        self.daq = daq
        self.callback = callback
        self.data = data
        self.from_index = from_index
        self.to_index = to_index

    def do_work(self):
        # Assign the data to our global store
        self.daq.assign_data(self.data, self.from_index, self.to_index)

        # Test our latest window
        self.daq.test_window(self.from_index, self.to_index)

        # Increment after so we are correctly indexed
        self.callback.callback_counter += 1
        print("Callback count: " + str(self.callback.callback_counter))

        # Handle outcomes
        if self.daq.shutting_down:
            print("We are ending because we have a successful lick.")
            self.callback.q.put(FinishWork(self.daq, self.callback))

        if self.callback.callback_counter >= self.callback.max_callbacks:
            print("We are ending because we have sampled enough.")
            self.callback.q.put(FinishWork(self.daq, self.callback))


class CallbackInterceptor:
    def __init__(self, daq, samples_per_callback, max_callbacks):
        self.daq = daq
        self.is_being_deleted = False

        # Callback props
        self.max_callbacks = max_callbacks
        self.has_finished = False
        self.callback_counter = 0
        self.samples_per_callback = samples_per_callback

        # Queue
        self.q = queue.Queue()

        # TODO: We could queue callbacks instead of locking
        #self.lock = threading.RLock()

    #@synchronized_with_attr("lock")  # Don't need a lock any more as we just add our work to the queue
    def DoCallback(self, handle, every_n_samples_event_type, n_samples, data_pointer):
        # This check allows registered callbacks to resolve without throwing memory errors
        if self.has_finished or self.daq.shutting_down:
            return

        # Get the data and put it into our daq object
        callback_data = get_callbackdata_from_id(data_pointer)
        from_index = self.callback_counter * self.samples_per_callback
        to_index = from_index + self.samples_per_callback
        # self.daq.acquire_data(callback_data, from_index, to_index)
        data = self.daq.acquire_data()

        # Queue our work
        work = WorkToDo(self.daq, self, data, from_index, to_index)
        self.q.put(work)

        # # Test our latest window
        # self.daq.test_window(from_index, to_index)
        # if self.daq.shutting_down:
        #     self.daq.ClearTask()
        #     self.has_finished = True
        #
        # # Increment after so we are correctly indexed
        # self.callback_counter += 1
        # self.current_length += self.samples_per_callback
        # if self.current_length >= self.total_length:
        #     self.has_finished = True

        return 0


class JonTask:
    def __init__(self, ai_device, ai_channels, do_device, samp_rate, secs, write, sync_clock, samps_per_callback, response_length_secs, response_start_secs, lick_fraction, lick_channel):
        self.ai_handle = TaskHandle(0)
        self.do_handle = TaskHandle(1)
        self.ai_channels = ai_channels
        self.total_length = numpy.uint64(samp_rate * secs)
        self.samps_per_callback = samps_per_callback
        self.response_length = response_length_secs * samp_rate
        self.response_start = response_start_secs * samp_rate
        self.lick_fraction = lick_fraction
        self.response_window = []
        self.last_pos = 0
        self.trial_length = samp_rate * secs
        self.lick_channel = lick_channel
        # self.data_of_interest = numpy.zeros(self.total_length, dtype=numpy.float64)

        # set up data buffers (analog)
        self.analog_data = numpy.zeros((self.ai_channels, self.total_length), dtype=numpy.float64)
        self.ai_read = int32()
        self.samps_per_chan_written = int32()

        # set up data buffers (digital)
        self.write = Util.binary_to_digital_map(write)
        self.samps_per_chan = self.write.shape[1]
        self.write = numpy.sum(self.write, axis=0)

        # data pointer to make callback happy
        self.data_pointer = create_callbackdata_id(self.analog_data)

        # set up tasks
        DAQmxCreateTask('', byref(self.ai_handle))
        DAQmxCreateTask('', byref(self.do_handle))

        DAQmxCreateAIVoltageChan(self.ai_handle, ai_device, '', DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
        DAQmxCreateDOChan(self.do_handle, do_device, '', DAQmx_Val_ChanForAllLines)

        #DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, self.total_length)
        DAQmxCfgSampClkTiming(self.ai_handle, '', samp_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, numpy.uint32(self.ai_channels * self.samps_per_callback))
        DAQmxCfgSampClkTiming(self.do_handle, sync_clock, samp_rate, DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, self.total_length)

        self.start = time.time()
        self.shutting_down = False
        self.debug = False

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.ClearTask()

    def __del__(self):
        """ Clear automatically the task to be able to reallocate resources """
        # Clear the task before deleting the object
        # If the task as already been manually cleared, nothing is done
        # This prevent to clear a task that has a Handle attributed to a new task
        # See this example
        # a = Task(), ..., a.ClearTask(), b = Task(), del(a)
        # b has the same taskHandle as a, and deleting a will clear the task of b
        try:
            self.ClearTask()
        except Exception:
            pass

    def ClearTask(self):
        if self.ai_handle:
            try:
                DAQmxClearTask(self.ai_handle)
            finally:
                self.ai_handle.value = 0

        if self.do_handle:
            try:
                DAQmxClearTask(self.do_handle)
            finally:
                self.do_handle.value = 1

    def __repr__(self):
        if self.ai_handle and self.do_handle:
            return "Task number %d and %d" % (self.ai_handle.value, self.do_handle.value)
        else:
            return "Invalid or cleared Task"

    def acquire_data(self):
        # Grab our data
        data = numpy.zeros((self.ai_channels, self.samps_per_callback), dtype=float64)  # Buffer
        DAQmxReadAnalogF64(self.ai_handle, self.samps_per_callback, -1, DAQmx_Val_GroupByChannel, data, numpy.uint32(self.ai_channels * self.samps_per_callback), byref(self.ai_read), None)

        locs = data == 0
        count = numpy.sum(locs)

        if count > 0:
            print("Data had zeros present....")

        # In place substitute data
        # callback_data[0 : self.ai_channels, from_index : to_index] = data
        return data

    def assign_data(self, data, from_index, to_index):
        self.analog_data[0: self.ai_channels, from_index: to_index] = data

    def test_window(self, from_index, to_index):
        # Run our analyse licks function
        mindex = to_index - self.response_length
        if mindex < 0:
            mindex = 0
        data_of_interest = self.analog_data[self.lick_channel][mindex: to_index]

        # Assess the responses using a sliding window over out current data fraction, allowing for a response length upstream
        below_threshold = data_of_interest > 2  # Parameterize
        count = numpy.sum(below_threshold)
        percentage = count / self.response_length

        # Detect signals of a specific shape smaller than the most recent data acquisition. If its larger we can just flat check it
        # shape = data_of_interest.shape[:-1] + (data_of_interest.shape[-1] - self.response_length + 1, self.response_length)
        # strides = data_of_interest.strides + (data_of_interest.strides[-1], )
        # values_in_windows = numpy.lib.stride_tricks.as_strided(data_of_interest, shape=shape, strides=strides)
        # window_totals = numpy.sum(values_in_windows, axis=1)
        # window_percentages = window_totals / self.response_length
        # valid_response_count = numpy.sum(window_percentages >= self.lick_fraction)

        if self.debug:
            self.shutting_down = numpy.random.rand() > 0.95
            return

        if percentage > self.lick_fraction:
            print("We have a lick!")
            self.shutting_down = True

    def DoTask(self):
        # Calculate the max number of callbacks
        total_samples = self.total_length
        number_of_callbacks = total_samples / self.samps_per_callback

        # Define and register callback function
        callback_interceptor = CallbackInterceptor(self, self.samps_per_callback, number_of_callbacks)
        EveryNCallback = DAQmxEveryNSamplesEventCallbackPtr(callback_interceptor.DoCallback)
        DAQmxRegisterEveryNSamplesEvent(self.ai_handle, DAQmx_Val_Acquired_Into_Buffer, self.samps_per_callback, 0, EveryNCallback, self.data_pointer)

        # Define write action
        DAQmxWriteDigitalU32(self.do_handle, self.samps_per_chan, 0, -1, DAQmx_Val_GroupByChannel, self.write, byref(self.samps_per_chan_written), None)

        # Start tasks
        DAQmxStartTask(self.do_handle)
        DAQmxStartTask(self.ai_handle)

        # Wait until we have taken enough samples or we have received a response
        print("start time: " + str(time.time() - self.start))
        while not callback_interceptor.has_finished:
            if callback_interceptor.q.qsize() > 0:

                # Shouldn't happen as we only have one worker thread but just in case
                try:
                    work_item = callback_interceptor.q.get()
                except queue.Empty:
                    time.sleep(0.01)
                    continue

                # Process our workload (i.e. dump data into daq analogue input & test)
                work_item.do_work()

            # Q is empty, wait for a callback to come in
            else:
                time.sleep(0.01)
        print("end time: " + str(time.time() - self.start))

        # self.ClearTasks()
        callback_interceptor.is_being_deleted = True
        return self.analog_data


# TODO TESTING
# region DoAiMultiTaskTest
# a = DoAiMultiTask('cDAQ1Mod3/ai0', 1, 'cDAQ1Mod1/port0/line0', 1000.0, 1.0, numpy.zeros((2, 1000)),
#                   '/cDAQ1/ai/SampleClock')
# analog = a.DoTask()
#
# plt.plot(analog[0])
# plt.show()
# endregion

# region simple digital test
# DigitalOutput test
# a = DigitalOut('cDAQ1Mod1/port0/line0:1', 1, 1000, numpy.zeros((2, 1000)), clock='')
# a.DoTask()

# DigitalInput test
# a = DigitalInput('cDAQ1Mod2/port0/line0', 1, 1000, 1)
# a.DoTask()
# endregion

# MultiTask test
# a = MultiTask('cDAQ1Mod3/ai0', 1, 'cDAQ1Mod2/port0/line0', 1, 'cDAQ1Mod1/port0/line0', 1000, 2, numpy.zeros((1, 2000),
#               dtype=numpy.uint32), '/cDAQ1/ai/SampleClock')
#
# a.DoTask()
#
# plt.plot(a.digitalData[0])
# plt.show()

# AnalogInput
# a = AnalogInput('cDAQ1Mod3/ai0', 1, 1000, 1)
# a.DoTask()
#
# plt.plot(a.analogRead[0])
# plt.show()
