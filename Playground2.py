from PyDAQmx import *
from ctypes import *
import daqface.Utils as Util
import numpy
import matplotlib.pyplot as plt
import time
import pandas as pd

#
# class CallbackTask(Task):
#     def __init__(self):
#         Task.__init__(self)
#         self.datasnip = np.zeros(1000)
#         self.a = []
#         self.CreateAIVoltageChan("Mod2/ai3", "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
#         self.CfgSampClkTiming("", 10000.0, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000)
#
#         self.AutoRegisterEveryNSamplesEvent(DAQmx_Val_Acquired_Into_Buffer, 1000, 0)
#         self.AutoRegisterDoneEvent(0)
#
#         self.windowlength = 20000
#         self.triallength = 60000
#
#     def EveryNCallback(self):
#         read = int32()
#         self.ReadAnalogF64(1000, 10.0, DAQmx_Val_GroupByChannel, self.datasnip, 1000, byref(read), None)
#
#         self.a.extend(self.datasnip.tolist())
#
#         if len(self.a) >= 35000:        # shortest time it takes for odour to arrive (1.5s) + window length (2s)
#             print('a getting big', len(self.a))
#             current_length = len(self.a)
#             self.window = pd.Series(self.a[(current_length-self.windowlength):current_length])
#             self.response = self.AnalyseLicks(self.window, 2, 0.1)
#             if self.response:
#                 self.Complete()
#                 print(self.response)
#                 print(self.window)
#             elif len(self.a) >= self.triallength:
#                 self.Complete()
#                 print(self.response)
#
#
#         return 0 # The function should return an integer
#
#     def DoneCallback(self, status):
#         print("Status", status.value)
#         return 0 # The function should return an integer
#
#     def Complete(self):
#         self.StopTask()
#         self.ClearTask()
#
#     def AnalyseLicks(self, lick_data, threshold, percent_accepted):
#         # first binarise the data
#         lick_response = pd.Series(np.zeros(self.windowlength))
#         lick_response[lick_data > threshold] = 1
#         # then determine percentage responded
#         percent_responded = np.sum(lick_response) / len(lick_response)
#         # return whether this is accepted as a response or not
#         return percent_responded >= percent_accepted
#
#
# task = CallbackTask()
# task.StartTask()
#
# input('Acquiring samples continuously. Press Enter to interrupt\n')
#


class CallbackTask(Task):
    def __init__(self, ai_device, ai_channels, samp_rate, secs, lick_fraction, windowlength, parent):
        Task.__init__(self)
        # self.ai_channels = ai_channels
        # self.analogData = np.zeros((self.ai_channels, self.total_length), dtype=np.float64)
        self.analogData = []
        self.parent = parent

        self.windowlength = samp_rate * windowlength
        self.triallength = secs*samp_rate
        self.lick_fraction = lick_fraction

        self.CreateAIVoltageChan(ai_device, "", DAQmx_Val_Cfg_Default, -10.0, 10.0, DAQmx_Val_Volts, None)

        self.AutoRegisterEveryNSamplesEvent(DAQmx_Val_Acquired_Into_Buffer, 1000, 0)
        self.AutoRegisterDoneEvent(0)

        print("Callback has started")

    def EveryNCallback(self):
#        read = int32()
#        self.ReadAnalogF64(1000, 10.0, DAQmx_Val_GroupByChannel, self.data, 1000, byref(read), None)
#        self.digital_output.WriteDigitalU32(self.sampsPerChan, 0, -1, DAQmx_Val_GroupByChannel, self.write, byref(self.sampsPerChanWritten), None)
        self.analogData.extend(self.data.tolist())
        # print("EveryNCallback is triggered")
        print(len(self.analogData))
        print(self.analogData)

        # if len(self.analogData) < 35000:
        #     print("More samples needed")

            # self.window = numpy.asarray(self.analogData)
            # self.response = self.AnalyseLicks(self.window, 2, self.lick_fraction)

        if len(self.analogData) >= 35000:        # shortest time it takes for odour to arrive (1.5s) + window length (2s)
            print('a getting big', len(self.analogData))
            current_length = len(self.analogData)
            self.window = numpy.asarray(self.analogData[(current_length-self.windowlength):current_length])
            self.response = self.AnalyseLicks(self.window, 2, self.lick_fraction)
            if self.response:
                self.parent.CallbackComplete()
                print(self.response)
                # print(self.window)
            elif len(self.analogData) >= self.triallength:
                self.parent.CallbackComplete()
                print(self.response)
                # print(self.window)

        return 0 # The function should return an integer

    def DoneCallback(self, status):
        print("Status", status.value)
        return 0 # The function should return an integer

    def AnalyseLicks(self, lick_data, threshold, percent_accepted):
        # first binarise the data
        lick_response = numpy.zeros(self.windowlength)
        lick_response[lick_data > threshold] = 1
        # then determine percentage responded
        percent_responded = numpy.sum(lick_response) / len(lick_response)
        # return whether this is accepted as a response or not
        return percent_responded >= percent_accepted


class DoAiMultiTaskCallback:
    def __init__(self, ai_device, ai_channels, do_device, samp_rate, secs, write, sync_clock, lick_fraction, windowlength):

        # create tasks
        self.analog_input = CallbackTask(ai_device, ai_channels, samp_rate, secs, lick_fraction, windowlength, self)
        self.digital_output = Task()

        # add channels
        # self.analog_input.CreateAIVoltageChan(ai_device, "", DAQmx_Val_Diff, -10.0, 10.0, DAQmx_Val_Volts, None)
        self.digital_output.CreateDOChan(do_device, "", DAQmx_Val_ChanForAllLines)

        # self.ai_read = int32()
        self.ai_channels = ai_channels
        self.sampsPerChanWritten = int32()

        self.write = Util.binary_to_digital_map(write)
        self.sampsPerChan = self.write.shape[1]
        self.write = numpy.sum(self.write, axis=0)

        self.totalLength = numpy.int32(samp_rate * secs)
        self.windowlength = numpy.int32(samp_rate * windowlength)
        self.lick_data = []
        self.read = int32()

        # set up sync clock
        self.analog_input.CfgSampClkTiming("", samp_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000)
        self.digital_output.CfgSampClkTiming(sync_clock, samp_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1000)

    def StartThisTask(self):

        # print("Task has been triggered")

        self.digital_output.WriteDigitalU32(self.sampsPerChan, 0, -1, DAQmx_Val_GroupByChannel, self.write,
                                            byref(self.sampsPerChanWritten), None)

        self.digital_output.StartTask()
        self.analog_input.StartTask()

        # print("Task has started")

        self.analog_input.ReadAnalogF64(self.totalLength, -1, DAQmx_Val_GroupByChannel, self.analog_input.data, self.totalLength, byref(self.read), None)


        # self.analog_input.ReadAnalogF64(1000 * self.ai_channels, -1, DAQmx_Val_GroupByChannel, self.analog_input.data, self.totalLength, byref(self.read), None)


    def CallbackComplete(self):
        self.lick_data = numpy.asarray(self.analog_input.analogData)
        print(self.lick_data)
        self.StopAll()

    def StopAll(self):
        self.analog_input.StopTask()
        self.digital_output.StopTask()
        self.analog_input.ClearTask()
        self.digital_output.ClearTask()


task = DoAiMultiTaskCallback("Mod2/ai3", "", "Mod1/port0/line0", 10000.0, 6.0, numpy.zeros((2, 1000)), "/cDaQ/ai/SampleClock", 0.1, 2.0)
task.StartThisTask()
