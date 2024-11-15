#!/bin/python3
import cv2 as cv
from pypylon import pylon

class VideoCapture:
    def __init__(self, camera_id):
        """
        DESCRIPTION:
            Initialize VideoCapture class.
        ARGUMENTS:
            camera_id (str): String representing camera id like the cv2.VideoCapture() class.
            logger (logging.Logger): Logger object for logging events and logs for debugging.
        RETURNS:
        """
        self.camera_id = camera_id
        #self.logger = logger
        self.factory = pylon.TlFactory.GetInstance()
        print([camera.GetFriendlyName() for camera in self.factory.EnumerateDevices()])
        #self.logger.debug([camera.GetFriendlyName() for camera in self.factory.EnumerateDevices()])
        # if camera_id > len(self.factory.EnumerateDevices()) - 1:
        #     #self.logger.error(f'TRYING TO ACCESS CAMERA ID {camera_id} FROM A LIST OF LENGTH {len(self.factory.EnumerateDevices())}')
        #     raise IndexError(f'TRYING TO ACCESS CAMERA ID {camera_id} FROM A LIST OF LENGTH {len(self.factory.EnumerateDevices())}')
        self.device = self.factory.EnumerateDevices()[self.camera_id]
        self.cap = pylon.InstantCamera(self.factory.CreateDevice(self.device))
        self.cap.Open()
        if not self.cap.IsGigE():
            self.cap.PixelFormat.Value = 'BGR8'
        else:
            self.cap.PixelFormat.Value = 'BayerRG12'
        self.converter = pylon.ImageFormatConverter()
        self.converter.ImagePixelFormat = pylon.PixelType_Mono16
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def isOpened(self):
        """
        DESCRIPTION:
            Checks of camera is open.
        ARGUMENTS:
        RETURNS:
            status: bool
        """
        if self.cap.IsOpen():
            return True
        return False

    def release(self):
        """
        DESCRIPTION:
            Closes a camera.
        ARGUMENTS:
        RETURNS:
        """
        self.cap.Close()

    def read(self):
        """
        DESCRIPTION:
            Tries to read a frame from camera.
        ARGUMENTS:
        RETURNS:
            ret: bool
            img: np.array/None
        """
        grabResult = self.cap.GrabOne(1000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            img = self.converter.Convert(grabResult)
            img = img.Array
            grabResult.Release()
            return True, img
        return False, None

    def get_exposure(self):
        """
        DESCRIPTION:
            Get current exposure value.
        ARGUMENTS:
        RETURNS:
            exposure: int
        """
        if not self.cap.IsGigE():
            return self.cap.ExposureTime.Value
        return self.cap.ExposureTimeAbs.Value

    def set_exposure(self, exposure):
        """
        DESCRIPTION:
            Sets current exposure value if it is a valid value else raises ValueError Exception.
        ARGUMENTS:
            exposure (int):
        RETURNS:
        """
        if not self.cap.IsGigE():
            if self.cap.ExposureTime.GetMin() < exposure < self.cap.ExposureTime.GetMax():
                self.cap.ExposureTime.Value = exposure
            else:
                raise ValueError(f"{self.cap.ExposureTime.GetMin()} > exposure > {self.cap.ExposureTime.GetMax()}")
        else:
            if self.cap.ExposureTimeAbs.GetMin() < exposure < self.cap.ExposureTimeAbs.GetMax():
                self.cap.ExposureTimeAbs.Value = exposure
            else:
                raise ValueError(f"{self.cap.ExposureTime.GetMin()} > exposure > {self.cap.ExposureTime.GetMax()}")

    def get_max_possible_exposure(self):
        """
        DESCRIPTION:
            Get the max possible exposure value.
        ARGUMENTS:
        RETURNS:
            max_exposure: int
        """
        if not self.cap.IsGigE():
            return self.cap.ExposureTime.GetMax()
        return self.cap.ExposureTimeAbs.GetMax()

    def get_min_possible_exposure(self):
        """
        DESCRIPTION:
            Get the min possible exposure value.
        ARGUMENTS:
        RETURNS:
            min_exposure: int
        """
        if not self.cap.IsGigE():
            return self.cap.ExposureTime.GetMin()
        return self.cap.ExposureTimeAbs.GetMin()

    def get_gain(self):
        """
        DESCRIPTION:
            Get current gain value.
        ARGUMENTS:
        RETURNS:
            gain: float
        """
        if not self.cap.IsGigE():
            return self.cap.Gain.Value
        return self.cap.GainRaw.Value

    def set_gain(self, gain):
        """
        DESCRIPTION:
            Sets current gain value if it is a valid value else raises ValueError Exception.
        ARGUMENTS:
        RETURNS:
        """
        if not self.cap.IsGigE():
            if self.cap.Gain.GetMin() < gain < self.cap.Gain.GetMax():
                self.cap.Gain.Value = gain
            else:
                raise ValueError(f"{self.cap.Gain.GetMin()} > gain > {self.cap.Gain.GetMax()}")
        else:
            if self.cap.GainRaw.GetMin() < gain < self.cap.GainRaw.GetMax():
                self.cap.GainRaw.Value = gain
            else:
                raise ValueError(f"{self.cap.GainRaw.GetMin()} > gain > {self.cap.GainRaw.GetMax()}")

    def get_max_possible_gain(self):
        """
        DESCRIPTION:
            Get the max possible gain value.
        ARGUMENTS:
        RETURNS:
            max_gain: int
        """
        if not self.cap.IsGigE():
            return self.cap.Gain.GetMax()
        return self.cap.GainRaw.GetMax()

    def get_min_possible_gain(self):
        """
        DESCRIPTION:
            Get the min possible gain value.
        ARGUMENTS:
        RETURNS:
            min_gain: int
        """
        if not self.cap.IsGigE():
            return self.cap.Gain.GetMin()
        return self.cap.GainRaw.GetMin()

    def get_packet_size(self):
        if self.cap.IsGigE():
            return self.cap.GevSCPSPacketSize
        return None

    def set_packet_size(self, value):
        if self.cap.IsGigE():
            self.cap.GevSCPSPacketSize.Value = value

if __name__ == '__main__':
    cap = VideoCapture(0)
    print(cap.cap.IsGigE())
    cap.set_exposure(20000)
    print(cap.get_exposure())
    cap.set_gain(10)
    print(cap.get_gain())
    while True:
        ret, frame = cap.read()
        if not ret:
            cap.release()
            cap = VideoCapture(1)
            continue
        frame = cv.resize(frame, (640, 640))
        cv.imwrite('gige.jpg', frame)
