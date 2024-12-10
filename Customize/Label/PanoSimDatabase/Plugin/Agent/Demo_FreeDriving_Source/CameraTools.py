import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2

class CameraTools:
    camRT = np.zeros((4,4))
    carRT = np.zeros((4,4))
    camPro = 0
    camera2img = 0
    imgWidth = 0
    imgHeight = 0

    def __init__(self):
        return
    def resetCameraPrameter(self,imgWidth = 1280,imdHeight = 720,HFov = 60.0):
        far = 1000
        near = 0.1
        VFov = math.degrees(math.atan(imdHeight * math.tan(math.radians(HFov) * 0.5) / imgWidth)) * 2
        a = 1 / math.tan(math.radians(HFov * 0.5))
        b = 1 / math.tan(math.radians(VFov * 0.5))
        c = (far + near) / (far - near)
        d = -2 * (near * far) / (far - near)

        projectionmatrix = np.array([a, 0, 0, 0,
                                     0, b, 0, 0,
                                     0, 0, c, d,
                                     0, 0, 1, 0]).reshape((4, 4))

        self.camPro = projectionmatrix
        self.camera2img = np.array([-imgWidth / 2, 0, imgWidth / 2,
                                    0, -imdHeight / 2, imdHeight / 2,
                                    0, 0, 1]).reshape(3, 3)
        self.imdHeight = imdHeight
        self.imgWidth = imgWidth
        return

    def resetCamRT(self,cameraPosition,cameraRotation):
        yaw,pitch,roll= cameraRotation
        x,y,z = cameraPosition

        ego_rotation = R.from_euler("xyz", (roll, pitch, yaw)).as_matrix()
        self.camRT[:3,:3] = ego_rotation
        self.camRT[:,3] = np.array([x,y,z,1])
        self.camRT = np.linalg.inv(self.camRT)
        return

    def resetCarRT(self,carPosition,carRotation):
        yaw,pitch,roll = carRotation
        x,y,z = carPosition
        ego_rotation = R.from_euler("xyz", (roll, pitch, yaw)).as_matrix()
        self.carRT[:3,:3] = ego_rotation
        self.carRT[:,3] = np.array([x,y,z,1])
        self.carRT = np.linalg.inv(self.carRT)
        return

    def getCamRT(self):
        return self.camRT

    def getCarRT(self):
        return self.carRT

    def getCamPro(self):
        return self.camPro

    def getWorld2CameraMat(self):
        return self.camRT @ self.carRT

    def getCoordinate(self,worldposition,flag = False):
        posShape = worldposition.shape
        if posShape[0] != 3 and posShape[0] != 4:
            m_worldposition = worldposition.T
        else:
            m_worldposition = worldposition
        if posShape == 3:
            np.row_stack((m_worldposition,np.ones((1,posShape[1]))))

        cameraCoordinate = self.camRT @ self.carRT @ m_worldposition
        return cameraCoordinate

    def getPixelSpacePoints(self,worldposition,flag = False):
        posShape = worldposition.shape
        if posShape[0] != 3 and posShape[0] != 4:
            m_worldposition = worldposition.T
        else:
            m_worldposition = worldposition
        if posShape == 3:
            np.row_stack((m_worldposition,np.ones((1,posShape[1]))))

        cameraCoordinate = self.camRT @ self.carRT @ m_worldposition
        cameraCoordinate[[0,1,2] ,:] = cameraCoordinate[[1,2,0] ,:]
        cameraCoordinate[3,:] = 0
        clipSpace = self.camPro @ cameraCoordinate
        screenSpace = clipSpace[:3, :] / np.abs(clipSpace[3, :])

        if(flag):
            idx = np.argwhere(np.any([screenSpace < -1, screenSpace > 1], axis=0))
            screenSpace = np.delete(screenSpace[:3, :], idx, axis=1)
        pixelSpace = self.camera2img @ screenSpace
        return pixelSpace

    def getCarPixelSpacePoints(self,worldposition,yaw,sizex,flag = False):
        posShape = worldposition.shape
        if posShape[0] != 3 and posShape[0] != 4:
            m_worldposition = worldposition.T
        else:
            m_worldposition = worldposition
        if posShape == 3:
            np.row_stack((m_worldposition,np.ones((1,posShape[1]))))

        cameraCoordinate = self.camRT @ self.carRT @ m_worldposition
        cameraCoordinate[[0,1,2] ,:] = cameraCoordinate[[1,2,0] ,:]
        cameraCoordinate[3,:] = 0

        cameraCoordinate[2,:] = cameraCoordinate[2,:] - sizex / 6.5 * math.cos(yaw)
        cameraCoordinate[0, :] = cameraCoordinate[0, :] + sizex / 6.5 * math.sin(yaw)

        clipSpace = self.camPro @ cameraCoordinate
        screenSpace = clipSpace[:3, :] / np.abs(clipSpace[3, :])
        if(flag):
            idx = np.argwhere(np.any([screenSpace < -1, screenSpace > 1], axis=0))
            screenSpace = np.delete(screenSpace[:3, :], idx, axis=1)
        pixelSpace = self.camera2img @ screenSpace
        return pixelSpace

    def world2img(self,img,worldposition):

        imgShape = img.shape

        if imgShape[0] != self.imgHeight or imgShape[1] != self.imgWidth:
            self.resetCameraPrameter(imgShape[1],imgShape[0])

        pixelSpace = self.getPixelSpacePoints(worldposition,True)

        for i in range(pixelSpace.shape[1]):
            cv2.circle(img,(int(pixelSpace[0,i]),int(pixelSpace[1,i])),10,(0,0,255),-1)

        return