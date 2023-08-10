import cv2
import numpy as np

class Evaluator:
    def __init__(self):
        
        pass

    def evaluate(self, originalImage):
        hsv = cv2.cvtColor(originalImage, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv, (36, 26, 25), (70, 255, 255))

        greenPixel = self.mask > 0
        greenExtract = np.zeros_like(originalImage, np.uint8)
        greenExtract[greenPixel] = originalImage[greenPixel]

        cv2.imshow("greenExtractor", greenExtract)