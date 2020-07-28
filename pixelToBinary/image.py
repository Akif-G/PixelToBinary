import numpy as np
from PIL import Image, UnidentifiedImageError
from cv2 import cvtColor, COLOR_BGR2RGB


class Bimage:
    def __init__(self, image=None):
        try:
            if(type(image) == str):
                try:
                    self.image = Image.open(image).convert('RGB')
                except:
                    raise FileNotFoundError(image)

            elif(type(image) == np.ndarray):
                img = cvtColor(image, COLOR_BGR2RGB)
                self.image = Image.fromarray(img).convert('RGB')

            elif(type(image) == Image.Image):
                self.image = image

            else:
                if(image == None):
                    return None
                else:
                    raise Exception

        except FileNotFoundError as err:
            raise Exception(f"file not found: {image}")

        except:
            raise Exception("use a file, pillow or cv2 images")

    def open(self, name=None, width=None, height=None, resample=Image.BILINEAR):
        try:
            if(name == None):
                raise ValueError("Image has missing a property: name.")
            else:
                self.name = name

            self.image = Image.open(name).convert('RGB')

            try:
                if(width == None):
                    raise ValueError("Image has missing a property: width")
                else:
                    self.width = width

                if(height == None):
                    raise ValueError("Image has missing a property: height.")
                else:
                    self.height = height

            except ValueError as err:
                print(f"for {name}: {err} Will use actual resolution instead.")
                self.width = self.image.size[0]
                self.height = self.image.size[1]

            self.image = self.image.resize((self.width, self.height), resample)

        except FileNotFoundError as err:
            raise Exception(f"FileNotFoundError:{err}")

        except UnidentifiedImageError as err:
            raise Exception(
                f"UnidentifiedImageError: file type is not supported. see: https://pillow.readthedocs.io/en/5.1.x/handbook/image-file-formats.html")

    def __convertCV2Gray(self):
        open_cv_image = np.array(self.image)
        originalImage = open_cv_image[:, :, ::-1].copy()

        # Convert RGB to BGR
        grayImage = cvtColor(originalImage, cv2.COLOR_BGR2GRAY)
        return grayImage

    def __blackAndWhite(self, tolarance=127, threshold=cv2.THRESH_BINARY):
        grayImage = self.__convertCV2Gray()
        (thresh, blackAndWhiteImage) = cv2.threshold(
            grayImage, tolarance, 255, threshold)

        return blackAndWhiteImage

    def convertBinary(self, tolarance=127):
        blackAndWhiteImage = self.__blackAndWhite()

        vfunc = np.vectorize(lambda t: 0 if t < 127 else 1)
        binary = []
        for i in blackAndWhiteImage:
            binary = np.concatenate((vfunc(i), binary))
        return binary

    def convertBinaryWithTolarance(self, tolarance=127):
        return self.convertBinary(tolarance)

    def convertBinaryWithRatio(self, ratio=0.5):
        tolarance = 127
        binary = self.convertBinary(tolarance)
        bwRatio = binary.sum()/((self.height*self.width))
        count = 126
        while count > 1:
            binary = self.convertBinary(tolarance)
            bwRatio = binary.sum()/((self.height*self.width)/2)
            if(bwRatio < ratio):
                tolarance -= 1
            else:
                tolarance -= 1
            count -= 1
        return binary

    def convertBinaryAdaptive(self, blockSize=101, C=2):
        grayImage = self.__convertCV2Gray()
        th = cv2.adaptiveThreshold(grayImage, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                   cv2.THRESH_BINARY, blockSize, C)
        vfunc = np.vectorize(lambda t: 0 if t < 127 else 1)
        binary = []
        for i in th:
            binary = np.concatenate((vfunc(i), binary))
        return binary, th

    def convertBinaryGaussian(self, blockSize=101, C=2):
        grayImage = self.__convertCV2Gray()
        th = cv2.adaptiveThreshold(grayImage, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, blockSize, C)
        vfunc = np.vectorize(lambda t: 0 if t < 127 else 1)
        binary = []
        for i in th:
            binary = np.concatenate((vfunc(i), binary))
        return binary, th
