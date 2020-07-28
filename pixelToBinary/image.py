import numpy as np
from PIL import Image, UnidentifiedImageError
from cv2 import cvtColor, COLOR_BGR2RGB


class image:
    def __init__(self, image):
        try:
            if(type(image) == str):
                try:
                    self.image = Image.open(image)
                except:
                    raise FileNotFoundError(image)

            elif(type(image) == np.ndarray):
                img = cvtColor(image, COLOR_BGR2RGB)
                self.image = Image.fromarray(img)

            elif(type(image) == Image.Image):
                self.image = image

            else:
                raise Exception

        except FileNotFoundError as err:
            raise Exception(f"file not found: {image}")

        except:
            raise Exception("use a file, pillow or cv2 images")

    def open(self, name=None, width=None, height=None):
        try:
            if(name == None):
                raise ValueError("Image has missing a property: name.")
            else:
                self.name = name

            self.image = Image.open(name)

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

            self.image.resize(width, height)

        except FileNotFoundError as err:
            raise Exception(f"FileNotFoundError:{err}")

        except UnidentifiedImageError as err:
            raise Exception(
                f"UnidentifiedImageError: file type is not supported. see: https://pillow.readthedocs.io/en/5.1.x/handbook/image-file-formats.html")
