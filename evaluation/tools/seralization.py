import numpy as np
from xml.etree.ElementTree import Element


class Pose3:

    def __init__(self):
        self.translation = np.array([0, 0, 0])
        self.rot = np.array([0, 0, 0, 1])

    @classmethod
    def from_xml(cls, element: Element):
        if element is None:
            return None
        #if R_ and t_
        rotation_element = element.find("R_")
        translation_element = element.find("t_")
        if rotation_element and translation_element:
            cls.parse_rotation_xml(rotation_element)
        else:
            return None
        # if "R_" in tags and "t_" in tags:
        #     # print(element["R"], element["t_"])
        # else:
        #     return None

    @staticmethod
    def parse_rotation_xml(rotation_element: Element):
        #assume rot11->rot33 form
        rot11 = rotation_element.find("rot11")

    @staticmethod
    def parse_translation_xml(rotation_element: Element):
        #assume rot11->rot33 form
        rot11 = rotation_element.find("rot11")


class ErrorPair:
    def __init__(self) -> None:
        self.translation = 0.0
        self.rot = 0.0

    def __repr__(self) -> str:
        return f'T: {self.translation} R: {self.rot}'

    @classmethod
    def from_xml(cls, element: Element):
        error_pair = cls()
        error_pair.rot = float(element.find("rot").text)
        error_pair.translation = float(element.find("translation").text)
        return error_pair

    
        
