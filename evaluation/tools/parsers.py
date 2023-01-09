import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element
from tools.seralization import Pose3, ErrorPair
import matplotlib.pyplot as plt



class ParseFrontendMetrics(object):

    def __init__(self, folder = "../output_logs", file = 'frontend_metrics.xml') -> None:
        path = folder + "/" + file
        self.tree = ET.parse(path)
        #list of dictioanries, with elements for each tag in the frontend_metrics file
        self.data_frames = self._parse(self.tree.getroot())

    def plot(self, **kwargs):
        # to save etc


        ate_r_before = list(map(lambda x: x["ate_before_flow"].rot, self.data_frames))
        ate_t_before = list(map(lambda x: x["ate_before_flow"].translation, self.data_frames))
        ate_r_after = map(lambda x: x["ate_after_flow"].rot, self.data_frames)
        ate_t_after = list(map(lambda x: x["ate_after_flow"].translation, self.data_frames))

        rte_r_before = list(map(lambda x: x["rte_before_flow"].rot, self.data_frames))
        rte_t_before = map(lambda x: x["rte_before_flow"].translation, self.data_frames)
        rte_r_after = list(map(lambda x: x["rte_after_flow"].rot, self.data_frames))
        rte_t_after = map(lambda x: x["rte_after_flow"].translation, self.data_frames)

        frame_id = list(map(lambda x: x["frame_id"], self.data_frames))
        

        #plt.plot(frames, camera_t_errors, "-b", label="PnP Camera")
        plt.plot(frame_id, ate_t_before, "-r", label="ATE t before flow")
        plt.plot(frame_id, ate_t_after, "-b", label="ATE t after flow")
        plt.title("Pose estimation using PnP in world vs camera frame")
        plt.xlabel("Frames")
        plt.ylabel("Absolute Rotation Error (r)")

        plt.legend(loc="upper left")
        plt.savefig("test.png")


    def _parse(self, root: Element):
        frames = []
        for x in root[0]:
            # print(x.tag)
            if x.tag == "item":
                frame_dict = {}
                for element in x:
                    data = self.constructDataFromTag(element)
                    if data:
                        frame_dict[element.tag] = data

                frames.append(frame_dict)
        return frames


    def constructDataFromTag(self, element: Element):
        tag = element.tag
        if tag == 'ate_before_flow' or \
           tag == 'rte_before_flow' or \
           tag == 'ate_after_flow' or \
           tag == 'rte_after_flow':
            return ErrorPair.from_xml(element)
        elif tag == 'frame_id':
            return int(element.text)
        elif tag == "timestamp":
            return float(element.text)
        else:
            return None


# metrics = ParseFrontendMetrics()
# metrics.plot()