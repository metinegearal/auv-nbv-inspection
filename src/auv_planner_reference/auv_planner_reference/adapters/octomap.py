import pyoctomap

class OctomapAdapter:
    def __init__(self, resolution):
        self._tree = pyoctomap.ColorOcTree(resolution)
    def extractPointCloud(self):
        return self._tree.extractPointCloud()
    def getNumLeafNodes(self):
        return self._tree.getNumLeafNodes()
    def insertPointCloud(self, *args, **kwargs):
        return self._tree.insertPointCloud(*args, **kwargs)
    def write(self, path):
        return self._tree.write(path)
