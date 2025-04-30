import Sofa

class MyController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.fem = kwargs.get("fem")

    def onAnimateEndEvent(self, param):
        print(f"elasticStrains at {param['dt']} is {self.fem.elasticStrains.value}")
        print(f"elasticStrains at {param['dt']} is of size {len(self.fem.elasticStrains.value)}")

        print(f"plasticStrains at {param['dt']} is {self.fem.plasticStrains.value}")
        print(f"plasticStrains at {param['dt']} is of size {len(self.fem.plasticStrains.value)}")


def createScene(root):
    root.addObject("EulerImplicitSolver")
    root.addObject("SparseLDLSolver")

    root.addObject("MeshGmshLoader", filename="mesh/liver.msh", name="loader")
    root.addObject("MechanicalObject", src=root.loader.linkpath)
    root.addObject("MeshTopology", src=root.loader.linkpath)
    root.addObject("TetrahedronFEMForceField", name="fem", method="small", computeGlobalMatrix=False, plasticMaxThreshold=1.000000000, plasticYieldThreshold=0.5000)
    root.addObject(MyController(fem=root.fem))
    

