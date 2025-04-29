import Sofa

class MyController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.fem = kwargs.get("fem")

    def onAnimateEndEvent(self, param):
        print(f"elasticStrain at {param['dt']} is {self.fem.elasticStrain.value}")
        print(f"elasticStrain at {param['dt']} is of size {len(self.fem.elasticStrain.value)}")

def createScene(root):
    root.addObject("EulerImplicitSolver")
    root.addObject("SparseLDLSolver")

    root.addObject("MeshGmshLoader", filename="mesh/liver.msh", name="loader")
    root.addObject("MechanicalObject", src=root.loader.linkpath)
    root.addObject("MeshTopology", src=root.loader.linkpath)
    root.addObject("TetrahedronFEMForceField", name="fem", method="small", computeGlobalMatrix=False)
    root.addObject(MyController(fem=root.fem))
    

