import Sofa
import SofaTypes
import matplotlib.pyplot as plt
import numpy as np
import drjit as dr
from drjit.auto.ad import Float, Array3f, TensorXf

dr.set_flag(dr.JitFlag.Debug, True)

# SDF de la sphère centrée à l’origine, rayon 1
def sdf_sphere(p):
    return dr.norm(p-[0.0,0.0,0.0]) - 1.0

# SDF du cube centré à l’origine, taille 1 (donc demi-taille 0.5)
def sdf_cube(p, size=0.5):
    q = dr.abs(p) - size
    outside_dist = dr.norm(dr.maximum(q, 0.0))
    inside_dist = dr.minimum(dr.maximum(q.x, dr.maximum(q.y, q.z)), 0.0)
    return outside_dist + inside_dist

def sdf(p: Array3f) -> Float:
    d_sphere = sdf_sphere(p)
    d_cube = sdf_cube(p, size=0.7)
    return dr.maximum(d_cube, -d_sphere)

@dr.syntax
def trace(o: Array3f, d: Array3f) -> Array3f:
    for i in range(100):
        o = dr.fma(d, sdf(o), o)
    return o

def shade(p: Array3f, l: Array3f) -> Float:
    dr.enable_grad(p)
    value = sdf(p)
    dr.set_grad(p, l)
    dr.forward_to(value)
    return dr.maximum(0, dr.grad(value))
    
class RayMarching(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args, **kwargs)
        x = dr.linspace(Float, -1, 1, 1000)
        x, y = dr.meshgrid(x, x)
        self.d = dr.normalize(Array3f(x, y, 1))
        self.onAnimateEndEvent(None)
        self.img = 0
        self.revision = 0

    def onAnimateEndEvent(self, params):
        p = trace(o=Array3f(0, 0, -2), d=self.d)
        sh = shade(p, l=Array3f(0,0,-1))
        
        sh[sdf(p) > 0] = 0
        img = Array3f(.1, .1, .2) + Array3f(.4, .4, .2) * sh

        self.img_flat = dr.ravel(img)
        
    def draw(self, visual_context):
        dt = visual_context.getDrawTool()

        dt.drawText(10,10, 16, "RayMarching demo")
        #self.revision+=1
        image = (self.img_flat.to_numpy() * 255).clip(0,255).astype(np.uint8)
        dt.drawRGBAImage("memory", 
                         self.revision, 
                         SofaTypes.Vec3d(-1.0,-1.0,0.0), 
                         10.0,
                         1000, 1000, 24, image)


def createScene(root):
    root.addObject("MechanicalObject", name="mo", position=[[2.0,0,0,0],
    [2.0,2,0,0],
    [2.0,0,4,0],
    [2.0,-1,4,0],
    [-2.0,-1,4,0],
    [-2.0,-1,4,-3]
    ])
    root.mo.showObject=True
    root.mo.showObjectScale=10.0
    
    root.addObject(RayMarching(name="raymarching"))
