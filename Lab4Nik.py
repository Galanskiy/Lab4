import NemAll_Python_Geometry as g
import NemAll_Python_BaseElements as base_tools
import NemAll_Python_BasisElements as basis_tools
import NemAll_Python_Utility as u
import GeometryValidate as val
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties

import StdReinfShapeBuilder.GeneralReinfShapeBuilder as gen
import StdReinfShapeBuilder.LinearBarPlacementBuilder as lin

from StdReinfShapeBuilder.ConcreteCoverProperties import ConcreteCoverProperties
from StdReinfShapeBuilder.ReinforcementShapeProperties import ReinforcementShapeProperties
from StdReinfShapeBuilder.RotationAngles import RotationAngles
import NemAll_Python_Reinforcement as rei


# Перевірка версії
def check_allplan_version(build_ele, version):
    del build_ele
    del version
    return True

# Створює об'єкт
def create_element(build_ele, doc):
    element = Armature(doc)
    return element.parts_union(build_ele)

# Створює ручку
def move_handle(build_ele, handle_prop, input_pnt, doc):
    build_ele.change_property(handle_prop, input_pnt)
    return create_element(build_ele, doc)

# Балка арматурою
class Armature:

    def __init__(self, doc):
        self.model_ele_list = []
        self.handle_list = []
        self.document = doc

    # створення властивостей
    def parts_union(self, build_ele):
        self.wd = build_ele.wd.value
        self.l = build_ele.l.value
        self.hb = build_ele.hb.value
        self.cbt = build_ele.cbt.value
        self.cbb = build_ele.cbb.value
        self.center_w = build_ele.center_w.value
        self.mh = build_ele.mh.value
        self.r = build_ele.r.value
        self.rr = build_ele.rr.value
        self.wt = build_ele.wt.value
        self.ht = build_ele.ht.value
        self.ps = build_ele.ps.value
        self.ph = build_ele.ph.value
        self.c = build_ele.c.value
        self.ctt = build_ele.ctt.value
        self.d = build_ele.d.value
        self.bs = build_ele.bs.value
        self.height = self.hb + self.mh + self.ht + self.ph + 300
        self.options()
        self.up()
        self.armature_1()
        self.armature_2()
        self.create_handles()
        return (self.model_ele_list, self.handle_list)

    def options(self):
        self.cp = base_tools.CommonProperties()
        self.cp.GetGlobalProperties()
        self.cp.Pen = 1
        self.cp.Color = self.c

    # Створення самої арматури з вигином, радіусом і заглибленням
    def armature_1(self):
        ang = RotationAngles(0, 90, 90)
        gr = rei.ReinforcementSettings.GetSteelGrade()
        pr = ReinforcementShapeProperties.rebar(self.rr, 4, gr, -1, rei.BendingShapeType.LongitudinalBar)
        ccp = ConcreteCoverProperties.left_right_bottom(self.rr * 2, self.rr * 2, self.rr)
        figure = gen.create_longitudinal_shape_with_hooks(self.d, ang, pr, ccp, 0, -1)
        self.model_ele_list.append(lin.create_linear_bar_placement_from_to_by_dist(1, figure, g.Point3D(self.wt / 6, 0, self.height), g.Point3D(self.wt / 6, self.l, self.height), 0, 0, self.bs))

    def armature_2(self):
        ang = RotationAngles(0, 90, 270)
        gr = rei.ReinforcementSettings.GetSteelGrade()
        pr = ReinforcementShapeProperties.rebar(self.rr, 4, gr, -1, rei.BendingShapeType.LongitudinalBar)
        ccp = ConcreteCoverProperties.left_right_bottom(self.rr * 2, self.rr * 2, self.rr)
        figure = gen.create_longitudinal_shape_with_hooks(self.d, ang, pr, ccp, 0, -1)
        self.model_ele_list.append(lin.create_linear_bar_placement_from_to_by_dist(1, figure, g.Point3D(self.wt - self.wt / 3, 0, self.height), g.Point3D(self.wt - self.wt / 3, self.l, self.height), 0, 0, self.bs))

    # Нижня частина
    def down(self):
        figure = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(0, 0, 0), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), self.wd, self.l, self.hb)
        figure_i = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(0, 0, 0), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), self.wd, self.l, self.hb)
        edges = u.VecSizeTList()
        edges.append(1)
        edges.append(3)
        _, figure = g.ChamferCalculus.Calculate(figure, edges, self.cbb, False)
        edges2 = u.VecSizeTList()
        edges2.append(8)
        edges2.append(10)
        _, figure_i = g.ChamferCalculus.Calculate(figure_i, edges2, self.cbt, False)
        _, part = g.MakeIntersection(figure, figure_i)
        return part

    # середня частина
    def center(self):
        figure = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(self.wd / 2 - self.center_w / 2, 0, self.hb), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), self.center_w, self.l, self.mh)
        cyc = g.BRep3D.CreateCylinder(g.AxisPlacement3D(g.Point3D(self.cbt, self.l / 8, self.hb + self.mh / 2), g.Vector3D(0, 0, 1), g.Vector3D(1, 0, 0)), self.r, self.center_w)
        cyc1 = g.BRep3D.CreateCylinder(g.AxisPlacement3D(g.Point3D(self.cbt, self.l - self.l / 8, self.hb + self.mh / 2), g.Vector3D(0, 0, 1), g.Vector3D(1, 0, 0)), self.r, self.center_w)
        _, figure = g.MakeSubtraction(figure, cyc)
        _, figure = g.MakeSubtraction(figure, cyc1)
        _, part = g.MakeUnion(figure, self.down())
        return part

    # Верхня частина
    def up(self):
        figure = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(0 - (self.wt - self.wd) / 2, 0, self.hb + self.mh), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), self.wt, self.l, self.ht)
        figure_p = g.BRep3D.CreateCuboid(g.AxisPlacement3D(g.Point3D(self.ps - (self.wt - self.wd) / 2, 0, self.hb + self.mh + self.ht), g.Vector3D(1, 0, 0), g.Vector3D(0, 0, 1)), self.wt - self.ps*2, self.l, self.ph)
        edges2 = u.VecSizeTList()
        edges2.append(8)
        edges2.append(10)
        _, figure = g.ChamferCalculus.Calculate(figure, edges2, self.ctt, False)
        _, part = g.MakeUnion(figure, self.center())
        _, part = g.MakeUnion(part, figure_p)
        self.model_ele_list.append(basis_tools.ModelElement3D(self.ctt, part))

    # Ручка тільки для довжини(щоб показати роботу створення арматури динамічно при зміні довжини)
    def create_handles(self):
        origin2 = g.Point3D(self.wd / 2, 0, self.hb / 2)
        self.handle_list.append(HandleProperties("l", g.Point3D(origin2.X, origin2.Y + self.l, origin2.Z), g.Point3D(origin2.X, origin2.Y, origin2.Z), [("l", HandleDirection.y_dir)], HandleDirection.y_dir, False))
