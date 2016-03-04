import gladys

class gladys2:
    def __init__(self, region, robot):
        self.ctmap = gladys.costmap(region, robot)
        self.graph = gladys.nav_graph(self.ctmap)
        self.gdmap = self.ctmap.get_map()
    def u2p(self, x, y):
        return gladys.point_utm2pix(self.gdmap, x, y)
    def p2u(self, x, y):
        return gladys.point_pix2utm(self.gdmap, x, y)
    def c2p(self, x, y):
        return gladys.point_custom2pix(self.gdmap, x, y)
    def p2c(self, x, y):
        return gladys.point_pix2custom(self.gdmap, x, y)
    def c2u(self, x, y):
        return gladys.point_custom2utm(self.gdmap, x, y)
    def u2c(self, x, y):
        return gladys.point_utm2custom(self.gdmap, x, y)
    def path(self, a, b):
        # A* a, b
        res_utm, cost = self.graph.search_with_cost(self.c2u(*a), self.c2u(*b))
        res = [self.u2c(x, y) for x, y in res_utm]
        return cost, res
