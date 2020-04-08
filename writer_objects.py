class SecondOrderParticle:
    def __init__(self, radius = 0.2):
        self.radius = radius
        self.rpatch = plt.Circle((0., 0.), radius = radius, fc='g')
        self.workspace = None
        self.children = []
    def draw_at(self, rx):
        self.rpatch.center = (rx[0], rx[1])
        if self.children:
            for child in children:
                
    def draw_init(self,ax):
        ax.add_patch(self.rpatch)
        ax.add_patch(self.gpatch)
    def draw_range(self, gx, world):
        self.rapatch.center = (gx[0], gx[1])
        self.rapatch.radius = closestCircPoint(world.center,world.radius,gx)
    def draw_init_range(self, ax):
        ax.add_patch(self.rapatch)