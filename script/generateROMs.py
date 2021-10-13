from solo_rbprm.solo import Robot
from hpp.gepetto import Viewer
from hpp.corbaserver.problem_solver import ProblemSolver
from numpy import array

from scipy.spatial import ConvexHull
from constants_and_tools import hull_to_obj
from plot_polytopes import plot_hull
import matplotlib.pyplot as plt

fullBody = Robot()

nbSamples = 100000

ps = ProblemSolver(fullBody)

r = Viewer(ps)

rootName = 'base_joint_xyz'

q_0 = fullBody.referenceConfig

r(q_0)
fullBody.setJointBounds("root_joint", [-20, 20, -20, 20, -20, 20])
fullBody.setConstrainedJointsBounds()
dict_heuristic = {
    fullBody.rLegId: "static",
    fullBody.lLegId: "static",
    fullBody.rArmId: "fixedStep04",
    fullBody.lArmId: "fixedStep04"
}
fullBody.loadAllLimbs(dict_heuristic, "ReferenceConfiguration", nbSamples=nbSamples)


def getEffPosition(limbId, nbSamples):
    positions = []
    limit = nbSamples - 1
    for i in range(0, limit):
        q = fullBody.getSamplePosition(limbId, i)
        positions += [q[:3]]
    return positions


for limbId in fullBody.limbs_names:
    positions = getEffPosition(limbId, nbSamples)
    rom_hull = ConvexHull(positions)
    hull_to_obj(rom_hull, positions, str(limbId) + "_rom.obj")
    fig = plt.figure()
    fig.suptitle(str(limbId) + "_rom.obj", fontsize=16)
    plot_hull(rom_hull, positions, array(positions), color="r", plot=False, fig=fig, ax=None)

    fig = plt.figure()
    fig.suptitle(str(limbId), fontsize=16)
    ax = None
    plt.show(block=False)
