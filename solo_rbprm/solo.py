#!/usr/bin/env python
# Copyright (c) 2020 CNRS
# Author: Pierre Fernbach
#
# This file is part of solo-rbprm.
# solo-rbprm is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_tutorial is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_tutorial.  If not, see
# <http://www.gnu.org/licenses/>.



import os
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody as Parent
from pinocchio import SE3, Quaternion
import numpy as np


class Robot (Parent):
    ##
    #  Information to retrieve urdf and srdf files.
    name = "solo"
    packageName = "solo_description"
    meshPackageName = "solo_description"
    rootJointType = "freeflyer"
    urdfName = "solo"
    urdfSuffix = "12"
    srdfSuffix = ""

    ## Information about the names of thes joints defining the limbs of the robot
    rLegId = 'FRleg'
    rleg = 'FR_HAA'
    rfoot = 'FR_FOOT'
    lLegId = 'FLleg'
    lleg = 'FL_HAA'
    lfoot = 'FL_FOOT'
    lArmId = 'HLleg'
    larm = 'HL_HAA'
    lhand = 'HL_FOOT'
    rArmId = 'HRleg'
    rarm = 'HR_HAA'
    rhand = 'HR_FOOT'

    referenceConfig = [0., 0., 0.241, 0., 0., 0., 1., # freeflyer
                        0., 0.8, -1.6,  #  FL
                        0., 0.8, -1.6,  #  FR
                        0., -0.8, 1.6,  #  HL
                        0., -0.8, 1.6]  #  HR

    postureWeights=[0,0,0,0,0,0, #FF
    100.,1.,20.,
    100.,1.,20.,
    100.,1.,20.,
    100.,1.,20.,]

    DEFAULT_COM_HEIGHT = 0.2

    # informations required to generate the limbs databases the limbs : 
    nbSamples = 50000 #  Number of sampled configuration for each limb in the database
    octreeSize = 0.002 #  Resolution of the octree leaf (as a cube, in meter)
    cType = "_3_DOF" #  6_dof constraints the orientation of the contacts, 3_dof only constraint the position
    offset = [0.,0.,-0.018] #  Contact position in the effector frame

    #  Position on the first joint of each limb (in the root frame) in the 'reference' configration, 
    #  Used by some heuristic when sorting candidates contacts positions
    rLegLimbOffset = [0.1946, 0.0875, 0.]
    lLegLimbOffset = [0.1946, -0.0875,0.]
    rArmLimbOffset = [-0.1946, 0.0875, 0.]
    lArmLimbOffset = [-0.1946, -0.0875, 0.]

    normal = [0,0,1] #  Contact normal, in the effector frame
    legx = 0.01; legy = 0.01  #  Contact patch size, cannot be 0 even for contact points

    kinematicConstraintsPath="package://solo-rbprm/com_inequalities/"
    # Path to constraints files used by SL1M:
    kinematic_constraints_path = os.environ["INSTALL_HPP_DIR"] + "/share/solo-rbprm/com_inequalities/feet_quasi_flat/"
    relative_feet_constraints_path = os.environ["INSTALL_HPP_DIR"] + "/share/solo-rbprm/relative_effector_positions/"
    minDist = 0.15 #  Minimal height of the CoM wrt to the contact height

    #  Data used by mlp
    limbs_names = [rArmId, lArmId, lLegId, rLegId] #  List of effector used to create contact
    dict_limb_rootJoint = {rLegId:rleg, lLegId:lleg, rArmId:rarm, lArmId:larm}
    dict_limb_joint = {rLegId:rfoot, lLegId:lfoot, rArmId:rhand, lArmId:lhand}
    dict_limb_color_traj = {rfoot:[0,1,0,1], lfoot:[1,0,0,1],rhand:[0,0,1,1],lhand:[0.9,0.5,0,1]}
    FOOT_SAFETY_SIZE = 0.01
    # size of the contact surface (x,y)
    dict_size={rfoot:[legx , legy], lfoot:[legx , legy],rhand:[legx , legy],lhand:[legx , legy]}
    #various offset used by scripts
    MRsole_offset = SE3.Identity()
    MRsole_offset.translation = np.matrix(offset).T
    MLsole_offset = MRsole_offset.copy()
    MRhand_offset = MRsole_offset.copy()
    MLhand_offset = MRsole_offset.copy()
    dict_offset = {rfoot:MRsole_offset, lfoot:MLsole_offset, rhand:MRhand_offset, lhand:MLhand_offset}
    dict_limb_offset= {rLegId:rLegLimbOffset, lLegId:lLegLimbOffset, rArmId:rArmLimbOffset, lArmId:lArmLimbOffset}
    dict_normal = {rfoot:normal, lfoot:normal, rhand:normal, lhand:normal}
    # Effector position in the reference configuration, in the root frame
    ref_EE_lLeg = np.array([0.1946, 0.14695, -0.223])
    ref_EE_rLeg = np.array([0.1946, -0.14695, -0.223])
    ref_EE_lArm = np.array([-0.1946, 0.14695, -0.223])
    ref_EE_rArm = np.array([-0.1946, -0.14695, -0.223])
    dict_ref_effector_from_root = {rLegId:ref_EE_rLeg,  
                                   lLegId:ref_EE_lLeg,
                                   rArmId:ref_EE_rArm,
                                   lArmId:ref_EE_lArm}
    # display transform :
    MRsole_display = MRsole_offset.copy()
    MLsole_display = MLsole_offset.copy()
    MRhand_display = MRhand_offset.copy()
    MLhand_display = MLhand_offset.copy()
    dict_display_offset = {rfoot:MRsole_display, lfoot:MLsole_display, rhand:MRhand_display, lhand:MLhand_display}

    kneeIds = {"FL":9,"HL":12,"FR":15,"HR":18}


    def __init__(self, name=None, load=True, client=None, clientRbprm=None):
        if name is not None:
            self.name = name
        Parent.__init__(self, self.name, self.rootJointType, load, client, None, clientRbprm)
        # save original bounds of the urdf for futur reset
        self.FL_HAA_bounds = self.getJointBounds('FL_HAA')
        self.FL_HFE_bounds = self.getJointBounds('FL_HFE')
        self.FL_KFE_bounds = self.getJointBounds('FL_KFE')

        self.FR_HAA_bounds = self.getJointBounds('FR_HAA')
        self.FR_HFE_bounds = self.getJointBounds('FR_HFE')
        self.FR_KFE_bounds = self.getJointBounds('FR_KFE')

        self.HL_HAA_bounds = self.getJointBounds('HL_HAA')
        self.HL_HFE_bounds = self.getJointBounds('HL_HFE')
        self.HL_KFE_bounds = self.getJointBounds('HL_KFE')

        self.HR_HAA_bounds = self.getJointBounds('HR_HAA')
        self.HR_HFE_bounds = self.getJointBounds('HR_HFE')
        self.HR_KFE_bounds = self.getJointBounds('HR_KFE')


    def loadAllLimbs(self,heuristic, analysis = None, nbSamples = nbSamples, octreeSize = octreeSize,disableEffectorCollision = False):
        if isinstance(heuristic,str):#only one heuristic name given assign it to all the limbs
            dict_heuristic = {}
            for id in self.limbs_names:
                dict_heuristic.update({id:heuristic})
        elif isinstance(heuristic,dict):
            dict_heuristic=heuristic
        else : 
            raise Exception("heuristic should be either a string or a map limbId:string")
        for id in self.limbs_names:
            print("add limb : ",id)
            eff = self.dict_limb_joint[id]
            print("effector name = ",eff)
            self.addLimb(id,
                         self.dict_limb_rootJoint[id],
                         eff,
                         self.dict_offset[eff].translation.tolist(),
                         self.dict_normal[eff],
                         self.dict_size[eff][0]/2.,
                         self.dict_size[eff][1]/2.,
                         nbSamples,
                         dict_heuristic[id],
                         octreeSize,
                         self.cType,
                         disableEffectorCollision = disableEffectorCollision,
                         #kinematicConstraintsPath=self.kinematicConstraintsPath+self.dict_limb_rootJoint[id]+"_06_com_constraints.obj",
                         limbOffset=self.dict_limb_offset[id],
                         kinematicConstraintsMin=self.minDist)
            if analysis :
                self.runLimbSampleAnalysis(id, analysis, True)


    def setConstrainedJointsBounds(self):
        self.setJointBounds('FL_HAA',[-0.5,0.5])
        self.setJointBounds('FL_HFE',[0.2,1.4])
        self.setJointBounds('FL_KFE',[-2.3,-0.4])

        self.setJointBounds('FR_HAA',[-0.5,0.5])
        self.setJointBounds('FR_HFE',[0.2,1.4])
        self.setJointBounds('FR_KFE',[-2.3,-0.4])

        self.setJointBounds('HL_HAA',[-0.5,0.5])
        self.setJointBounds('HL_HFE',[-1.4,-0.2])
        self.setJointBounds('HL_KFE',[0.4,2.3])

        self.setJointBounds('HR_HAA',[-0.5,0.5])
        self.setJointBounds('HR_HFE',[-1.4,-0.2])
        self.setJointBounds('HR_KFE',[0.4,2.3])

    def setConstrainedShoulder(self, max_amplitude = 0.05):
        self.setJointBounds('FL_HAA',[-max_amplitude,max_amplitude])
        self.setJointBounds('FR_HAA',[-max_amplitude,max_amplitude])
        self.setJointBounds('HL_HAA',[-max_amplitude,max_amplitude])
        self.setJointBounds('HR_HAA',[-max_amplitude,max_amplitude])


    def resetJointsBounds(self):
        self.setJointBounds('FL_HAA',self.FL_HAA_bounds)
        self.setJointBounds('FL_HFE',self.FL_HFE_bounds)
        self.setJointBounds('FL_KFE',self.FL_KFE_bounds)

        self.setJointBounds('FR_HAA',self.FR_HAA_bounds)
        self.setJointBounds('FR_HFE',self.FR_HFE_bounds)
        self.setJointBounds('FR_KFE',self.FR_KFE_bounds)

        self.setJointBounds('HL_HAA',self.HL_HAA_bounds)
        self.setJointBounds('HL_HFE',self.HL_HFE_bounds)
        self.setJointBounds('HL_KFE',self.HL_KFE_bounds)

        self.setJointBounds('HR_HAA',self.HR_HAA_bounds)
        self.setJointBounds('HR_HFE',self.HR_HFE_bounds)
        self.setJointBounds('HR_KFE',self.HR_KFE_bounds)

