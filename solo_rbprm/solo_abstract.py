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

from hpp.corbaserver.rbprm.rbprmbuilder import Builder as Parent


class Robot(Parent):

    ##
    #  Information to retrieve urdf and srdf files.
    rootJointType = "freeflyer"
    packageName = "solo-rbprm"
    meshPackageName = "solo-rbprm"
    # URDF file describing the trunk of the robot solo
    urdfName = "solo_trunk"
    # URDF files describing the reachable workspace of each limb of solo
    urdfNameRom = [
        "solo_RFleg_rom",
        "solo_LHleg_rom",
        "solo_LFleg_rom",
        "solo_RHleg_rom",
    ]
    urdfSuffix = ""
    srdfSuffix = ""
    name = urdfName

    ref_height = 0.241

    rLegId = "solo_RFleg_rom"
    lLegId = "solo_LFleg_rom"
    rArmId = "solo_RHleg_rom"
    lArmId = "solo_LHleg_rom"

    #  Effector position in the reference configuration, in the root frame
    ref_EE_lLeg = [0.1946, 0.14695, -0.223]
    ref_EE_rLeg = [0.1946, -0.14695, -0.223]
    ref_EE_lArm = [-0.1946, 0.14695, -0.223]
    ref_EE_rArm = [-0.1946, -0.14695, -0.223]
    dict_ref_effector_from_root = {
        rLegId: ref_EE_rLeg,
        lLegId: ref_EE_lLeg,
        rArmId: ref_EE_rArm,
        lArmId: ref_EE_lArm,
    }

    def __init__(self, name=None, load=True, client=None, clientRbprm=None):
        if name is not None:
            self.name = name
        Parent.__init__(
            self, self.name, self.rootJointType, load, client, None, clientRbprm
        )
        self.setReferenceEndEffector("solo_LFleg_rom", self.ref_EE_lLeg)
        self.setReferenceEndEffector("solo_RFleg_rom", self.ref_EE_rLeg)
        self.setReferenceEndEffector("solo_LHleg_rom", self.ref_EE_lArm)
        self.setReferenceEndEffector("solo_RHleg_rom", self.ref_EE_rArm)
