#!/bin/bash

cd $DEVEL_DIR/src/solo-rbprm/script;
starthpp relativeFootPositionQuasiFlat.py
starthpp generateROMS.py
/local/pfernbac/blender-2.82a-linux64/blender --background --python reduce.py

#for f in ./output/*reduced.obj; do mv "$f" "${f%.obj}.obj"; done
#cp ./output/* $DEVEL_DIR/src/solo-rbprm/data/
#cd -
