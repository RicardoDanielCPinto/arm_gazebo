#!/bin/bash

#Ricardo Pinto
#Este programa serve para garantir que os ficheiros URDF e SDF são gerados e atualizados
#através dos seus ficheiros PAI .xacro e .erb respetivamente

erb model.erb > model.sdf
xacro model.urdf.xacro > model.urdf
