"""Polyhedron Potential model functions

Extended description of the module

Notes
-----
    This is an example of an indented section. It's like any other section,
    but the body is indented to help it stand out from surrounding text.

If a section is indented, then a section break is created by
resuming unindented text.

Attributes
----------
module_level_variable1 : int
    Descrption of the variable

Author
------
Shankar Kulumani		GWU		skulumani@gwu.edu
"""
import numpy as np

def face_contribution_loop(r_v, Fa, F_face, w_face):
    U_face = 0
    U_grad_face = np.zeros(3)
    U_grad_mat_face = np.zeros((3,3))

    for ii in range(Fa.shape[0]):

        U_face = U_face + \
            r_v[Fa[ii], :].dot(F_face[:, :, ii]).dot(
                r_v[Fa[ii], :].T) * w_face[ii, 0]
        U_grad_face = U_grad_face + \
            F_face[:, :, ii].dot(r_v[Fa[ii], :].T) * w_face[ii, 0]
        U_grad_mat_face = U_grad_mat_face + \
            F_face[:, :, ii] * w_face[ii]
    
    return U_face, U_grad_face, U_grad_mat_face

def face_contribution(r_v, Fa, F_face, w_face):

    return
