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

# TODO: Add documentation
def face_contribution(r_v, Fa, F_face, w_face):

    ra = r_v[Fa, :]
    # rv^T F_face = np.einsum('...j,jk...', ra, F_face)
    radotFface = np.einsum('...j,jk...', ra, F_face)

    U_face = np.sum(np.einsum('...j,...j', radotFface, ra) * w_face[:,0])
    U_grad_face = np.sum(radotFface * w_face, axis=0)
    U_grad_mat_face = np.sum(F_face * w_face[:, 0], axis=2)

    return U_face, U_grad_face, U_grad_mat_face

# TODO: Add documentation
def laplacian_factor(r_v, Fa, Fb, Fc):
    num_f = Fa.shape[0]

    # vectorize w_face calculation
    ri = r_v[Fa, :]
    ri_norm = np.sqrt(np.sum(ri**2, axis=1))

    rj = r_v[Fb, :]
    rj_norm = np.sqrt(np.sum(rj**2, axis=1))

    rk = r_v[Fc, :]
    rk_norm = np.sqrt(np.sum(rk**2, axis=1))

    rjrk_cross = np.cross(rj, rk)
    num = np.sum(ri * rjrk_cross, axis=1)

    rjrk_dot = np.sum(rj * rk, axis=1)
    rkri_dot = np.sum(rk * ri, axis=1)
    rirj_dot = np.sum(ri * rj, axis=1)

    den = ri_norm * rj_norm * rk_norm + ri_norm * \
        rjrk_dot + rj_norm * rkri_dot + rk_norm * rirj_dot

    w_face = 2.0 * np.arctan2(num, den).reshape((num_f, 1))
