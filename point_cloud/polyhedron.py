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

    return w_face

def edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map):
    
    num_f = e1.shape[0]

    r1i = r_v[e1_vertex_map[:, 0], :]
    r1j = r_v[e1_vertex_map[:, 1], :]
    r1i_norm = np.sqrt(np.sum(r1i**2, axis=1))
    r1j_norm = np.sqrt(np.sum(r1j**2, axis=1))
    e1_norm = np.sqrt(np.sum(e1**2, axis=1))
    L1_edge = np.log((r1i_norm + r1j_norm + e1_norm) /
                        (r1i_norm + r1j_norm - e1_norm)).reshape((num_f, 1))

    r2i = r_v[e2_vertex_map[:, 0], :]
    r2j = r_v[e2_vertex_map[:, 1], :]
    r2i_norm = np.sqrt(np.sum(r2i**2, axis=1))
    r2j_norm = np.sqrt(np.sum(r2j**2, axis=1))
    e2_norm = np.sqrt(np.sum(e2**2, axis=1))
    L2_edge = np.log((r2i_norm + r2j_norm + e2_norm) /
                        (r2i_norm + r2j_norm - e2_norm)).reshape((num_f, 1))

    r3i = r_v[e3_vertex_map[:, 0], :]
    r3j = r_v[e3_vertex_map[:, 1], :]
    r3i_norm = np.sqrt(np.sum(r3i**2, axis=1))
    r3j_norm = np.sqrt(np.sum(r3j**2, axis=1))
    e3_norm = np.sqrt(np.sum(e3**2, axis=1))
    L3_edge = np.log((r3i_norm + r3j_norm + e3_norm) /
                        (r3i_norm + r3j_norm - e3_norm)).reshape((num_f, 1))
    
    return L1_edge, L2_edge, L3_edge

def map_edge_factor(r_v, e1, e2, e3, e1_vertex_map, e2_vertex_map, e3_vertex_map):

    num_f = e1.shape[0]
    
    def map_edge(edge_tuple):
        ri, rj, e = edge_tuple
        ri_norm = np.sqrt(np.sum(ri**2, axis=1))
        rj_norm = np.sqrt(np.sum(rj**2, axis=1))
        e_norm = np.sqrt(np.sum(e**2, axis=1))
        L_edge = np.log((ri_norm + rj_norm + e_norm) / 
                        (ri_norm + rj_norm - e_norm)).reshape((num_f, 1))
        
        return L_edge

    L1_edge = map_edge((r_v[e1_vertex_map[:, 0], :], r_v[e1_vertex_map[:, 1], :], e1))

    L2_edge = map_edge((r_v[e2_vertex_map[:, 0], :], r_v[e2_vertex_map[:, 1], :], e2))
    L3_edge = map_edge((r_v[e3_vertex_map[:, 0], :], r_v[e3_vertex_map[:, 1], :], e3))
   
    return L1_edge, L2_edge, L3_edge
