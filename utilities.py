from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np
import sys
import pdb

def save_to_interactive(dct):
    """Save variables from debugger to main interpreter
    
    import utilities
    utilities.save_to_interactive({'F1':F1, 'F2':F2, 'M1':M1, 'M2':M2})
    """

    # Be safe and define a maximum of frames we're trying to walk up
    MAX_FRAMES = 20

    n = 0
    # Walk up the stack looking for '__name__'
    # with a value of '__main__' in frame globals
    for n in range(MAX_FRAMES):
        cur_frame = sys._getframe(n)
        name = cur_frame.f_globals.get('__name__')
        if name == '__main__':
            # Yay - we're in the stack frame of the interactive interpreter!
            # So we update its frame globals with the dict containing our data
            cur_frame.f_globals.update(dct)
            break

def ismember_rows(a, b):
    '''Equivalent of 'ismember' from Matlab
    a.shape = (nRows_a, nCol)
    b.shape = (nRows_b, nCol)
    return the idx where b[idx] == a
    '''
    invalid = -1
    indx = np.full(a.shape[0],invalid,dtype='int')

    indxa, indxb = np.nonzero(np.all(b == a[:,np.newaxis], axis=2))

    indx[indxa] = indxb
    indx[indxb] = indxa

    return indx

def ismember(a,b):

    indx = np.nonzero(np.all(b == a[:,np.newaxis], axis=2))[0]

    return indx

def asvoid(arr):
    """
    View the array as dtype np.void (bytes)
    This views the last axis of ND-arrays as bytes so you can perform comparisons on
    the entire row.
    http://stackoverflow.com/a/16840350/190597 (Jaime, 2013-05)
    Warning: When using asvoid for comparison, note that float zeros may compare UNEQUALLY
    >>> asvoid([-0.]) == asvoid([0.])
    array([False], dtype=bool)
    """
    arr = np.ascontiguousarray(arr)
    return arr.view(np.dtype((np.void, arr.dtype.itemsize * arr.shape[-1])))

def in1d_index(a, b):
    voida, voidb = map(asvoid, (a, b))
    return np.where(np.in1d(voidb, voida))[0]

# TODO: Add documentation and exmaple
def ismember_index(a,b):
    """Find index of matching elements
    
    Actually find the index of elements and their match. 
    """
    invalid = -1

    a[a==-0.0] = 0
    b[b==-0.0] = 0
    
    voida, voidb = map(asvoid,(a,b))

    index = np.full(a.shape[0], invalid, dtype='int')
    
    for ii in range(a.shape[0]):
        match = np.where(voida[ii] == voidb)[0]

        if match.size:
            index[ii] = match[0]

    return index

# TODO: Documentation
def search_index(a, b):
    """Memory intensive way to find matches. have to search across vertex map

    https://stackoverflow.com/questions/8251541/numpy-for-every-element-in-one-array-find-the-index-in-another-array
    """
    invalid = -1
    lenb = len(b)
    lena = len(a)

    ae = np.broadcast_to(b, (lena, lenb))
    be = np.broadcast_to(a, (lenb, lena)).T

    inda, indb = np.where(np.equal(ae, be))
    
    # returns locations where there are matches ( and the elements to find them)
    # inda - index for each element corresponds to the match given in indb
    # so a[inda[0]] = b[indb[0]] and a[inda[1]] = b[indb[1]]
    return inda, indb

if __name__ == "__main__":
    
    print("Some versions of trying to duplicate Matlab's ismember function.")
