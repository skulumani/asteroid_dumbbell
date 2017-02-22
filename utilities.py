import numpy as np

def ismember_rows(a, b):
    '''Equivalent of 'ismember' from Matlab
    a.shape = (nRows_a, nCol)
    b.shape = (nRows_b, nCol)
    return the idx where b[idx] == a
    '''
    invalid = -1
    indx = np.full(a.shape[0],invalid,dtype='long')

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

def ismember_index(a,b):
    """Find index of matching elements
    
    Actually find the index of elements and their match. 
    """
    invalid = -1

    a[a==-0.0] = 0
    b[b==-0.0] = 0
    
    voida, voidb = map(asvoid,(a,b))

    index = np.full(a.shape[0], invalid, dtype='long')
    
    for ii in range(a.shape[0]):
        match = np.where(voida[ii] == voidb)[0]

        if match.size:
            index[ii] = match[0]

    return index

if __name__ == "__main__":
    
    print("Some versions of trying to duplicate Matlab's ismember function.")
