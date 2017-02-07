import numpy as np

def ismember_rows(a, b):
    '''Equivalent of 'ismember' from Matlab
    a.shape = (nRows_a, nCol)
    b.shape = (nRows_b, nCol)
    return the idx where b[idx] == a
    '''
    indx = np.nonzero(np.all(b == a[:,np.newaxis], axis=2))[1]

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

    voida, voidb = map(asvoid,(a,b))

    index = np.full(a.shape[0], -1, dtype='int8')
    
    for ii in range(a.shape[0]):
        match = np.where(voida[ii] == voidb)[0]

        if match.size:
            index[ii] = match[0]

    return index

if __name__ == "__main__":
    a = np.array([[4, 6,5],[2, 6,5],[5, 2,5]])
    b = np.array([[1, 7,5],[1, 8,5],[2, 6,5],[2, 1,5],[2, 4,5],[4, 6,5],[4, 7,5],[5, 9,5],[5, 2,5],[5, 1,5]])
    idx = ismember_rows(a, b)
    print(idx)
    print(np.all(b[idx] == a))

    indx = in1d_index(a,b)
    print(indx)

    e1 = np.array([[1, 2,3],[4,5,6],[7,8,9],[-1,-2,-3]])
    print(ismember_rows(e1,-e1))
    print(in1d_index(e1,-e1))
